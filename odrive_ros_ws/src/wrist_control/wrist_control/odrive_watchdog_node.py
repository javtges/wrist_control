#!/usr/bin/env python3
"""
odrive_watchdog_node: trips a latched fault and E-stops every ODrive if any
axis breaks a safety limit.

Limits (all parameters, checked per axis):
  1. FET temperature above max_temperature_c (80 °C). Also the motor
     thermistor if check_motor_temperature is true.
  2. Instantaneous |current| above max_current_a (3.5 A).
  3. 1-minute average |current| above avg_1min_limit_a (3.0 A).
  4. 5-minute average |current| above avg_5min_limit_a (2.0 A).
  Plus: an axis whose status messages stop arriving for stale_timeout_s, or
  that reports a NaN reading, also trips. Without data the limits can't be
  checked.

"current" is the motor phase current (controller_status.iq_measured) when
current_source is "phase", or the DC bus current (odrive_status.bus_current)
when it is "bus". Averages are time-weighted. Until a window has filled, they
cover only the time seen so far, and are checked once they span
min_avg_duration_s.

On a trip (any axis, any limit):
  * Sends the CAN Simple Estop frame (cmd 0x002) to all six ODrives,
    directly on SocketCAN rather than through odrive_can_node, so it still
    works if a CAN node has died. Each ODrive disarms and records
    ESTOP_REQUESTED in active_errors.
  * Latches the fault. While latched, any axis seen out of IDLE gets another
    Estop (10 Hz). That catches an enable that bypasses hsa_odrive_control,
    e.g. odrivetool or calling request_axis_state directly, which clears
    errors before arming.
  * hsa_odrive_control refuses ~/enable until the fault is reset.

A fault is NOT cleared by the limit going away. Call ~/reset_fault. It only
succeeds once every axis has fresh data, temperatures are
temperature_reset_margin_c below the limit, and current and both averages
are back under their limits. The 5-minute average decays slowly, so a reset
can be refused for minutes after a current trip. Then it sends Clear_Errors
to every ODrive.

Before every axis has reported at least once, status is STALE ("waiting")
and enable is refused, but nothing is latched. Power on the ODrives and it
turns OK on its own.

Topics subscribed:
  /odrive_axis{N}/odrive_status      (odrive_can/ODriveStatus)   temps, bus current
  /odrive_axis{N}/controller_status  (odrive_can/ControllerStatus) iq, axis_state

Topics published:
  ~/status  (diagnostic_msgs/DiagnosticStatus, latched, 10 Hz)
            level OK = all limits OK, ERROR = fault latched (message says why),
            STALE = still waiting for data. values = per-axis readings.

Services provided:
  ~/reset_fault  (std_srvs/Trigger)  clear a latched fault if all limits are OK
  ~/trip         (std_srvs/Trigger)  trip a fault by hand, to test the chain

Parameters:
  can_interface              (str)    SocketCAN interface for Estop frames, "can0"
  node_ids                   (int[])  ODrive CAN node_id of axis 0..N-1, [0..5]
  current_source             (str)    "phase" (iq_measured) or "bus" (bus_current)
  max_temperature_c          (float)  80.0
  check_motor_temperature    (bool)   false. Enable only with a motor thermistor fitted
  max_current_a              (float)  3.5
  avg_1min_limit_a           (float)  3.0
  avg_5min_limit_a           (float)  2.0
  min_avg_duration_s         (float)  10.0
  stale_timeout_s            (float)  1.0
  temperature_reset_margin_c (float)  5.0

Usage:
  Started by `ros2 launch odrive_can wrist_launch.yaml`. Standalone:
    ros2 run wrist_control odrive_watchdog_node
  Watch it:     ros2 topic echo /odrive_watchdog/status
  Test a trip:  ros2 service call /odrive_watchdog/trip std_srvs/srv/Trigger
  Reset:        ros2 service call /odrive_watchdog/reset_fault std_srvs/srv/Trigger
"""

import math
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from std_srvs.srv import Trigger
from odrive_can.msg import ControllerStatus, ODriveStatus

from .watchdog_limits import AxisMonitor, Limits

AXIS_STATE_IDLE = 1

CMD_ESTOP = 0x002
CMD_CLEAR_ERRORS = 0x018

TICK_HZ = 10.0

STATUS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def can_frame(node_id: int, cmd_id: int, data: bytes = b'') -> bytes:
    """Pack a classic CAN frame (struct can_frame) for a SocketCAN raw socket,
    with the ODrive CAN Simple id layout node_id << 5 | cmd_id."""
    return struct.pack('=IB3x8s', (node_id << 5) | cmd_id, len(data), data.ljust(8, b'\x00'))


class CanSender:
    """Send-only raw SocketCAN socket. Needs no root; the interface must be up."""

    def __init__(self, interface: str) -> None:
        self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # Empty filter: receive nothing, so the unread receive buffer never fills.
        self._sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, b'')
        self._sock.bind((interface,))

    def send(self, node_id: int, cmd_id: int, data: bytes = b'') -> None:
        self._sock.send(can_frame(node_id, cmd_id, data))

    def close(self) -> None:
        self._sock.close()


class ODriveWatchdog(Node):
    def __init__(self) -> None:
        super().__init__('odrive_watchdog')

        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('node_ids', [0, 1, 2, 3, 4, 5])
        self.declare_parameter('current_source', 'phase')
        self.declare_parameter('max_temperature_c', 80.0)
        self.declare_parameter('check_motor_temperature', False)
        self.declare_parameter('max_current_a', 3.5)
        self.declare_parameter('avg_1min_limit_a', 3.0)
        self.declare_parameter('avg_5min_limit_a', 2.0)
        self.declare_parameter('min_avg_duration_s', 10.0)
        self.declare_parameter('stale_timeout_s', 1.0)
        self.declare_parameter('temperature_reset_margin_c', 5.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._interface: str = p('can_interface')
        self._node_ids: list[int] = list(p('node_ids'))
        self._current_source: str = p('current_source')
        if self._current_source not in ('phase', 'bus'):
            raise ValueError(f"current_source must be 'phase' or 'bus', got {self._current_source!r}")

        self._limits = Limits(
            max_temperature_c=float(p('max_temperature_c')),
            check_motor_temperature=bool(p('check_motor_temperature')),
            max_current_a=float(p('max_current_a')),
            avg_windows=[(60.0, float(p('avg_1min_limit_a'))), (300.0, float(p('avg_5min_limit_a')))],
            min_avg_duration_s=float(p('min_avg_duration_s')),
            stale_timeout_s=float(p('stale_timeout_s')),
            temperature_reset_margin_c=float(p('temperature_reset_margin_c')),
        )

        self.n = len(self._node_ids)
        self._monitors = [AxisMonitor(self._limits) for _ in range(self.n)]
        self._axis_states: list[int | None] = [None] * self.n

        self._faulted = False
        self._fault_reason = ''

        self._status_pub = self.create_publisher(DiagnosticStatus, '~/status', STATUS_QOS)

        for i in range(self.n):
            self.create_subscription(
                ODriveStatus, f'/odrive_axis{i}/odrive_status',
                lambda msg, idx=i: self._on_odrive_status(idx, msg), SENSOR_QOS)
            self.create_subscription(
                ControllerStatus, f'/odrive_axis{i}/controller_status',
                lambda msg, idx=i: self._on_controller_status(idx, msg), SENSOR_QOS)

        self.create_service(Trigger, '~/reset_fault', self._svc_reset_fault)
        self.create_service(Trigger, '~/trip', self._svc_trip)

        self._can: CanSender | None = None
        self._open_can()

        self.create_timer(1.0 / TICK_HZ, self._tick)

        lim = self._limits
        self.get_logger().info(
            f'Watching {self.n} axes (node_ids {self._node_ids}) on {self._interface}: '
            f'temp > {lim.max_temperature_c:g} °C, {self._current_source} current > '
            f'{lim.max_current_a:g} A, 1-min avg > {lim.avg_windows[0][1]:g} A, '
            f'5-min avg > {lim.avg_windows[1][1]:g} A, data stale > {lim.stale_timeout_s:g} s.'
        )
        self._publish_status()

    # ------------------------------------------------------------------
    # CAN
    # ------------------------------------------------------------------

    def _open_can(self) -> bool:
        if self._can is not None:
            return True
        try:
            self._can = CanSender(self._interface)
            return True
        except OSError as e:
            self._can = None
            # Can't E-stop without CAN, so this is itself a fault.
            self._trip(f'cannot open CAN interface {self._interface!r} for Estop: {e}')
            return False

    def _send_all(self, cmd_id: int, data: bytes, indices) -> None:
        if self._can is None and not self._open_can():
            return
        for i in indices:
            try:
                self._can.send(self._node_ids[i], cmd_id, data)
            except OSError as e:
                self.get_logger().error(
                    f'CAN send (cmd 0x{cmd_id:03x}) to axis {i} failed: {e}',
                    throttle_duration_sec=1.0)

    def _estop(self, indices) -> None:
        self._send_all(CMD_ESTOP, b'', indices)

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    def _on_odrive_status(self, i: int, msg: ODriveStatus) -> None:
        now = time.monotonic()
        self._monitors[i].update_temperature(now, msg.fet_temperature, msg.motor_temperature)
        if self._current_source == 'bus':
            self._monitors[i].update_current(now, msg.bus_current)
        self._check_axis(i, now)

    def _on_controller_status(self, i: int, msg: ControllerStatus) -> None:
        now = time.monotonic()
        self._axis_states[i] = msg.axis_state
        if self._current_source == 'phase':
            self._monitors[i].update_current(now, msg.iq_measured)
        self._check_axis(i, now)

    def _check_axis(self, i: int, now: float) -> None:
        if self._faulted:
            return
        violations = self._monitors[i].violations(now)
        if violations:
            self._trip(f'axis {i}: ' + '; '.join(violations))

    # ------------------------------------------------------------------
    # Fault handling
    # ------------------------------------------------------------------

    def _trip(self, reason: str) -> None:
        if self._faulted:
            return
        self._faulted = True
        self._fault_reason = reason
        self.get_logger().error(f'SAFETY FAULT, E-stopping all ODrives: {reason}')
        self._estop(range(self.n))
        self._publish_status()

    def _tick(self) -> None:
        now = time.monotonic()
        if self._faulted:
            # Keep any axis that comes out of IDLE disarmed.
            not_idle = [i for i, s in enumerate(self._axis_states) if s != AXIS_STATE_IDLE]
            if not_idle:
                self._estop(not_idle)
        else:
            # Catches data that has stopped arriving (no callback fires then).
            for i in range(self.n):
                self._check_axis(i, now)
        self._publish_status()

    def _svc_reset_fault(self, _req: Trigger.Request, resp: Trigger.Response):
        if not self._faulted:
            resp.success = True
            resp.message = 'No fault latched.'
            return resp
        now = time.monotonic()
        problems = [f'axis {i}: ' + '; '.join(v)
                    for i, m in enumerate(self._monitors)
                    if (v := m.violations(now, resetting=True))]
        if self._can is None and not self._open_can():
            problems.append(f'CAN interface {self._interface!r} unavailable')
        if problems:
            resp.success = False
            resp.message = 'Fault NOT reset, still out of limits: ' + ' | '.join(problems)
            self.get_logger().warning(resp.message)
            return resp

        self._faulted = False
        self._send_all(CMD_CLEAR_ERRORS, b'\x00', range(self.n))
        self.get_logger().warning(f'Fault reset (was: {self._fault_reason}). ODrive errors cleared.')
        self._fault_reason = ''
        self._publish_status()
        resp.success = True
        resp.message = 'Fault reset; ODrive errors cleared. Enable is allowed again.'
        return resp

    def _svc_trip(self, _req: Trigger.Request, resp: Trigger.Response):
        self._trip('manual trip via ~/trip')
        resp.success = True
        resp.message = f'Fault latched: {self._fault_reason}'
        return resp

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def _publish_status(self) -> None:
        msg = DiagnosticStatus()
        msg.name = 'odrive_watchdog'
        msg.hardware_id = self._interface
        waiting = [i for i, m in enumerate(self._monitors) if not m.has_data()]
        if self._faulted:
            msg.level = DiagnosticStatus.ERROR
            msg.message = f'FAULT: {self._fault_reason}'
        elif waiting:
            msg.level = DiagnosticStatus.STALE
            msg.message = f'waiting for data from axes {waiting}'
        else:
            msg.level = DiagnosticStatus.OK
            msg.message = 'all limits OK'

        def f(v):
            return 'none' if v is None else f'{v:.3f}'

        values = [KeyValue(key='current_source', value=self._current_source)]
        for i, m in enumerate(self._monitors):
            avg1, avg5 = m.average_values()
            values += [
                KeyValue(key=f'axis{i}.fet_temperature_c', value=f(m.fet_temperature)),
                KeyValue(key=f'axis{i}.current_a', value=f(m.current)),
                KeyValue(key=f'axis{i}.avg_1min_a', value=f(avg1)),
                KeyValue(key=f'axis{i}.avg_5min_a', value=f(avg5)),
                KeyValue(key=f'axis{i}.axis_state', value=str(self._axis_states[i])),
            ]
        msg.values = values
        self._status_pub.publish(msg)

    def destroy_node(self):
        if self._can is not None:
            self._can.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ODriveWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
