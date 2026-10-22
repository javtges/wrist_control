#!/usr/bin/env python3
"""
wrist_state_publisher — everything the wrist reports, in one stamped message at a fixed rate.

Per axis it gathers hsa_odrive_control's measured extension angles and the last commanded
angles, the ODrive CAN nodes' ControllerStatus (position, velocity, torque, Iq, errors) and
ODriveStatus (bus voltage and current, temperatures, errors) into hsa_wrist_msgs/WristState on
/hsa/wrist/state. One topic to record, plot or feed a policy, instead of fourteen unstamped ones.

The ODrive messages carry no header, so every source is keyed on its receive time; the message
header is this node's clock at publish time and each *_age_s field says how old that source's
newest sample was then. Values are held (not cleared) when a source goes quiet: the age is the
freshness signal.

Topics subscribed:
  /hsa_odrive_control/joint_states    (std_msgs/Float32MultiArray)  measured extension angles, deg
  /hsa_odrive_control/joint_commands  (std_msgs/Float32MultiArray)  commanded extension angles, deg
  /odrive_axis{N}/controller_status   (odrive_can/ControllerStatus)
  /odrive_axis{N}/odrive_status       (odrive_can/ODriveStatus)

Topic published:
  /hsa/wrist/state  (hsa_wrist_msgs/WristState)

Parameters:
  num_axes  (int)    6
  rate      (float)  100.0 Hz
  topic     (str)    /hsa/wrist/state
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float32MultiArray
from odrive_can.msg import ControllerStatus, ODriveStatus
from hsa_wrist_msgs.msg import WristState

NAN = float('nan')

#: ControllerStatus fields copied per axis, in message order: (source attribute, WristState field).
CONTROLLER_FIELDS = (
    ('pos_estimate', 'motor_pos_turns'),
    ('vel_estimate', 'motor_vel_turns_s'),
    ('torque_target', 'torque_target_nm'),
    ('torque_estimate', 'torque_estimate_nm'),
    ('iq_setpoint', 'iq_setpoint_a'),
    ('iq_measured', 'iq_measured_a'),
)
#: ODriveStatus fields copied per axis.
ODRIVE_FIELDS = (
    ('bus_voltage', 'bus_voltage_v'),
    ('bus_current', 'bus_current_a'),
    ('fet_temperature', 'fet_temperature_c'),
    ('motor_temperature', 'motor_temperature_c'),
)


def _age(now: float, t: float | None) -> float:
    return NAN if t is None else max(0.0, now - t)


def build_state(
    now: float,
    n: int,
    joints: tuple[float, list[float]] | None,
    command: tuple[float, list[float]] | None,
    controller: list[tuple[float, dict] | None],
    odrive: list[tuple[float, dict] | None],
) -> dict:
    """
    Assemble the WristState fields from the newest sample of each source.

    ``joints`` / ``command`` are ``(receive_time, values)``; ``controller[i]`` / ``odrive[i]`` are
    ``(receive_time, {attribute: value})`` per axis, or None before the first sample. Pure, so the
    aggregation is testable without ROS. Returns {field name: list} ready to set on the message.
    """
    out: dict[str, list] = {
        'extension_deg': list(joints[1]) if joints else [NAN] * n,
        'command_deg': list(command[1]) if command else [NAN] * n,
        'extension_age_s': _age(now, joints[0] if joints else None),
        'command_age_s': _age(now, command[0] if command else None),
    }
    for _, field in CONTROLLER_FIELDS:
        out[field] = [NAN] * n
    out['axis_state'] = [0] * n
    out['controller_errors'] = [0] * n
    out['controller_age_s'] = [NAN] * n
    for _, field in ODRIVE_FIELDS:
        out[field] = [NAN] * n
    out['odrive_errors'] = [0] * n
    out['disarm_reason'] = [0] * n
    out['odrive_age_s'] = [NAN] * n

    for i in range(n):
        c = controller[i] if i < len(controller) else None
        if c is not None:
            t, d = c
            for src, field in CONTROLLER_FIELDS:
                out[field][i] = float(d[src])
            out['axis_state'][i] = int(d['axis_state'])
            out['controller_errors'][i] = int(d['active_errors'])
            out['controller_age_s'][i] = _age(now, t)
        o = odrive[i] if i < len(odrive) else None
        if o is not None:
            t, d = o
            for src, field in ODRIVE_FIELDS:
                out[field][i] = float(d[src])
            out['odrive_errors'][i] = int(d['active_errors'])
            out['disarm_reason'][i] = int(d['disarm_reason'])
            out['odrive_age_s'][i] = _age(now, t)
    # Pad or trim the two Float32MultiArray sources to n, so a short message cannot shift columns.
    for key in ('extension_deg', 'command_deg'):
        vals = out[key][:n]
        out[key] = vals + [NAN] * (n - len(vals))
    return out


class WristStatePublisher(Node):
    """Aggregate the per-axis topics and publish WristState at ``rate``."""

    def __init__(self) -> None:
        super().__init__('wrist_state_publisher')
        self.declare_parameter('num_axes', 6)
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('topic', '/hsa/wrist/state')
        self.n = int(self.get_parameter('num_axes').value)
        rate = float(self.get_parameter('rate').value)
        topic = str(self.get_parameter('topic').value)

        self._joints: tuple[float, list[float]] | None = None
        self._command: tuple[float, list[float]] | None = None
        self._controller: list[tuple[float, dict] | None] = [None] * self.n
        self._odrive: list[tuple[float, dict] | None] = [None] * self.n

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Float32MultiArray, '/hsa_odrive_control/joint_states', self._on_joints, qos)
        self.create_subscription(Float32MultiArray, '/hsa_odrive_control/joint_commands', self._on_command, 10)
        for i in range(self.n):
            self.create_subscription(
                ControllerStatus, f'/odrive_axis{i}/controller_status',
                lambda msg, idx=i: self._on_controller(idx, msg), qos)
            self.create_subscription(
                ODriveStatus, f'/odrive_axis{i}/odrive_status',
                lambda msg, idx=i: self._on_odrive(idx, msg), qos)
        self._pub = self.create_publisher(WristState, topic, 10)
        self.create_timer(1.0 / rate, self._publish)
        self.create_timer(5.0, self._report)
        self.get_logger().info(f'wrist_state_publisher: {self.n} axes -> {topic} @ {rate:g} Hz')

    # ------------------------------------------------------------------ callbacks

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_joints(self, msg: Float32MultiArray) -> None:
        self._joints = (self._now(), [float(v) for v in msg.data])

    def _on_command(self, msg: Float32MultiArray) -> None:
        self._command = (self._now(), [float(v) for v in msg.data])

    def _on_controller(self, idx: int, msg: ControllerStatus) -> None:
        self._controller[idx] = (self._now(), {
            **{src: getattr(msg, src) for src, _ in CONTROLLER_FIELDS},
            'axis_state': msg.axis_state, 'active_errors': msg.active_errors,
        })

    def _on_odrive(self, idx: int, msg: ODriveStatus) -> None:
        self._odrive[idx] = (self._now(), {
            **{src: getattr(msg, src) for src, _ in ODRIVE_FIELDS},
            'active_errors': msg.active_errors, 'disarm_reason': msg.disarm_reason,
        })

    # ------------------------------------------------------------------ publish

    def _publish(self) -> None:
        now = self.get_clock().now()
        fields = build_state(now.nanoseconds * 1e-9, self.n, self._joints, self._command,
                             self._controller, self._odrive)
        msg = WristState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'hsa_wrist'
        for key, value in fields.items():
            setattr(msg, key, value)
        self._pub.publish(msg)

    def _report(self) -> None:
        now = self._now()
        ctrl = sum(1 for c in self._controller if c is not None and now - c[0] < 1.0)
        odrv = sum(1 for o in self._odrive if o is not None and now - o[0] < 1.0)
        ext = 'none' if self._joints is None else ' '.join(f'{v:.0f}' for v in self._joints[1])
        self.get_logger().info(
            f'extension [{ext}] deg | controller_status fresh on {ctrl}/{self.n} axes, '
            f'odrive_status on {odrv}/{self.n}' + (' | no joint_commands yet' if self._command is None else ''))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WristStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
