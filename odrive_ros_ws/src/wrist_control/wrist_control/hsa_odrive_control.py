#!/usr/bin/env python3
"""
hsa_odrive_control — ROS2 node for HSA wrist control via ODrive CAN.

Subscribes to extension-angle commands (degrees), converts them through the
HSA kinematics model, and publishes ControlMessage to each ODrive CAN node.

Topics subscribed:
  ~/joint_commands  (std_msgs/Float32MultiArray)  — 6 extension angles, degrees,
                     clamped to each HSA's [min_angle, max_angle]
  ~/jog             (std_msgs/Float32MultiArray)  — 6 extension-angle steps,
                     degrees (0 = leave that axis alone). Moves each stepped axis
                     from its current setpoint; may go below min_angle, down to
                     JOG_MIN_DEG, so the physical zero can be found below the
                     current zero. Only joint_commander_gui's jog buttons use it.

Topics published:
  /odrive_axis{N}/control_message  (odrive_can/ControlMessage)
  /hsa_zero_offsets (std_msgs/Float32MultiArray, transient local) — software
                     zero offset per axis in motor turns; odrive_position_monitor
                     adds it to /odrive_positions to get /hsa_positions

Zero offsets are loaded from each HSA's `zero_offset` in wrist.yaml at startup
and written back there by ~/zero_encoders and ~/reset_zero, so the physical
zero survives a restart of this node.

Services provided:
  ~/zero_encoders  (std_srvs/Empty)   — set the physical zero: snapshot current
                     positions as each axis's zero_point and save the offsets to
                     wrist.yaml. Called by joint_commander_gui's "Set Physical Zero".
  ~/reset_zero     (std_srvs/Empty)   — clear the software zero offset (in wrist.yaml
                     too), so extension angle 0 goes back to meaning the ODrive's
                     own raw pos_estimate 0
  ~/enable         (std_srvs/Trigger) — put all axes in CLOSED_LOOP_CONTROL
  ~/disable        (std_srvs/Trigger) — put all axes in IDLE

Services called:
  /odrive_axis{N}/request_axis_state  (odrive_can/AxisState)

Parameters:
  robot_config   (str)   — path to wrist.yaml; defaults to hsa_robots/wrist.yaml
  control_mode   (int)   — ODrive ControlMode (3 = position, 2 = velocity, 1 = torque)
  input_mode     (int)   — ODrive InputMode   (1 = passthrough, 5 = trap_traj)
  vel_ff         (float) — velocity feed-forward in turns/s (position mode only)
  torque_ff      (float) — torque feed-forward in Nm
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty, Trigger
from odrive_can.msg import ControlMessage, ControllerStatus
from odrive_can.srv import AxisState

from hsa_robots import HSARobot
from .conversion import (
    compute_extension_angle,
    compute_jog_setpoint,
    compute_setpoint,
    compute_zero_offset,
)
from .robot_config import default_robot_config

# ODrive axis state constants
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP = 8

# ODrive control mode defaults
CONTROL_MODE_POSITION = 3
INPUT_MODE_PASSTHROUGH = 1

# Latched so late subscribers (position monitor, GUI) get the current offsets.
ZERO_OFFSETS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class HSAODriveControl(Node):
    def __init__(self) -> None:
        super().__init__('hsa_odrive_control')

        # --- Parameters ---
        self.declare_parameter('robot_config', '')
        self.declare_parameter('control_mode', CONTROL_MODE_POSITION)
        self.declare_parameter('input_mode', INPUT_MODE_PASSTHROUGH)
        self.declare_parameter('vel_ff', 0.0)
        self.declare_parameter('torque_ff', 0.0)

        config_path: str = self.get_parameter('robot_config').value or default_robot_config()

        self.robot = HSARobot(config_path)
        self.n: int = len(self.robot.hsas)
        self.get_logger().info(f'Loaded {self.n}-axis robot from {config_path}')

        # Per-axis state: latest position estimates, software zero offsets and
        # the last setpoint sent (raw ODrive turns; None = not commanded since
        # the last enable/disable, so a jog starts from the measured position)
        self._pos_estimate: list[float | None] = [None] * self.n
        self._zero_offsets: list[float] = [h.zero_offset for h in self.robot.hsas]
        self._setpoints: list[float | None] = [None] * self.n

        # QoS: reliable, keep-last-10 for control topics
        ctrl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers: one ControlMessage publisher per ODrive axis
        self._ctrl_pubs = [
            self.create_publisher(
                ControlMessage,
                f'/odrive_axis{i}/control_message',
                ctrl_qos,
            )
            for i in range(self.n)
        ]

        # Publisher: software zero offsets (motor turns), one per axis
        self._zero_offsets_pub = self.create_publisher(
            Float32MultiArray, '/hsa_zero_offsets', ZERO_OFFSETS_QOS
        )

        # Subscribers: controller_status per axis for position feedback
        for i in range(self.n):
            self.create_subscription(
                ControllerStatus,
                f'/odrive_axis{i}/controller_status',
                lambda msg, idx=i: self._on_controller_status(idx, msg),
                ctrl_qos,
            )

        # Subscriber: joint commands
        self.create_subscription(
            Float32MultiArray,
            '~/joint_commands',
            self._on_joint_commands,
            10,
        )
        self.create_subscription(Float32MultiArray, '~/jog', self._on_jog, 10)

        # Services provided by this node
        self.create_service(Empty, '~/zero_encoders', self._svc_zero_encoders)
        self.create_service(Empty, '~/reset_zero', self._svc_reset_zero)
        self.create_service(Trigger, '~/enable', self._svc_enable)
        self.create_service(Trigger, '~/disable', self._svc_disable)

        # Service clients for per-axis state transitions (fire-and-forget)
        self._state_clients = [
            self.create_client(AxisState, f'/odrive_axis{i}/request_axis_state')
            for i in range(self.n)
        ]

        self._publish_zero_offsets()

        self.get_logger().info('hsa_odrive_control ready.')
        self.get_logger().info(
            f'Zero offsets from {config_path} (motor turns): '
            + ', '.join(f'{o:+.4f}' for o in self._zero_offsets)
        )
        self.get_logger().info(
            'These assume the ODrive positions are unchanged since the zero was set. '
            "If an ODrive has rebooted, check /hsa_positions and use joint_commander_gui's "
            'Set Physical Zero to re-zero before commanding motion.'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_controller_status(self, axis_idx: int, msg: ControllerStatus) -> None:
        self._pos_estimate[axis_idx] = msg.pos_estimate

    def _on_joint_commands(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != self.n:
            self.get_logger().error(
                f'joint_commands: expected {self.n} values, got {len(msg.data)}'
            )
            return
        self._publish_commands(list(msg.data))

    def _on_jog(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != self.n:
            self.get_logger().error(f'jog: expected {self.n} values, got {len(msg.data)}')
            return
        for i, (hsa, step) in enumerate(zip(self.robot.hsas, msg.data)):
            if step == 0.0 or math.isnan(step):
                continue
            # Step from what the axis is being held at; fall back to where it
            # actually is if it hasn't been commanded since enable.
            base_turns = self._setpoints[i]
            if base_turns is None:
                base_turns = self._pos_estimate[i]
            if base_turns is None:
                self.get_logger().warning(f'jog: axis {i} has no position feedback yet; ignored.')
                continue
            base_deg = compute_extension_angle(base_turns, hsa, self._zero_offsets[i])
            target_deg = base_deg + float(step)
            setpoint = compute_jog_setpoint(target_deg, hsa, self._zero_offsets[i])
            self._send_setpoint(i, setpoint)
            reached = compute_extension_angle(setpoint, hsa, self._zero_offsets[i])
            self.get_logger().info(f'jog axis {i}: {base_deg:+.2f}° → {reached:+.2f}°')

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _publish_commands(self, extension_angles_deg: list[float]) -> None:
        for i, (hsa, angle_deg) in enumerate(zip(self.robot.hsas, extension_angles_deg)):
            self._send_setpoint(i, compute_setpoint(angle_deg, hsa, self._zero_offsets[i]))

    def _send_setpoint(self, axis_idx: int, setpoint_turns: float) -> None:
        cmd = ControlMessage()
        cmd.control_mode = int(self.get_parameter('control_mode').value)
        cmd.input_mode = int(self.get_parameter('input_mode').value)
        cmd.input_pos = float(setpoint_turns)
        cmd.input_vel = float(self.get_parameter('vel_ff').value)
        cmd.input_torque = float(self.get_parameter('torque_ff').value)
        self._ctrl_pubs[axis_idx].publish(cmd)
        self._setpoints[axis_idx] = float(setpoint_turns)

    def _publish_zero_offsets(self) -> None:
        msg = Float32MultiArray()
        msg.data = [float(o) for o in self._zero_offsets]
        self._zero_offsets_pub.publish(msg)

    def _log_previous_offsets(self, action: str) -> None:
        """Log the offsets about to be replaced at full precision, so a zero
        replaced by mistake can be restored by hand into wrist.yaml."""
        self.get_logger().info(
            f'{action}: previous zero_offset values (motor turns): '
            + ', '.join(repr(float(o)) for o in self._zero_offsets)
        )

    def _save_zero_offsets(self) -> None:
        """Persist the offsets as each HSA's zero_offset in wrist.yaml."""
        try:
            self.robot.save_zero_offsets(self._zero_offsets)
        except Exception as e:
            self.get_logger().error(
                f'Could not save zero offsets to {self.robot.config_path}: {e}. '
                'They apply until this node restarts.'
            )
            return
        self.get_logger().info(f'Zero offsets saved to {self.robot.config_path}.')

    def _request_axis_state_all(self, state: int) -> list[int]:
        """Request `state` on every axis, one at a time with a small stagger
        between them.

        Each request can produce a multi-frame CAN burst on the target axis
        (send_startup_gains + ClearErrors + SetAxisState). Firing all six
        axes' requests back-to-back lets those bursts overlap on the bus;
        since CAN arbitration is `node_id << 5 | cmd_id`, higher-node_id
        axes (3, 4, 5) always lose that contention and can intermittently
        get starved. Spacing the requests out keeps only one axis's burst
        on the bus at a time.

        Returns the indices of axes whose service never became available,
        so callers can report the failure instead of leaving those axes
        silently stuck in their prior state."""
        req = AxisState.Request()
        req.axis_requested_state = state

        per_axis_timeout_sec = 2.0
        stagger_sec = 0.05

        unavailable: list[int] = []
        for i, client in enumerate(self._state_clients):
            if client.wait_for_service(timeout_sec=per_axis_timeout_sec):
                client.call_async(req)
            else:
                self.get_logger().error(
                    f'Axis {i} request_axis_state service not available after '
                    f'{per_axis_timeout_sec:.0f}s; axis {i} was NOT commanded to state {state}.'
                )
                unavailable.append(i)
            time.sleep(stagger_sec)
        return unavailable

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _svc_zero_encoders(self, _request: Empty.Request, response: Empty.Response):
        """Record current ODrive positions as the physical zero for each axis.

        An axis with no position feedback keeps its previous offset rather
        than being zeroed at a guessed position."""
        missing = [i for i, p in enumerate(self._pos_estimate) if p is None]
        if missing:
            self.get_logger().warning(
                f'No position feedback yet for axes {missing}; their zero is unchanged.'
            )
        self._log_previous_offsets('Set Physical Zero')
        for i, hsa in enumerate(self.robot.hsas):
            current = self._pos_estimate[i]
            if current is None:
                continue
            self._zero_offsets[i] = compute_zero_offset(current, hsa)
            self.get_logger().info(
                f'Axis {i} ({hsa.handedness}): '
                f'pos={current:.4f} turns  →  offset={self._zero_offsets[i]:.4f} turns'
            )
        self._publish_zero_offsets()
        self._save_zero_offsets()
        return response

    def _svc_reset_zero(self, _request: Empty.Request, response: Empty.Response):
        """Clear the software zero offset for every axis, so extension angle 0
        again means the ODrive's own raw pos_estimate 0 — undoes any prior
        ~/zero_encoders calibration without needing to restart the node."""
        self._log_previous_offsets('Clear Physical Zero')
        self._zero_offsets = [0.0] * self.n
        self.get_logger().info(
            'Zero offsets reset to 0.0 for all axes — extension angle now follows '
            "each ODrive's own raw pos_estimate directly."
        )
        self._publish_zero_offsets()
        self._save_zero_offsets()
        return response

    def _svc_enable(self, _request: Trigger.Request, response: Trigger.Response):
        # Each ODrive holds its current position on entering closed loop, so
        # any setpoint remembered from before is stale.
        self._setpoints = [None] * self.n
        unavailable = self._request_axis_state_all(AXIS_STATE_CLOSED_LOOP)
        response.success = not unavailable
        response.message = (
            'CLOSED_LOOP_CONTROL requested for all axes.'
            if not unavailable
            else f'CLOSED_LOOP_CONTROL request FAILED for axes {unavailable} (service unavailable).'
        )
        return response

    def _svc_disable(self, _request: Trigger.Request, response: Trigger.Response):
        self._setpoints = [None] * self.n
        unavailable = self._request_axis_state_all(AXIS_STATE_IDLE)
        response.success = not unavailable
        response.message = (
            'IDLE requested for all axes.'
            if not unavailable
            else f'IDLE request FAILED for axes {unavailable} (service unavailable).'
        )
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HSAODriveControl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
