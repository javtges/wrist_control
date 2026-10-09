#!/usr/bin/env python3
"""
hsa_odrive_control — ROS2 node for HSA wrist control via ODrive CAN.

Subscribes to extension-angle commands (degrees), converts them through the
HSA kinematics model, and publishes ControlMessage to each ODrive CAN node.

Topics subscribed:
  ~/joint_commands  (std_msgs/Float32MultiArray)  — 6 extension angles, degrees

Topics published:
  /odrive_axis{N}/control_message  (odrive_can/ControlMessage)
  ~/joint_states    (std_msgs/Float32MultiArray) — 6 measured extension angles,
                     degrees, converted from each axis's live encoder position
                     estimate (NaN for an axis with no feedback yet)

Services provided:
  ~/zero_encoders  (std_srvs/Empty)   — snapshot current positions as software zero
  ~/reset_zero     (std_srvs/Empty)   — clear the software zero offset, so extension
                     angle 0 goes back to meaning the ODrive's own raw pos_estimate 0
  ~/enable         (std_srvs/Trigger) — put all axes in CLOSED_LOOP_CONTROL
  ~/disable        (std_srvs/Trigger) — put all axes in IDLE

Services called:
  /odrive_axis{N}/request_axis_state  (odrive_can/AxisState)

Parameters:
  robot_config   (str)   — path to wrist.yaml; defaults to package config/wrist.yaml
  control_mode   (int)   — ODrive ControlMode (3 = position, 2 = velocity, 1 = torque)
  input_mode     (int)   — ODrive InputMode   (1 = passthrough, 5 = trap_traj)
  vel_ff         (float) — velocity feed-forward in turns/s (position mode only)
  torque_ff      (float) — torque feed-forward in Nm
"""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty, Trigger
from odrive_can.msg import ControlMessage, ControllerStatus
from odrive_can.srv import AxisState

from hsa_robots import HSARobot
from .conversion import compute_setpoint, compute_zero_offset, compute_extension_angle

# ODrive axis state constants
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP = 8

# ODrive control mode defaults
CONTROL_MODE_POSITION = 3
INPUT_MODE_PASSTHROUGH = 1


class HSAODriveControl(Node):
    def __init__(self) -> None:
        super().__init__('hsa_odrive_control')

        # --- Parameters ---
        self.declare_parameter('robot_config', '')
        self.declare_parameter('control_mode', CONTROL_MODE_POSITION)
        self.declare_parameter('input_mode', INPUT_MODE_PASSTHROUGH)
        self.declare_parameter('vel_ff', 0.0)
        self.declare_parameter('torque_ff', 0.0)

        config_path: str = self.get_parameter('robot_config').value
        if not config_path:
            pkg_share = get_package_share_directory('wrist_control')
            config_path = os.path.join(pkg_share, 'config', 'wrist.yaml')

        self.robot = HSARobot(config_path)
        self.n: int = len(self.robot.hsas)
        self.get_logger().info(f'Loaded {self.n}-axis robot from {config_path}')

        # Per-axis state: latest position estimates and software zero offsets
        self._pos_estimate: list[float | None] = [None] * self.n
        self._zero_offsets: list[float] = [0.0] * self.n

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

        # Publisher: live measured extension angles (degrees), one per axis
        self._joint_state_pub = self.create_publisher(
            Float32MultiArray, '~/joint_states', ctrl_qos
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

        self.get_logger().info('hsa_odrive_control ready.')
        self.get_logger().info(
            'Call ~/zero_encoders before commanding motion to establish the home position.'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_controller_status(self, axis_idx: int, msg: ControllerStatus) -> None:
        self._pos_estimate[axis_idx] = msg.pos_estimate
        self._publish_joint_states()

    def _on_joint_commands(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != self.n:
            self.get_logger().error(
                f'joint_commands: expected {self.n} values, got {len(msg.data)}'
            )
            return
        self._publish_commands(list(msg.data))

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _publish_commands(self, extension_angles_deg: list[float]) -> None:
        ctrl_mode: int = int(self.get_parameter('control_mode').value)
        in_mode: int = int(self.get_parameter('input_mode').value)
        vel_ff: float = float(self.get_parameter('vel_ff').value)
        torque_ff: float = float(self.get_parameter('torque_ff').value)

        for i, (hsa, angle_deg) in enumerate(zip(self.robot.hsas, extension_angles_deg)):
            setpoint = compute_setpoint(angle_deg, hsa, self._zero_offsets[i])

            cmd = ControlMessage()
            cmd.control_mode = ctrl_mode
            cmd.input_mode = in_mode
            cmd.input_pos = float(setpoint)
            cmd.input_vel = vel_ff
            cmd.input_torque = torque_ff
            self._ctrl_pubs[i].publish(cmd)

    def _publish_joint_states(self) -> None:
        """Publish each axis's live measured extension angle (degrees),
        converted from its encoder position estimate. NaN where no feedback
        has arrived yet for that axis."""
        angles: list[float] = []
        for i, hsa in enumerate(self.robot.hsas):
            pos = self._pos_estimate[i]
            if pos is None:
                angles.append(float('nan'))
            else:
                angles.append(compute_extension_angle(pos, hsa, self._zero_offsets[i]))
        msg = Float32MultiArray()
        msg.data = angles
        self._joint_state_pub.publish(msg)

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
        """Record current ODrive positions as the software zero for each axis."""
        missing = [i for i, p in enumerate(self._pos_estimate) if p is None]
        if missing:
            self.get_logger().warning(
                f'No position feedback yet for axes {missing}. '
                'Those axes will be zeroed at 0.0 turns.'
            )
        for i, hsa in enumerate(self.robot.hsas):
            current = self._pos_estimate[i] if self._pos_estimate[i] is not None else 0.0
            self._zero_offsets[i] = compute_zero_offset(current, hsa)
            self.get_logger().info(
                f'Axis {i} ({hsa.handedness}): '
                f'pos={current:.4f} turns  →  offset={self._zero_offsets[i]:.4f} turns'
            )
        self._publish_joint_states()
        return response

    def _svc_reset_zero(self, _request: Empty.Request, response: Empty.Response):
        """Clear the software zero offset for every axis, so extension angle 0
        again means the ODrive's own raw pos_estimate 0 — undoes any prior
        ~/zero_encoders calibration without needing to restart the node."""
        self._zero_offsets = [0.0] * self.n
        self.get_logger().info(
            'Zero offsets reset to 0.0 for all axes — extension angle now follows '
            "each ODrive's own raw pos_estimate directly."
        )
        self._publish_joint_states()
        return response

    def _svc_enable(self, _request: Trigger.Request, response: Trigger.Response):
        unavailable = self._request_axis_state_all(AXIS_STATE_CLOSED_LOOP)
        response.success = not unavailable
        response.message = (
            'CLOSED_LOOP_CONTROL requested for all axes.'
            if not unavailable
            else f'CLOSED_LOOP_CONTROL request FAILED for axes {unavailable} (service unavailable).'
        )
        return response

    def _svc_disable(self, _request: Trigger.Request, response: Trigger.Response):
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
