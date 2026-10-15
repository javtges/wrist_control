#!/usr/bin/env python3
"""
odrive_position_monitor — aggregate every ODrive axis position onto one topic.

Subscribes to each axis's controller_status and republishes all of the raw
pos_estimate values together, so `ros2 topic echo /odrive_positions` shows the
whole wrist at once. Also adds the software zero offsets from
hsa_odrive_control and converts the result to HSA extension angles through
hsa_robots, so `/hsa_positions` reads 0–360 measured from the physical zero.

Topics subscribed:
  /odrive_axis{N}/controller_status  (odrive_can/ControllerStatus)
  /hsa_zero_offsets  (std_msgs/Float32MultiArray, transient local) — per-axis
                     software zero offset in motor turns (wrist.yaml's
                     zero_offset until received)

Topics published:
  /odrive_positions  (std_msgs/Float32MultiArray) — num_axes raw pos_estimate
                     values in turns (NaN for an axis with no feedback yet)
  /hsa_positions     (std_msgs/Float32MultiArray) — odrive_positions +
                     hsa_zero_offsets, as HSA extension angles in degrees
                     (gear ratio and handedness from wrist.yaml; NaN for an
                     axis with no feedback yet)

Parameters:
  robot_config (str)   — path to wrist.yaml; defaults to hsa_robots/wrist.yaml
  num_axes     (int)   — number of ODrive axes to watch; must match robot_config
  publish_rate (float) — publish frequency in Hz
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float32MultiArray
from odrive_can.msg import ControllerStatus

from hsa_robots import HSARobot

from .conversion import compute_extension_angle
from .robot_config import default_robot_config

ZERO_OFFSETS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class ODrivePositionMonitor(Node):
    def __init__(self) -> None:
        super().__init__('odrive_position_monitor')

        self.declare_parameter('robot_config', '')
        cfg = self.get_parameter('robot_config').value or default_robot_config()
        self.robot = HSARobot(cfg)

        self.declare_parameter('num_axes', len(self.robot.hsas))
        self.declare_parameter('publish_rate', 20.0)

        self.n: int = self.get_parameter('num_axes').value
        rate: float = self.get_parameter('publish_rate').value
        if self.n != len(self.robot.hsas):
            raise ValueError(
                f'num_axes={self.n} but {cfg} defines {len(self.robot.hsas)} HSAs'
            )

        self._positions: list[float] = [math.nan] * self.n
        self._zero_offsets: list[float] = [h.zero_offset for h in self.robot.hsas]

        self._pub = self.create_publisher(Float32MultiArray, '/odrive_positions', 10)
        self._hsa_pub = self.create_publisher(Float32MultiArray, '/hsa_positions', 10)

        for i in range(self.n):
            self.create_subscription(
                ControllerStatus,
                f'/odrive_axis{i}/controller_status',
                lambda msg, idx=i: self._on_controller_status(idx, msg),
                10,
            )
        self.create_subscription(
            Float32MultiArray, '/hsa_zero_offsets', self._on_zero_offsets, ZERO_OFFSETS_QOS
        )

        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'Watching {self.n} axes, publishing /odrive_positions and /hsa_positions '
            f'at {rate} Hz (config {cfg}).'
        )

    def _on_controller_status(self, axis_idx: int, msg: ControllerStatus) -> None:
        self._positions[axis_idx] = msg.pos_estimate

    def _on_zero_offsets(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != self.n:
            self.get_logger().error(
                f'hsa_zero_offsets: expected {self.n} values, got {len(msg.data)}'
            )
            return
        self._zero_offsets = list(msg.data)

    def _publish(self) -> None:
        self._pub.publish(Float32MultiArray(data=self._positions))
        self._hsa_pub.publish(Float32MultiArray(data=[
            math.nan if math.isnan(pos) else compute_extension_angle(pos, hsa, offset)
            for pos, hsa, offset in zip(self._positions, self.robot.hsas, self._zero_offsets)
        ]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ODrivePositionMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
