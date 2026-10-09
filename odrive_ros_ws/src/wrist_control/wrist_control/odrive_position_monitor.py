#!/usr/bin/env python3
"""
odrive_position_monitor — aggregate every ODrive axis position onto one topic.

Subscribes to each axis's controller_status and republishes all of the raw
pos_estimate values together, so `ros2 topic echo /odrive_positions` shows the
whole wrist at once.

Topics subscribed:
  /odrive_axis{N}/controller_status  (odrive_can/ControllerStatus)

Topics published:
  /odrive_positions  (std_msgs/Float32MultiArray) — num_axes raw pos_estimate
                     values in turns (NaN for an axis with no feedback yet)

Parameters:
  num_axes    (int)   — number of ODrive axes to watch
  publish_rate (float) — publish frequency in Hz
"""

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from odrive_can.msg import ControllerStatus


class ODrivePositionMonitor(Node):
    def __init__(self) -> None:
        super().__init__('odrive_position_monitor')

        self.declare_parameter('num_axes', 6)
        self.declare_parameter('publish_rate', 20.0)

        self.n: int = self.get_parameter('num_axes').value
        rate: float = self.get_parameter('publish_rate').value

        self._positions: list[float] = [math.nan] * self.n

        self._pub = self.create_publisher(Float32MultiArray, '/odrive_positions', 10)

        for i in range(self.n):
            self.create_subscription(
                ControllerStatus,
                f'/odrive_axis{i}/controller_status',
                lambda msg, idx=i: self._on_controller_status(idx, msg),
                10,
            )

        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'Watching {self.n} axes, publishing /odrive_positions at {rate} Hz.'
        )

    def _on_controller_status(self, axis_idx: int, msg: ControllerStatus) -> None:
        self._positions[axis_idx] = msg.pos_estimate

    def _publish(self) -> None:
        self._pub.publish(Float32MultiArray(data=self._positions))


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
