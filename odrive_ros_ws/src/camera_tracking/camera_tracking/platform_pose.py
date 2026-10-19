#!/usr/bin/env python3
"""
platform_pose — publish the platform AprilTag pose in the camera frame.

AprilTagNode only broadcasts tag poses over TF (its detections message carries
pixel corners, not a pose), so this node looks up camera -> tag and republishes
it as a PoseStamped that is easy to echo, plot or bag.

Topics published:
  /platform_pose          (geometry_msgs/PoseStamped) — tag pose in camera_frame,
                           only while the TF is fresher than max_age
  /platform_tag_visible   (std_msgs/Bool) — true while the pose is fresh

Parameters:
  camera_frame (str)   — TF parent, the usb_cam frame_id (default "camera")
  tag_id       (int)   — platform tag id; frame becomes tag36h11:<id>
  tag_frame    (str)   — explicit TF child name, overrides tag_id if non-empty
  rate         (float) — publish rate in Hz (default 30)
  max_age      (float) — seconds before a stale TF counts as lost (default 1.0)
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class PlatformPose(Node):
    def __init__(self) -> None:
        super().__init__('platform_pose')

        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('tag_id', 0)
        self.declare_parameter('tag_frame', '')
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('max_age', 1.0)

        self.camera_frame: str = self.get_parameter('camera_frame').value
        tag_frame: str = self.get_parameter('tag_frame').value
        self.tag_frame: str = tag_frame or f"tag36h11:{self.get_parameter('tag_id').value}"
        self.max_age = Duration(seconds=self.get_parameter('max_age').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pose_pub = self.create_publisher(PoseStamped, '/platform_pose', 10)
        self._visible_pub = self.create_publisher(Bool, '/platform_tag_visible', 10)

        self._visible: bool | None = None
        self.create_timer(1.0 / self.get_parameter('rate').value, self._tick)

        self.get_logger().info(
            f'Publishing {self.camera_frame} -> {self.tag_frame} on /platform_pose')

    def _tick(self) -> None:
        visible = False
        try:
            tf = self._tf_buffer.lookup_transform(self.camera_frame, self.tag_frame, Time())
            age = self.get_clock().now() - Time.from_msg(tf.header.stamp)
            if age <= self.max_age:
                visible = True
                msg = PoseStamped()
                msg.header = tf.header
                msg.pose.position.x = tf.transform.translation.x
                msg.pose.position.y = tf.transform.translation.y
                msg.pose.position.z = tf.transform.translation.z
                msg.pose.orientation = tf.transform.rotation
                self._pose_pub.publish(msg)
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

        self._visible_pub.publish(Bool(data=visible))
        if visible != self._visible:
            self._visible = visible
            self.get_logger().info(
                f'{self.tag_frame} ' + ('visible' if visible else 'LOST'))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlatformPose()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
