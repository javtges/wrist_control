#!/usr/bin/env python3
"""
platform_pose — pose of the wrist platform from its AprilTags, relative to a stored zero.

The plate carries four tags (16h5, ids 0-3): ID0 in the middle, ID1/ID2/ID3
40 mm out at 120 degrees. Each detection frame gives the camera -> platform
transform from ID0 when it is visible, otherwise from the first visible outer
tag shifted by its known offset (see platform_geometry.TagLayout).

Zeroing: with the HSAs zeroed and the platform flat on the table, call
/set_platform_zero. The current camera -> platform transform is stored as the
`platform_zero` frame (and saved to `zero_file`, so it survives restarts).
From then on /platform_pose is the platform expressed in platform_zero: the
plate's displacement from its rest pose.

Subscribes:
  detections_topic    (apriltag_msgs/AprilTagDetectionArray) default /apriltag/detections
  camera_info_topic   (sensor_msgs/CameraInfo)                default /tracking_cam/camera_info
  joint_states_topic  (std_msgs/Float32MultiArray) HSA extension angles, degrees,
                      default /hsa_odrive_control/joint_states (zeroing guard)

Publishes:
  /platform_pose          (geometry_msgs/PoseStamped) platform in platform_zero; only
                          once a zero is set and while a tag is visible
  /platform_pose_camera   (geometry_msgs/PoseStamped) platform in the camera frame, always
  /platform_tag_visible   (std_msgs/Bool)   a layout tag was seen in the latest frame
  /platform_source_tag    (std_msgs/Int32)  tag id the pose is anchored on, -1 = none
  /platform_zero_set      (std_msgs/Bool)   latched; true once a zero exists
  ~/tag_consistency_mm    (std_msgs/Float32MultiArray) per layout tag, distance between
                          the platform origin as that tag alone sees it and the
                          published pose (NaN = not visible). Checks the offsets.
  TF: camera -> platform_zero (static, once set), platform_zero -> platform
      (or camera -> platform until a zero exists)

Services:
  /set_platform_zero  (std_srvs/Trigger) store the current pose as the zero

Parameters: see config/platform.yaml.
"""

from __future__ import annotations

import datetime as _dt
import math
import os
import time

import numpy as np
import yaml

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, Float32MultiArray, Int32
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from camera_tracking import platform_geometry as pg


def _stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class PlatformPose(Node):
    def __init__(self) -> None:
        super().__init__('platform_pose')

        self.declare_parameter('detections_topic', '/apriltag/detections')
        self.declare_parameter('camera_info_topic', '/tracking_cam/camera_info')
        self.declare_parameter('joint_states_topic', '/hsa_odrive_control/joint_states')
        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('platform_frame', 'platform')
        self.declare_parameter('zero_frame', 'platform_zero')
        self.declare_parameter('tag_size', pg.DEFAULT_TAG_SIZE_M)
        self.declare_parameter('tag_ids', pg.DEFAULT_TAG_IDS)
        self.declare_parameter('tag_x_mm', pg.DEFAULT_TAG_X_MM)
        self.declare_parameter('tag_y_mm', pg.DEFAULT_TAG_Y_MM)
        self.declare_parameter('tag_priority', pg.DEFAULT_TAG_IDS)
        self.declare_parameter('pose_source', 'priority')   # priority | all
        self.declare_parameter('max_age', 0.1)               # s, pose freshness for zeroing
        self.declare_parameter('stale_after', 1.0)           # s without detections -> not visible
        self.declare_parameter('zero_file', '~/.config/camera_tracking/platform_zero.yaml')
        self.declare_parameter('require_joints_zero', True)
        self.declare_parameter('joints_zero_tol_deg', 1.0)
        self.declare_parameter('joints_max_age', 1.0)
        self.declare_parameter('publish_consistency', True)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.camera_frame: str = p('camera_frame')
        self.platform_frame: str = p('platform_frame')
        self.zero_frame: str = p('zero_frame')
        self.layout = pg.TagLayout(
            size=float(p('tag_size')),
            ids=[int(i) for i in p('tag_ids')],
            x_mm=[float(v) for v in p('tag_x_mm')],
            y_mm=[float(v) for v in p('tag_y_mm')],
        )
        self.priority = [int(i) for i in p('tag_priority')]
        unknown = [i for i in self.priority if i not in self.layout.ids]
        if unknown:
            raise ValueError(f'tag_priority ids {unknown} are not in tag_ids')
        self.pose_source: str = p('pose_source')
        if self.pose_source not in ('priority', 'all'):
            raise ValueError("pose_source must be 'priority' or 'all'")
        self.max_age = float(p('max_age'))
        self.stale_after = float(p('stale_after'))
        self.zero_file = os.path.expanduser(str(p('zero_file')))
        self.require_joints_zero = bool(p('require_joints_zero'))
        self.joints_tol = float(p('joints_zero_tol_deg'))
        self.joints_max_age = float(p('joints_max_age'))
        self.publish_consistency = bool(p('publish_consistency'))

        self._K: np.ndarray | None = None
        self._T_cam_platform: np.ndarray | None = None   # latest pose
        self._pose_stamp: float = 0.0                     # its image stamp (s)
        self._source_tag: int | None = None
        self._last_detections_wall: float = 0.0
        self._visible: bool | None = None
        self._joints: list[float] | None = None
        self._joints_wall: float = 0.0
        self._T_cam_zero: np.ndarray | None = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pose_pub = self.create_publisher(PoseStamped, '/platform_pose', 10)
        self._pose_cam_pub = self.create_publisher(PoseStamped, '/platform_pose_camera', 10)
        self._visible_pub = self.create_publisher(Bool, '/platform_tag_visible', 10)
        self._source_pub = self.create_publisher(Int32, '/platform_source_tag', 10)
        self._zero_set_pub = self.create_publisher(Bool, '/platform_zero_set', latched)
        self._consistency_pub = self.create_publisher(Float32MultiArray, '~/tag_consistency_mm', 10)
        self._tf = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)

        self.create_subscription(CameraInfo, str(p('camera_info_topic')), self._camera_info_cb, 10)
        self.create_subscription(AprilTagDetectionArray, str(p('detections_topic')), self._detections_cb, 10)
        self.create_subscription(Float32MultiArray, str(p('joint_states_topic')), self._joints_cb, 10)
        self.create_service(Trigger, '/set_platform_zero', self._set_zero_cb)
        self.create_timer(0.2, self._watchdog)

        self._load_zero()
        self._zero_set_pub.publish(Bool(data=self._T_cam_zero is not None))
        self.get_logger().info(
            f'tags {self.layout.ids} ({self.layout.size * 1e3:.0f} mm), priority {self.priority}, '
            f'source={self.pose_source}; zero '
            + ('loaded from ' + self.zero_file if self._T_cam_zero is not None
               else 'NOT set: /platform_pose stays silent until /set_platform_zero'))

    # ------------------------------------------------------------ callbacks ---

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self._K = pg.intrinsics_from_projection(msg.p)

    def _joints_cb(self, msg: Float32MultiArray) -> None:
        self._joints = [float(v) for v in msg.data]
        self._joints_wall = time.monotonic()

    def _detections_cb(self, msg: AprilTagDetectionArray) -> None:
        self._last_detections_wall = time.monotonic()
        if self._K is None:
            self.get_logger().warning('no camera_info yet, cannot estimate a pose',
                                      throttle_duration_sec=5.0)
            return
        detections = {
            int(d.id): np.array([[c.x, c.y] for c in d.corners], dtype=float)
            for d in msg.detections if int(d.id) in self.layout.ids
        }
        T, tag_id = pg.platform_pose(self.layout, detections, self._K, self.priority, self.pose_source)
        self._set_visible(T is not None, tag_id)
        if T is None:
            return
        self._T_cam_platform = T
        self._pose_stamp = _stamp_s(msg.header.stamp)
        self._source_tag = tag_id
        self._publish_pose(msg.header.stamp)
        if self.publish_consistency and len(detections) > 1:
            errs = pg.tag_consistency_mm(self.layout, detections, self._K, T)
            self._consistency_pub.publish(Float32MultiArray(data=[float(e) for e in errs]))

    def _watchdog(self) -> None:
        if self._visible and time.monotonic() - self._last_detections_wall > self.stale_after:
            self._set_visible(False, None)

    def _set_visible(self, visible: bool, tag_id: int | None) -> None:
        self._visible_pub.publish(Bool(data=visible))
        self._source_pub.publish(Int32(data=tag_id if tag_id is not None else -1))
        if visible != self._visible:
            self._visible = visible
            self.get_logger().info(f'platform ' + (f'visible (tag {tag_id})' if visible else 'LOST'))

    # ------------------------------------------------------------ publishing ---

    def _publish_pose(self, stamp) -> None:
        T = self._T_cam_platform
        self._pose_cam_pub.publish(self._pose_msg(stamp, self.camera_frame, T))
        if self._T_cam_zero is None:
            self._tf.sendTransform(self._tf_msg(stamp, self.camera_frame, self.platform_frame, T))
            return
        T_zero_platform = pg.relative_to_zero(self._T_cam_zero, T)
        self._pose_pub.publish(self._pose_msg(stamp, self.zero_frame, T_zero_platform))
        self._tf.sendTransform(self._tf_msg(stamp, self.zero_frame, self.platform_frame, T_zero_platform))

    @staticmethod
    def _pose_msg(stamp, frame: str, T: np.ndarray) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        x, y, z = T[:3, 3]
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
        qx, qy, qz, qw = pg.matrix_to_quat(T)
        msg.pose.orientation.x, msg.pose.orientation.y = float(qx), float(qy)
        msg.pose.orientation.z, msg.pose.orientation.w = float(qz), float(qw)
        return msg

    @staticmethod
    def _tf_msg(stamp, parent: str, child: str, T: np.ndarray) -> TransformStamped:
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent
        msg.child_frame_id = child
        x, y, z = T[:3, 3]
        msg.transform.translation.x, msg.transform.translation.y = float(x), float(y)
        msg.transform.translation.z = float(z)
        qx, qy, qz, qw = pg.matrix_to_quat(T)
        msg.transform.rotation.x, msg.transform.rotation.y = float(qx), float(qy)
        msg.transform.rotation.z, msg.transform.rotation.w = float(qz), float(qw)
        return msg

    # ------------------------------------------------------------------ zero ---

    def _joints_check(self) -> str | None:
        """None when zeroing may proceed, else the reason it may not."""
        if not self.require_joints_zero:
            return None
        if self._joints is None or time.monotonic() - self._joints_wall > self.joints_max_age:
            return ('no fresh HSA joint states: start hsa_odrive_control (or set '
                    'require_joints_zero:=false)')
        bad = [f'{i}:{v:.1f}' for i, v in enumerate(self._joints)
               if not math.isfinite(v) or abs(v) > self.joints_tol]
        if bad:
            return (f'HSAs not at zero (tolerance {self.joints_tol:g} deg): ' + ' '.join(bad)
                    + '; zero the encoders and command 0 deg first')
        return None

    def _set_zero_cb(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        reason = self._joints_check()
        if reason is not None:
            res.success, res.message = False, reason
            self.get_logger().warning('set_platform_zero refused: ' + reason)
            return res
        if self._T_cam_platform is None:
            res.success, res.message = False, 'no platform pose yet (tag never seen)'
            return res
        # age by receipt time, not header stamp: usb_cam stamps frames on a clock
        # that can sit a few hundred ms behind the node clock
        age = time.monotonic() - self._last_detections_wall
        if age > self.max_age or not self._visible:
            res.success = False
            res.message = f'no fresh platform pose ({age:.2f} s since the last detections, max_age {self.max_age:g}); tag not visible?'
            return res
        self._T_cam_zero = self._T_cam_platform.copy()
        self._broadcast_zero()
        self._save_zero()
        self._zero_set_pub.publish(Bool(data=True))
        t = self._T_cam_zero[:3, 3] * 1e3
        res.success = True
        res.message = (f'platform zero set from tag {self._source_tag} at camera '
                       f'({t[0]:.1f}, {t[1]:.1f}, {t[2]:.1f}) mm; saved to {self.zero_file}')
        self.get_logger().info(res.message)
        return res

    def _broadcast_zero(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._static_tf.sendTransform(
            self._tf_msg(stamp, self.camera_frame, self.zero_frame, self._T_cam_zero))

    def _save_zero(self) -> None:
        q = pg.matrix_to_quat(self._T_cam_zero)
        data = {
            'camera_frame': self.camera_frame,
            'zero_frame': self.zero_frame,
            'translation': [float(v) for v in self._T_cam_zero[:3, 3]],
            'rotation_xyzw': [float(v) for v in q],
            'source_tag': self._source_tag,
            'set_at': _dt.datetime.now().isoformat(timespec='seconds'),
            'joint_states_deg': self._joints,
        }
        os.makedirs(os.path.dirname(self.zero_file), exist_ok=True)
        with open(self.zero_file, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)

    def _load_zero(self) -> None:
        if not os.path.isfile(self.zero_file):
            return
        try:
            with open(self.zero_file) as f:
                data = yaml.safe_load(f)
            if data.get('camera_frame', self.camera_frame) != self.camera_frame:
                self.get_logger().warning(
                    f'{self.zero_file} was set for camera frame {data["camera_frame"]!r}, '
                    f'this node uses {self.camera_frame!r}; using it anyway')
            self._T_cam_zero = pg.pose_from_quat_xyz(data['translation'], data['rotation_xyzw'])
            self._broadcast_zero()
            self.get_logger().info(f'platform zero from {self.zero_file} (set {data.get("set_at")})')
        except Exception as e:  # malformed file: start without a zero
            self.get_logger().error(f'could not read {self.zero_file}: {e}; zero not set')
            self._T_cam_zero = None


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
