#!/usr/bin/env python3
"""
plate_check — is the top plate mounted the right way round?

Two answers, printed to the terminal while the tracking stack runs:

1. Where each tag sits in the camera image: the clock position of ID1, ID2
   and ID3 around the plate centre (12 = top of the image), whether the
   three outer tags run 1 -> 2 -> 3 clockwise 120 deg apart as in the
   drawing, and the distance of each tag from the camera.

2. Which leg pair sits under each tag: move one pair of HSAs at a time and
   the tag whose camera distance changes most is the one above that pair.
   By default you move the pairs yourself (joint_commander_gui); with
   `drive:=true` this node commands each pair to `wiggle_deg` in turn
   (others at 0), waits `settle_s`, and prints the mapping.

Run:
  ros2 launch camera_tracking tracking.launch.py          # camera + detector
  ros2 run camera_tracking plate_check                     # watch while you move pairs
  ros2 run camera_tracking plate_check --ros-args -p drive:=true   # moves the wrist itself

Subscribes: detections_topic, camera_info_topic, joint_states_topic.
Publishes (drive only): joint_commands_topic (std_msgs/Float32MultiArray, degrees).
"""

from __future__ import annotations

import math
import time

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray

from camera_tracking import platform_geometry as pg


class PlateCheck(Node):
    def __init__(self) -> None:
        super().__init__('plate_check')
        self.declare_parameter('detections_topic', '/apriltag/detections')
        self.declare_parameter('camera_info_topic', '/tracking_cam/camera_info')
        self.declare_parameter('joint_states_topic', '/hsa_odrive_control/joint_states')
        self.declare_parameter('joint_commands_topic', '/hsa_odrive_control/joint_commands')
        self.declare_parameter('tag_size', pg.DEFAULT_TAG_SIZE_M)
        self.declare_parameter('tag_ids', pg.DEFAULT_TAG_IDS)
        self.declare_parameter('tag_x_mm', pg.DEFAULT_TAG_X_MM)
        self.declare_parameter('tag_y_mm', pg.DEFAULT_TAG_Y_MM)
        self.declare_parameter('pairs', [0, 1, 2, 3, 4, 5])  # flat: (0,1) (2,3) (4,5)
        self.declare_parameter('drive', False)
        self.declare_parameter('wiggle_deg', 20.0)
        self.declare_parameter('settle_s', 2.0)
        self.declare_parameter('min_move_mm', 2.0)
        self.declare_parameter('report_period', 1.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.layout = pg.TagLayout(size=float(p('tag_size')), ids=[int(i) for i in p('tag_ids')],
                                   x_mm=[float(v) for v in p('tag_x_mm')],
                                   y_mm=[float(v) for v in p('tag_y_mm')])
        flat = [int(i) for i in p('pairs')]
        self.pairs = [tuple(flat[k:k + 2]) for k in range(0, len(flat) - 1, 2)]
        self.drive = bool(p('drive'))
        self.wiggle = float(p('wiggle_deg'))
        self.settle = float(p('settle_s'))
        self.min_move = float(p('min_move_mm'))

        self._K: np.ndarray | None = None
        self._centres: dict[int, np.ndarray] = {}
        self._dist_mm: dict[int, float] = {}       # per tag, camera distance now
        self._rest_mm: dict[int, float] | None = None
        self._joints: list[float] | None = None
        self._n_joints = 6
        self._stage = 'wait'                        # drive sequence state
        self._stage_t = 0.0
        self._pair_i = 0
        self._mapping: dict[tuple[int, int], tuple[int | None, float]] = {}

        self.create_subscription(CameraInfo, str(p('camera_info_topic')), self._ci_cb, 10)
        self.create_subscription(AprilTagDetectionArray, str(p('detections_topic')), self._det_cb, 10)
        self.create_subscription(Float32MultiArray, str(p('joint_states_topic')), self._js_cb, 10)
        self._cmd_pub = self.create_publisher(Float32MultiArray, str(p('joint_commands_topic')), 10)
        self._report_timer = self.create_timer(float(p('report_period')), self._report)
        if self.drive:
            self.create_timer(0.1, self._drive_step)
        self.get_logger().info('waiting for camera_info and detections'
                               + (f'; will move pairs {self.pairs} by {self.wiggle:g} deg'
                                  if self.drive else '; move one leg pair at a time and watch'))

    # ------------------------------------------------------------ inputs ---

    def _ci_cb(self, msg: CameraInfo) -> None:
        self._K = pg.intrinsics_from_projection(msg.p)

    def _js_cb(self, msg: Float32MultiArray) -> None:
        self._joints = [float(v) for v in msg.data]
        self._n_joints = len(self._joints)

    def _det_cb(self, msg: AprilTagDetectionArray) -> None:
        if self._K is None:
            return
        centres, dist = {}, {}
        for d in msg.detections:
            if int(d.id) not in self.layout.ids:
                continue
            corners = np.array([[c.x, c.y] for c in d.corners], dtype=float)
            centres[int(d.id)] = corners.mean(axis=0)
            T = pg.solve_tag_centre_pose(self.layout, corners, self._K)
            if T is not None:
                dist[int(d.id)] = float(np.linalg.norm(T[:3, 3]) * 1e3)
        self._centres, self._dist_mm = centres, dist
        if self._rest_mm is None and len(dist) == len(self.layout.ids):
            self._rest_mm = dict(dist)
            self.get_logger().info('rest distances taken (all tags visible): '
                                   + ' '.join(f'ID{i}={v:.1f}mm' for i, v in sorted(dist.items())))

    # ------------------------------------------------------------ report ---

    def _report(self) -> None:
        if self._K is None or not self._centres:
            self.get_logger().info('no tags in view', throttle_duration_sec=5.0)
            return
        lines = []
        centre = pg.plate_centre_px(self._centres, self.layout)
        outer = [i for i in self.layout.ids[1:] if i in self._centres]
        if centre is not None and outer:
            bearings = {i: pg.clock_angle_deg(centre, self._centres[i]) for i in outer}
            ids, gaps = pg.outer_tag_order(bearings)
            for i in self.layout.ids[1:]:
                if i in bearings:
                    lines.append(f'ID{i}: {pg.clock_position(bearings[i]):2d} o\'clock '
                                 f'({bearings[i]:5.1f} deg cw from image up)')
                else:
                    lines.append(f'ID{i}: not visible')
            if len(ids) == 3:
                order_ok = pg.is_layout_order(ids, self.layout.ids[1:])
                gap_ok = all(abs(g - 120.0) < 10.0 for g in gaps)
                lines.append(f'clockwise order on screen {"->".join(f"ID{i}" for i in ids)}, '
                             f'gaps {" ".join(f"{g:.0f}" for g in gaps)} deg: '
                             + ('matches the drawing' if order_ok and gap_ok else
                                'DOES NOT match the drawing (1->2->3 clockwise, 120 deg)'))
        else:
            lines.append('need ID0 or all three outer tags for bearings')
        if self._dist_mm:
            lines.append('distance from camera: ' + ' '.join(
                f'ID{i}={self._dist_mm[i]:.1f}mm' for i in self.layout.ids if i in self._dist_mm))
        if self._rest_mm is not None:
            dz = {i: self._dist_mm[i] - self._rest_mm[i] for i in self._dist_mm if i in self._rest_mm}
            tag, move = pg.tag_over_pair(dz, self.min_move)
            lines.append('change since rest: ' + ' '.join(f'ID{i}={v:+.1f}' for i, v in sorted(dz.items()))
                         + (f' mm -> ID{tag} moved most' if tag is not None else ' mm (nothing moved)'))
        if self._joints is not None:
            lines.append('joints deg: ' + ' '.join(f'{v:.0f}' for v in self._joints))
        self.get_logger().info('\n  ' + '\n  '.join(lines))

    # ------------------------------------------------------------- drive ---

    def _command(self, angles: list[float]) -> None:
        self._cmd_pub.publish(Float32MultiArray(data=[float(a) for a in angles]))

    def _drive_step(self) -> None:
        now = time.monotonic()
        if self._stage == 'wait':
            if self._rest_mm is None:
                return
            self._command([0.0] * self._n_joints)
            self._stage, self._stage_t = 'rest', now
        elif self._stage == 'rest':
            if now - self._stage_t < self.settle:
                return
            self._rest_mm = dict(self._dist_mm) if len(self._dist_mm) == len(self.layout.ids) else self._rest_mm
            if self._pair_i >= len(self.pairs):
                self._stage = 'done'
                self._report_timer.cancel()
                self._report()
                self._print_mapping()
                self.get_logger().info('done, all joints back at 0; Ctrl-C to exit')
                return
            a, b = self.pairs[self._pair_i]
            cmd = [0.0] * self._n_joints
            cmd[a] = cmd[b] = self.wiggle
            self._command(cmd)
            self.get_logger().info(f'pair ({a},{b}) -> {self.wiggle:g} deg')
            self._stage, self._stage_t = 'moved', now
        elif self._stage == 'moved':
            if now - self._stage_t < self.settle:
                return
            dz = {i: self._dist_mm[i] - self._rest_mm[i] for i in self._dist_mm if i in self._rest_mm}
            self._mapping[self.pairs[self._pair_i]] = pg.tag_over_pair(dz, self.min_move)
            self._pair_i += 1
            self._command([0.0] * self._n_joints)
            self._stage, self._stage_t = 'rest', now

    def _print_mapping(self) -> None:
        lines = ['leg pair -> tag above it:']
        for pair, (tag, move) in self._mapping.items():
            lines.append(f'  ({pair[0]},{pair[1]}): ' + (f'ID{tag} ({move:+.1f} mm)' if tag is not None
                                                        else f'no tag moved more than {self.min_move:g} mm'))
        self.get_logger().info('\n'.join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlateCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
