#!/usr/bin/env python3
"""
plate_guard — warns when the top plate is not mounted the expected way round.

Reads the tag detections, measures the screen bearing of every visible outer
tag around the plate centre and compares it with the expected mounting
(`expected_id1_bearing_deg`: where ID1 sits, clockwise from the top of the
image; the other tags follow from the layout). One visible outer tag is
enough, so a tag clipped by the frame edge does not block the check.

Publishes, at `rate`:
  /plate_mount_ok      (std_msgs/Bool)    true while the plate is as expected
  /plate_mount_status  (std_msgs/String)  "OK ..." / "WRONG ..." / "NO TAGS ..." /
                                          "MIRRORED ..."; shown as a banner in
                                          joint_commander_gui
Logs the status on every change (ERROR when wrong) and repeats a WARN every
`warn_period` seconds while it stays wrong.

Parameters (config/platform.yaml, `plate_guard:`): detections_topic, tag_size,
tag_ids, tag_x_mm, tag_y_mm, expected_id1_bearing_deg, tolerance_deg (15),
stale_after (1 s), warn_period (5 s), rate (2 Hz).
"""

from __future__ import annotations

import time

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import Bool, String

from camera_tracking import platform_geometry as pg


class PlateGuard(Node):
    def __init__(self) -> None:
        super().__init__('plate_guard')
        self.declare_parameter('detections_topic', '/apriltag/detections')
        self.declare_parameter('tag_size', pg.DEFAULT_TAG_SIZE_M)
        self.declare_parameter('tag_ids', pg.DEFAULT_TAG_IDS)
        self.declare_parameter('tag_x_mm', pg.DEFAULT_TAG_X_MM)
        self.declare_parameter('tag_y_mm', pg.DEFAULT_TAG_Y_MM)
        self.declare_parameter('expected_id1_bearing_deg', 283.5)
        self.declare_parameter('tolerance_deg', 15.0)
        self.declare_parameter('stale_after', 1.0)
        self.declare_parameter('warn_period', 5.0)
        self.declare_parameter('rate', 2.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.layout = pg.TagLayout(size=float(p('tag_size')), ids=[int(i) for i in p('tag_ids')],
                                   x_mm=[float(v) for v in p('tag_x_mm')],
                                   y_mm=[float(v) for v in p('tag_y_mm')])
        self.expected = pg.expected_bearings(self.layout, float(p('expected_id1_bearing_deg')))
        self.tol = float(p('tolerance_deg'))
        self.stale_after = float(p('stale_after'))
        self.warn_period = float(p('warn_period'))

        self._centres: dict[int, np.ndarray] = {}
        self._last_det_wall = 0.0
        self._key: tuple | None = None      # (ok, verdict, clock positions): what counts as a change
        self._last_warn_wall = 0.0

        self._ok_pub = self.create_publisher(Bool, '/plate_mount_ok', 10)
        self._status_pub = self.create_publisher(String, '/plate_mount_status', 10)
        self.create_subscription(AprilTagDetectionArray, str(p('detections_topic')), self._det_cb, 10)
        self.create_timer(1.0 / float(p('rate')), self._tick)
        self.get_logger().info('expected mounting: ' + ', '.join(
            f'ID{i} at {pg.clock_position(b)} o\'clock' for i, b in sorted(self.expected.items()))
            + f' (tolerance {self.tol:g} deg)')

    def _det_cb(self, msg: AprilTagDetectionArray) -> None:
        self._last_det_wall = time.monotonic()
        self._centres = {int(d.id): np.array([[c.x, c.y] for c in d.corners]).mean(axis=0)
                         for d in msg.detections if int(d.id) in self.layout.ids}

    def evaluate(self) -> tuple[bool, str, tuple]:
        """(ok, status text, change key) for the latest detections. The key
        is (ok, verdict, clock positions): degree jitter is not a change."""
        if time.monotonic() - self._last_det_wall > self.stale_after:
            return False, 'NO TAGS: no detections (camera_tracking running? plate in view?)', (False, 'NO TAGS', '')
        centre = pg.plate_centre_px(self._centres, self.layout)
        outer = [i for i in self.layout.ids[1:] if i in self._centres]
        if centre is None or not outer:
            seen = ' '.join(f'ID{i}' for i in sorted(self._centres)) or 'none'
            return (False, f'NO TAGS: need ID0 (or all outer tags) plus one outer tag; seen {seen}',
                    (False, 'NO TAGS', seen))
        bearings = {i: pg.clock_angle_deg(centre, self._centres[i]) for i in outer}
        where = ', '.join(f'ID{i} {pg.clock_position(b)} o\'clock' for i, b in sorted(bearings.items()))
        if len(outer) == 3:
            ids, _ = pg.outer_tag_order(bearings)
            if not pg.is_layout_order(ids, self.layout.ids[1:]):
                return (False, f'MIRRORED: tags run {"->".join(f"ID{i}" for i in ids)} clockwise; {where}',
                        (False, 'MIRRORED', where))
        err, _ = pg.plate_spin_error(bearings, self.expected)
        if err is None:
            return False, f'NO TAGS: no expected outer tag visible; {where}', (False, 'NO TAGS', where)
        if abs(err) <= self.tol:
            return True, f'OK: plate as expected ({int(round(err)):+d} deg); {where}', (True, 'OK', where)
        return False, f'WRONG: plate turned {pg.describe_spin(err)}; {where}', (False, 'WRONG', where)

    def _tick(self) -> None:
        ok, status, key = self.evaluate()
        self._ok_pub.publish(Bool(data=ok))
        self._status_pub.publish(String(data=status))
        now = time.monotonic()
        if key != self._key:
            self._key = key
            self._last_warn_wall = now
            # separate call sites: rclpy pins one severity per source line
            if ok:
                self.get_logger().info('plate ' + status)
            else:
                self.get_logger().error('plate ' + status)
        elif not ok and now - self._last_warn_wall >= self.warn_period:
            self._last_warn_wall = now
            self.get_logger().warning('plate ' + status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlateGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
