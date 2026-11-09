#!/usr/bin/env python3
"""
platform_meter — read the platform's height, tilt and twist off the camera.

Prints one line per `report_period` while the tracking stack runs: the HSA
angles, the plate pose relative to the rest zero (z, lateral offset, tilt, the
direction it leans, twist about the normal) and the session maxima. Type a
label and press Enter (or just Enter) to log a snapshot to `csv`: every field
averaged over the last `avg_s`, with the tilt and z spread. Type `r` + Enter
to reset the maxima.

Typical session: Enable, command 0 deg, snapshot `rest`; Extend Fully,
snapshot `max_ext`; drive single legs / pairs to find the largest tilt and
snapshot each; back to 0, snapshot `rest_after` (hysteresis).

Run:
  ros2 launch wrist_control wrist_control.launch.py      # wrist + camera + platform_pose (zeroed)
  ros2 run camera_tracking platform_meter
  ros2 run camera_tracking platform_meter --ros-args -p pose_topic:=/platform_pose_camera   # before a zero exists

Subscribes: pose_topic (geometry_msgs/PoseStamped), joint_states_topic
(std_msgs/Float32MultiArray, degrees).
"""

from __future__ import annotations

import csv
import math
import os
import sys
import threading
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

from camera_tracking import platform_geometry as pg

FIELDS = ['z_mm', 'x_mm', 'y_mm', 'lateral_mm', 'tilt_deg', 'tilt_dir_deg', 'twist_deg']


class PlatformMeter(Node):
    def __init__(self) -> None:
        super().__init__('platform_meter')
        self.declare_parameter('pose_topic', '/platform_pose')
        self.declare_parameter('joint_states_topic', '/hsa_positions')
        self.declare_parameter('report_period', 1.0)
        self.declare_parameter('avg_s', 0.5)
        self.declare_parameter('stale_after', 1.0)
        self.declare_parameter('csv', os.path.expanduser(
            time.strftime('~/.local/share/camera_tracking/platform_meter_%Y%m%d_%H%M%S.csv')))

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.avg_s = float(p('avg_s'))
        self.stale_after = float(p('stale_after'))
        self.csv_path = str(p('csv'))

        self._lock = threading.Lock()
        self._samples: deque[tuple[float, dict[str, float]]] = deque()   # (monotonic t, metrics)
        self._joints: list[float] | None = None
        self._max: dict[str, tuple[float, list[float] | None]] = {}      # field -> (value, joints then)
        self._n_logged = 0

        self.create_subscription(PoseStamped, str(p('pose_topic')), self._pose_cb, 10)
        self.create_subscription(Float32MultiArray, str(p('joint_states_topic')), self._js_cb, 10)
        self.create_timer(float(p('report_period')), self._report)
        threading.Thread(target=self._stdin_loop, daemon=True).start()
        self.get_logger().info(f'reading {p("pose_topic")}; type a label + Enter to log a snapshot '
                               f'to {self.csv_path}, r + Enter resets the maxima')

    # ------------------------------------------------------------ inputs ---

    def _js_cb(self, msg: Float32MultiArray) -> None:
        self._joints = [float(v) for v in msg.data]

    def _pose_cb(self, msg: PoseStamped) -> None:
        o, q = msg.pose.position, msg.pose.orientation
        m = pg.plate_metrics(pg.pose_from_quat_xyz([o.x, o.y, o.z], [q.x, q.y, q.z, q.w]))
        now = time.monotonic()
        with self._lock:
            self._samples.append((now, m))
            while self._samples and now - self._samples[0][0] > max(self.avg_s, self.stale_after):
                self._samples.popleft()
            for k in ('tilt_deg', 'lateral_mm'):
                if k not in self._max or m[k] > self._max[k][0]:
                    self._max[k] = (m[k], self._joints)
            if 'abs_z_mm' not in self._max or abs(m['z_mm']) > abs(self._max['abs_z_mm'][0]):
                self._max['abs_z_mm'] = (m['z_mm'], self._joints)

    def _recent(self) -> list[dict[str, float]]:
        now = time.monotonic()
        with self._lock:
            return [m for t, m in self._samples if now - t <= self.avg_s]

    # ------------------------------------------------------------ output ---

    @staticmethod
    def _fmt_joints(j: list[float] | None) -> str:
        return '-' if j is None else ' '.join(f'{v:5.0f}' for v in j)

    def _report(self) -> None:
        recent = self._recent()
        joints = f'HSA {self._fmt_joints(self._joints)} deg'
        if not recent:
            print(f'{joints} | no pose (tags not seen, or no platform zero yet)', flush=True)
            return
        m = recent[-1]
        line = (f'{joints} | z {m["z_mm"]:+6.2f} mm  lat {m["lateral_mm"]:5.2f} mm '
                f'({m["x_mm"]:+.1f}, {m["y_mm"]:+.1f})  tilt {m["tilt_deg"]:5.2f} deg '
                f'toward {m["tilt_dir_deg"]:+4.0f}  twist {m["twist_deg"]:+5.2f} deg')
        with self._lock:
            mx = dict(self._max)
        if mx:
            line += (f' | max tilt {mx["tilt_deg"][0]:.2f} deg, max |z| {abs(mx["abs_z_mm"][0]):.2f} mm, '
                     f'max lat {mx["lateral_mm"][0]:.2f} mm')
        print(line, flush=True)

    def _snapshot(self, label: str) -> None:
        recent = self._recent()
        if not recent:
            print('snapshot skipped: no pose in the last {:.1f} s'.format(self.avg_s), flush=True)
            return
        mean = {k: float(np.mean([m[k] for m in recent])) for k in FIELDS}
        # Circular mean for the lean direction; it is meaningless at ~0 tilt anyway.
        mean['tilt_dir_deg'] = math.degrees(math.atan2(
            sum(math.sin(math.radians(m['tilt_dir_deg'])) for m in recent),
            sum(math.cos(math.radians(m['tilt_dir_deg'])) for m in recent)))
        row = {'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'label': label, 'n': len(recent),
               **{k: round(v, 3) for k, v in mean.items()},
               'z_std_mm': round(float(np.std([m['z_mm'] for m in recent])), 3),
               'tilt_std_deg': round(float(np.std([m['tilt_deg'] for m in recent])), 3)}
        joints = self._joints or []
        for i, v in enumerate(joints):
            row[f'hsa{i}_deg'] = round(v, 2)
        os.makedirs(os.path.dirname(self.csv_path) or '.', exist_ok=True)
        new = not os.path.exists(self.csv_path)
        with open(self.csv_path, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(row))
            if new:
                w.writeheader()
            w.writerow(row)
        self._n_logged += 1
        print(f'logged #{self._n_logged} "{label}": z {mean["z_mm"]:+.2f} mm (sd {row["z_std_mm"]:.2f}), '
              f'tilt {mean["tilt_deg"]:.2f} deg (sd {row["tilt_std_deg"]:.2f}) toward '
              f'{mean["tilt_dir_deg"]:+.0f}, lat {mean["lateral_mm"]:.2f} mm, twist {mean["twist_deg"]:+.2f} deg, '
              f'{len(recent)} samples -> {self.csv_path}', flush=True)

    def _stdin_loop(self) -> None:
        for line in sys.stdin:
            text = line.strip()
            if text == 'r':
                with self._lock:
                    self._max.clear()
                print('maxima reset', flush=True)
            else:
                self._snapshot(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlatformMeter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
