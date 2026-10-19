#!/usr/bin/env python3
"""
tag_monitor — live camera view with motor positions; screenshots when the tag is lost.

Shows the baseplate camera image with the AprilTag detection state and the
wrist motor positions drawn on top. Whenever the platform tag stops being
detected it saves the annotated frame, the raw frame and a yaml/csv record of
the motor positions at that moment, so the pose that pushed the tag out of view
can be inspected afterwards.

Topics subscribed:
  image_topic             (sensor_msgs/Image)                 default /tracking_cam/image_rect
  detections_topic        (apriltag_msgs/AprilTagDetectionArray) default /apriltag/detections
  joint_states_topic      (std_msgs/Float32MultiArray) extension angles, degrees
  odrive_positions_topic  (std_msgs/Float32MultiArray) raw ODrive pos_estimate, turns
  joint_commands_topic    (std_msgs/Float32MultiArray) last commanded angles, degrees

Topics published:
  ~/image_annotated       (sensor_msgs/Image) — the overlaid view, for rqt_image_view
  /platform_tag_detected  (std_msgs/Bool)

Services:
  ~/capture               (std_srvs/Trigger) — save a snapshot now

Keys (live window): s = save snapshot, q = quit

Parameters:
  tag_id                 (int)   platform tag id, -1 = any tag (default -1)
  lost_after             (float) seconds without a detection before "lost" (0.5);
                                 the detector must also have reported a frame
                                 without the tag, so a stalled image stream
                                 (no detections messages at all) never counts as lost
  stale_after            (float) seconds without any detections message before the
                                 overlay warns NO DETECTIONS (2.0)
  capture_on_lost        (bool)  save captures automatically while lost (true)
  capture_interval       (float) min seconds between automatic captures (2.0)
  require_prior_detection(bool)  only auto-capture after the tag was seen once (true)
  output_dir             (str)   capture folder (~/wrist_captures)
  save_raw               (bool)  also save the un-annotated frame (true)
  show_window            (bool)  open an OpenCV window (true)
  publish_annotated      (bool)  publish ~/image_annotated (true)
"""

import csv
import datetime as _dt
import math
import os
import time

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import Trigger

CSV_NAME = 'captures.csv'
CSV_FIELDS = [
    'timestamp', 'reason', 'tag_id', 'detected_ids', 'since_seen_s',
    'joint_states_deg', 'odrive_positions_turns', 'joint_commands_deg',
    'image_file',
]

GREEN = (0, 200, 0)
RED = (0, 0, 230)
ORANGE = (0, 160, 255)
WHITE = (255, 255, 255)
YELLOW = (0, 255, 255)


# ---------------------------------------------------------------- helpers ---

def fmt_values(values, fmt='{:7.1f}') -> str:
    """Format a list of floats compactly; None/empty -> 'n/a'."""
    if not values:
        return 'n/a'
    return ' '.join('    nan' if (v is None or math.isnan(v)) else fmt.format(v) for v in values)


def build_record(timestamp: str, reason: str, tag_id: int, detected_ids, since_seen,
                 joint_states, odrive_positions, joint_commands,
                 image_topic: str, image_file: str, raw_file: str | None) -> dict:
    """Everything worth knowing about one capture, as plain python types."""
    def clean(values):
        return None if values is None else [float(v) for v in values]
    return {
        'timestamp': timestamp,
        'reason': reason,
        'tag_id': int(tag_id),
        'detected_ids': [int(i) for i in detected_ids],
        'since_seen_s': None if since_seen is None else round(float(since_seen), 3),
        'joint_states_deg': clean(joint_states),
        'odrive_positions_turns': clean(odrive_positions),
        'joint_commands_deg': clean(joint_commands),
        'image_topic': image_topic,
        'image_file': image_file,
        'raw_file': raw_file,
    }


def record_to_csv_row(rec: dict) -> dict:
    def join(values):
        return '' if values is None else ' '.join(f'{v:.4f}' for v in values)
    return {
        'timestamp': rec['timestamp'],
        'reason': rec['reason'],
        'tag_id': rec['tag_id'],
        'detected_ids': ' '.join(str(i) for i in rec['detected_ids']),
        'since_seen_s': '' if rec['since_seen_s'] is None else rec['since_seen_s'],
        'joint_states_deg': join(rec['joint_states_deg']),
        'odrive_positions_turns': join(rec['odrive_positions_turns']),
        'joint_commands_deg': join(rec['joint_commands_deg']),
        'image_file': rec['image_file'],
    }


def _put_text(img, text, org, color=WHITE, scale=0.55):
    """Text with a dark backing box so it reads on any image."""
    (w, h), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x, y = org
    cv2.rectangle(img, (x - 3, y - h - 3), (x + w + 3, y + base + 2), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def draw_overlay(frame: np.ndarray, detected: bool | None, since_seen, tags,
                 joint_states, odrive_positions, joint_commands,
                 n_captures: int, last_capture: str | None, stale: bool = False) -> np.ndarray:
    """Return a copy of frame with tag outlines and status/motor text drawn on it.

    tags: list of (id, corners[4x2], centre[2]) currently detected.
    detected: True / False / None (never seen yet).
    """
    img = frame.copy()
    for tag_id, corners, centre in tags:
        pts = np.asarray(corners, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(img, [pts], True, GREEN, 2)
        cx, cy = int(centre[0]), int(centre[1])
        cv2.circle(img, (cx, cy), 4, GREEN, -1)
        _put_text(img, f'id {tag_id}', (cx + 8, cy - 8), GREEN)

    if detected and tags:
        status, color = 'TAG DETECTED ' + ' '.join(str(t[0]) for t in tags), GREEN
    elif detected:
        # last frame had no tag but lost_after has not elapsed yet
        since = '' if since_seen is None else f' {since_seen:.1f}s'
        status, color = 'TAG MISSING' + since, ORANGE
    elif detected is None:
        status, color = 'TAG NOT SEEN YET', ORANGE
    else:
        since = '' if since_seen is None else f' {since_seen:.1f}s'
        status, color = 'TAG LOST' + since, RED

    if stale:
        status, color = status + '  (NO DETECTIONS MSGS)', ORANGE

    y = 24
    _put_text(img, status, (10, y), color, 0.8)
    y += 28
    _put_text(img, _dt.datetime.now().strftime('%H:%M:%S.%f')[:-3], (10, y))
    y += 26
    _put_text(img, 'ext deg : ' + fmt_values(joint_states), (10, y), YELLOW)
    y += 22
    _put_text(img, 'turns   : ' + fmt_values(odrive_positions, '{:7.3f}'), (10, y), YELLOW)
    y += 22
    _put_text(img, 'cmd deg : ' + fmt_values(joint_commands), (10, y), YELLOW)

    foot = f'captures: {n_captures}'
    if last_capture:
        foot += f'   last: {os.path.basename(last_capture)}'
    foot += '   [s] snapshot  [q] quit'
    _put_text(img, foot, (10, img.shape[0] - 10), WHITE, 0.5)
    return img


# ------------------------------------------------------------------- node ---

class TagMonitor(Node):
    def __init__(self) -> None:
        super().__init__('tag_monitor')

        self.declare_parameter('image_topic', '/tracking_cam/image_rect')
        self.declare_parameter('detections_topic', '/apriltag/detections')
        self.declare_parameter('joint_states_topic', '/hsa_odrive_control/joint_states')
        self.declare_parameter('odrive_positions_topic', '/odrive_positions')
        self.declare_parameter('joint_commands_topic', '/hsa_odrive_control/joint_commands')
        self.declare_parameter('tag_id', -1)
        self.declare_parameter('lost_after', 0.5)
        self.declare_parameter('stale_after', 2.0)
        self.declare_parameter('capture_on_lost', True)
        self.declare_parameter('capture_interval', 2.0)
        self.declare_parameter('require_prior_detection', True)
        self.declare_parameter('output_dir', '~/wrist_captures')
        self.declare_parameter('save_raw', True)
        self.declare_parameter('show_window', True)
        self.declare_parameter('publish_annotated', True)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.image_topic: str = p('image_topic')
        self.tag_id: int = p('tag_id')
        self.lost_after: float = p('lost_after')
        self.stale_after: float = p('stale_after')
        self.capture_on_lost: bool = p('capture_on_lost')
        self.capture_interval: float = p('capture_interval')
        self.require_prior_detection: bool = p('require_prior_detection')
        self.output_dir: str = os.path.expanduser(p('output_dir'))
        self.save_raw: bool = p('save_raw')
        self.show_window: bool = p('show_window')
        self.publish_annotated: bool = p('publish_annotated')

        os.makedirs(self.output_dir, exist_ok=True)

        self._bridge = CvBridge()
        self._frame: np.ndarray | None = None
        self._annotated: np.ndarray | None = None
        self._tags: list = []              # (id, corners, centre) of matching tags
        self._detected_ids: list[int] = []  # every id in the last detections msg
        self._last_seen: float | None = None
        self._last_msg: float | None = None      # any detections message
        self._last_msg_had_tag = False
        self._detected: bool | None = None  # None until first detection
        self._last_capture_time = -math.inf
        self._n_captures = 0
        self._last_capture: str | None = None

        self._joint_states: list[float] | None = None
        self._odrive_positions: list[float] | None = None
        self._joint_commands: list[float] | None = None

        self.create_subscription(Image, self.image_topic, self._on_image, 1)
        self.create_subscription(AprilTagDetectionArray, p('detections_topic'),
                                 self._on_detections, 10)
        self.create_subscription(Float32MultiArray, p('joint_states_topic'),
                                 lambda m: setattr(self, '_joint_states', list(m.data)), 10)
        self.create_subscription(Float32MultiArray, p('odrive_positions_topic'),
                                 lambda m: setattr(self, '_odrive_positions', list(m.data)), 10)
        self.create_subscription(Float32MultiArray, p('joint_commands_topic'),
                                 lambda m: setattr(self, '_joint_commands', list(m.data)), 10)

        self._annotated_pub = self.create_publisher(Image, '~/image_annotated', 1)
        self._detected_pub = self.create_publisher(Bool, '/platform_tag_detected', 10)
        self.create_service(Trigger, '~/capture', self._on_capture_srv)

        self.create_timer(0.05, self._tick)

        self._window = 'tag_monitor'
        self.get_logger().info(
            f'Watching {self.image_topic}, tag id '
            f'{"any" if self.tag_id < 0 else self.tag_id}; captures -> {self.output_dir}')

    # --- callbacks ---------------------------------------------------------

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        self._detected_ids = [d.id for d in msg.detections]
        tags = []
        for d in msg.detections:
            if self.tag_id >= 0 and d.id != self.tag_id:
                continue
            corners = [(c.x, c.y) for c in d.corners]
            tags.append((d.id, corners, (d.centre.x, d.centre.y)))
        self._tags = tags
        self._last_msg = time.monotonic()
        self._last_msg_had_tag = bool(tags)
        if tags:
            self._last_seen = self._last_msg

    def _on_image(self, msg: Image) -> None:
        try:
            self._frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # cv_bridge conversion problems
            self.get_logger().warn(f'image conversion failed: {e}', throttle_duration_sec=5.0)
            return

        self._annotated = self._render()

        if self.publish_annotated:
            out = self._bridge.cv2_to_imgmsg(self._annotated, encoding='bgr8')
            out.header = msg.header
            self._annotated_pub.publish(out)

        if self.show_window:
            try:
                cv2.imshow(self._window, self._annotated)
                key = cv2.waitKey(1) & 0xFF
            except cv2.error as e:
                self.get_logger().warn(f'no display, disabling window: {e}')
                self.show_window = False
                return
            if key == ord('s'):
                self.capture('manual')
            elif key == ord('q'):
                self.get_logger().info('q pressed, shutting down')
                raise SystemExit

    def _render(self) -> np.ndarray:
        return draw_overlay(
            self._frame, self._detected, self._since_seen(), self._tags,
            self._joint_states, self._odrive_positions, self._joint_commands,
            self._n_captures, self._last_capture, stale=self._stale())

    def _on_capture_srv(self, _req, res):
        path = self.capture('manual')
        res.success = path is not None
        res.message = path or 'no image received yet'
        return res

    # --- lost/detected state -----------------------------------------------

    def _since_seen(self) -> float | None:
        return None if self._last_seen is None else time.monotonic() - self._last_seen

    def _stale(self) -> bool:
        return self._last_msg is None or time.monotonic() - self._last_msg > self.stale_after

    def _tick(self) -> None:
        since = self._since_seen()
        if since is None:
            detected = None
        elif since < self.lost_after or self._last_msg_had_tag:
            # Still seen recently, or the detector has not yet reported a
            # frame without the tag (message gap, not a real loss).
            detected = True
        else:
            detected = False
        if detected is False:
            self._tags = []  # stale corners; stop drawing them

        self._detected_pub.publish(Bool(data=bool(detected)))

        transition = detected != self._detected
        if transition:
            self._detected = detected
            if detected:
                self.get_logger().info('tag detected')
            elif detected is False:
                self.get_logger().warn('tag LOST')

        if detected is False and self.capture_on_lost:
            self._auto_capture(transition)
        elif detected is None and self.capture_on_lost and not self.require_prior_detection:
            self._auto_capture(False)

    def _auto_capture(self, transition: bool) -> None:
        now = time.monotonic()
        if transition or now - self._last_capture_time >= self.capture_interval:
            self.capture('lost')

    # --- capture -----------------------------------------------------------

    def capture(self, reason: str) -> str | None:
        """Save annotated + raw frame and a yaml/csv record. Returns the png path."""
        if self._frame is None:
            self.get_logger().warn('capture requested but no image received yet')
            return None
        now = _dt.datetime.now()
        stamp = now.strftime('%Y%m%d_%H%M%S_') + f'{now.microsecond // 1000:03d}'
        base = os.path.join(self.output_dir, f'{stamp}_{reason}')

        # Redraw so the saved frame shows the state at capture time, not the
        # state when the last image arrived.
        annotated = self._render()
        image_file = base + '.png'
        cv2.imwrite(image_file, annotated)
        raw_file = None
        if self.save_raw:
            raw_file = base + '_raw.png'
            cv2.imwrite(raw_file, self._frame)

        rec = build_record(
            now.isoformat(timespec='milliseconds'), reason, self.tag_id,
            self._detected_ids, self._since_seen(),
            self._joint_states, self._odrive_positions, self._joint_commands,
            self.image_topic, image_file, raw_file)
        with open(base + '.yaml', 'w') as f:
            yaml.safe_dump(rec, f, sort_keys=False)

        csv_path = os.path.join(self.output_dir, CSV_NAME)
        new = not os.path.exists(csv_path)
        with open(csv_path, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            if new:
                w.writeheader()
            w.writerow(record_to_csv_row(rec))

        self._last_capture_time = time.monotonic()
        self._n_captures += 1
        self._last_capture = image_file
        self.get_logger().info(
            f'[{reason}] saved {os.path.basename(image_file)}  '
            f'ext deg: {fmt_values(self._joint_states)}')
        return image_file

    def destroy_node(self):
        if self.show_window:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TagMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
