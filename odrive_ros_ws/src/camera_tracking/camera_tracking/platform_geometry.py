"""
platform_geometry — tag layout, PnP and frame maths for the platform tracker.

Pure numpy/OpenCV, no ROS, so the maths is unit-testable.

Frames
  camera    : the rectified camera frame (OpenCV convention, z forward).
  platform  : the ID0 tag frame: origin at the centre of ID0, x to the right
              and y up along the printed tag, z out of the plate towards the
              camera. Every other tag lies in the same plane, printed with
              the same orientation, at a known (x, y) offset from ID0.
  zero      : the platform frame captured when the HSAs were zeroed and the
              platform sat flat. `/platform_pose` is platform expressed in zero.

Tag corner order follows apriltag (and apriltag_ros): the four corners wrap
counter-clockwise starting bottom-left, so with a tag of edge `s` centred at
(cx, cy):  p0 = (cx-s/2, cy-s/2)  p1 = (cx+s/2, cy-s/2)
           p2 = (cx+s/2, cy+s/2)  p3 = (cx-s/2, cy+s/2)
The single-tag pose solved here therefore matches the TF AprilTagNode
broadcasts for that tag.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

# Default plate layout (metres, in the ID0 frame): ID1 straight above ID0,
# ID2 lower right and ID3 lower left, all 40 mm from the centre at 120 degrees.
DEFAULT_TAG_IDS = [0, 1, 2, 3]
DEFAULT_TAG_X_MM = [0.0, 0.0, 40.0 * math.cos(math.radians(-30.0)), -40.0 * math.cos(math.radians(-30.0))]
DEFAULT_TAG_Y_MM = [0.0, 40.0, -20.0, -20.0]
DEFAULT_TAG_SIZE_M = 0.012


@dataclass
class TagLayout:
    """Where each tag sits on the plate, in the platform (ID0) frame."""

    size: float = DEFAULT_TAG_SIZE_M
    ids: list[int] = field(default_factory=lambda: list(DEFAULT_TAG_IDS))
    x_mm: list[float] = field(default_factory=lambda: list(DEFAULT_TAG_X_MM))
    y_mm: list[float] = field(default_factory=lambda: list(DEFAULT_TAG_Y_MM))

    def __post_init__(self) -> None:
        if not (len(self.ids) == len(self.x_mm) == len(self.y_mm)):
            raise ValueError('tag_ids, tag_x_mm and tag_y_mm must have the same length')
        if len(set(self.ids)) != len(self.ids):
            raise ValueError('tag_ids must be unique')
        if self.size <= 0:
            raise ValueError('tag size must be positive')

    def offset(self, tag_id: int) -> np.ndarray:
        """Centre of `tag_id` in the platform frame, metres (x, y, 0)."""
        i = self.ids.index(tag_id)
        return np.array([self.x_mm[i] * 1e-3, self.y_mm[i] * 1e-3, 0.0])

    def object_points(self, tag_id: int) -> np.ndarray:
        """(4, 3) corners of `tag_id` in the platform frame, apriltag order."""
        cx, cy, _ = self.offset(tag_id)
        h = self.size / 2.0
        return np.array([
            [cx - h, cy - h, 0.0],
            [cx + h, cy - h, 0.0],
            [cx + h, cy + h, 0.0],
            [cx - h, cy + h, 0.0],
        ])

    def centred_object_points(self) -> np.ndarray:
        """(4, 3) corners of a tag about its own centre, apriltag order."""
        h = self.size / 2.0
        return np.array([[-h, -h, 0.0], [h, -h, 0.0], [h, h, 0.0], [-h, h, 0.0]])


# ------------------------------------------------------------------ SE(3) ---

def rt_to_matrix(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """Rodrigues vector + translation -> 4x4 homogeneous transform."""
    T = np.eye(4)
    T[:3, :3], _ = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))
    T[:3, 3] = np.asarray(tvec, dtype=float).reshape(3)
    return T


def matrix_to_rt(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    rvec, _ = cv2.Rodrigues(np.ascontiguousarray(T[:3, :3]))
    return rvec.reshape(3), T[:3, 3].copy()


def invert(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ T[:3, 3]
    return out


def translation(xyz) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = np.asarray(xyz, dtype=float).reshape(3)
    return T


def matrix_to_quat(T: np.ndarray) -> np.ndarray:
    """Rotation of T as a unit quaternion (x, y, z, w)."""
    R = T[:3, :3]
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def quat_to_matrix(q) -> np.ndarray:
    """Unit quaternion (x, y, z, w) -> 4x4 transform with zero translation."""
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    T = np.eye(4)
    T[:3, :3] = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return T


def pose_from_quat_xyz(xyz, q) -> np.ndarray:
    T = quat_to_matrix(q)
    T[:3, 3] = np.asarray(xyz, dtype=float).reshape(3)
    return T


def rotation_angle_deg(T: np.ndarray) -> float:
    """Magnitude of the rotation in T, degrees."""
    c = (np.trace(T[:3, :3]) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


# ------------------------------------------------------------------- PnP ---

def intrinsics_from_projection(p: np.ndarray) -> np.ndarray:
    """3x3 K from a CameraInfo P (3x4, row-major, 12 values).

    The detector runs on the rectified image, so P (not K) describes the pixels
    the corners live in and there is no distortion left to model.
    """
    return np.asarray(p, dtype=float).reshape(3, 4)[:, :3].copy()


def solve_tag_pose(layout: TagLayout, tag_id: int, corners: np.ndarray, K: np.ndarray) -> np.ndarray | None:
    """camera -> platform from one tag's four corners ((4, 2) rectified pixels).

    Solves the tag about its own centre with IPPE_SQUARE (the planar-square
    solver, unambiguous for one tag) then shifts by the tag's plate offset.
    """
    img = np.asarray(corners, dtype=float).reshape(4, 2)
    obj = layout.centred_object_points()
    # IPPE_SQUARE wants top-left, top-right, bottom-right, bottom-left.
    order = [3, 2, 1, 0]
    ok, rvec, tvec = cv2.solvePnP(obj[order], img[order], K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    T_cam_tag = rt_to_matrix(rvec, tvec)
    return T_cam_tag @ translation(-layout.offset(tag_id))


def solve_multi_tag_pose(layout: TagLayout, detections: dict[int, np.ndarray], K: np.ndarray,
                         guess: np.ndarray | None = None) -> np.ndarray | None:
    """camera -> platform from every visible tag at once (>= 1 tag).

    With one tag this is the same as `solve_tag_pose`; with more it refines a
    single pose against all corners, which averages out per-tag corner noise.
    """
    ids = [i for i in layout.ids if i in detections]
    if not ids:
        return None
    if guess is None:
        guess = solve_tag_pose(layout, ids[0], detections[ids[0]], K)
        if guess is None:
            return None
    if len(ids) == 1:
        return guess
    obj = np.vstack([layout.object_points(i) for i in ids])
    img = np.vstack([np.asarray(detections[i], dtype=float).reshape(4, 2) for i in ids])
    rvec0, tvec0 = matrix_to_rt(guess)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, None, rvec0.reshape(3, 1), tvec0.reshape(3, 1),
                                  useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return guess
    return rt_to_matrix(rvec, tvec)


def choose_source_tag(priority: list[int], visible: set[int]) -> int | None:
    """First id in `priority` that is visible (ID0 before the outer tags)."""
    for tag_id in priority:
        if tag_id in visible:
            return tag_id
    return None


def platform_pose(layout: TagLayout, detections: dict[int, np.ndarray], K: np.ndarray,
                  priority: list[int], source: str = 'priority') -> tuple[np.ndarray | None, int | None]:
    """camera -> platform plus the tag id it was anchored on.

    source='priority': the pose comes from the first visible tag in `priority`
    (the spec's hierarchy: ID0 when it is seen, else one of the others).
    source='all': the priority tag seeds a refinement over every visible tag.
    """
    visible = {i for i in detections if i in layout.ids}
    tag_id = choose_source_tag(priority, visible)
    if tag_id is None:
        return None, None
    T = solve_tag_pose(layout, tag_id, detections[tag_id], K)
    if T is None:
        return None, None
    if source == 'all' and len(visible) > 1:
        refined = solve_multi_tag_pose(layout, detections, K, guess=T)
        if refined is not None:
            T = refined
    return T, tag_id


def tag_consistency_mm(layout: TagLayout, detections: dict[int, np.ndarray], K: np.ndarray,
                       T_ref: np.ndarray) -> list[float]:
    """Per layout tag: distance (mm) between the platform origin as that tag
    alone sees it and the origin of `T_ref`; NaN when the tag is not visible.

    Non-zero values with every tag visible mean the configured offsets (or
    the assumption that all tags are printed in the same orientation) are off.
    """
    out = []
    for tag_id in layout.ids:
        if tag_id not in detections:
            out.append(math.nan)
            continue
        T = solve_tag_pose(layout, tag_id, detections[tag_id], K)
        if T is None:
            out.append(math.nan)
            continue
        out.append(float(np.linalg.norm(T[:3, 3] - T_ref[:3, 3]) * 1e3))
    return out


def relative_to_zero(T_cam_zero: np.ndarray, T_cam_platform: np.ndarray) -> np.ndarray:
    """platform expressed in the zero frame: zero -> platform."""
    return invert(T_cam_zero) @ T_cam_platform


def project_points(T_cam_obj: np.ndarray, points_obj: np.ndarray, K: np.ndarray) -> np.ndarray:
    """(N, 2) pixels of (N, 3) object points; test helper and overlay helper."""
    rvec, tvec = matrix_to_rt(T_cam_obj)
    px, _ = cv2.projectPoints(np.asarray(points_obj, dtype=float), rvec, tvec, K, None)
    return px.reshape(-1, 2)
