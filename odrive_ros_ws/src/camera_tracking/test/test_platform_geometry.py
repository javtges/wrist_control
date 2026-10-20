import math

import numpy as np
import pytest

from camera_tracking import platform_geometry as pg


K = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]])


def rot_x(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    T = np.eye(4)
    T[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]
    return T


def rot_y(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    T = np.eye(4)
    T[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return T


def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    return T


def plate_pose(x=0.01, y=-0.005, z=0.15, tilt_x=180 + 8, tilt_y=-5, spin=12):
    """A plate facing the camera (platform z towards the camera), slightly tilted."""
    return pg.translation([x, y, z]) @ rot_x(tilt_x) @ rot_y(tilt_y) @ rot_z(spin)


def synth_detections(layout, T, ids=None):
    ids = layout.ids if ids is None else ids
    return {i: pg.project_points(T, layout.object_points(i), K) for i in ids}


def assert_close(T, T_ref, pos_tol_m=1e-4, rot_tol_deg=0.05):
    assert np.linalg.norm(T[:3, 3] - T_ref[:3, 3]) < pos_tol_m
    assert pg.rotation_angle_deg(pg.invert(T_ref) @ T) < rot_tol_deg


def test_default_layout_matches_the_drawing():
    lay = pg.TagLayout()
    assert lay.ids == [0, 1, 2, 3]
    assert lay.offset(0).tolist() == [0.0, 0.0, 0.0]
    for i in (1, 2, 3):
        assert np.linalg.norm(lay.offset(i)) == pytest.approx(0.040, abs=1e-9)
    assert lay.offset(1)[1] == pytest.approx(0.040)
    assert lay.offset(2)[0] > 0 and lay.offset(3)[0] < 0
    # 120 degrees apart
    a1 = math.atan2(lay.offset(1)[1], lay.offset(1)[0])
    a2 = math.atan2(lay.offset(2)[1], lay.offset(2)[0])
    assert math.degrees(a1 - a2) == pytest.approx(120.0)


def test_layout_validation():
    with pytest.raises(ValueError):
        pg.TagLayout(ids=[0, 1], x_mm=[0.0], y_mm=[0.0, 1.0])
    with pytest.raises(ValueError):
        pg.TagLayout(ids=[0, 0], x_mm=[0.0, 1.0], y_mm=[0.0, 1.0])
    with pytest.raises(ValueError):
        pg.TagLayout(size=0.0)


def test_object_points_follow_apriltag_corner_order():
    lay = pg.TagLayout(size=0.02)
    pts = lay.object_points(1)  # centred at (0, 40 mm)
    assert pts[0].tolist() == pytest.approx([-0.01, 0.03, 0.0])  # bottom-left
    assert pts[1].tolist() == pytest.approx([0.01, 0.03, 0.0])   # bottom-right
    assert pts[2].tolist() == pytest.approx([0.01, 0.05, 0.0])   # top-right
    assert pts[3].tolist() == pytest.approx([-0.01, 0.05, 0.0])  # top-left


def test_quaternion_roundtrip():
    for T in (np.eye(4), rot_x(170), rot_y(-100) @ rot_z(30), plate_pose()):
        q = pg.matrix_to_quat(T)
        assert np.linalg.norm(q) == pytest.approx(1.0)
        back = pg.quat_to_matrix(q)
        assert np.allclose(back[:3, :3], T[:3, :3], atol=1e-9)
    T = plate_pose()
    T2 = pg.pose_from_quat_xyz(T[:3, 3], pg.matrix_to_quat(T))
    assert np.allclose(T2, T, atol=1e-9)


def test_intrinsics_from_projection():
    P = [758.3, 0, 621.7, 0, 0, 788.6, 360, 0, 0, 0, 1, 0]
    Kp = pg.intrinsics_from_projection(P)
    assert Kp.shape == (3, 3)
    assert Kp[0, 0] == 758.3 and Kp[1, 2] == 360 and Kp[2, 2] == 1


def test_each_tag_alone_recovers_the_platform_pose():
    lay = pg.TagLayout()
    T = plate_pose()
    det = synth_detections(lay, T)
    for i in lay.ids:
        assert_close(pg.solve_tag_pose(lay, i, det[i], K), T)


def test_priority_uses_id0_then_the_outer_tags():
    lay = pg.TagLayout()
    T = plate_pose()
    det = synth_detections(lay, T)
    pose, tag = pg.platform_pose(lay, det, K, [0, 1, 2, 3])
    assert tag == 0
    assert_close(pose, T)

    det0_hidden = {i: c for i, c in det.items() if i != 0}
    pose, tag = pg.platform_pose(lay, det0_hidden, K, [0, 1, 2, 3])
    assert tag == 1
    assert_close(pose, T)

    pose, tag = pg.platform_pose(lay, {3: det[3], 9: det[0]}, K, [0, 1, 2, 3])
    assert tag == 3
    assert_close(pose, T)

    pose, tag = pg.platform_pose(lay, {9: det[0]}, K, [0, 1, 2, 3])
    assert pose is None and tag is None


def test_all_mode_refines_over_every_tag():
    lay = pg.TagLayout()
    T = plate_pose()
    det = synth_detections(lay, T)
    rng = np.random.default_rng(0)
    noisy = {i: c + rng.normal(0, 0.3, c.shape) for i, c in det.items()}
    single, _ = pg.platform_pose(lay, noisy, K, [0, 1, 2, 3], source='priority')
    fused, tag = pg.platform_pose(lay, noisy, K, [0, 1, 2, 3], source='all')
    assert tag == 0
    assert_close(fused, T, pos_tol_m=2e-3, rot_tol_deg=1.5)
    err_single = np.linalg.norm(single[:3, 3] - T[:3, 3])
    err_fused = np.linalg.norm(fused[:3, 3] - T[:3, 3])
    assert err_fused <= err_single * 1.5  # fusing never makes it much worse


def test_consistency_is_zero_for_a_correct_layout_and_flags_a_wrong_one():
    lay = pg.TagLayout()
    T = plate_pose()
    det = synth_detections(lay, T)
    errs = pg.tag_consistency_mm(lay, det, K, T)
    assert len(errs) == 4
    assert all(e < 0.2 for e in errs)

    errs = pg.tag_consistency_mm(lay, {0: det[0], 2: det[2]}, K, T)
    assert math.isnan(errs[1]) and math.isnan(errs[3])

    wrong = pg.TagLayout(x_mm=[0.0, 0.0, 34.641, -34.641], y_mm=[0.0, 45.0, -20.0, -20.0])
    errs = pg.tag_consistency_mm(wrong, det, K, T)
    assert errs[1] == pytest.approx(5.0, abs=0.3)


def test_relative_to_zero_is_the_displacement_from_rest():
    T_zero = plate_pose()
    delta = pg.translation([0.002, -0.001, 0.015]) @ rot_z(4)  # 15 mm up, 4 deg twist
    T_now = T_zero @ delta
    rel = pg.relative_to_zero(T_zero, T_now)
    assert np.allclose(rel, delta, atol=1e-12)
    assert np.allclose(pg.relative_to_zero(T_zero, T_zero), np.eye(4), atol=1e-12)


# ------------------------------------------------ plate orientation checks ---

def test_clock_angle_and_position():
    c = (640, 360)
    assert pg.clock_angle_deg(c, (640, 300)) == pytest.approx(0.0)      # up
    assert pg.clock_angle_deg(c, (700, 360)) == pytest.approx(90.0)     # right
    assert pg.clock_angle_deg(c, (640, 420)) == pytest.approx(180.0)    # down
    assert pg.clock_angle_deg(c, (580, 360)) == pytest.approx(270.0)    # left
    assert pg.clock_position(0.0) == 12
    assert pg.clock_position(355.0) == 12
    assert pg.clock_position(90.0) == 3
    assert pg.clock_position(200.0) == 7


def test_plate_centre_px_falls_back_to_outer_centroid():
    lay = pg.TagLayout()
    T = plate_pose()
    centres = {i: pg.project_points(T, lay.offset(i)[None, :], K)[0] for i in lay.ids}
    assert np.allclose(pg.plate_centre_px(centres, lay), centres[0])
    no0 = {i: c for i, c in centres.items() if i != 0}
    # perspective on a tilted plate shifts the centroid a few px (~1 deg of bearing); fine for a clock position
    assert np.allclose(pg.plate_centre_px(no0, lay), centres[0], atol=8.0)
    assert pg.plate_centre_px({1: centres[1]}, lay) is None


def test_outer_tag_order_matches_the_drawing_when_viewed_face_on():
    lay = pg.TagLayout()
    T = plate_pose(tilt_x=180, tilt_y=0, spin=0)   # face on, no spin: tag y up = screen up
    centres = {i: pg.project_points(T, lay.offset(i)[None, :], K)[0] for i in lay.ids}
    bearings = {i: pg.clock_angle_deg(centres[0], centres[i]) for i in (1, 2, 3)}
    assert bearings[1] == pytest.approx(0.0, abs=0.5)
    ids, gaps = pg.outer_tag_order(bearings)
    assert ids == [1, 2, 3]
    assert all(g == pytest.approx(120.0, abs=0.5) for g in gaps)
    assert pg.is_layout_order(ids, [1, 2, 3])
    assert pg.is_layout_order([2, 3, 1], [1, 2, 3])
    assert not pg.is_layout_order([1, 3, 2], [1, 2, 3])
    assert not pg.is_layout_order([1, 2], [1, 2, 3])


def test_tag_over_pair_picks_the_largest_move():
    assert pg.tag_over_pair({1: 0.4, 2: -6.0, 3: 1.0}) == (2, -6.0)
    tag, dz = pg.tag_over_pair({1: 0.4, 2: -1.0, 3: 1.0})
    assert tag is None
    assert pg.tag_over_pair({}) == (None, 0.0)


def test_solve_tag_centre_pose_lands_on_the_tag_centre():
    lay = pg.TagLayout()
    T = plate_pose()
    for i in lay.ids:
        corners = pg.project_points(T, lay.object_points(i), K)
        T_tag = pg.solve_tag_centre_pose(lay, corners, K)
        expected = T @ pg.translation(lay.offset(i))
        assert_close(T_tag, expected)
