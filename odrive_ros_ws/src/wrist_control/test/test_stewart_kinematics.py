"""
Unit tests for wrist_control.stewart_kinematics.

These tests exercise the pure kinematics and do not require a running ROS2
environment, CAN bus or SpaceMouse.
"""

import numpy as np
import pytest

from wrist_control.stewart_kinematics import (
    MAX_LIN,
    MAX_ROT,
    anchor_angles,
    clamp_to_limits,
    leg_lengths,
    leg_lengths_to_joint_angles,
    spacemouse_to_joint_angles,
    spacemouse_to_pose,
)

# Inputs used to compare against rigid_dynamics.py: (x, y, z, roll, pitch, yaw)
SAMPLE_INPUTS = [
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    (0.0, 0.0, 0.5, 0.0, 0.0, 0.0),
    (0.3, -0.2, -0.4, 0.25, -0.6, 0.8),
]


def _rigid_dynamics_reference(x, y, z, roll, pitch, yaw):
    """Verbatim math from resource/rigid_dynamics.py (counter-clockwise legs),
    including the spacemouse_node euler2quat → quat2euler round trip."""
    transforms3d = pytest.importorskip("transforms3d")
    pi = np.pi
    gamma_B, gamma_P = 0.3639382215786, 0.3818744117959
    psi_B = np.array([-gamma_B, gamma_B, 2*pi/3 - gamma_B, 2*pi/3 + gamma_B,
                      2*pi/3 + 2*pi/3 - gamma_B, 2*pi/3 + 2*pi/3 + gamma_B])
    psi_P = np.array([pi/3 + 2*pi/3 + 2*pi/3 + gamma_P, pi/3 + -gamma_P, pi/3 + gamma_P,
                      pi/3 + 2*pi/3 - gamma_P, pi/3 + 2*pi/3 + gamma_P,
                      pi/3 + 2*pi/3 + 2*pi/3 - gamma_P])
    B = 11.28616996 * np.array([[np.cos(a), np.sin(a), 0] for a in psi_B]).T
    P = 7.874 * np.array([[np.cos(a), np.sin(a), 0] for a in psi_P]).T
    home_pos = np.array([0, 0, 6.1341])

    quat = transforms3d.euler.euler2quat(roll, pitch, yaw)
    r, p, yw = transforms3d.euler.quat2euler(quat)
    rot = np.array([r * pi/5 * -1, p * pi/5 * -1, yw * pi/5])
    trans = np.array([0, 0, z * 10])

    def rx(t): return np.array([[1, 0, 0], [0, np.cos(t), -np.sin(t)], [0, np.sin(t), np.cos(t)]])
    def ry(t): return np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]])
    def rz(t): return np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]])

    R = rx(rot[0]) @ ry(rot[1]) @ rz(rot[2])
    l = np.repeat(trans[:, None], 6, axis=1) + np.repeat(home_pos[:, None], 6, axis=1) + R @ P - B
    lll = np.linalg.norm(l, axis=0)
    deg = np.around(np.degrees((lll - 6.814) / 1.23), decimals=2)
    deg += -1*min(deg) if min(deg) < 0 else 0
    return deg


# ---------------------------------------------------------------------------
# Matches rigid_dynamics.py
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("inputs", SAMPLE_INPUTS)
def test_counter_clockwise_matches_rigid_dynamics(inputs):
    ours = spacemouse_to_joint_angles(*inputs, clockwise=False)
    np.testing.assert_allclose(ours, _rigid_dynamics_reference(*inputs), atol=1e-9)


def test_home_pose_is_equal_on_all_joints():
    angles = spacemouse_to_joint_angles(0, 0, 0, 0, 0, 0)
    assert np.allclose(angles, angles[0])
    assert angles[0] == pytest.approx(35.14)


def test_full_down_z_is_all_zero():
    np.testing.assert_allclose(spacemouse_to_joint_angles(0, 0, -1, 0, 0, 0), 0.0)


def test_x_and_y_translation_are_ignored():
    base = spacemouse_to_joint_angles(0, 0, 0.2, 0.3, 0, 0)
    moved = spacemouse_to_joint_angles(1, -1, 0.2, 0.3, 0, 0)
    np.testing.assert_allclose(base, moved)


def test_pose_scaling_and_signs():
    assert spacemouse_to_pose(0, 0, 1, 1, 1, 1) == pytest.approx(
        (-MAX_ROT, -MAX_ROT, MAX_ROT, MAX_LIN)
    )


# ---------------------------------------------------------------------------
# Clockwise numbering
# ---------------------------------------------------------------------------

def test_clockwise_anchors_run_clockwise_from_above():
    psi_B, _ = anchor_angles(clockwise=True)
    # Unwrapped polar angle must strictly decrease motor-to-motor (clockwise).
    assert np.all(np.diff(np.unwrap(psi_B)) < 0)
    psi_B_ccw, _ = anchor_angles(clockwise=False)
    assert np.all(np.diff(np.unwrap(psi_B_ccw)) > 0)


def test_clockwise_is_same_physical_anchor_set():
    """Mirroring only renumbers legs; the set of anchor points is unchanged."""
    for cw_psi, ccw_psi in zip(anchor_angles(True), anchor_angles(False)):
        cw = np.sort(np.mod(np.round(cw_psi, 9), 2*np.pi))
        ccw = np.sort(np.mod(np.round(ccw_psi, 9), 2*np.pi))
        np.testing.assert_allclose(cw, ccw, atol=1e-9)


def _gap(a, b):
    return np.abs(np.angle(np.exp(1j * (a - b))))


def test_clockwise_pairs_apart_on_base_meet_at_top():
    """On the wrist, motors (0,1), (2,3), (4,5) are apart on the base and
    meet together at the top of the platform."""
    psi_B, psi_P = anchor_angles(clockwise=True)
    for a, b in [(0, 1), (2, 3), (4, 5)]:
        assert _gap(psi_P[a], psi_P[b]) == pytest.approx(2 * 0.3818744117959)
        assert _gap(psi_B[a], psi_B[b]) == pytest.approx(2*np.pi/3 - 2 * 0.3639382215786)


def test_clockwise_motor_0_is_rigid_dynamics_leg_0():
    cw_B, cw_P = anchor_angles(clockwise=True)
    ccw_B, ccw_P = anchor_angles(clockwise=False)
    assert cw_B[0] == ccw_B[0] and cw_P[0] == ccw_P[0]


@pytest.mark.parametrize("inputs", SAMPLE_INPUTS)
def test_clockwise_is_rigid_dynamics_renumbered(inputs):
    """Motor k gets what rigid_dynamics.py computes for its leg (-k) % 6."""
    cw = spacemouse_to_joint_angles(*inputs, clockwise=True)
    ccw = spacemouse_to_joint_angles(*inputs, clockwise=False)
    np.testing.assert_allclose(cw, ccw[[0, 5, 4, 3, 2, 1]])


def test_leg_offset_rotates_motor_assignment():
    pose = (0.1, -0.2, 0.15, 0.0)
    base = leg_lengths(*pose, leg_offset=0)
    for k in range(6):
        np.testing.assert_allclose(leg_lengths(*pose, leg_offset=k), np.roll(base, -k))


# ---------------------------------------------------------------------------
# Joint limits
# ---------------------------------------------------------------------------

def test_shift_only_when_negative():
    np.testing.assert_allclose(
        leg_lengths_to_joint_angles(np.full(6, 6.814 + 1.23)),
        np.full(6, 57.3),
    )


def test_clamp_to_limits():
    low, high = np.zeros(6), np.full(6, 360.0)
    out = clamp_to_limits([-5, 0, 100, 360, 400, 820.96], low, high)
    np.testing.assert_allclose(out, [0, 0, 100, 360, 360, 360])


def test_full_deflection_exceeds_range_so_clamp_matters():
    angles = spacemouse_to_joint_angles(0, 0, 1, 0, 0, 0)
    assert np.all(angles > 360)
