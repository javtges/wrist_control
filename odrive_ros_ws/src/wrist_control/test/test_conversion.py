"""
Unit tests for wrist_control.conversion.

These tests exercise the pure mathematical functions and do not require
a running ROS2 environment or CAN bus.
"""

import pytest
from wrist_control.conversion import (
    SAFETY_MIN_TURNS,
    SAFETY_MAX_TURNS,
    DEGREES_PER_TURN,
    clamp,
    degrees_to_turns,
    compute_setpoint,
    compute_zero_offset,
)


# ---------------------------------------------------------------------------
# Minimal HSA stub — mirrors the interface used by conversion functions
# ---------------------------------------------------------------------------

class _HSA:
    CW = 'CW'
    CCW = 'CCW'

    def __init__(self, handedness, min_angle, max_angle, zero_point, gear_ratio=1.0):
        self.handedness = handedness
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.zero_point = zero_point
        self.gear_ratio = gear_ratio

    def move(self, angle: float) -> float:
        if self.handedness == self.CW:
            motor = angle
        else:
            motor = self.max_angle - angle
        return max(self.min_angle, min(self.max_angle, motor))


# ---------------------------------------------------------------------------
# degrees_to_turns
# ---------------------------------------------------------------------------

def test_degrees_to_turns_full_rotation():
    assert degrees_to_turns(360.0) == pytest.approx(1.0)


def test_degrees_to_turns_half():
    assert degrees_to_turns(180.0) == pytest.approx(0.5)


def test_degrees_to_turns_zero():
    assert degrees_to_turns(0.0) == pytest.approx(0.0)


def test_degrees_to_turns_negative():
    assert degrees_to_turns(-180.0) == pytest.approx(-0.5)


# ---------------------------------------------------------------------------
# clamp
# ---------------------------------------------------------------------------

def test_clamp_within():
    assert clamp(5.0, 0.0, 10.0) == pytest.approx(5.0)


def test_clamp_below():
    assert clamp(-1.0, 0.0, 10.0) == pytest.approx(0.0)


def test_clamp_above():
    assert clamp(11.0, 0.0, 10.0) == pytest.approx(10.0)


# ---------------------------------------------------------------------------
# compute_setpoint — CW actuator, gear_ratio=1 (no scaling)
# ---------------------------------------------------------------------------

def test_setpoint_cw_zero_deg():
    hsa = _HSA('CW', 0.0, 360.0, 0.0)
    assert compute_setpoint(0.0, hsa, 0.0) == pytest.approx(0.0)


def test_setpoint_cw_180_deg():
    hsa = _HSA('CW', 0.0, 360.0, 0.0)
    assert compute_setpoint(180.0, hsa, 0.0) == pytest.approx(0.5)


def test_setpoint_cw_360_deg():
    hsa = _HSA('CW', 0.0, 360.0, 0.0)
    assert compute_setpoint(360.0, hsa, 0.0) == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# compute_setpoint — CCW actuator, gear_ratio=1
# ---------------------------------------------------------------------------

def test_setpoint_ccw_zero_ext_is_one_turn():
    hsa = _HSA('CCW', 0.0, 360.0, 0.0)
    # extension=0 → output=360 deg = 1.0 turns
    assert compute_setpoint(0.0, hsa, 0.0) == pytest.approx(1.0)


def test_setpoint_ccw_180_ext_is_half_turn():
    hsa = _HSA('CCW', 0.0, 360.0, 0.0)
    # extension=180 → output=180 deg = 0.5 turns
    assert compute_setpoint(180.0, hsa, 0.0) == pytest.approx(0.5)


def test_setpoint_ccw_360_ext_is_zero_turns():
    hsa = _HSA('CCW', 0.0, 360.0, 0.0)
    # extension=360 → output=0 deg = 0.0 turns
    assert compute_setpoint(360.0, hsa, 0.0) == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# compute_setpoint — gear_ratio=2 (wrist config)
# ---------------------------------------------------------------------------

def test_setpoint_gear_ratio_cw_180():
    hsa = _HSA('CW', 0.0, 360.0, 0.0, gear_ratio=2.0)
    # 180° → 0.5 output turns * 2.0 = 1.0 motor turns
    assert compute_setpoint(180.0, hsa, 0.0) == pytest.approx(1.0)


def test_setpoint_gear_ratio_cw_360():
    hsa = _HSA('CW', 0.0, 360.0, 0.0, gear_ratio=2.0)
    # 360° → 1.0 output turns * 2.0 = 2.0 motor turns
    assert compute_setpoint(360.0, hsa, 0.0) == pytest.approx(2.0)


def test_setpoint_gear_ratio_ccw_zero():
    hsa = _HSA('CCW', 0.0, 360.0, 0.0, gear_ratio=2.0)
    # extension=0 → output=1.0 turns * 2.0 = 2.0 motor turns
    assert compute_setpoint(0.0, hsa, 0.0) == pytest.approx(2.0)


def test_setpoint_gear_ratio_ccw_180():
    hsa = _HSA('CCW', 0.0, 360.0, 0.0, gear_ratio=2.0)
    # extension=180 → output=0.5 turns * 2.0 = 1.0 motor turns
    assert compute_setpoint(180.0, hsa, 0.0) == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# compute_setpoint — safety clamps (applied on output turns, before gear ratio)
# ---------------------------------------------------------------------------

def test_safety_upper_clamp_via_input():
    # Wide config (max_angle=720), gear_ratio=1; input above 540 → clamped to 1.5 output turns
    hsa = _HSA('CW', 0.0, 720.0, 0.0)
    assert compute_setpoint(900.0, hsa, 0.0) == pytest.approx(SAFETY_MAX_TURNS)


def test_safety_lower_clamp_via_input():
    hsa = _HSA('CW', -360.0, 360.0, 0.0)
    assert compute_setpoint(-720.0, hsa, 0.0) == pytest.approx(SAFETY_MIN_TURNS)


def test_safety_clamp_scales_with_gear_ratio():
    # Safety clamp is on output turns; result is still multiplied by gear_ratio
    hsa = _HSA('CW', 0.0, 720.0, 0.0, gear_ratio=2.0)
    # 900° → safety clamp to 540° = 1.5 output turns * 2.0 = 3.0 motor turns
    assert compute_setpoint(900.0, hsa, 0.0) == pytest.approx(SAFETY_MAX_TURNS * 2.0)


def test_robot_config_clamp_applied():
    # max_angle=180; commanding 360 → clamped to 180 → 0.5 turns
    hsa = _HSA('CW', 0.0, 180.0, 0.0)
    assert compute_setpoint(360.0, hsa, 0.0) == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# compute_setpoint — zero offset
# ---------------------------------------------------------------------------

def test_setpoint_with_positive_offset():
    hsa = _HSA('CW', 0.0, 360.0, 0.0)
    # 180 deg → 0.5 turns + 2.0 offset = 2.5 turns
    assert compute_setpoint(180.0, hsa, 2.0) == pytest.approx(2.5)


def test_setpoint_with_negative_offset():
    hsa = _HSA('CW', 0.0, 360.0, 0.0)
    assert compute_setpoint(180.0, hsa, -0.25) == pytest.approx(0.25)


# ---------------------------------------------------------------------------
# compute_zero_offset — gear_ratio=1
# ---------------------------------------------------------------------------

def test_zero_offset_cw_at_zero_extension():
    hsa = _HSA('CW', 0.0, 360.0, 0.0)
    # zero_point=0 → zero_motor_turns=0*1=0; current=2.0 → offset=2.0
    assert compute_zero_offset(2.0, hsa) == pytest.approx(2.0)


def test_zero_offset_ccw_at_zero_extension():
    hsa = _HSA('CCW', 0.0, 360.0, 0.0)
    # zero_point=0 → output=1.0 turns * 1.0 = 1.0; current=2.5 → offset=1.5
    assert compute_zero_offset(2.5, hsa) == pytest.approx(1.5)


# ---------------------------------------------------------------------------
# compute_zero_offset — gear_ratio=2
# ---------------------------------------------------------------------------

def test_zero_offset_cw_gear_ratio():
    hsa = _HSA('CW', 0.0, 360.0, 0.0, gear_ratio=2.0)
    # zero_point=0 → output=0 turns * 2.0 = 0; current=5.0 → offset=5.0
    assert compute_zero_offset(5.0, hsa) == pytest.approx(5.0)


def test_zero_offset_ccw_gear_ratio():
    hsa = _HSA('CCW', 0.0, 360.0, 0.0, gear_ratio=2.0)
    # zero_point=0 → output=1.0 turns * 2.0 = 2.0; current=3.0 → offset=1.0
    assert compute_zero_offset(3.0, hsa) == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# Round-trip: zero then command zero_point → setpoint == current pos
# ---------------------------------------------------------------------------

def test_round_trip_cw_gr1():
    hsa = _HSA('CW', 0.0, 360.0, 0.0)
    current = 3.7
    offset = compute_zero_offset(current, hsa)
    assert compute_setpoint(hsa.zero_point, hsa, offset) == pytest.approx(current)


def test_round_trip_ccw_gr1():
    hsa = _HSA('CCW', 0.0, 360.0, 0.0)
    current = 1.3
    offset = compute_zero_offset(current, hsa)
    assert compute_setpoint(hsa.zero_point, hsa, offset) == pytest.approx(current)


def test_round_trip_cw_gr2():
    hsa = _HSA('CW', 0.0, 360.0, 0.0, gear_ratio=2.0)
    current = 4.2
    offset = compute_zero_offset(current, hsa)
    assert compute_setpoint(hsa.zero_point, hsa, offset) == pytest.approx(current)


def test_round_trip_ccw_gr2():
    hsa = _HSA('CCW', 0.0, 360.0, 0.0, gear_ratio=2.0)
    current = 3.0
    offset = compute_zero_offset(current, hsa)
    assert compute_setpoint(hsa.zero_point, hsa, offset) == pytest.approx(current)
