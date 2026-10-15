"""
Unit tests for wrist_control.conversion.

These tests exercise the pure mathematical functions and do not require
a running ROS2 environment or CAN bus.
"""

import pytest
from hsa_robots import HSA

from wrist_control.conversion import (
    JOG_MIN_DEG,
    SAFETY_MIN_TURNS,
    SAFETY_MAX_TURNS,
    DEGREES_PER_TURN,
    clamp,
    degrees_to_turns,
    compute_extension_angle,
    compute_jog_setpoint,
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

# The offset is added to the raw ODrive position to measure from the physical
# zero, so it is subtracted when going the other way, to a raw setpoint.

def test_setpoint_with_positive_offset():
    hsa = _HSA('CW', 0.0, 360.0, 0.0)
    # 180 deg → 0.5 turns - 2.0 offset = -1.5 raw turns
    assert compute_setpoint(180.0, hsa, 2.0) == pytest.approx(-1.5)


def test_setpoint_with_negative_offset():
    hsa = _HSA('CW', 0.0, 360.0, 0.0)
    assert compute_setpoint(180.0, hsa, -0.25) == pytest.approx(0.75)


# ---------------------------------------------------------------------------
# compute_zero_offset — gear_ratio=1
# ---------------------------------------------------------------------------

def test_zero_offset_cw_at_zero_extension():
    hsa = _HSA('CW', 0.0, 360.0, 0.0)
    # zero_point=0 → zero_motor_turns=0*1=0; current=2.0 → offset=-2.0
    assert compute_zero_offset(2.0, hsa) == pytest.approx(-2.0)


def test_zero_offset_ccw_at_zero_extension():
    hsa = _HSA('CCW', 0.0, 360.0, 0.0)
    # zero_point=0 → output=1.0 turns * 1.0 = 1.0; current=2.5 → offset=-1.5
    assert compute_zero_offset(2.5, hsa) == pytest.approx(-1.5)


# ---------------------------------------------------------------------------
# compute_zero_offset — gear_ratio=2
# ---------------------------------------------------------------------------

def test_zero_offset_cw_gear_ratio():
    hsa = _HSA('CW', 0.0, 360.0, 0.0, gear_ratio=2.0)
    # zero_point=0 → output=0 turns * 2.0 = 0; current=5.0 → offset=-5.0
    assert compute_zero_offset(5.0, hsa) == pytest.approx(-5.0)


def test_zero_offset_ccw_gear_ratio():
    hsa = _HSA('CCW', 0.0, 360.0, 0.0, gear_ratio=2.0)
    # zero_point=0 → output=1.0 turns * 2.0 = 2.0; current=3.0 → offset=-1.0
    assert compute_zero_offset(3.0, hsa) == pytest.approx(-1.0)


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


# ---------------------------------------------------------------------------
# hsa position = odrive position + zero offset (then hsa_robots → degrees)
# ---------------------------------------------------------------------------

HANDS_AND_RATIOS = [('CW', 1.0), ('CCW', 1.0), ('CW', 2.0), ('CCW', 2.0)]


@pytest.mark.parametrize('hand,gr', HANDS_AND_RATIOS)
def test_zeroed_position_reads_zero_point(hand, gr):
    hsa = _HSA(hand, 0.0, 360.0, 0.0, gear_ratio=gr)
    current = 0.8137
    offset = compute_zero_offset(current, hsa)
    assert compute_extension_angle(current, hsa, offset) == pytest.approx(0.0)


@pytest.mark.parametrize('hand,gr', HANDS_AND_RATIOS)
@pytest.mark.parametrize('angle', [0.0, 45.0, 180.0, 359.0, 360.0])
def test_extension_angle_inverts_setpoint(hand, gr, angle):
    hsa = _HSA(hand, 0.0, 360.0, 0.0, gear_ratio=gr)
    offset = compute_zero_offset(-1.234, hsa)
    sp = compute_setpoint(angle, hsa, offset)
    assert compute_extension_angle(sp, hsa, offset) == pytest.approx(angle)


def test_slider_range_is_physical_after_zeroing():
    """After zeroing at an arbitrary encoder position, commanding 0..360 moves
    the motor 0..2 motor turns (gear ratio 2) from that position."""
    hsa = _HSA('CW', 0.0, 360.0, 0.0, gear_ratio=2.0)
    zero_pos = -0.37
    offset = compute_zero_offset(zero_pos, hsa)
    assert compute_setpoint(0.0, hsa, offset) == pytest.approx(zero_pos)
    assert compute_setpoint(360.0, hsa, offset) == pytest.approx(zero_pos + 2.0)


def test_extension_angle_reports_below_zero_unclamped():
    hsa = _HSA('CW', 0.0, 360.0, 0.0, gear_ratio=2.0)
    # 0.1 motor turns below zero = 0.05 output turns = 18 degrees
    assert compute_extension_angle(-0.1, hsa, 0.0) == pytest.approx(-18.0)


# ---------------------------------------------------------------------------
# compute_jog_setpoint — may go below min_angle, sliders may not
# ---------------------------------------------------------------------------

def test_slider_setpoint_clamps_below_min():
    hsa = _HSA('CW', 0.0, 360.0, 0.0, gear_ratio=2.0)
    assert compute_setpoint(-10.0, hsa, 0.0) == pytest.approx(0.0)


def test_jog_goes_below_min_cw():
    hsa = _HSA('CW', 0.0, 360.0, 0.0, gear_ratio=2.0)
    # -10° extension → -10° output → -10/360*2 motor turns
    assert compute_jog_setpoint(-10.0, hsa, 0.0) == pytest.approx(-20.0 / 360.0)


def test_jog_goes_below_min_ccw():
    hsa = _HSA('CCW', 0.0, 360.0, 0.0, gear_ratio=2.0)
    # CCW: extension -10° → output 370° → 370/360*2 motor turns
    assert compute_jog_setpoint(-10.0, hsa, 0.0) == pytest.approx(740.0 / 360.0)


@pytest.mark.parametrize('hand,gr', HANDS_AND_RATIOS)
@pytest.mark.parametrize('angle', [0.0, 90.0, 360.0])
def test_jog_matches_slider_inside_range(hand, gr, angle):
    hsa = _HSA(hand, 0.0, 360.0, 0.0, gear_ratio=gr)
    assert compute_jog_setpoint(angle, hsa, 0.3) == pytest.approx(
        compute_setpoint(angle, hsa, 0.3)
    )


@pytest.mark.parametrize('hand,gr', HANDS_AND_RATIOS)
def test_jog_round_trips_below_min(hand, gr):
    hsa = _HSA(hand, 0.0, 360.0, 0.0, gear_ratio=gr)
    sp = compute_jog_setpoint(-42.0, hsa, 0.5)
    assert compute_extension_angle(sp, hsa, 0.5) == pytest.approx(-42.0)


def test_jog_lower_limit():
    hsa = _HSA('CW', 0.0, 360.0, 0.0, gear_ratio=2.0)
    assert compute_jog_setpoint(-1000.0, hsa, 0.0) == pytest.approx(
        degrees_to_turns(JOG_MIN_DEG) * 2.0
    )


def test_jog_upper_limit_is_max_angle():
    hsa = _HSA('CW', 0.0, 360.0, 0.0, gear_ratio=2.0)
    assert compute_jog_setpoint(400.0, hsa, 0.0) == pytest.approx(2.0)


# ---------------------------------------------------------------------------
# Same conversions through the real hsa_robots HSA class
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('hand', [HSA.CW, HSA.CCW])
def test_real_hsa_round_trip(hand):
    hsa = HSA(handedness=hand, motor='odrive', min_angle_extension=0.0,
              max_angle_extension=360.0, zero_point=0.0, gear_ratio=2.0)
    offset = compute_zero_offset(1.5, hsa)
    for angle in (0.0, 123.0, 360.0):
        sp = compute_setpoint(angle, hsa, offset)
        assert compute_extension_angle(sp, hsa, offset) == pytest.approx(angle)
    sp = compute_jog_setpoint(-6.0, hsa, offset)
    assert compute_extension_angle(sp, hsa, offset) == pytest.approx(-6.0)
