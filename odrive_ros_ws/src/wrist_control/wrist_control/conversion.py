"""
Pure functions for HSA–ODrive angle conversion.

ODrive native position unit is turns (1 turn = 360 degrees).
All setpoints sent to the driver are in turns.

Zero offsets are in motor-shaft turns and are *added* to the raw ODrive
position: odrive_position + zero_offset = motor turns measured from the
physical zero, which hsa_robots' HSA.move() then maps to extension degrees.
"""

DEGREES_PER_TURN: float = 360.0

# Absolute motor travel limits regardless of robot config, measured from the
# physical zero. -180 deg = -0.5 turns, +540 deg = +1.5 turns.
SAFETY_MIN_TURNS: float = -0.5
SAFETY_MAX_TURNS: float = 1.5

# Lowest extension angle a jog may command. Jogging is the only way to go
# below an HSA's min_angle, so the physical zero can be found below the
# current zero.
JOG_MIN_DEG: float = SAFETY_MIN_TURNS * DEGREES_PER_TURN


def degrees_to_turns(degrees: float) -> float:
    return degrees / DEGREES_PER_TURN


def turns_to_degrees(turns: float) -> float:
    return turns * DEGREES_PER_TURN


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _move_line(hsa) -> tuple[float, float]:
    """(motor angle at min_angle, slope) of hsa.move() over [min_angle, max_angle].

    HSA.move() is a straight line there (slope +1 for CW, -1 for CCW); this lets
    angles outside the range, and the inverse, follow the same mapping.
    """
    lo, hi = hsa.min_angle, hsa.max_angle
    m_lo = hsa.move(lo)
    slope = (hsa.move(hi) - m_lo) / (hi - lo) if hi != lo else 1.0
    return m_lo, slope


def extension_to_motor_deg(extension_deg: float, hsa) -> float:
    """hsa.move() without its clamp: extension degrees → output-shaft degrees."""
    m_lo, slope = _move_line(hsa)
    return m_lo + slope * (extension_deg - hsa.min_angle)


def motor_deg_to_extension(motor_deg: float, hsa) -> float:
    """Inverse of hsa.move(): output-shaft degrees → extension degrees."""
    m_lo, slope = _move_line(hsa)
    return hsa.min_angle + (motor_deg - m_lo) / slope


def _output_turns_to_setpoint(output_turns: float, hsa, zero_offset_turns: float) -> float:
    # Backstop safety clamp on output-shaft turns (catches wide robot configs)
    output_turns = clamp(output_turns, SAFETY_MIN_TURNS, SAFETY_MAX_TURNS)
    # Scale to motor-shaft turns via gear ratio, then back into raw ODrive turns
    return output_turns * hsa.gear_ratio - zero_offset_turns


def compute_setpoint(extension_angle_deg: float, hsa, zero_offset_turns: float) -> float:
    """
    Convert a commanded extension angle to an ODrive position setpoint in turns.

    Args:
        extension_angle_deg: Commanded extension angle in degrees.
        hsa: HSA instance (provides handedness, min/max/zero_point, gear_ratio, move()).
        zero_offset_turns: Offset (in motor-shaft turns) added to the raw ODrive
            position to measure from the physical zero.

    Returns:
        Absolute ODrive position setpoint in motor-shaft turns.
    """
    # 1. Hard safety clamp on input angle (output-shaft degrees)
    angle = clamp(extension_angle_deg, -180.0, 540.0)
    # 2. Robot-config range clamp
    angle = clamp(angle, hsa.min_angle, hsa.max_angle)
    # 3. HSA kinematics: applies handedness and re-clamps to config range
    output_deg = hsa.move(angle)
    # 4. Convert to output-shaft turns, then to a raw ODrive setpoint
    return _output_turns_to_setpoint(degrees_to_turns(output_deg), hsa, zero_offset_turns)


def compute_jog_setpoint(extension_angle_deg: float, hsa, zero_offset_turns: float) -> float:
    """
    Like compute_setpoint, but the extension angle may go below hsa.min_angle,
    down to JOG_MIN_DEG. Used only by the jog buttons, to drive a motor back
    past the current zero while looking for the physical zero.
    """
    angle = clamp(extension_angle_deg, JOG_MIN_DEG, hsa.max_angle)
    output_deg = extension_to_motor_deg(angle, hsa)
    return _output_turns_to_setpoint(degrees_to_turns(output_deg), hsa, zero_offset_turns)


def compute_extension_angle(pos_estimate_turns: float, hsa, zero_offset_turns: float) -> float:
    """
    Convert a raw ODrive position estimate (motor-shaft turns) back into the
    extension angle (degrees) it corresponds to — the inverse of
    compute_setpoint's handedness/gear-ratio/zero-offset mapping.

    This is a readout conversion (no clamping): an actual position outside
    the configured range is reported as-is rather than hidden.
    """
    motor_turns = pos_estimate_turns + zero_offset_turns
    output_turns = motor_turns / hsa.gear_ratio if hsa.gear_ratio else motor_turns
    return motor_deg_to_extension(turns_to_degrees(output_turns), hsa)


def compute_zero_offset(current_pos_turns: float, hsa) -> float:
    """
    Compute the offset (motor-shaft turns) so that *current_pos_turns* corresponds
    to hsa.zero_point.

    current_pos_turns is the ODrive pos_estimate, which is in motor-shaft turns.
    Calling this at the physical home position means all subsequent setpoints
    are relative to that home.
    """
    zero_output_turns = degrees_to_turns(hsa.move(hsa.zero_point))
    zero_motor_turns = zero_output_turns * hsa.gear_ratio
    return zero_motor_turns - current_pos_turns
