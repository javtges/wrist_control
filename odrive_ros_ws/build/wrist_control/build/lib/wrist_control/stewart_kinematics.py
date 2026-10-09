"""
Pure Stewart-platform inverse kinematics for the HSA wrist.

Ported from resource/rigid_dynamics.py (credit: https://github.com/Yeok-c/Stewart_Py,
MIT licence). Geometry, rotation order, spacemouse scaling/signs and the
leg-length → angle conversion are unchanged. The one change is the leg
numbering: rigid_dynamics.py numbers its legs counter-clockwise viewed from
above, while the wrist's ODrives 0–5 run clockwise viewed from above. With
`clockwise=True`, motor k drives rigid_dynamics.py leg (-k) % 6: motor 0 is
rigid_dynamics.py's leg 0 and the rest follow clockwise from it. This puts
motors (0,1), (2,3), (4,5) apart on the base and meeting at the top of the
platform, as they are on the wrist. The geometry itself is unchanged.
"""

import numpy as np

# These are in cm
PLATFORM_RADIUS_TOP = 7.874
PLATFORM_RADIUS_BOTTOM = 11.28616996
TOP_TO_BOTTOM = 6.1341
STATIC_LENGTH = 6.814

# These are in radians
GAMMA_B = 0.3639382215786  # half angle between adjacent legs on the base
GAMMA_P = 0.3818744117959  # half angle between adjacent legs on the platform
MAX_ROT = np.pi / 5

# This is in cm
MAX_LIN = 10.0

# Leg length change (cm) per radian of joint rotation
CM_PER_RAD = 1.23

N_LEGS = 6


def rot_x(theta: float) -> np.ndarray:
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]])


def rot_y(theta: float) -> np.ndarray:
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]])


def rot_z(theta: float) -> np.ndarray:
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]])


def anchor_angles(clockwise: bool = True, leg_offset: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """
    Polar angles (rad) of the base (psi_B) and platform (psi_P) anchors, one
    per leg, indexed by motor number.

    Args:
        clockwise: number legs clockwise viewed from above (ODrive wiring),
            starting from rigid_dynamics.py's leg 0. False reproduces
            rigid_dynamics.py's counter-clockwise order.
        leg_offset: rotate the numbering so motor k drives what would
            otherwise be motor (k + leg_offset) % 6's leg. Use it if
            motor 0 is not the leg rigid_dynamics.py calls leg 0.
    """
    pi = np.pi
    gamma_B, gamma_P = GAMMA_B, GAMMA_P

    psi_B = np.array([
        -gamma_B,
        gamma_B,
        2*pi/3 - gamma_B,
        2*pi/3 + gamma_B,
        2*pi/3 + 2*pi/3 - gamma_B,
        2*pi/3 + 2*pi/3 + gamma_B])

    psi_P = np.array([
        pi/3 + 2*pi/3 + 2*pi/3 + gamma_P,
        pi/3 + -gamma_P,
        pi/3 + gamma_P,
        pi/3 + 2*pi/3 - gamma_P,
        pi/3 + 2*pi/3 + gamma_P,
        pi/3 + 2*pi/3 + 2*pi/3 - gamma_P])

    motors = (np.arange(N_LEGS) + leg_offset) % N_LEGS
    order = (-motors) % N_LEGS if clockwise else motors
    return psi_B[order], psi_P[order]


def spacemouse_to_pose(
    x: float, y: float, z: float, roll: float, pitch: float, yaw: float,
    max_rot: float = MAX_ROT, max_lin: float = MAX_LIN,
) -> tuple[float, float, float, float]:
    """
    Scale spacemouse axes ([-1, 1]) to the platform command used by
    rigid_dynamics.py: (roll, pitch, yaw) in rad and z translation in cm.

    rigid_dynamics.py only applies z translation; x and y are accepted for
    parity but unused. The original spacemouse_node → rigid_dynamics path
    passed roll/pitch/yaw through euler2quat → quat2euler, which returns the
    same angles for these input ranges, so they are used directly here.
    """
    del x, y
    return (
        roll * max_rot * -1,
        pitch * max_rot * -1,
        yaw * max_rot,
        z * max_lin,
    )


def leg_lengths(
    roll: float, pitch: float, yaw: float, z_trans: float,
    clockwise: bool = True, leg_offset: int = 0,
) -> np.ndarray:
    """Leg lengths (cm), indexed by motor number, for a platform pose."""
    psi_B, psi_P = anchor_angles(clockwise, leg_offset)

    # Coordinates of the points where the legs attach to the base
    B = PLATFORM_RADIUS_BOTTOM * np.array([np.cos(psi_B), np.sin(psi_B), np.zeros(N_LEGS)])
    # Coordinates of the points where the legs attach to the platform
    P = PLATFORM_RADIUS_TOP * np.array([np.cos(psi_P), np.sin(psi_P), np.zeros(N_LEGS)])

    home_pos = np.array([0, 0, TOP_TO_BOTTOM])
    trans = np.array([0, 0, z_trans])

    R = rot_x(roll) @ rot_y(pitch) @ rot_z(yaw)

    l = trans[:, np.newaxis] + home_pos[:, np.newaxis] + R @ P - B
    return np.linalg.norm(l, axis=0)


def leg_lengths_to_joint_angles(lengths: np.ndarray) -> np.ndarray:
    """
    Convert leg lengths (cm) to joint angles (deg) exactly as rigid_dynamics.py
    does, including shifting everything up so the smallest angle is zero
    whenever any angle would otherwise be negative.
    """
    length_difference = np.asarray(lengths) - STATIC_LENGTH
    motor_angles_radians = length_difference / CM_PER_RAD
    motor_angles_degrees = np.around(np.degrees(motor_angles_radians), decimals=2)
    if min(motor_angles_degrees) < 0:
        motor_angles_degrees = motor_angles_degrees - min(motor_angles_degrees)
    return motor_angles_degrees


def clamp_to_limits(angles: np.ndarray, low: np.ndarray, high: np.ndarray) -> np.ndarray:
    """Clamp each joint angle to its [low, high] range."""
    return np.clip(np.asarray(angles, dtype=float), low, high)


def spacemouse_to_joint_angles(
    x: float, y: float, z: float, roll: float, pitch: float, yaw: float,
    max_rot: float = MAX_ROT, max_lin: float = MAX_LIN,
    clockwise: bool = True, leg_offset: int = 0,
) -> np.ndarray:
    """Spacemouse axes ([-1, 1]) → unclamped joint angles (deg), by motor number."""
    r, p, yw, zt = spacemouse_to_pose(x, y, z, roll, pitch, yaw, max_rot, max_lin)
    return leg_lengths_to_joint_angles(leg_lengths(r, p, yw, zt, clockwise, leg_offset))
