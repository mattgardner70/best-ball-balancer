"""Closed-form inverse kinematics for the 3-RRS platform.

Each leg's motor shaft sits at radius `motor_radius`, axis tangent to the
circle. The arm swings in the leg's radial plane; arm angle 0 is horizontal
pointing outward, positive lifts the elbow.
"""

import numpy as np

from .params import GEOMETRY, Geometry


def leg_directions(g: Geometry = GEOMETRY) -> np.ndarray:
    """Unit radial vector (x, y) of each leg, shape (3, 2)."""
    a = np.radians(g.leg_angles_deg)
    return np.stack([np.cos(a), np.sin(a)], axis=1)


def plate_rotation(roll: float, pitch: float) -> np.ndarray:
    """Plate orientation: roll about x, then pitch about y."""
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return ry @ rx


def arm_angles(roll: float, pitch: float, height: float | None = None,
               g: Geometry = GEOMETRY) -> np.ndarray:
    """Arm angles (rad) that put the plate at this tilt and rod-end height."""
    if height is None:
        height = g.home_plate_height
    rot = plate_rotation(roll, pitch)
    u = leg_directions(g)
    angles = np.empty(3)
    for i in range(3):
        ui = np.array([u[i, 0], u[i, 1], 0.0])
        joint = rot @ (g.plate_joint_radius * ui) + np.array([0, 0, height])
        shaft = g.motor_radius * ui + np.array([0, 0, g.motor_height])
        d = joint - shaft
        du, dz = d @ ui, d[2]
        k = (d @ d + g.arm_length**2 - g.rod_length**2) / (2 * g.arm_length)
        r = np.hypot(du, dz)
        angles[i] = np.arctan2(dz, du) - np.arccos(np.clip(k / r, -1, 1))
    return angles
