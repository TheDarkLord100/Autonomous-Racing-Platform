"""SE(2) helpers shared by the filter modules. No ROS imports."""

import numpy as np


def wrap_angle(a):
    """Wrap angle(s) to [-pi, pi)."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def yaw_to_quat(yaw):
    """Yaw (rad) -> quaternion (x, y, z, w) for a rotation about z."""
    half = 0.5 * yaw
    return 0.0, 0.0, float(np.sin(half)), float(np.cos(half))


def quat_to_yaw(x, y, z, w):
    """Quaternion -> yaw (rad). Assumes rotation is (mostly) about z."""
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def relative_pose(p0, p1):
    """Motion from pose p0 to pose p1, expressed in p0's local frame.

    p0, p1: (x, y, theta). Returns (dx, dy, dtheta) with dx forward, dy left.
    """
    c, s = np.cos(p0[2]), np.sin(p0[2])
    ex, ey = p1[0] - p0[0], p1[1] - p0[1]
    return np.array([c * ex + s * ey, -s * ex + c * ey, wrap_angle(p1[2] - p0[2])])


def compose(a, b):
    """SE(2) composition a (+) b: apply local motion b on top of pose a.

    a: (3,) or (N, 3). b: (3,) or (N, 3). Broadcasts.
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    c, s = np.cos(a[..., 2]), np.sin(a[..., 2])
    x = a[..., 0] + c * b[..., 0] - s * b[..., 1]
    y = a[..., 1] + s * b[..., 0] + c * b[..., 1]
    th = wrap_angle(a[..., 2] + b[..., 2])
    return np.stack([x, y, th], axis=-1)


def interpolate(p0, p1, alpha):
    """Pose a fraction alpha (0..1) of the way from p0 to p1 (shortest-angle for theta)."""
    dth = wrap_angle(p1[2] - p0[2])
    return np.array([p0[0] + alpha * (p1[0] - p0[0]),
                     p0[1] + alpha * (p1[1] - p0[1]),
                     wrap_angle(p0[2] + alpha * dth)])


def inverse(a):
    """SE(2) inverse of a single pose a = (x, y, theta)."""
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap_angle(-a[2])])