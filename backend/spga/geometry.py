from __future__ import annotations
import math
import numpy as np

EPS = 1e-12


def normalize(v: np.ndarray, fallback=None) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    if n < EPS:
        if fallback is None:
            raise ValueError("Cannot normalize near-zero vector")
        return normalize(np.asarray(fallback, dtype=float))
    return v / n


def cap_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    if n <= max_norm or n < EPS:
        return v
    return v * (max_norm / n)


def min_rotation(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Minimum SO(3) rotation mapping unit vector a to unit vector b."""
    a = normalize(a)
    b = normalize(b)
    c = float(np.clip(np.dot(a, b), -1.0, 1.0))
    v = np.cross(a, b)
    s = float(np.linalg.norm(v))
    if s < 1e-10:
        if c > 0.0:
            return np.eye(3)
        # antiparallel: rotate pi about any axis perpendicular to a
        basis = np.array([1.0, 0.0, 0.0])
        if abs(a[0]) > 0.8:
            basis = np.array([0.0, 1.0, 0.0])
        axis = normalize(np.cross(a, basis))
        K = skew(axis)
        return np.eye(3) + 2.0 * (K @ K)
    K = skew(v)
    return np.eye(3) + K + K @ K * ((1.0 - c) / (s * s))


def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(v, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def frame_from_blade_and_tool_axis(blade_x: np.ndarray, tool_y: np.ndarray) -> np.ndarray:
    """Build a right-handed frame whose x is blade direction and y is regulated tool axis."""
    x = normalize(blade_x)
    y0 = np.asarray(tool_y, dtype=float)
    y = normalize(y0 - np.dot(y0, x) * x)
    z = normalize(np.cross(x, y))
    y = normalize(np.cross(z, x))
    return np.column_stack([x, y, z])


def rotation_matrix_to_euler_xyz(R: np.ndarray) -> np.ndarray:
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-8
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0.0
    return np.array([x, y, z])