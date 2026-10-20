"""Homogeneous transformation and rotation matrix utilities."""

import numpy as np


def rotation_matrix_x(angle: float) -> np.ndarray:
    """Rotation about the X-axis, in radians."""
    c = np.cos(angle)
    s = np.sin(angle)

    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s, c],
    ])


def rotation_matrix_y(angle: float) -> np.ndarray:
    """Rotation about the Y-axis, in radians."""
    c = np.cos(angle)
    s = np.sin(angle)

    return np.array([
        [c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c],
    ])


def rotation_matrix_z(angle: float) -> np.ndarray:
    """Rotation about the Z-axis, in radians."""
    c = np.cos(angle)
    s = np.sin(angle)

    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ])


def translation_matrix(x: float, y: float, z: float) -> np.ndarray:
    """Create a 4x4 homogeneous translation matrix."""
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


def homogeneous_transform(
    rotation: np.ndarray,
    translation: np.ndarray,
) -> np.ndarray:
    """Combine a 3x3 rotation matrix and a 3D translation."""
    rotation = np.asarray(rotation, dtype=float)
    translation = np.asarray(translation, dtype=float)

    if rotation.shape != (3, 3):
        raise ValueError("Rotation must be a 3x3 matrix.")

    if translation.shape != (3,):
        raise ValueError("Translation must be a 3-element vector.")

    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = translation

    return T


def dh_transform(
    theta: float,
    d: float,
    a: float,
    alpha: float,
) -> np.ndarray:
    """
    Standard Denavit-Hartenberg homogeneous transformation.

    All angles are in radians.
    All lengths use the same unit.
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def inverse_transform(T: np.ndarray) -> np.ndarray:
    """Compute the inverse of a rigid homogeneous transform."""
    T = np.asarray(T, dtype=float)

    if T.shape != (4, 4):
        raise ValueError("Transformation must be a 4x4 matrix.")

    R = T[:3, :3]
    p = T[:3, 3]

    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ p

    return T_inv
