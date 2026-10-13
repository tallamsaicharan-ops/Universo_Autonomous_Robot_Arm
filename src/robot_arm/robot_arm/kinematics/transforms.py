"""Coordinate transformation utilities for the robot arm."""

import numpy as np


def translation_matrix(x: float, y: float, z: float) -> np.ndarray:
    """Create a 4x4 homogeneous translation matrix."""
    transform = np.eye(4, dtype=float)
    transform[:3, 3] = [x, y, z]
    return transform


def rotation_matrix_x(angle: float) -> np.ndarray:
    """Create a 4x4 rotation matrix about the X-axis."""
    c, s = np.cos(angle), np.sin(angle)

    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c, -s, 0.0],
        [0.0, s,  c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def rotation_matrix_y(angle: float) -> np.ndarray:
    """Create a 4x4 rotation matrix about the Y-axis."""
    c, s = np.cos(angle), np.sin(angle)

    return np.array([
        [ c, 0.0, s, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-s, 0.0, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def rotation_matrix_z(angle: float) -> np.ndarray:
    """Create a 4x4 rotation matrix about the Z-axis."""
    c, s = np.cos(angle), np.sin(angle)

    return np.array([
        [c, -s, 0.0, 0.0],
        [s,  c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def transform_point(
    transform: np.ndarray,
    point: np.ndarray,
) -> np.ndarray:
    """Transform a 3D point using a 4x4 homogeneous matrix."""
    transform = np.asarray(transform, dtype=float)
    point = np.asarray(point, dtype=float).reshape(3)

    if transform.shape != (4, 4):
        raise ValueError("Transform must be a 4x4 matrix.")

    homogeneous_point = np.append(point, 1.0)
    return (transform @ homogeneous_point)[:3]


def compose_transforms(*transforms: np.ndarray) -> np.ndarray:
    """Compose transformations in the supplied multiplication order."""
    result = np.eye(4, dtype=float)

    for transform in transforms:
        transform = np.asarray(transform, dtype=float)

        if transform.shape != (4, 4):
            raise ValueError("Every transform must be a 4x4 matrix.")

        result = result @ transform

    return result


def inverse_transform(transform: np.ndarray) -> np.ndarray:
    """Calculate the inverse of a 4x4 transformation matrix."""
    transform = np.asarray(transform, dtype=float)

    if transform.shape != (4, 4):
        raise ValueError("Transform must be a 4x4 matrix.")

    return np.linalg.inv(transform)
