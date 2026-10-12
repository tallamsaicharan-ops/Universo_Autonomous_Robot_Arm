import numpy as np


def translation_matrix(x: float, y: float, z: float) -> np.ndarray:
    """Create a 4x4 homogeneous translation matrix."""
    matrix = np.eye(4)
    matrix[:3, 3] = [x, y, z]
    return matrix


def rotation_matrix_x(angle: float) -> np.ndarray:
    """Create a 4x4 rotation matrix around the X-axis."""
    c, s = np.cos(angle), np.sin(angle)

    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1],
    ], dtype=float)


def rotation_matrix_y(angle: float) -> np.ndarray:
    """Create a 4x4 rotation matrix around the Y-axis."""
    c, s = np.cos(angle), np.sin(angle)

    return np.array([
        [c, 0, s, 0],
        [0, 1, 0, 0],
        [-s, 0, c, 0],
        [0, 0, 0, 1],
    ], dtype=float)


def rotation_matrix_z(angle: float) -> np.ndarray:
    """Create a 4x4 rotation matrix around the Z-axis."""
    c, s = np.cos(angle), np.sin(angle)

    return np.array([
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ], dtype=float)


def transform_point(
    transform: np.ndarray,
    point: np.ndarray,
) -> np.ndarray:
    """Transform a 3D point using a 4x4 homogeneous matrix."""
    point = np.asarray(point, dtype=float).reshape(3)

    homogeneous_point = np.append(point, 1.0)
    transformed = transform @ homogeneous_point

    return transformed[:3]