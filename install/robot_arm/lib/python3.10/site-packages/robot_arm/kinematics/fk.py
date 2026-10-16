"""Forward kinematics for a 3-DOF robot arm."""

import numpy as np

from .transforms import (
    rotation_matrix_z,
    rotation_matrix_y,
    translation_matrix,
)


def forward_kinematics(
    base_angle: float,
    shoulder_angle: float,
    elbow_angle: float,
    upper_arm_length: float,
    forearm_length: float,
) -> np.ndarray:
    """
    Calculate the end-effector homogeneous transformation matrix.

    Angles are in radians.
    Lengths are in metres.

    Assumptions:
        - Base rotates about the Z-axis.
        - Shoulder and elbow rotate about their local Y-axes.
        - Both links extend along their local X-axes.
        - The base origin is at (0, 0, 0).

    Returns:
        A 4x4 homogeneous transformation matrix.
    """
    if upper_arm_length <= 0 or forearm_length <= 0:
        raise ValueError("Link lengths must be positive.")

    base_transform = rotation_matrix_z(base_angle)

    shoulder_transform = (
        rotation_matrix_y(shoulder_angle)
        @ translation_matrix(upper_arm_length, 0.0, 0.0)
    )

    elbow_transform = (
        rotation_matrix_y(elbow_angle)
        @ translation_matrix(forearm_length, 0.0, 0.0)
    )

    return base_transform @ shoulder_transform @ elbow_transform


def get_end_effector_position(transform: np.ndarray) -> np.ndarray:
    """Extract the XYZ position from a homogeneous transformation."""
    transform = np.asarray(transform, dtype=float)

    if transform.shape != (4, 4):
        raise ValueError("Transform must be a 4x4 matrix.")

    return transform[:3, 3].copy()

