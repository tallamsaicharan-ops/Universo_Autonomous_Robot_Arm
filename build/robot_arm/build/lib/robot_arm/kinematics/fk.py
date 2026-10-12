import numpy as np

from .transforms import (
    translation_matrix,
    rotation_matrix_z,
    rotation_matrix_y,
)


def forward_kinematics(
    base_angle: float,
    shoulder_angle: float,
    elbow_angle: float,
    upper_arm_length: float,
    forearm_length: float,
) -> np.ndarray:
    """
    Calculate the end-effector position and orientation.

    Angles are in radians.
    Lengths are in metres.

    Returns:
        A 4x4 homogeneous transformation matrix.
    """

    # Rotate the arm around the vertical base axis.
    base_rotation = rotation_matrix_z(base_angle)

    # Rotate the shoulder and move along the upper arm.
    shoulder_transform = (
        rotation_matrix_y(shoulder_angle)
        @ translation_matrix(upper_arm_length, 0.0, 0.0)
    )

    # Rotate the elbow and move along the forearm.
    elbow_transform = (
        rotation_matrix_y(elbow_angle)
        @ translation_matrix(forearm_length, 0.0, 0.0)
    )

    # Combine the transformations.
    end_effector_transform = (
        base_rotation
        @ shoulder_transform
        @ elbow_transform
    )

    return end_effector_transform


def get_end_effector_position(
    transform: np.ndarray,
) -> np.ndarray:
    """Extract the XYZ position from a 4x4 transformation matrix."""
    return transform[:3, 3].copy()