import numpy as np


def calculate_jacobian(
    base_angle: float,
    shoulder_angle: float,
    elbow_angle: float,
    upper_arm_length: float,
    forearm_length: float,
) -> np.ndarray:
    """
    Calculate the positional Jacobian for a 3-DOF arm.

    Angles are in radians.
    Lengths are in metres.

    Returns:
        A 3x3 Jacobian mapping joint velocities to
        end-effector linear velocity.

    Assumptions:
        - Base rotates around the vertical Z-axis.
        - Shoulder and elbow rotate around local Y-axes.
        - Shoulder is located at the origin.
        - Geometry matches the preliminary FK model.
    """

    q1 = base_angle
    q2 = shoulder_angle
    q3 = elbow_angle

    l1 = upper_arm_length
    l2 = forearm_length

    # Radial distance from the base axis.
    radial = (
        l1 * np.cos(q2)
        + l2 * np.cos(q2 + q3)
    )

    # Height of the end effector.
    height = (
        l1 * np.sin(q2)
        + l2 * np.sin(q2 + q3)
    )

    # Derivatives with respect to the shoulder angle.
    dr_dq2 = (
        -l1 * np.sin(q2)
        -l2 * np.sin(q2 + q3)
    )

    dz_dq2 = (
        l1 * np.cos(q2)
        + l2 * np.cos(q2 + q3)
    )

    # Derivatives with respect to the elbow angle.
    dr_dq3 = -l2 * np.sin(q2 + q3)
    dz_dq3 = l2 * np.cos(q2 + q3)

    # Construct the positional Jacobian.
    jacobian = np.array([
        [
            -radial * np.sin(q1),
            dr_dq2 * np.cos(q1),
            dr_dq3 * np.cos(q1),
        ],
        [
            radial * np.cos(q1),
            dr_dq2 * np.sin(q1),
            dr_dq3 * np.sin(q1),
        ],
        [
            0.0,
            dz_dq2,
            dz_dq3,
        ],
    ])

    return jacobian