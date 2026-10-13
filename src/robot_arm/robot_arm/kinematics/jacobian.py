"""Calculate the position Jacobian for a simplified 3-DOF arm."""

import math
import numpy as np


def calculate_jacobian(
    base_angle: float,
    shoulder_angle: float,
    elbow_angle: float,
    upper_arm_length: float,
    forearm_length: float,
) -> np.ndarray:
    """Calculate the 3x3 translational Jacobian.

    Maps joint velocities [q1_dot, q2_dot, q3_dot] to
    end-effector velocity [x_dot, y_dot, z_dot].

    Angles are in radians; lengths are in metres.
    """
    if upper_arm_length <= 0 or forearm_length <= 0:
        raise ValueError("Link lengths must be positive.")

    q1 = base_angle
    q2 = shoulder_angle
    q3 = elbow_angle

    l1 = upper_arm_length
    l2 = forearm_length

    radial = l1 * math.cos(q2) + l2 * math.cos(q2 + q3)

    dr_dq2 = (
        -l1 * math.sin(q2)
        -l2 * math.sin(q2 + q3)
    )

    dr_dq3 = -l2 * math.sin(q2 + q3)

    # The FK model uses rotation_matrix_y(), so positive shoulder
    # and elbow angles produce negative Z for links along local +X.
    dz_dq2 = -(
        l1 * math.cos(q2)
        +l2 * math.cos(q2 + q3)
    )

    dz_dq3 = -l2 * math.cos(q2 + q3)

    return np.array([
        [
            -radial * math.sin(q1),
            dr_dq2 * math.cos(q1),
            dr_dq3 * math.cos(q1),
        ],
        [
            radial * math.cos(q1),
            dr_dq2 * math.sin(q1),
            dr_dq3 * math.sin(q1),
        ],
        [
            0.0,
            dz_dq2,
            dz_dq3,
        ],
    ], dtype=float)
