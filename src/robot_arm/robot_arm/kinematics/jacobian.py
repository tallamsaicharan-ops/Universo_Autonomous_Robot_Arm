"""Position Jacobian for the 3-DOF robotic arm."""

import numpy as np

from .fk import (
    BASE_FORWARD_OFFSET_M,
    UPPER_ARM_LENGTH_M,
    FOREARM_LENGTH_M,
    upper_arm_elevation,
    forearm_elevation,
    validate_joint_angles,
)


def position_jacobian(
    q_base: float,
    q_shoulder: float,
    q_elbow: float,
    check_limits: bool = True,
) -> np.ndarray:
    """
    Calculate the 3x3 translational Jacobian.

    Parameters
    ----------
    q_base : float
        Base rotation angle in radians.
    q_shoulder : float
        Shoulder joint angle in radians.
    q_elbow : float
        Elbow joint angle in radians.
    check_limits : bool
        If True, validate the shoulder and elbow joint limits.

    Returns
    -------
    numpy.ndarray
        Position Jacobian in metres per radian.

        Rows:    [x, y, z]
        Columns: [base, shoulder, elbow]
    """

    angles = np.array(
        [q_base, q_shoulder, q_elbow],
        dtype=float,
    )

    if not np.all(np.isfinite(angles)):
        raise ValueError("Joint angles must be finite.")

    if check_limits:
        validate_joint_angles(
            q_base,
            q_shoulder,
            q_elbow,
        )

    alpha = upper_arm_elevation(q_shoulder)
    beta = forearm_elevation(q_shoulder, q_elbow)

    # Full radial distance, including the base forward offset.
    radial = (
        BASE_FORWARD_OFFSET_M
        + UPPER_ARM_LENGTH_M * np.cos(alpha)
        + FOREARM_LENGTH_M * np.cos(beta)
    )

    # Radial derivative with respect to shoulder angle.
    dr_dshoulder = (
        -UPPER_ARM_LENGTH_M * np.sin(alpha)
        - FOREARM_LENGTH_M * np.sin(beta)
    )

    # Radial derivative with respect to elbow angle.
    dr_delbow = FOREARM_LENGTH_M * np.sin(beta)

    # Vertical derivatives.
    dz_dshoulder = (
        UPPER_ARM_LENGTH_M * np.cos(alpha)
        + FOREARM_LENGTH_M * np.cos(beta)
    )

    dz_delbow = -FOREARM_LENGTH_M * np.cos(beta)

    c = np.cos(q_base)
    s = np.sin(q_base)

    # Rows: x, y, z
    # Columns: base, shoulder, elbow
    J = np.array(
        [
            [
                -radial * s,
                dr_dshoulder * c,
                dr_delbow * c,
            ],
            [
                radial * c,
                dr_dshoulder * s,
                dr_delbow * s,
            ],
            [
                0.0,
                dz_dshoulder,
                dz_delbow,
            ],
        ],
        dtype=float,
    )

    return J


if __name__ == "__main__":
    # Example configuration.
    angles_deg = [30.0, -20.0, 15.0]

    q_base, q_shoulder, q_elbow = np.deg2rad(
        angles_deg
    )

    J = position_jacobian(
        q_base,
        q_shoulder,
        q_elbow,
    )

    print("3-DOF ROBOTIC ARM - POSITION JACOBIAN")
    print("=" * 45)
    print("Joint angles (degrees):", angles_deg)
    print("\nJacobian (m/rad):")
    print(np.round(J, 6))
    print("\nRows: x, y, z")
    print("Columns: base, shoulder, elbow")
