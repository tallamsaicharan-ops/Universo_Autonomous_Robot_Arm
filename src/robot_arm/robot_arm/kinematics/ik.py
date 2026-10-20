"""Closed-form inverse kinematics for the 3-DOF robotic arm."""

import numpy as np

from .fk import (
    BASE_FORWARD_OFFSET_M,
    SHOULDER_HEIGHT_M,
    UPPER_ARM_LENGTH_M,
    FOREARM_LENGTH_M,
    FOREARM_ZERO_ELEVATION_RAD,
    SHOULDER_LIMITS,
    ELBOW_LIMITS,
    fk_position,
    within_joint_limits,
)


POSITION_TOLERANCE_M = 1e-4


def inverse_kinematics(
    target_position,
    tolerance_m=POSITION_TOLERANCE_M,
    check_limits=True,
):
    """
    Calculate joint angles for a target XYZ position.

    Parameters
    ----------
    target_position : array-like
        Target [x, y, z] in metres.
    tolerance_m : float
        Maximum accepted position error in metres.
    check_limits : bool
        If True, enforce the Excel shoulder and elbow limits.

    Returns
    -------
    dict
        Contains success status, joint angles, calculated position,
        and position error.

    Notes
    -----
    Joint angles are returned in radians and degrees.
    The base joint is continuous.
    """

    target = np.asarray(target_position, dtype=float)

    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError(
            "target_position must contain three finite values: [x, y, z]."
        )

    if not np.isfinite(tolerance_m) or tolerance_m <= 0:
        raise ValueError("tolerance_m must be positive and finite.")

    x, y, z = target

    # Base rotation.
    q_base = float(np.arctan2(y, x))

    # Convert the Cartesian target into the arm's radial plane.
    radial = float(np.hypot(x, y) - BASE_FORWARD_OFFSET_M)
    vertical = float(z - SHOULDER_HEIGHT_M)

    # The shoulder axis is at the origin of the planar arm geometry.
    # The forearm's absolute elevation is:
    # beta = FOREARM_ZERO_ELEVATION_RAD + q_shoulder - q_elbow
    #
    # The upper-arm elevation is:
    # alpha = pi/2 + q_shoulder
    #
    # Define:
    # theta1 = q_shoulder
    # theta2 = q_shoulder - q_elbow
    #
    # The planar target relative to the shoulder is:
    # radial = L1*cos(pi/2 + theta1) + L2*cos(35deg + theta2)
    # vertical = L1*sin(pi/2 + theta1) + L2*sin(35deg + theta2)

    # Transform into conventional planar-link coordinates.
    # Let u = pi/2 + q_shoulder and v = 35deg + q_shoulder - q_elbow.
    # Then:
    # radial = L1*cos(u) + L2*cos(v)
    # vertical = L1*sin(u) + L2*sin(v)

    L1 = UPPER_ARM_LENGTH_M
    L2 = FOREARM_LENGTH_M

    distance_squared = radial**2 + vertical**2

    cos_difference = (
        distance_squared - L1**2 - L2**2
    ) / (2.0 * L1 * L2)

    # Geometric reachability check.
    if cos_difference < -1.0 - 1e-9 or cos_difference > 1.0 + 1e-9:
        return {
            "success": False,
            "message": "Target is outside the arm's geometric workspace.",
            "target_position_m": target.copy(),
        }

    cos_difference = float(np.clip(cos_difference, -1.0, 1.0))

    # Both elbow branches are considered.
    candidates = []

    for sign in (1.0, -1.0):
        # Difference between upper-arm and forearm elevations.
        delta = float(
            np.arctan2(
                sign * np.sqrt(max(0.0, 1.0 - cos_difference**2)),
                cos_difference,
            )
        )

        # Direction of the shoulder-to-target vector.
        target_angle = float(np.arctan2(vertical, radial))

        # Absolute upper-arm elevation.
        upper_elevation = target_angle - np.arctan2(
            L2 * np.sin(delta),
            L1 + L2 * np.cos(delta),
        )

        q_shoulder = float(upper_elevation - np.pi / 2.0)

        # From delta = upper_elevation - forearm_elevation:
        # forearm_elevation = 35deg + q_shoulder - q_elbow
        # Therefore q_elbow = 35deg + q_shoulder - forearm_elevation.
        forearm_elevation = float(upper_elevation - delta)

        q_elbow = float(
            FOREARM_ZERO_ELEVATION_RAD
            + q_shoulder
            - forearm_elevation
        )

        candidates.append(
            np.array([q_base, q_shoulder, q_elbow], dtype=float)
        )

    valid_solutions = []

    for q in candidates:
        if check_limits and not within_joint_limits(q[1], q[2]):
            continue

        position = fk_position(
            q[0],
            q[1],
            q[2],
            check_limits=check_limits,
        )

        error = float(np.linalg.norm(position - target))

        if error <= tolerance_m:
            valid_solutions.append((error, q, position))

    if not valid_solutions:
        return {
            "success": False,
            "message": (
                "No inverse-kinematics solution satisfies the target "
                "and the selected joint-limit settings."
            ),
            "target_position_m": target.copy(),
        }

    # Return the solution with the smallest Cartesian error.
    error, q, position = min(
        valid_solutions,
        key=lambda solution: solution[0],
    )

    return {
        "success": True,
        "joint_angles_rad": q.copy(),
        "joint_angles_deg": np.rad2deg(q),
        "position_m": position.copy(),
        "target_position_m": target.copy(),
        "position_error_m": error,
        "message": "Inverse-kinematics solution found.",
    }


if __name__ == "__main__":
    target = np.array([0.40, 0.0, 0.65])

    result = inverse_kinematics(target)

    print("3-DOF ROBOTIC ARM - INVERSE KINEMATICS")
    print("=" * 45)
    print("Target position (m):", target)
    print("Success:", result["success"])
    print("Message:", result["message"])

    if result["success"]:
        print(
            "Joint angles (degrees):",
            np.round(result["joint_angles_deg"], 4),
        )
        print(
            "Calculated position (m):",
            np.round(result["position_m"], 6),
        )
        print(
            "Position error (mm):",
            round(result["position_error_m"] * 1000, 6),
        )
