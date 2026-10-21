"""
Inverse kinematics for the 3-DOF robotic arm.

Joint order:
    q_base     : base rotation (rad)
    q_shoulder : shoulder rotation (rad)
    q_elbow    : elbow rotation (rad)

Target position:
    [x, y, z] in metres.

Features:
    - Analytical inverse kinematics
    - Joint-limit checking
    - Forward-kinematics verification
    - Actuator lengths in millimetres
    - Closest-reachable-pose fallback
"""

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
)

from .actuator_conversion import actuator_lengths, length_in_range


def _validate_target(target_position):
    """Validate and convert the target position."""

    try:
        target = np.asarray(target_position, dtype=float)
    except (TypeError, ValueError):
        raise ValueError(
            "Target must contain three finite numeric values: [x, y, z]."
        )

    if target.shape != (3,):
        raise ValueError(
            "Target must contain exactly three values: [x, y, z]."
        )

    if not np.all(np.isfinite(target)):
        raise ValueError("Target coordinates must be finite.")

    return target


def _get_actuator_data(q_shoulder, q_elbow):
    """
    Calculate actuator lengths using actuator_conversion.py.

    Expected interface:
        actuator_lengths(q_shoulder, q_elbow)

    Expected return:
        shoulder length, elbow length, and optionally a range flag.

    Lengths must be in millimetres.
    """

    result = actuator_lengths(q_shoulder, q_elbow)

    if isinstance(result, dict):
        shoulder_mm = result.get("shoulder_actuator_mm")
        elbow_mm = result.get("elbow_actuator_mm")

        if shoulder_mm is None or elbow_mm is None:
            raise ValueError(
                "actuator_lengths() returned a dictionary without "
                "'shoulder_actuator_mm' and 'elbow_actuator_mm'."
            )

    else:
        try:
            # actuator_lengths() may return three values:
            # shoulder length, elbow length, and a range flag.
            shoulder_mm, elbow_mm = result[:2]
        except (TypeError, ValueError, IndexError, KeyError):
            raise ValueError(
                "actuator_lengths() must return shoulder and elbow "
                "actuator lengths in millimetres."
            )

    shoulder_mm = float(shoulder_mm)
    elbow_mm = float(elbow_mm)

    if not np.isfinite(shoulder_mm) or not np.isfinite(elbow_mm):
        raise ValueError("Actuator lengths must be finite.")

    # Use the shared range check, including its configured tolerance.
    actuators_ok = (
        length_in_range(shoulder_mm)
        and length_in_range(elbow_mm)
    )

    return shoulder_mm, elbow_mm, actuators_ok


def _make_result(
    target,
    joint_angles,
    calculated_position,
    status,
    message,
    tolerance_m,
):
    """Build a consistent IK result dictionary."""

    position_error = calculated_position - target
    error_norm = float(np.linalg.norm(position_error))

    _, q_shoulder, q_elbow = joint_angles

    shoulder_mm, elbow_mm, actuators_ok = _get_actuator_data(
        q_shoulder,
        q_elbow,
    )

    return {
        "success": status == "exact",
        "status": status,
        "message": message,
        "target_position_m": target.copy(),
        "joint_angles_rad": joint_angles.copy(),
        "joint_angles_deg": np.rad2deg(joint_angles),
        "calculated_position_m": calculated_position.copy(),
        "position_error_m": position_error,
        "position_error_norm_m": error_norm,
        "within_tolerance": error_norm <= tolerance_m,
        "shoulder_actuator_mm": shoulder_mm,
        "elbow_actuator_mm": elbow_mm,
        "actuators_ok": actuators_ok,
    }


def _position_error_squared(q_base, q_shoulder, q_elbow, target):
    """Calculate squared Cartesian position error."""

    position = fk_position(
        q_base,
        q_shoulder,
        q_elbow,
        check_limits=False,
    )

    difference = position - target

    return float(np.dot(difference, difference))


def _closest_reachable_pose(target):
    """
    Find an approximate closest pose within the shoulder/elbow limits.

    Uses a deterministic grid search followed by coordinate refinement.

    This is an approximate fallback, not a guarantee of the global
    minimum. Actuator lengths are checked separately.
    """

    shoulder_min, shoulder_max = SHOULDER_LIMITS
    elbow_min, elbow_max = ELBOW_LIMITS

    # Point the base toward the target's horizontal direction.
    q_base = float(np.arctan2(target[1], target[0]))

    shoulder_grid = np.linspace(
        shoulder_min,
        shoulder_max,
        101,
    )

    elbow_grid = np.linspace(
        elbow_min,
        elbow_max,
        101,
    )

    best_shoulder = None
    best_elbow = None
    best_error = float("inf")

    # Search the permitted joint space.
    for shoulder in shoulder_grid:
        for elbow in elbow_grid:
            error = _position_error_squared(
                q_base,
                shoulder,
                elbow,
                target,
            )

            if error < best_error:
                best_error = error
                best_shoulder = float(shoulder)
                best_elbow = float(elbow)

    # Refine the best grid point.
    shoulder_step = (
        shoulder_max - shoulder_min
    ) / 100.0

    elbow_step = (
        elbow_max - elbow_min
    ) / 100.0

    for _ in range(40):
        improved = False

        candidates = (
            (best_shoulder + shoulder_step, best_elbow),
            (best_shoulder - shoulder_step, best_elbow),
            (best_shoulder, best_elbow + elbow_step),
            (best_shoulder, best_elbow - elbow_step),
            (
                best_shoulder + shoulder_step,
                best_elbow + elbow_step,
            ),
            (
                best_shoulder + shoulder_step,
                best_elbow - elbow_step,
            ),
            (
                best_shoulder - shoulder_step,
                best_elbow + elbow_step,
            ),
            (
                best_shoulder - shoulder_step,
                best_elbow - elbow_step,
            ),
        )

        for shoulder, elbow in candidates:
            shoulder = float(
                np.clip(
                    shoulder,
                    shoulder_min,
                    shoulder_max,
                )
            )

            elbow = float(
                np.clip(
                    elbow,
                    elbow_min,
                    elbow_max,
                )
            )

            error = _position_error_squared(
                q_base,
                shoulder,
                elbow,
                target,
            )

            if error < best_error:
                best_error = error
                best_shoulder = shoulder
                best_elbow = elbow
                improved = True

        if not improved:
            shoulder_step *= 0.5
            elbow_step *= 0.5

        if max(shoulder_step, elbow_step) < 1e-9:
            break

    joint_angles = np.array(
        [q_base, best_shoulder, best_elbow],
        dtype=float,
    )

    calculated_position = fk_position(
        *joint_angles,
        check_limits=False,
    )

    return joint_angles, calculated_position


def inverse_kinematics(
    target_position,
    tolerance_m=1e-4,
    check_limits=True,
    closest_fallback=True,
):
    """
    Calculate inverse kinematics for a target end-effector position.

    Parameters
    ----------
    target_position : array-like
        Desired [x, y, z] position in metres.
    tolerance_m : float
        Maximum position error for an exact solution.
    check_limits : bool
        If True, enforce shoulder and elbow joint limits.
    closest_fallback : bool
        If True, search for an approximate closest pose if no exact
        valid solution is available.

    Returns
    -------
    dict
        Contains solution status, joint angles, position error,
        actuator lengths, and actuator-range status.

    Status values
    -------------
    exact
        The target was reached within tolerance.
    closest_reachable
        The exact target could not be reached, but an approximate
        fallback pose was calculated.
    no_solution
        No solution or fallback could be calculated.
    """

    try:
        target = _validate_target(target_position)
    except ValueError as exc:
        return {
            "success": False,
            "status": "no_solution",
            "message": str(exc),
        }

    # Reject booleans, strings, invalid values, and non-positive tolerances.
    if (
        isinstance(tolerance_m, (bool, np.bool_))
        or not isinstance(
            tolerance_m,
            (int, float, np.integer, np.floating),
        )
        or not np.isfinite(tolerance_m)
        or tolerance_m <= 0
    ):
        return {
            "success": False,
            "status": "no_solution",
            "message": "tolerance_m must be a positive finite number.",
            "target_position_m": target,
        }

    x, y, z = target

    # Base rotation.
    q_base = float(np.arctan2(y, x))

    # Convert target to coordinates relative to the shoulder axis.
    radial = float(
        np.hypot(x, y) - BASE_FORWARD_OFFSET_M
    )

    vertical = float(
        z - SHOULDER_HEIGHT_M
    )

    l1 = UPPER_ARM_LENGTH_M
    l2 = FOREARM_LENGTH_M

    distance_squared = radial**2 + vertical**2

    # Law of cosines.
    cos_gamma = (
        distance_squared - l1**2 - l2**2
    ) / (2.0 * l1 * l2)

    geometrically_reachable = (
        -1.0 - 1e-9 <= cos_gamma <= 1.0 + 1e-9
    )

    candidates = []

    if geometrically_reachable:
        cos_gamma = float(
            np.clip(cos_gamma, -1.0, 1.0)
        )

        gamma_magnitude = float(
            np.arccos(cos_gamma)
        )

        target_direction = float(
            np.arctan2(vertical, radial)
        )

        # Try both geometric branches.
        for gamma in (
            -gamma_magnitude,
            gamma_magnitude,
        ):
            alpha = target_direction - np.arctan2(
                l2 * np.sin(gamma),
                l1 + l2 * np.cos(gamma),
            )

            # Convert upper-arm elevation to shoulder joint angle.
            q_shoulder = float(
                alpha - np.pi / 2.0
            )

            # Consistent with fk.py:
            # alpha = pi/2 + q_shoulder
            # beta = FOREARM_ZERO_ELEVATION_RAD
            #        + q_shoulder - q_elbow
            # gamma = beta - alpha
            q_elbow = float(
                FOREARM_ZERO_ELEVATION_RAD
                - np.pi / 2.0
                - gamma
            )

            joint_angles = np.array(
                [q_base, q_shoulder, q_elbow],
                dtype=float,
            )

            if not np.all(np.isfinite(joint_angles)):
                continue

            # Apply joint limits.
            if check_limits:
                shoulder_min, shoulder_max = SHOULDER_LIMITS
                elbow_min, elbow_max = ELBOW_LIMITS

                if not (
                    shoulder_min - 1e-9
                    <= q_shoulder
                    <= shoulder_max + 1e-9
                ):
                    continue

                if not (
                    elbow_min - 1e-9
                    <= q_elbow
                    <= elbow_max + 1e-9
                ):
                    continue

            # Verify the candidate with FK.
            calculated_position = fk_position(
                *joint_angles,
                check_limits=False,
            )

            error_norm = float(
                np.linalg.norm(calculated_position - target)
            )

            if error_norm <= tolerance_m:
                candidates.append(
                    (joint_angles, calculated_position)
                )

    # Return the first verified exact solution.
    if candidates:
        joint_angles, calculated_position = candidates[0]

        try:
            return _make_result(
                target=target,
                joint_angles=joint_angles,
                calculated_position=calculated_position,
                status="exact",
                message="Target reached within tolerance.",
                tolerance_m=tolerance_m,
            )

        except (TypeError, ValueError, KeyError) as exc:
            return {
                "success": False,
                "status": "no_solution",
                "message": (
                    "IK found a valid pose, but actuator conversion "
                    "failed: " + str(exc)
                ),
                "target_position_m": target,
            }

    # No exact solution was found.
    if not closest_fallback:
        return {
            "success": False,
            "status": "no_solution",
            "message": (
                "No exact IK solution satisfies the target and "
                "selected joint limits."
            ),
            "target_position_m": target,
        }

    try:
        joint_angles, calculated_position = _closest_reachable_pose(
            target
        )

        result = _make_result(
            target=target,
            joint_angles=joint_angles,
            calculated_position=calculated_position,
            status="closest_reachable",
            message=(
                "The target could not be reached exactly. An approximate "
                "closest pose was calculated. Check actuator limits and "
                "approve the fallback before commanding the hardware."
            ),
            tolerance_m=tolerance_m,
        )

        # A fallback must never be reported as an exact solution.
        result["success"] = False

        return result

    except (TypeError, ValueError, KeyError, RuntimeError) as exc:
        return {
            "success": False,
            "status": "no_solution",
            "message": (
                "Could not calculate a valid fallback pose: "
                + str(exc)
            ),
            "target_position_m": target,
        }


if __name__ == "__main__":
    print("3-DOF ROBOTIC ARM - INVERSE KINEMATICS")
    print("=" * 45)

    # Use a reachable target generated by forward kinematics.
    reference_angles = np.deg2rad(
        [0.0, -30.0, 20.0]
    )

    target = fk_position(*reference_angles)

    print(
        "Reference joint angles (degrees):",
        np.rad2deg(reference_angles),
    )
    print("Target position (m):", target)

    result = inverse_kinematics(target)

    print("\nIK result:")
    print("Success:", result["success"])
    print("Status:", result["status"])
    print("Message:", result["message"])

    if "joint_angles_deg" in result:
        print(
            "Joint angles (degrees):",
            np.round(result["joint_angles_deg"], 4),
        )
        print(
            "Calculated position (m):",
            np.round(result["calculated_position_m"], 6),
        )
        print(
            "Position error (m):",
            result["position_error_norm_m"],
        )
        print(
            "Shoulder actuator (mm):",
            result["shoulder_actuator_mm"],
        )
        print(
            "Elbow actuator (mm):",
            result["elbow_actuator_mm"],
        )
        print(
            "Actuators within range:",
            result["actuators_ok"],
        )
