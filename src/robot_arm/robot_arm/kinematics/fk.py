#!/usr/bin/env python3
"""
fk.py

Forward kinematics for the 3-DOF robotic arm:
    1. Base
    2. Shoulder
    3. Elbow

Geometry and joint limits are based on the Excel measurement sheet.

Units:
    Length: metres
    Joint angles: radians

Coordinate frame:
    Origin: base rotation axis at the bottom face of the base block.
    +Z: upward.
    +X: forward when the base angle is zero.
    +Y: left when viewed from above.

Zero pose:
    Upper arm vertical.
    Forearm 35 degrees above horizontal.

Note:
    The zero pose is a geometric reference and is outside the
    permitted shoulder joint range.
"""

import numpy as np


# =========================================================
# 1. GEOMETRY FROM EXCEL
# =========================================================

BASE_FORWARD_OFFSET_M = 0.0975
SHOULDER_HEIGHT_M = 0.020

UPPER_ARM_LENGTH_M = 0.540
FOREARM_LENGTH_M = 0.430

FOREARM_ZERO_ELEVATION_RAD = np.deg2rad(35.0)


# =========================================================
# 2. JOINT LIMITS FROM EXCEL
# =========================================================

# Base joint is continuous.
SHOULDER_LIMITS = (-1.570796, -0.022426)
ELBOW_LIMITS = (-0.149650, 1.007196)

LIMIT_TOLERANCE_RAD = 1e-6


# =========================================================
# 3. EXCEL ZERO-POSE VALIDATION TARGET
# =========================================================

ZERO_POSE_TARGET_M = np.array([
    0.4498,
    0.0,
    0.8066
])


# =========================================================
# 4. VALIDATION FUNCTIONS
# =========================================================

def _validate_finite_angles(q_base, q_shoulder, q_elbow):
    """Raise ValueError if any joint angle is not finite."""

    angles = {
        "Base": q_base,
        "Shoulder": q_shoulder,
        "Elbow": q_elbow
    }

    for name, angle in angles.items():
        if not np.isfinite(angle):
            raise ValueError(f"{name} angle must be finite.")


def within_joint_limits(
    q_shoulder,
    q_elbow,
    tol=LIMIT_TOLERANCE_RAD
):
    """
    Return True if shoulder and elbow are within their Excel limits.

    The base is continuous and is not checked here.
    """

    if not np.isfinite(q_shoulder) or not np.isfinite(q_elbow):
        return False

    if not np.isfinite(tol) or tol < 0:
        raise ValueError("Tolerance must be finite and non-negative.")

    return bool(
        SHOULDER_LIMITS[0] - tol
        <= q_shoulder
        <= SHOULDER_LIMITS[1] + tol

        and

        ELBOW_LIMITS[0] - tol
        <= q_elbow
        <= ELBOW_LIMITS[1] + tol
    )


def clamp_to_limits(q_shoulder, q_elbow):
    """
    Return shoulder and elbow angles clipped to their Excel limits.

    Note:
        Clamping changes the requested angles. It does not guarantee
        that the resulting end-effector position reaches a target.
    """

    if not np.isfinite(q_shoulder) or not np.isfinite(q_elbow):
        raise ValueError("Joint angles must be finite.")

    return (
        float(np.clip(q_shoulder, *SHOULDER_LIMITS)),
        float(np.clip(q_elbow, *ELBOW_LIMITS))
    )


def validate_joint_angles(q_base, q_shoulder, q_elbow):
    """
    Raise ValueError if any angle is non-finite or if the shoulder
    or elbow exceeds the Excel joint limits.
    """

    _validate_finite_angles(q_base, q_shoulder, q_elbow)

    if not within_joint_limits(q_shoulder, q_elbow):
        raise ValueError(
            "Joint angle outside Excel limits.\n"
            f"Shoulder: {np.rad2deg(q_shoulder):.4f} degrees; "
            f"allowed {np.rad2deg(SHOULDER_LIMITS[0]):.4f} to "
            f"{np.rad2deg(SHOULDER_LIMITS[1]):.4f} degrees.\n"
            f"Elbow: {np.rad2deg(q_elbow):.4f} degrees; "
            f"allowed {np.rad2deg(ELBOW_LIMITS[0]):.4f} to "
            f"{np.rad2deg(ELBOW_LIMITS[1]):.4f} degrees."
        )


# =========================================================
# 5. JOINT ANGLE RELATIONSHIPS
# =========================================================

def upper_arm_elevation(q_shoulder):
    """Upper-arm elevation above horizontal, in radians."""

    return np.pi / 2.0 + q_shoulder


def forearm_elevation(q_shoulder, q_elbow):
    """Forearm elevation above horizontal, in radians."""

    return (
        FOREARM_ZERO_ELEVATION_RAD
        + q_shoulder
        - q_elbow
    )


# =========================================================
# 6. PLANAR FORWARD KINEMATICS
# =========================================================

def fk_planar(q_shoulder, q_elbow):
    """
    Calculate the end-effector position in the arm plane.

    Returns:
        r: horizontal radial distance from the base rotation axis (m)
        z: height above the base_link origin (m)

    This function deliberately does not enforce joint limits.
    It can therefore be used by numerical IK algorithms while
    evaluating candidate angles.

    Positive r points forward in the base-zero arm plane.
    """

    if not np.isfinite(q_shoulder) or not np.isfinite(q_elbow):
        raise ValueError("Joint angles must be finite.")

    alpha = upper_arm_elevation(q_shoulder)
    beta = forearm_elevation(q_shoulder, q_elbow)

    radial_distance = (
        BASE_FORWARD_OFFSET_M
        + UPPER_ARM_LENGTH_M * np.cos(alpha)
        + FOREARM_LENGTH_M * np.cos(beta)
    )

    z = (
        SHOULDER_HEIGHT_M
        + UPPER_ARM_LENGTH_M * np.sin(alpha)
        + FOREARM_LENGTH_M * np.sin(beta)
    )

    return float(radial_distance), float(z)


# =========================================================
# 7. THREE-DIMENSIONAL FORWARD KINEMATICS
# =========================================================

def fk_position(
    q_base,
    q_shoulder,
    q_elbow,
    check_limits=True
):
    """
    Calculate end-effector position in the base_link frame.

    Parameters:
        q_base: base rotation in radians.
        q_shoulder: shoulder angle in radians.
        q_elbow: elbow angle in radians.
        check_limits: enforce shoulder and elbow limits if True.

    Returns:
        NumPy array [x, y, z], in metres.
    """

    if check_limits:
        validate_joint_angles(q_base, q_shoulder, q_elbow)
    else:
        _validate_finite_angles(q_base, q_shoulder, q_elbow)

    radial_distance, z = fk_planar(
        q_shoulder,
        q_elbow
    )

    x = radial_distance * np.cos(q_base)
    y = radial_distance * np.sin(q_base)

    return np.array([x, y, z], dtype=float)


# =========================================================
# 8. POSITIONS OF ARM JOINTS AND TIP
# =========================================================

def fk_joint_positions(
    q_base,
    q_shoulder,
    q_elbow,
    check_limits=True
):
    """
    Calculate the positions of the shoulder axis, elbow axis and tip.

    Returns:
        Dictionary with:
            shoulder: [x, y, z] in metres
            elbow:    [x, y, z] in metres
            tip:      [x, y, z] in metres
    """

    if check_limits:
        validate_joint_angles(q_base, q_shoulder, q_elbow)
    else:
        _validate_finite_angles(q_base, q_shoulder, q_elbow)

    alpha = upper_arm_elevation(q_shoulder)

    # Shoulder-axis position in the arm plane.
    r_shoulder = BASE_FORWARD_OFFSET_M
    z_shoulder = SHOULDER_HEIGHT_M

    # Elbow-axis position in the arm plane.
    r_elbow = (
        r_shoulder
        + UPPER_ARM_LENGTH_M * np.cos(alpha)
    )

    z_elbow = (
        z_shoulder
        + UPPER_ARM_LENGTH_M * np.sin(alpha)
    )

    # End-effector position.
    r_tip, z_tip = fk_planar(
        q_shoulder,
        q_elbow
    )

    c = np.cos(q_base)
    s = np.sin(q_base)

    def to_3d(r, z):
        return np.array([
            r * c,
            r * s,
            z
        ])

    return {
        "shoulder": to_3d(r_shoulder, z_shoulder),
        "elbow": to_3d(r_elbow, z_elbow),
        "tip": to_3d(r_tip, z_tip)
    }


# =========================================================
# 9. END-EFFECTOR POSE INFORMATION
# =========================================================

def fk_pose(
    q_base,
    q_shoulder,
    q_elbow,
    check_limits=True
):
    """
    Return end-effector position and simplified orientation information.

    This does not return a complete six-dimensional pose.
    """

    position = fk_position(
        q_base,
        q_shoulder,
        q_elbow,
        check_limits
    )

    return {
        "position_m": position,
        "base_rotation_rad": float(q_base),
        "forearm_elevation_rad": forearm_elevation(
            q_shoulder,
            q_elbow
        ),
        "joint_angles_rad": np.array([
            q_base,
            q_shoulder,
            q_elbow
        ])
    }


# =========================================================
# 10. ZERO-POSE VALIDATION
# =========================================================

def validate_zero_pose(tolerance_m=0.001):
    """
    Compare the calculated reference-pose position with the Excel target.

    The reference pose is evaluated without enforcing joint limits because
    the zero shoulder angle lies outside the permitted shoulder range.
    """

    if not np.isfinite(tolerance_m) or tolerance_m < 0:
        raise ValueError(
            "Tolerance must be finite and non-negative."
        )

    calculated = fk_position(
        0.0,
        0.0,
        0.0,
        check_limits=False
    )

    error = calculated - ZERO_POSE_TARGET_M
    error_distance = float(np.linalg.norm(error))

    return {
        "calculated_position_m": calculated,
        "excel_target_m": ZERO_POSE_TARGET_M.copy(),
        "error_m": error,
        "error_distance_m": error_distance,
        "passed": error_distance <= tolerance_m
    }


# =========================================================
# 11. SELF-TEST
# =========================================================

if __name__ == "__main__":

    print("3-DOF ROBOTIC ARM - FORWARD KINEMATICS")
    print("=" * 45)

    # Reference-pose validation.
    result = validate_zero_pose()

    print("\nExcel reference-pose validation:")
    print(
        "Calculated position (mm):",
        np.round(result["calculated_position_m"] * 1000, 4)
    )
    print(
        "Excel target (mm):       ",
        np.round(result["excel_target_m"] * 1000, 4)
    )
    print(
        "Coordinate error (mm):   ",
        np.round(result["error_m"] * 1000, 4)
    )
    print(
        "Total position error (mm):",
        round(result["error_distance_m"] * 1000, 6)
    )
    print("Within tolerance:         ", result["passed"])

    # Example configuration within the specified limits.
    q_base, q_shoulder, q_elbow = np.deg2rad([
        30.0,
        -20.0,
        15.0
    ])

    position = fk_position(
        q_base,
        q_shoulder,
        q_elbow
    )

    print("\nExample joint angles:")
    print(f"Base:     {np.rad2deg(q_base):.3f} degrees")
    print(f"Shoulder: {np.rad2deg(q_shoulder):.3f} degrees")
    print(f"Elbow:    {np.rad2deg(q_elbow):.3f} degrees")

    print("\nEnd-effector position:")
    print(f"X = {position[0]:.6f} m")
    print(f"Y = {position[1]:.6f} m")
    print(f"Z = {position[2]:.6f} m")

    print("\nJoint-axis positions:")
    for name, point in fk_joint_positions(
        q_base,
        q_shoulder,
        q_elbow
    ).items():
        print(
            f"{name.capitalize():8s}: "
            f"{np.round(point, 6)} m"
        )
