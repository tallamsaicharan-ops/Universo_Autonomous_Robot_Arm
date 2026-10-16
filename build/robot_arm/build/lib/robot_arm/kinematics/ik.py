"""Inverse kinematics for a 3-DOF robot arm."""

import math


def inverse_kinematics(
    x,
    y,
    z,
    upper_arm_length,
    forearm_length,
):
    """
    Calculate joint angles for a 3-DOF arm.

    Coordinate conventions must match fk.py:
      - Base rotates about Z.
      - Shoulder and elbow rotate about local Y.
      - Links extend along local +X.
      - Positive shoulder/elbow angles move the arm toward -Z.

    Returns:
        (base_angle, shoulder_angle, elbow_angle), or None
        if the target is unreachable.
    """
    if upper_arm_length <= 0 or forearm_length <= 0:
        raise ValueError("Arm link lengths must be positive.")

    values = (x, y, z, upper_arm_length, forearm_length)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Coordinates and link lengths must be finite.")

    l1 = upper_arm_length
    l2 = forearm_length

    # Base rotation and distance in the horizontal plane.
    base_angle = math.atan2(y, x)
    radial = math.hypot(x, y)

    # FK's positive joint angles produce negative Z, so reflect
    # the requested vertical coordinate for the standard 2-link solution.
    vertical = -z

    distance_squared = radial**2 + vertical**2
    distance = math.sqrt(distance_squared)

    tolerance = 1e-9
    if distance > l1 + l2 + tolerance:
        return None
    if distance < abs(l1 - l2) - tolerance:
        return None

    # Law of cosines.
    cos_elbow = (
        distance_squared - l1**2 - l2**2
    ) / (2.0 * l1 * l2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))

    # One of the two possible elbow configurations.
    elbow_angle = math.acos(cos_elbow)

    shoulder_angle = (
        math.atan2(vertical, radial)
        - math.atan2(
            l2 * math.sin(elbow_angle),
            l1 + l2 * math.cos(elbow_angle),
        )
    )

    return base_angle, shoulder_angle, elbow_angle
