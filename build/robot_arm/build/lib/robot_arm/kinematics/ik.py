import math
from typing import Optional, Tuple


def inverse_kinematics(
    x: float,
    y: float,
    z: float,
    upper_arm_length: float,
    forearm_length: float,
) -> Optional[Tuple[float, float, float]]:
    """
    Calculate base, shoulder, and elbow angles for a target position.

    Args:
        x, y, z: Target position in metres, relative to the arm base.
        upper_arm_length: Upper arm length in metres.
        forearm_length: Forearm length in metres.

    Returns:
        (base_angle, shoulder_angle, elbow_angle) in radians,
        or None if the target is unreachable.

    Assumptions:
        - Base rotates around the vertical Z-axis.
        - Shoulder and elbow rotate in a vertical plane.
        - Shoulder angle is measured from the horizontal.
        - The arm geometry matches the preliminary FK model.
    """

    if upper_arm_length <= 0 or forearm_length <= 0:
        raise ValueError("Arm lengths must be positive.")

    # Calculate the base rotation.
    base_angle = math.atan2(y, x)

    # Reduce the 3D target to the arm's vertical plane.
    radial_distance = math.hypot(x, y)
    vertical_distance = z

    # Distance from the shoulder to the target.
    distance_squared = (
        radial_distance**2 + vertical_distance**2
    )
    distance = math.sqrt(distance_squared)

    # Check whether the target is reachable.
    if distance > upper_arm_length + forearm_length:
        return None

    if distance < abs(upper_arm_length - forearm_length):
        return None

    # Calculate the elbow angle using the cosine rule.
    cos_elbow = (
        distance_squared
        - upper_arm_length**2
        - forearm_length**2
    ) / (2 * upper_arm_length * forearm_length)

    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    elbow_angle = math.acos(cos_elbow)

    # Calculate the shoulder angle.
    target_angle = math.atan2(
        vertical_distance, radial_distance
    )

    correction_angle = math.atan2(
        forearm_length * math.sin(elbow_angle),
        upper_arm_length
        + forearm_length * math.cos(elbow_angle),
    )

    shoulder_angle = target_angle - correction_angle

    return base_angle, shoulder_angle, elbow_angle