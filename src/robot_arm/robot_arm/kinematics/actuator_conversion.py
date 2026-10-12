from dataclasses import dataclass
from typing import Tuple
import math


@dataclass
class ActuatorGeometry:
    fixed_x: float
    fixed_y: float
    moving_radius: float
    moving_offset: float = 0.0
    min_length: float = 0.0
    max_length: float = float("inf")


def calculate_actuator_length(
    joint_angle: float,
    geometry: ActuatorGeometry,
) -> float:
    """
    Calculate actuator pin-to-pin length for a simplified
    planar mounting geometry.

    All distances are in metres.
    joint_angle is in radians.

    The moving mount is assumed to be located at:
        x = moving_radius * cos(angle + offset)
        y = moving_radius * sin(angle + offset)

    The fixed mount is located at (fixed_x, fixed_y).
    """

    moving_x = geometry.moving_radius * math.cos(
        joint_angle + geometry.moving_offset
    )
    moving_y = geometry.moving_radius * math.sin(
        joint_angle + geometry.moving_offset
    )

    dx = moving_x - geometry.fixed_x
    dy = moving_y - geometry.fixed_y

    length = math.hypot(dx, dy)

    if not geometry.min_length <= length <= geometry.max_length:
        raise ValueError(
            f"Calculated actuator length {length:.4f} m "
            "is outside configured limits."
        )

    return length


def calculate_actuator_lengths(
    shoulder_angle: float,
    elbow_angle: float,
    shoulder_geometry: ActuatorGeometry,
    elbow_geometry: ActuatorGeometry,
) -> Tuple[float, float]:
    """Calculate shoulder and elbow actuator lengths."""

    shoulder_length = calculate_actuator_length(
        shoulder_angle, shoulder_geometry
    )
    elbow_length = calculate_actuator_length(
        elbow_angle, elbow_geometry
    )

    return shoulder_length, elbow_length