"""Convert joint angles to linear actuator lengths."""

from dataclasses import dataclass
from typing import Tuple
import math


@dataclass
class ActuatorGeometry:
    """Geometry of a linear actuator mounted between two pivot points.

    All distances are in metres.
    All angles are in radians.

    fixed_x, fixed_y:
        Fixed actuator pivot relative to the joint pivot.

    moving_radius:
        Distance from the joint pivot to the moving actuator attachment.

    moving_offset:
        Angular offset of the moving attachment at zero joint angle.

    min_length, max_length:
        Permitted actuator lengths, in metres.
    """

    fixed_x: float
    fixed_y: float
    moving_radius: float
    moving_offset: float = 0.0
    min_length: float = 0.0
    max_length: float = math.inf

    def __post_init__(self) -> None:
        if self.moving_radius <= 0:
            raise ValueError("moving_radius must be positive.")

        if self.min_length < 0:
            raise ValueError("min_length cannot be negative.")

        if self.max_length < self.min_length:
            raise ValueError("max_length must be >= min_length.")


def calculate_actuator_length(
    joint_angle: float,
    geometry: ActuatorGeometry,
) -> float:
    """Calculate actuator length for a given joint angle.

    This simplified model assumes that both mounting points lie in
    the same plane and that the moving attachment rotates around
    the joint pivot.
    """
    moving_angle = joint_angle + geometry.moving_offset

    moving_x = geometry.moving_radius * math.cos(moving_angle)
    moving_y = geometry.moving_radius * math.sin(moving_angle)

    length = math.hypot(
        moving_x - geometry.fixed_x,
        moving_y - geometry.fixed_y,
    )

    if not geometry.min_length <= length <= geometry.max_length:
        raise ValueError(
            f"Calculated actuator length {length:.4f} m is outside "
            f"the permitted range "
            f"[{geometry.min_length:.4f}, {geometry.max_length:.4f}] m."
        )

    return length


def calculate_actuator_lengths(
    shoulder_angle: float,
    elbow_angle: float,
    shoulder_geometry: ActuatorGeometry,
    elbow_geometry: ActuatorGeometry,
) -> Tuple[float, float]:
    """Return shoulder and elbow actuator lengths, in metres."""
    shoulder_length = calculate_actuator_length(
        shoulder_angle,
        shoulder_geometry,
    )

    elbow_length = calculate_actuator_length(
        elbow_angle,
        elbow_geometry,
    )

    return shoulder_length, elbow_length
