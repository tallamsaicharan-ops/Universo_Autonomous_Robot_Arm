"""Basic grasp target planning for a robot arm."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


@dataclass
class GraspTarget:
    """Represents a proposed grasp position in 3D space."""

    x: float
    y: float
    z: float
    approach_height: float = 0.05


class GraspPlanner:
    """Creates a basic grasp approach target from an object position."""

    def __init__(self, approach_height: float = 0.05):
        if not math.isfinite(approach_height) or approach_height < 0:
            raise ValueError(
                "Approach height must be finite and non-negative."
            )

        self.approach_height = float(approach_height)

    def plan_grasp(
        self,
        object_position: Tuple[float, float, float],
    ) -> Optional[GraspTarget]:
        """Create a grasp target from a finite 3D object position."""

        try:
            if len(object_position) != 3:
                return None

            x, y, z = (float(value) for value in object_position)
        except (TypeError, ValueError):
            return None

        if not all(math.isfinite(value) for value in (x, y, z)):
            return None

        return GraspTarget(
            x=x,
            y=y,
            z=z,
            approach_height=self.approach_height,
        )

    def get_approach_position(
        self,
        target: GraspTarget,
    ) -> Tuple[float, float, float]:
        """Return a position above the grasp target along positive Z."""

        values = (
            target.x,
            target.y,
            target.z,
            target.approach_height,
        )

        if not all(math.isfinite(value) for value in values):
            raise ValueError("Grasp target values must be finite.")

        if target.approach_height < 0:
            raise ValueError("Approach height cannot be negative.")

        return (
            target.x,
            target.y,
            target.z + target.approach_height,
        )

