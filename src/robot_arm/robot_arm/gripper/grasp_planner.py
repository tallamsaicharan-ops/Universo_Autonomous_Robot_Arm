from dataclasses import dataclass
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
        if approach_height < 0:
            raise ValueError("Approach height cannot be negative.")

        self.approach_height = approach_height

    def plan_grasp(
        self,
        object_position: Tuple[float, float, float],
    ) -> Optional[GraspTarget]:
        """Create a target above the detected object."""

        if len(object_position) != 3:
            return None

        x, y, z = object_position

        return GraspTarget(
            x=float(x),
            y=float(y),
            z=float(z),
            approach_height=self.approach_height,
        )

    def get_approach_position(
        self,
        target: GraspTarget,
    ) -> Tuple[float, float, float]:
        """Return a position vertically above the grasp target."""

        return (
            target.x,
            target.y,
            target.z + target.approach_height,
        )