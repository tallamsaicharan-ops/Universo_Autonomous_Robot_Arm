from dataclasses import dataclass
from typing import List, Tuple
import math


@dataclass
class SphereObstacle:
    """A spherical obstacle represented in the arm's coordinate frame."""

    x: float
    y: float
    z: float
    radius: float


class CollisionChecker:
    """Basic point-to-sphere collision checker."""

    def __init__(self, safety_margin: float = 0.02):
        if safety_margin < 0:
            raise ValueError("Safety margin cannot be negative.")

        self.safety_margin = safety_margin
        self.obstacles: List[SphereObstacle] = []

    def add_obstacle(self, obstacle: SphereObstacle) -> None:
        if obstacle.radius <= 0:
            raise ValueError("Obstacle radius must be positive.")

        self.obstacles.append(obstacle)

    def clear_obstacles(self) -> None:
        self.obstacles.clear()

    def point_in_collision(
        self,
        point: Tuple[float, float, float],
        robot_radius: float = 0.0,
    ) -> bool:
        """Check whether a point intersects any inflated obstacle."""

        if robot_radius < 0:
            raise ValueError("Robot radius cannot be negative.")

        px, py, pz = point

        for obstacle in self.obstacles:
            distance = math.sqrt(
                (px - obstacle.x) ** 2
                + (py - obstacle.y) ** 2
                + (pz - obstacle.z) ** 2
            )

            required_distance = (
                obstacle.radius
                + robot_radius
                + self.safety_margin
            )

            if distance <= required_distance:
                return True

        return False