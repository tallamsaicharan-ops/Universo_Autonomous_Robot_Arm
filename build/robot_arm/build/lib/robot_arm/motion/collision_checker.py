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
        if not math.isfinite(safety_margin) or safety_margin < 0:
            raise ValueError("Safety margin must be finite and non-negative.")

        self.safety_margin = safety_margin
        self.obstacles: List[SphereObstacle] = []

    def add_obstacle(self, obstacle: SphereObstacle) -> None:
        values = (obstacle.x, obstacle.y, obstacle.z, obstacle.radius)

        if not all(math.isfinite(value) for value in values):
            raise ValueError("Obstacle values must be finite.")

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
        """Check whether a point intersects an inflated spherical obstacle."""

        if len(point) != 3 or not all(math.isfinite(v) for v in point):
            raise ValueError("Point must contain three finite coordinates.")

        if not math.isfinite(robot_radius) or robot_radius < 0:
            raise ValueError("Robot radius must be finite and non-negative.")

        px, py, pz = point

        for obstacle in self.obstacles:
            distance_squared = (
                (px - obstacle.x) ** 2
                + (py - obstacle.y) ** 2
                + (pz - obstacle.z) ** 2
            )

            required_distance = (
                obstacle.radius
                + robot_radius
                + self.safety_margin
            )

            if distance_squared <= required_distance ** 2:
                return True

        return False
