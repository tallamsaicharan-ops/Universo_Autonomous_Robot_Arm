from typing import Dict, Optional
import math

from .trajectory import Trajectory


class MotionPlanner:
    """Generates simple joint-space trajectories for the robot arm."""

    def __init__(self, max_velocity: float = 0.5):
        if max_velocity <= 0:
            raise ValueError("Maximum velocity must be positive.")

        self.max_velocity = max_velocity

    def plan_trajectory(
        self,
        start_positions: Dict[str, float],
        target_positions: Dict[str, float],
        time_step: float = 0.1,
    ) -> Optional[Trajectory]:
        """
        Generate a synchronized linear joint-space trajectory.

        Joint positions are assumed to be in radians.
        max_velocity is in radians per second.

        Returns None if the joint sets do not match.
        """

        if time_step <= 0:
            raise ValueError("Time step must be positive.")

        if (
            not start_positions
            or set(start_positions) != set(target_positions)
        ):
            return None

        joint_names = list(start_positions.keys())

        max_distance = max(
            abs(target_positions[name] - start_positions[name])
            for name in joint_names
        )

        duration = max_distance / self.max_velocity
        trajectory = Trajectory(joint_names=joint_names)

        if duration == 0:
            trajectory.add_point(start_positions.copy(), 0.0)
            return trajectory

        steps = max(1, math.ceil(duration / time_step))

        for step in range(steps + 1):
            ratio = step / steps
            positions = {
                name: (
                    start_positions[name]
                    + ratio * (
                        target_positions[name] - start_positions[name]
                    )
                )
                for name in joint_names
            }

            trajectory.add_point(
                positions=positions,
                time_from_start=step * duration / steps,
            )

        return trajectory