"""Generate validated joint-space trajectories."""

import math
from typing import Dict, Optional

from .trajectory import Trajectory


class MotionPlanner:
    """Generate synchronized linear joint-space trajectories."""

    def __init__(self, max_velocity: float = 0.5):
        if (
            not isinstance(max_velocity, (int, float))
            or isinstance(max_velocity, bool)
            or not math.isfinite(max_velocity)
            or max_velocity <= 0
        ):
            raise ValueError("Maximum velocity must be finite and positive.")

        self.max_velocity = float(max_velocity)

    def plan_trajectory(
        self,
        start_positions: Dict[str, float],
        target_positions: Dict[str, float],
        time_step: float = 0.1,
    ) -> Optional[Trajectory]:
        """Plan a linear trajectory between two joint configurations.

        Joint positions are assumed to be radians.
        Maximum velocity is in radians per second.

        Returns None when joint names do not match or are missing.
        """

        if (
            not isinstance(time_step, (int, float))
            or isinstance(time_step, bool)
            or not math.isfinite(time_step)
            or time_step <= 0
        ):
            raise ValueError("Time step must be finite and positive.")

        if not isinstance(start_positions, dict) or not isinstance(
            target_positions, dict
        ):
            raise TypeError("Joint positions must be dictionaries.")

        if (
            not start_positions
            or set(start_positions) != set(target_positions)
        ):
            return None

        joint_names = list(start_positions.keys())

        for name in joint_names:
            if not isinstance(name, str) or not name:
                raise ValueError("Joint names must be non-empty strings.")

            for label, positions in (
                ("start", start_positions),
                ("target", target_positions),
            ):
                value = positions[name]

                if (
                    not isinstance(value, (int, float))
                    or isinstance(value, bool)
                    or not math.isfinite(value)
                ):
                    raise ValueError(
                        f"{label} position for joint '{name}' "
                        "must be finite."
                    )

        max_distance = max(
            abs(
                target_positions[name] - start_positions[name]
            )
            for name in joint_names
        )

        duration = max_distance / self.max_velocity
        trajectory = Trajectory(joint_names=joint_names)

        if duration == 0:
            trajectory.add_point(
                positions={
                    name: float(start_positions[name])
                    for name in joint_names
                },
                time_from_start=0.0,
            )
            return trajectory

        steps = max(1, math.ceil(duration / time_step))

        for step in range(steps + 1):
            ratio = step / steps

            positions = {
                name: (
                    start_positions[name]
                    + ratio
                    * (
                        target_positions[name]
                        - start_positions[name]
                    )
                )
                for name in joint_names
            }

            trajectory.add_point(
                positions=positions,
                time_from_start=step * duration / steps,
            )

        return trajectory
