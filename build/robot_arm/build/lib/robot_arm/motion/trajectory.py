"""Store a sequence of validated joint-position waypoints."""

from dataclasses import dataclass, field
import math
from typing import Dict, List


@dataclass
class Trajectory:
    """Stores a sequence of joint positions for arm movement."""

    joint_names: List[str]
    points: List[Dict[str, float]] = field(default_factory=list)
    times: List[float] = field(default_factory=list)

    def add_point(
        self,
        positions: Dict[str, float],
        time_from_start: float,
    ) -> None:
        """Add a validated joint-position waypoint."""

        if (
            not isinstance(time_from_start, (int, float))
            or isinstance(time_from_start, bool)
            or not math.isfinite(time_from_start)
        ):
            raise ValueError("Waypoint time must be finite.")

        if time_from_start < 0:
            raise ValueError("Time cannot be negative.")

        if self.times and time_from_start <= self.times[-1]:
            raise ValueError(
                "Each waypoint time must be greater than the previous time."
            )

        if not isinstance(positions, dict):
            raise TypeError("Joint positions must be provided as a dictionary.")

        if not isinstance(self.joint_names, list) or not self.joint_names:
            raise ValueError("Joint names must be a non-empty list.")

        if any(
            not isinstance(name, str) or not name
            for name in self.joint_names
        ):
            raise ValueError("Joint names must be non-empty strings.")

        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("Joint names must be unique.")

        missing = set(self.joint_names) - set(positions)
        if missing:
            raise ValueError(f"Missing joint positions: {sorted(missing)}")

        validated_positions = {}

        for name in self.joint_names:
            value = positions[name]

            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
            ):
                raise ValueError(
                    f"Position for joint '{name}' must be finite."
                )

            validated_positions[name] = float(value)

        self.points.append(validated_positions)
        self.times.append(float(time_from_start))

    def clear(self) -> None:
        """Remove all trajectory waypoints."""
        self.points.clear()
        self.times.clear()

    @property
    def duration(self) -> float:
        """Return the trajectory duration in seconds."""
        return self.times[-1] if self.times else 0.0

    @property
    def is_empty(self) -> bool:
        """Return whether the trajectory contains no waypoints."""
        return len(self.points) == 0
