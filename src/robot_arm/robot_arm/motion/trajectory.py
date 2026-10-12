from dataclasses import dataclass, field
from typing import List, Dict


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
        """Add a joint-position waypoint to the trajectory."""

        if time_from_start < 0:
            raise ValueError("Time cannot be negative.")

        if self.times and time_from_start <= self.times[-1]:
            raise ValueError(
                "Each waypoint time must be greater than the previous time."
            )

        missing = set(self.joint_names) - set(positions)
        if missing:
            raise ValueError(f"Missing joint positions: {sorted(missing)}")

        self.points.append({
            name: positions[name] for name in self.joint_names
        })
        self.times.append(time_from_start)

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
        return len(self.points) == 0