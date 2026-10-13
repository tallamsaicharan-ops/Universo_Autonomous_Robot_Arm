"""Validate joint limits, actuator stroke limits, and command bounds."""

import math
from dataclasses import dataclass
from typing import Dict, Optional


@dataclass
class JointLimit:
    minimum: Optional[float] = None
    maximum: Optional[float] = None
    continuous: bool = False

    def __post_init__(self):
        if not isinstance(self.continuous, bool):
            raise ValueError("Continuous must be a boolean.")

        if self.continuous:
            if self.minimum is not None or self.maximum is not None:
                raise ValueError(
                    "Continuous joints must not define position bounds."
                )
            return

        if self.minimum is None or self.maximum is None:
            raise ValueError(
                "Non-continuous joints require minimum and maximum limits."
            )

        if (
            isinstance(self.minimum, bool)
            or isinstance(self.maximum, bool)
            or not isinstance(self.minimum, (int, float))
            or not isinstance(self.maximum, (int, float))
            or not math.isfinite(self.minimum)
            or not math.isfinite(self.maximum)
        ):
            raise ValueError("Joint limits must be finite numbers.")

        if self.minimum >= self.maximum:
            raise ValueError("Minimum limit must be less than maximum.")

    def contains(self, position: float) -> bool:
        if (
            isinstance(position, bool)
            or not isinstance(position, (int, float))
            or not math.isfinite(position)
        ):
            return False

        if self.continuous:
            return True

        tolerance = 1e-12
        return (
            self.minimum - tolerance
            <= position
            <= self.maximum + tolerance
        )

class LimitManager:
    def __init__(
        self,
        joint_limits: Dict[str, JointLimit],
        actuator_limits: Optional[Dict[str, JointLimit]] = None,
    ):
        if not isinstance(joint_limits, dict) or not joint_limits:
            raise ValueError("At least one joint limit must be configured.")

        if not all(
            isinstance(name, str)
            and name
            and isinstance(limit, JointLimit)
            for name, limit in joint_limits.items()
        ):
            raise ValueError("Joint limits must map valid names to JointLimit objects.")

        self.joint_limits = dict(joint_limits)
        self.actuator_limits = dict(actuator_limits or {})

        if not all(
            isinstance(name, str)
            and name
            and isinstance(limit, JointLimit)
            for name, limit in self.actuator_limits.items()
        ):
            raise ValueError(
                "Actuator limits must map valid names to JointLimit objects."
            )

    def validate_joint_position(
        self, joint_name: str, position: float
    ) -> bool:
        limit = self.joint_limits.get(joint_name)

        if limit is None:
            raise ValueError(
                f"No limits configured for joint '{joint_name}'."
            )

        return limit.contains(position)

    def validate_joint_targets(self, targets: Dict[str, float]) -> bool:
        if not isinstance(targets, dict) or not targets:
            return False

        # Require every configured joint and reject unknown joints.
        if set(targets) != set(self.joint_limits):
            return False

        return all(
            self.validate_joint_position(name, position)
            for name, position in targets.items()
        )

    def validate_actuator_stroke(
        self, actuator_name: str, stroke: float
    ) -> bool:
        limit = self.actuator_limits.get(actuator_name)

        if limit is None:
            raise ValueError(
                f"No stroke limits configured for actuator '{actuator_name}'."
            )

        return limit.contains(stroke)
