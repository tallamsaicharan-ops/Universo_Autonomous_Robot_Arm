"""Validate joint limits, actuator stroke limits, and command bounds."""

from dataclasses import dataclass
from typing import Dict, Optional


@dataclass
class JointLimit:
    minimum: Optional[float] = None
    maximum: Optional[float] = None
    continuous: bool = False

    def __post_init__(self):
        if self.continuous:
            return

        if self.minimum is None or self.maximum is None:
            raise ValueError(
                "Non-continuous joints require minimum and maximum limits."
            )

        if self.minimum >= self.maximum:
            raise ValueError("Minimum limit must be less than maximum.")

    def contains(self, position: float) -> bool:
        if self.continuous:
            return True

        return self.minimum <= position <= self.maximum


class LimitManager:
    def __init__(
        self,
        joint_limits: Dict[str, JointLimit],
        actuator_limits: Optional[Dict[str, JointLimit]] = None,
    ):
        self.joint_limits = joint_limits
        self.actuator_limits = actuator_limits or {}

    def validate_joint_position(self, joint_name: str, position: float) -> bool:
        limit = self.joint_limits.get(joint_name)

        if limit is None:
            raise ValueError(f"No limits configured for joint '{joint_name}'.")

        return limit.contains(position)

    def validate_joint_targets(self, targets: Dict[str, float]) -> bool:
        if not targets:
            return False

        return all(
            self.validate_joint_position(name, position)
            for name, position in targets.items()
        )

    def validate_actuator_stroke(self, actuator_name: str, stroke: float) -> bool:
        limit = self.actuator_limits.get(actuator_name)

        if limit is None:
            raise ValueError(
                f"No stroke limits configured for actuator '{actuator_name}'."
            )

        return limit.contains(stroke)