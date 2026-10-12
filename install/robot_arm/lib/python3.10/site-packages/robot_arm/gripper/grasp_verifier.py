from dataclasses import dataclass
from typing import Optional

from ..state.gripper_state import GripperState, GripperStatus


@dataclass
class GraspVerificationResult:
    """Result of checking whether an object was grasped."""

    success: bool
    message: str


class GraspVerifier:
    """Verifies a grasp using available sensor feedback."""

    def __init__(self, state: GripperState):
        self.state = state

    def verify(
        self,
        object_detected: Optional[bool] = None,
        position_reached: Optional[bool] = None,
    ) -> GraspVerificationResult:
        """
        Check grasp feedback.

        Sensor values must come from real hardware or perception.
        Missing feedback is treated as inconclusive, not success.
        """

        if object_detected is None or position_reached is None:
            return GraspVerificationResult(
                success=False,
                message="Grasp verification inconclusive: sensor feedback missing.",
            )

        if not position_reached:
            return GraspVerificationResult(
                success=False,
                message="Gripper has not reached its target position.",
            )

        if not object_detected:
            self.state.clear_object()
            return GraspVerificationResult(
                success=False,
                message="No object detected in the gripper.",
            )

        self.state.confirm_object()

        return GraspVerificationResult(
            success=True,
            message="Object grasp confirmed by supplied sensor feedback.",
        )