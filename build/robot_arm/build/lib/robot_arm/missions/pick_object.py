"""Pick-object mission workflow."""

from enum import Enum
from typing import Callable, Optional


class PickObjectStatus(Enum):
    IDLE = "idle"
    APPROACHING = "approaching"
    GRASPING = "grasping"
    VERIFYING = "verifying"
    COMPLETED = "completed"
    FAILED = "failed"


class PickObjectMission:
    def __init__(self):
        self.status = PickObjectStatus.IDLE
        self.error_message: Optional[str] = None

    def execute(
        self,
        approach: Callable[[], bool],
        grasp: Callable[[], bool],
        verify_grasp: Callable[[], bool],
    ) -> bool:
        """Run the pick sequence using supplied operation callbacks."""
        if self.status in (
            PickObjectStatus.APPROACHING,
            PickObjectStatus.GRASPING,
            PickObjectStatus.VERIFYING,
        ):
            return False

        self.error_message = None

        try:
            self.status = PickObjectStatus.APPROACHING
            if not approach():
                return self._fail("Failed to approach the object.")

            self.status = PickObjectStatus.GRASPING
            if not grasp():
                return self._fail("Gripper operation failed.")

            self.status = PickObjectStatus.VERIFYING
            if not verify_grasp():
                return self._fail("Grasp could not be verified.")

            self.status = PickObjectStatus.COMPLETED
            return True

        except Exception as exc:
            return self._fail(f"Pick mission error: {exc}")

    def _fail(self, message: str) -> bool:
        self.status = PickObjectStatus.FAILED
        self.error_message = message
        return False

    def reset(self) -> None:
        self.status = PickObjectStatus.IDLE
        self.error_message = None