"""Position the arm for a manual grasp."""

from enum import Enum
from typing import Callable, Optional


class ManualGraspStatus(Enum):
    IDLE = "idle"
    POSITIONING = "positioning"
    READY = "ready"
    FAILED = "failed"


class PositionForManualGrasp:
    def __init__(self):
        self.status = ManualGraspStatus.IDLE
        self.error_message: Optional[str] = None

    def execute(
        self,
        position_arm: Callable[[], bool],
        confirm_ready: Callable[[], bool],
    ) -> bool:
        """Position the arm and confirm it is ready for manual grasping."""
        if self.status == ManualGraspStatus.POSITIONING:
            return False

        self.error_message = None

        try:
            self.status = ManualGraspStatus.POSITIONING

            if not position_arm():
                return self._fail("Failed to position the arm.")

            if not confirm_ready():
                return self._fail(
                    "Arm positioning completed, but readiness was not confirmed."
                )

            self.status = ManualGraspStatus.READY
            return True

        except Exception as exc:
            return self._fail(f"Manual-grasp positioning error: {exc}")

    def _fail(self, message: str) -> bool:
        self.status = ManualGraspStatus.FAILED
        self.error_message = message
        return False

    def reset(self) -> None:
        self.status = ManualGraspStatus.IDLE
        self.error_message = None