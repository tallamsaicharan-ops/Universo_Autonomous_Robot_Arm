"""Place-object mission workflow."""

from enum import Enum
from typing import Callable, Optional


class PlaceObjectStatus(Enum):
    IDLE = "idle"
    APPROACHING = "approaching"
    RELEASING = "releasing"
    VERIFYING = "verifying"
    COMPLETED = "completed"
    FAILED = "failed"


class PlaceObjectMission:
    def __init__(self):
        self.status = PlaceObjectStatus.IDLE
        self.error_message: Optional[str] = None

    def execute(
        self,
        approach_target: Callable[[], bool],
        release_object: Callable[[], bool],
        verify_release: Callable[[], bool],
    ) -> bool:
        """Run the placement sequence using supplied operation callbacks."""
        if self.status in (
            PlaceObjectStatus.APPROACHING,
            PlaceObjectStatus.RELEASING,
            PlaceObjectStatus.VERIFYING,
        ):
            return False

        self.error_message = None

        try:
            self.status = PlaceObjectStatus.APPROACHING
            if not approach_target():
                return self._fail("Failed to reach the placement position.")

            self.status = PlaceObjectStatus.RELEASING
            if not release_object():
                return self._fail("Failed to release the object.")

            self.status = PlaceObjectStatus.VERIFYING
            if not verify_release():
                return self._fail("Object release could not be verified.")

            self.status = PlaceObjectStatus.COMPLETED
            return True

        except Exception as exc:
            return self._fail(f"Place mission error: {exc}")

    def _fail(self, message: str) -> bool:
        self.status = PlaceObjectStatus.FAILED
        self.error_message = message
        return False

    def reset(self) -> None:
        self.status = PlaceObjectStatus.IDLE
        self.error_message = None