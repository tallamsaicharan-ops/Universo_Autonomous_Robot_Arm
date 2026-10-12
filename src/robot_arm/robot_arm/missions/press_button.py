"""Press-button mission workflow."""

from enum import Enum
from typing import Callable, Optional


class PressButtonStatus(Enum):
    IDLE = "idle"
    APPROACHING = "approaching"
    PRESSING = "pressing"
    VERIFYING = "verifying"
    COMPLETED = "completed"
    FAILED = "failed"


class PressButtonMission:
    def __init__(self):
        self.status = PressButtonStatus.IDLE
        self.error_message: Optional[str] = None

    def execute(
        self,
        approach_button: Callable[[], bool],
        press_button: Callable[[], bool],
        verify_press: Callable[[], bool],
    ) -> bool:
        """Execute the button-press sequence using supplied callbacks."""
        if self.status in (
            PressButtonStatus.APPROACHING,
            PressButtonStatus.PRESSING,
            PressButtonStatus.VERIFYING,
        ):
            return False

        self.error_message = None

        try:
            self.status = PressButtonStatus.APPROACHING
            if not approach_button():
                return self._fail("Failed to approach the button.")

            self.status = PressButtonStatus.PRESSING
            if not press_button():
                return self._fail("Button press operation failed.")

            self.status = PressButtonStatus.VERIFYING
            if not verify_press():
                return self._fail("Button press could not be verified.")

            self.status = PressButtonStatus.COMPLETED
            return True

        except Exception as exc:
            return self._fail(f"Button-press mission error: {exc}")

    def _fail(self, message: str) -> bool:
        self.status = PressButtonStatus.FAILED
        self.error_message = message
        return False

    def reset(self) -> None:
        self.status = PressButtonStatus.IDLE
        self.error_message = None