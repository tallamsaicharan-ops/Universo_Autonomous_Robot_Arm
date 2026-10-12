"""Soil-collection mission workflow."""

from enum import Enum
from typing import Callable, Optional


class SoilCollectionStatus(Enum):
    IDLE = "idle"
    APPROACHING = "approaching"
    COLLECTING = "collecting"
    VERIFYING = "verifying"
    COMPLETED = "completed"
    FAILED = "failed"


class SoilCollectionMission:
    def __init__(self):
        self.status = SoilCollectionStatus.IDLE
        self.error_message: Optional[str] = None

    def execute(
        self,
        approach_site: Callable[[], bool],
        collect_sample: Callable[[], bool],
        verify_sample: Callable[[], bool],
    ) -> bool:
        """Execute soil collection using supplied operation callbacks."""
        if self.status in (
            SoilCollectionStatus.APPROACHING,
            SoilCollectionStatus.COLLECTING,
            SoilCollectionStatus.VERIFYING,
        ):
            return False

        self.error_message = None

        try:
            self.status = SoilCollectionStatus.APPROACHING
            if not approach_site():
                return self._fail("Failed to reach the soil sampling location.")

            self.status = SoilCollectionStatus.COLLECTING
            if not collect_sample():
                return self._fail("Soil collection operation failed.")

            self.status = SoilCollectionStatus.VERIFYING
            if not verify_sample():
                return self._fail("Soil sample collection could not be verified.")

            self.status = SoilCollectionStatus.COMPLETED
            return True

        except Exception as exc:
            return self._fail(f"Soil-collection mission error: {exc}")

    def _fail(self, message: str) -> bool:
        self.status = SoilCollectionStatus.FAILED
        self.error_message = message
        return False

    def reset(self) -> None:
        self.status = SoilCollectionStatus.IDLE
        self.error_message = None