"""Recovery management for autonomous robot-arm missions."""

from enum import Enum
from typing import Optional


class RecoveryStatus(Enum):
    IDLE = "idle"
    RECOVERY_REQUIRED = "recovery_required"
    RECOVERING = "recovering"
    RECOVERED = "recovered"
    FAILED = "failed"
    EMERGENCY_STOP = "emergency_stop"


class RecoveryManager:
    def __init__(self, max_attempts: int = 3):
        if max_attempts < 1:
            raise ValueError("max_attempts must be at least 1.")

        self.max_attempts = max_attempts
        self.attempts = 0
        self.status = RecoveryStatus.IDLE
        self.last_error: Optional[str] = None

    def report_failure(self, error: str) -> None:
        """Record a failure that may require recovery."""
        self.last_error = error
        self.attempts = 0
        self.status = RecoveryStatus.RECOVERY_REQUIRED

    def begin_recovery(self) -> bool:
        if self.status != RecoveryStatus.RECOVERY_REQUIRED:
            return False

        if self.attempts >= self.max_attempts:
            self.status = RecoveryStatus.FAILED
            return False

        self.attempts += 1
        self.status = RecoveryStatus.RECOVERING
        return True

    def complete_recovery(self, success: bool) -> None:
        if self.status != RecoveryStatus.RECOVERING:
            raise RuntimeError("Recovery has not been started.")

        if success:
            self.status = RecoveryStatus.RECOVERED
            self.last_error = None
        elif self.attempts >= self.max_attempts:
            self.status = RecoveryStatus.FAILED
        else:
            self.status = RecoveryStatus.RECOVERY_REQUIRED

    def require_emergency_stop(self, reason: str) -> None:
        """Latch the software recovery state as an emergency stop."""
        self.last_error = reason
        self.status = RecoveryStatus.EMERGENCY_STOP