from dataclasses import dataclass
from enum import Enum
from typing import Optional


class GripperStatus(Enum):
    UNKNOWN = "unknown"
    OPEN = "open"
    CLOSED = "closed"
    MOVING = "moving"
    HOLDING_OBJECT = "holding_object"
    ERROR = "error"


@dataclass
class GripperState:
    status: GripperStatus = GripperStatus.UNKNOWN
    position: Optional[float] = None
    target_position: Optional[float] = None
    object_detected: bool = False
    error_message: Optional[str] = None

    def set_status(self, status: GripperStatus) -> None:
        self.status = status

    def update_position(self, measured_position: float) -> None:
        self.position = measured_position

    def set_target(self, target_position: float) -> None:
        self.target_position = target_position

    def confirm_object(self) -> None:
        self.object_detected = True
        self.status = GripperStatus.HOLDING_OBJECT

    def clear_object(self) -> None:
        self.object_detected = False

        if self.status == GripperStatus.HOLDING_OBJECT:
            self.status = GripperStatus.CLOSED

    def set_error(self, message: str) -> None:
        self.error_message = message
        self.status = GripperStatus.ERROR