"""Track the overall robot-arm system status."""

from dataclasses import dataclass
from enum import Enum


class SystemStatus(Enum):
    INITIALIZING = "initializing"
    READY = "ready"
    BUSY = "busy"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class SystemState:
    status: SystemStatus = SystemStatus.INITIALIZING
    jetson_connected: bool = True
    stm32_connected: bool = False
    can_connected: bool = False
    camera_connected: bool = False
    last_error: str = ""

    def set_status(self, status: SystemStatus) -> None:
        self.status = status

    def update_connection(self, component: str, connected: bool) -> None:
        if component not in {
            "jetson", "stm32", "can", "camera"
        }:
            raise ValueError(f"Unknown component: {component}")

        setattr(self, f"{component}_connected", connected)

    def set_error(self, message: str) -> None:
        self.last_error = message
        self.status = SystemStatus.ERROR

    def clear_error(self) -> None:
        self.last_error = ""
        if self.status == SystemStatus.ERROR:
            self.status = SystemStatus.INITIALIZING

    @property
    def communications_ready(self) -> bool:
        return self.jetson_connected and self.stm32_connected and self.can_connected

    @property
    def all_required_connections_ready(self) -> bool:
        return self.communications_ready and self.camera_connected
