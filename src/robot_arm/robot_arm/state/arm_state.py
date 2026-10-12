from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Dict

from .joint_state import JointState


class ArmStatus(Enum):
    IDLE = "idle"
    MOVING = "moving"
    EXECUTING_TASK = "executing_task"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class ArmState:
    status: ArmStatus = ArmStatus.IDLE
    joints: Dict[str, JointState] = field(default_factory=dict)
    current_task: Optional[str] = None
    error_message: Optional[str] = None

    def add_joint(self, joint: JointState) -> None:
        self.joints[joint.name] = joint

    def get_joint(self, name: str) -> Optional[JointState]:
        return self.joints.get(name)

    def set_status(self, status: ArmStatus) -> None:
        self.status = status

    def set_error(self, message: str) -> None:
        self.status = ArmStatus.ERROR
        self.error_message = message

    def clear_error(self) -> None:
        self.error_message = None
        if self.status == ArmStatus.ERROR:
            self.status = ArmStatus.IDLE