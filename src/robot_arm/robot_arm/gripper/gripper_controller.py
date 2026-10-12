from typing import Optional

from ..state.gripper_state import GripperState, GripperStatus


class GripperController:
    """Manages gripper commands and software state."""

    def __init__(self, state: Optional[GripperState] = None):
        self.state = state or GripperState()

    def open_gripper(self) -> None:
        """Request the gripper to open."""
        self.state.set_status(GripperStatus.MOVING)

    def close_gripper(self) -> None:
        """Request the gripper to close."""
        self.state.set_status(GripperStatus.MOVING)

    def update_position(self, position: float) -> None:
        """Update the gripper position using a measured value."""
        self.state.update_position(position)

    def confirm_open(self) -> None:
        """Mark the gripper as fully open."""
        self.state.set_status(GripperStatus.OPEN)

    def confirm_closed(self) -> None:
        """Mark the gripper as closed."""
        self.state.set_status(GripperStatus.CLOSED)

    def stop(self) -> None:
        """Stop the software operation and mark the state unknown."""
        self.state.set_status(GripperStatus.UNKNOWN)