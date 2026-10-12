"""High-level state machine for autonomous arm operation."""

from enum import Enum


class AutonomyState(Enum):
    IDLE = "idle"
    INITIALIZING = "initializing"
    READY = "ready"
    EXECUTING_MISSION = "executing_mission"
    RECOVERING = "recovering"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"


class StateMachine:
    def __init__(self):
        self.state = AutonomyState.IDLE

    def transition(self, new_state: AutonomyState) -> bool:
        """Transition only when the requested state change is allowed."""
        allowed_transitions = {
            AutonomyState.IDLE: {
                AutonomyState.INITIALIZING,
                AutonomyState.EMERGENCY_STOP,
            },
            AutonomyState.INITIALIZING: {
                AutonomyState.READY,
                AutonomyState.ERROR,
                AutonomyState.EMERGENCY_STOP,
            },
            AutonomyState.READY: {
                AutonomyState.EXECUTING_MISSION,
                AutonomyState.IDLE,
                AutonomyState.ERROR,
                AutonomyState.EMERGENCY_STOP,
            },
            AutonomyState.EXECUTING_MISSION: {
                AutonomyState.READY,
                AutonomyState.RECOVERING,
                AutonomyState.ERROR,
                AutonomyState.EMERGENCY_STOP,
            },
            AutonomyState.RECOVERING: {
                AutonomyState.READY,
                AutonomyState.ERROR,
                AutonomyState.EMERGENCY_STOP,
            },
            AutonomyState.ERROR: {
                AutonomyState.IDLE,
                AutonomyState.EMERGENCY_STOP,
            },
            AutonomyState.EMERGENCY_STOP: set(),
        }

        if new_state not in allowed_transitions[self.state]:
            return False

        self.state = new_state
        return True

    def reset(self) -> bool:
        """Reset software state after an error; not a hardware reset."""
        if self.state != AutonomyState.ERROR:
            return False

        self.state = AutonomyState.IDLE
        return True