from dataclasses import dataclass
from typing import Optional


@dataclass
class JointState:
    name: str
    position: float = 0.0
    target_position: Optional[float] = None
    velocity: float = 0.0
    is_moving: bool = False
    is_calibrated: bool = False

    @property
    def position_error(self) -> Optional[float]:
        if self.target_position is None:
            return None

        return self.target_position - self.position

    def update_position(self, measured_position: float) -> None:
        self.position = measured_position

    def set_target(self, target_position: float) -> None:
        self.target_position = target_position

    def stop(self) -> None:
        self.is_moving = False
        self.velocity = 0.0