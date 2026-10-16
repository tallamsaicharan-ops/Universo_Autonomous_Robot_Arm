from dataclasses import dataclass


@dataclass
class EncoderCalibration:
    """Stores calibration settings for one joint encoder."""

    joint_name: str
    zero_offset: float = 0.0
    direction: int = 1
    is_calibrated: bool = False

    def __post_init__(self) -> None:
        if self.direction not in (-1, 1):
            raise ValueError("direction must be either 1 or -1")

    def calibrate(self, raw_position: float, known_position: float) -> None:
        """Set the offset using a known joint position."""
        self.zero_offset = (
            known_position - self.direction * raw_position
        )
        self.is_calibrated = True

    def get_position(self, raw_position: float) -> float:
        """Convert a raw encoder reading to a calibrated position."""
        if not self.is_calibrated:
            raise RuntimeError(
                f"Encoder for '{self.joint_name}' is not calibrated."
            )

        return self.direction * raw_position + self.zero_offset

    def reset(self) -> None:
        """Mark the encoder as requiring calibration."""
        self.zero_offset = 0.0
        self.is_calibrated = False