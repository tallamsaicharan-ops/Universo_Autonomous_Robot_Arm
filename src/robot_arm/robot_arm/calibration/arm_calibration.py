from dataclasses import dataclass, field
from typing import Dict, Optional


@dataclass
class ArmCalibration:
    """Stores calibration status and reference positions for arm joints."""

    reference_positions: Dict[str, float] = field(default_factory=dict)
    calibrated_joints: Dict[str, bool] = field(default_factory=dict)

    def set_reference(
        self,
        joint_name: str,
        position: float,
    ) -> None:
        """Save the known reference position for a joint."""
        self.reference_positions[joint_name] = position
        self.calibrated_joints[joint_name] = True

    def get_reference(self, joint_name: str) -> Optional[float]:
        """Return a joint's reference position, if available."""
        return self.reference_positions.get(joint_name)

    def is_calibrated(self, joint_name: str) -> bool:
        """Check whether a joint has been calibrated."""
        return self.calibrated_joints.get(joint_name, False)

    def is_arm_calibrated(self, required_joints: list[str]) -> bool:
        """Check whether all required joints are calibrated."""
        return all(self.is_calibrated(name) for name in required_joints)

    def reset_joint(self, joint_name: str) -> None:
        """Clear calibration for one joint."""
        self.reference_positions.pop(joint_name, None)
        self.calibrated_joints[joint_name] = False