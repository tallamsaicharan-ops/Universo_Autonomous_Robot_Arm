"""
Kinematics package for the 3-DOF robotic arm.

Provides:
- Forward Kinematics (FK)
- Inverse Kinematics (IK)
- Position Jacobian
"""

from .fk import (
    fk_position,
    fk_planar,
    fk_pose,
    fk_joint_positions,
    validate_joint_angles,
    within_joint_limits,
)

from .ik import inverse_kinematics

from .jacobian import position_jacobian


__all__ = [
    "fk_position",
    "fk_planar",
    "fk_pose",
    "fk_joint_positions",
    "validate_joint_angles",
    "within_joint_limits",
    "inverse_kinematics",
    "position_jacobian",
]
