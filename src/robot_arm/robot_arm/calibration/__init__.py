"""Calibration: raw encoder counts <-> URDF joint angles (Phase 1). Self-contained (numpy-free, no ROS)."""
from .encoder_calibration import (
    EncoderModel,
    load_encoders,
    counts_to_joint_angles,
    status_report,
)
from .encoder_sources import EncoderSource, ManualEntrySource, Stm32Source, SimulatedSource
from .arm_calibration import (
    FitResult,
    fit_zero_and_direction,
    shoulder_from_inclinometer,
    elbow_from_inclinometer,
    measure_counts_per_rev,
    check_endstops,
    check_tip,
)

__all__ = [
    'EncoderModel', 'load_encoders', 'counts_to_joint_angles', 'status_report',
    'EncoderSource', 'ManualEntrySource', 'Stm32Source', 'SimulatedSource',
    'FitResult', 'fit_zero_and_direction', 'shoulder_from_inclinometer', 'elbow_from_inclinometer',
    'measure_counts_per_rev', 'check_endstops', 'check_tip',
]
