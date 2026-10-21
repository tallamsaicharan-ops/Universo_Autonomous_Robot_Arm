"""Actuator pin-to-pin length at the joint limits must be 300 mm (retracted) / 500 mm (extended).

1. Geometry check (always runs): rotates the CAD pin holes to the URDF limits.
2. Code check (skipped until actuator_conversion.py has actuator_lengths()).
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import urdf_reference as ref  # noqa: E402

J = ref.load_joints()
SH = ref.limits(J, 'shoulder_joint')
EL = ref.limits(J, 'elbow_joint')
RETRACTED_MM, EXTENDED_MM = 300.0, 500.0

# Pin-hole centres in the arm plane [x, z] (base_link, metres) at joint angles = 0, from the URDF meshes.
SHOULDER_AXIS = np.array([0.0975003, 0.0200000])
ELBOW_AXIS = np.array([0.0975243, 0.5599954])
BASE_PIN = np.array([-0.0950000, 0.0750000])        # base block rear lug (fixed)
UPPER_PIN_SHOULDER = np.array([0.1280118, 0.2700081])  # upper-arm plate, shoulder actuator
UPPER_PIN_ELBOW = np.array([0.1280115, 0.1899984])     # upper-arm plate, elbow actuator
FOREARM_PIN = np.array([0.2747671, 0.6468530])         # forearm plate, elbow actuator


def _rot(v, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def shoulder_mm(q_shoulder):
    moving = SHOULDER_AXIS + _rot(UPPER_PIN_SHOULDER - SHOULDER_AXIS, q_shoulder)
    return float(np.linalg.norm(moving - BASE_PIN)) * 1000


def elbow_mm(q_elbow):
    moving = ELBOW_AXIS + _rot(FOREARM_PIN - ELBOW_AXIS, -q_elbow)
    return float(np.linalg.norm(moving - UPPER_PIN_ELBOW)) * 1000


def test_urdf_limits_are_actuator_end_stops():
    assert shoulder_mm(SH[1]) == pytest.approx(RETRACTED_MM, abs=0.05)   # shoulder upper = retracted
    assert elbow_mm(EL[0]) == pytest.approx(EXTENDED_MM, abs=0.05)       # elbow lower = extended
    assert elbow_mm(EL[1]) == pytest.approx(RETRACTED_MM, abs=0.05)      # elbow upper = retracted


def test_shoulder_never_reaches_full_extension():
    # Shoulder lower limit (-90 deg) is a CAD limit; the actuator is 450.7 mm there and can never reach 500.
    assert shoulder_mm(SH[0]) == pytest.approx(450.70, abs=0.05)
    worst = max(shoulder_mm(q) for q in np.linspace(SH[0], SH[1], 500))
    assert worst < EXTENDED_MM


def test_actuator_lengths_function_matches_geometry():
    try:
        from robot_arm.kinematics.actuator_conversion import actuator_lengths
    except ImportError as exc:
        pytest.skip(f'actuator_lengths() not available yet: {exc}')
    for q1, q2 in ((SH[1], EL[0]), (SH[1], EL[1]), (-0.8, 0.4)):
        result = actuator_lengths(q1, q2)
        if isinstance(result, dict):
            s, e = result['shoulder_actuator_mm'], result['elbow_actuator_mm']
        else:
            s, e = result[0], result[1]
        assert s == pytest.approx(shoulder_mm(q1), abs=0.1)
        assert e == pytest.approx(elbow_mm(q2), abs=0.1)
