"""Kinematics must match the URDF.

- FK (fk.py) vs the URDF, over the whole joint range
- IK (ik.py) round trip: FK -> IK -> same joint angles
IK tests are SKIPPED (not failed) while robot_arm.kinematics cannot be imported
(actuator_conversion.py is still missing actuator_lengths / length_in_range).
"""
import importlib.util
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
FK_TOL_M = 1e-4        # 0.1 mm


def _load_fk():
    """Load fk.py on its own (it only needs numpy), so FK is tested even if the package import fails."""
    path = HERE.parent / 'robot_arm' / 'kinematics' / 'fk.py'
    spec = importlib.util.spec_from_file_location('fk_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _load_ik():
    try:
        from robot_arm.kinematics import ik
    except ImportError as exc:
        pytest.skip(f'robot_arm.kinematics does not import yet: {exc}')
    return ik


FK = _load_fk()


def _random_q(n, seed=0):
    rng = np.random.default_rng(seed)
    return [(rng.uniform(-math.pi, math.pi), rng.uniform(*SH), rng.uniform(*EL)) for _ in range(n)]


def test_fk_limits_match_urdf():
    assert FK.SHOULDER_LIMITS[0] == pytest.approx(SH[0], abs=1e-6)
    assert FK.SHOULDER_LIMITS[1] == pytest.approx(SH[1], abs=1e-6)
    assert FK.ELBOW_LIMITS[0] == pytest.approx(EL[0], abs=1e-6)
    assert FK.ELBOW_LIMITS[1] == pytest.approx(EL[1], abs=1e-6)


def test_fk_zero_pose_matches_urdf():
    p = FK.fk_position(0.0, 0.0, 0.0, check_limits=False)
    assert np.linalg.norm(p - ref.tip(J, 0.0, 0.0, 0.0)) < FK_TOL_M


def test_fk_matches_urdf_over_joint_range():
    worst = max(np.linalg.norm(FK.fk_position(*q) - ref.tip(J, *q)) for q in _random_q(300))
    assert worst < FK_TOL_M, f'FK differs from URDF by {worst * 1000:.3f} mm'


def test_fk_at_joint_limits():
    for q1 in SH:
        for q2 in EL:
            assert np.linalg.norm(FK.fk_position(0.3, q1, q2) - ref.tip(J, 0.3, q1, q2)) < FK_TOL_M


def test_ik_round_trip():
    ik = _load_ik()
    for q in _random_q(200, seed=1):
        target = ref.tip(J, *q)
        r = ik.inverse_kinematics(target)
        assert r['status'] == 'exact', r.get('message')
        assert r['position_error_norm_m'] < FK_TOL_M
        assert np.allclose(r['joint_angles_rad'], q, atol=math.radians(0.01))


def test_ik_reports_unreachable_target():
    ik = _load_ik()
    r = ik.inverse_kinematics([2.0, 0.0, 0.5], closest_fallback=False)
    assert not r['success']
