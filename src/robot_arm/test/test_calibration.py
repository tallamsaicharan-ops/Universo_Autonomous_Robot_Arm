"""Calibration must recover hidden encoder offsets/directions and convert counts exactly."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from robot_arm.calibration import (EncoderModel, load_encoders, fit_zero_and_direction, SimulatedSource,
                                   shoulder_from_inclinometer, elbow_from_inclinometer, check_tip)
from robot_arm.calibration.arm_calibration import simulate, reference_cfg

REF = reference_cfg()


def test_config_is_tbd_until_hardware_known():
    enc = load_encoders()
    assert not enc['shoulder_joint'].ready and not enc['base_joint'].ready
    with pytest.raises(RuntimeError):
        enc['shoulder_joint'].counts_to_angle(100)


@pytest.mark.parametrize('kind,cpr,d,zero', [('absolute', 4096, 1, 3000), ('absolute', 16384, -1, 15000),
                                             ('incremental', 16380, -1, -777)])
def test_counts_angle_round_trip(kind, cpr, d, zero):
    m = EncoderModel('j', kind, float(cpr), d, float(zero))
    for deg in (-170, -90, -10.869, 0, 3.0, 56.254, 120):
        a = math.radians(deg)
        assert m.counts_to_angle(m.angle_to_counts(a)) == pytest.approx(a, abs=1e-12)


def test_absolute_wraps_across_zero_count():
    m = EncoderModel('j', 'absolute', 16384.0, 1, 16300.0)          # zero just below the wrap point
    assert math.degrees(m.counts_to_angle(100)) == pytest.approx((100 + 84) * 360 / 16384)


@pytest.mark.parametrize('d', [+1, -1])
def test_fit_recovers_truth(d):
    truth = {'j': dict(kind='absolute', counts_per_rev=16384.0, direction=d, zero_offset_counts=9000.0)}
    src = SimulatedSource(truth)
    samples = []
    for deg in (-60, -30, -5, 10):
        src.set_angles(j=math.radians(deg))
        samples.append((src.read_counts()['j'], math.radians(deg)))
    fit = fit_zero_and_direction(EncoderModel('j', 'absolute', 16384.0, None, None), samples)
    assert fit.accepted and fit.direction == d
    assert fit.zero_offset_counts == pytest.approx(9000.0, abs=0.6)


def test_fit_rejects_small_spread():
    m = EncoderModel('j', 'absolute', 4096.0, None, None)
    fit = fit_zero_and_direction(m, [(100, 0.0), (102, 0.002), (104, 0.004)])
    assert not fit.accepted


def test_inclinometer_conversions_match_urdf_convention():
    assert math.degrees(shoulder_from_inclinometer(60.0, REF)) == pytest.approx(-29.99745, abs=1e-6)
    # shoulder -30, elbow 15 -> forearm elevation 35 - 30 - 15 = -10 deg
    q2 = elbow_from_inclinometer(89.99745 - 30.0, 34.99797 - 30.0 - 15.0, REF)
    assert math.degrees(q2) == pytest.approx(15.0, abs=1e-6)


def test_tip_check_at_cad_pose():
    assert '0.0 mm' in check_tip(0.0, 0.0, 0.0, [0.4497665, 0.0, 0.8066195], REF)


def test_end_to_end_simulation():
    results = simulate(seed=5, inclinometer_noise_deg=0.1)
    assert all(r.accepted for r in results)
    assert {r.joint: r.direction for r in results} == {'shoulder_joint': -1, 'elbow_joint': 1, 'base_joint': 1}
