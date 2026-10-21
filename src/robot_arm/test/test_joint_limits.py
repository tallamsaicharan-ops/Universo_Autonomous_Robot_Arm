"""Every file that holds joint limits must agree with the URDF (300 / 500 mm actuators)."""
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import urdf_reference as ref  # noqa: E402

J = ref.load_joints()
CONFIG = ref.PKG / 'config'
YAML_NAME = {'base_joint': 'base', 'shoulder_joint': 'shoulder', 'elbow_joint': 'elbow'}   # joints.yaml keys


def test_urdf_limit_values():
    assert ref.limits(J, 'shoulder_joint') == pytest.approx((-1.570796, -0.022426), abs=1e-6)
    assert ref.limits(J, 'elbow_joint') == pytest.approx((-0.149650, 1.007196), abs=1e-6)
    assert J['base_joint']['type'] == 'continuous'


def test_joints_yaml_matches_urdf():
    cfg = yaml.safe_load((CONFIG / 'joints.yaml').read_text())['joints']
    assert cfg[YAML_NAME['base_joint']]['type'] == 'continuous'
    for name in ('shoulder_joint', 'elbow_joint'):
        lo, hi = ref.limits(J, name)
        entry = cfg[YAML_NAME[name]]
        assert entry['min_position_rad'] == pytest.approx(lo, abs=1e-6), name
        assert entry['max_position_rad'] == pytest.approx(hi, abs=1e-6), name


def test_calibration_end_stops_match_urdf():
    r = yaml.safe_load((CONFIG / 'calibration.yaml').read_text())['reference']
    assert r['shoulder_at_full_retraction_deg'] == pytest.approx(math.degrees(ref.limits(J, 'shoulder_joint')[1]), abs=0.001)
    assert r['elbow_at_full_extension_deg'] == pytest.approx(math.degrees(ref.limits(J, 'elbow_joint')[0]), abs=0.001)
    assert r['elbow_at_full_retraction_deg'] == pytest.approx(math.degrees(ref.limits(J, 'elbow_joint')[1]), abs=0.001)


def _inside(q):
    for name in ('shoulder_joint', 'elbow_joint'):
        lo, hi = ref.limits(J, name)
        if not lo < q[name] < hi:
            return False
    return True


def test_moveit_start_pose_inside_limits():
    ip = yaml.safe_load((ref.MOVEIT / 'config' / 'initial_positions.yaml').read_text())['initial_positions']
    assert _inside(ip)


def test_moveit_saved_poses_inside_limits():
    root = ET.parse(ref.MOVEIT / 'config' / 'robot_arm.srdf').getroot()
    states = root.findall('group_state')
    assert states
    for s in states:
        q = {j.get('name'): float(j.get('value')) for j in s.findall('joint')}
        assert _inside(q), s.get('name')


def test_moveit_speeds_not_above_urdf():
    jl = yaml.safe_load((ref.MOVEIT / 'config' / 'joint_limits.yaml').read_text())['joint_limits']
    for name in ('shoulder_joint', 'elbow_joint'):
        assert jl[name]['max_velocity'] <= J[name]['limit']['velocity'] + 1e-9, name
