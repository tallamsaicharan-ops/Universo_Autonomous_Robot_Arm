"""Independent helpers for the tests: read description/urdf/robot_arm.urdf and compute link poses.

Not a test file (no test_ prefix). Uses only the URDF, so it checks the kinematics code against
the robot description instead of against itself.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

PKG = Path(__file__).resolve().parent.parent              # src/robot_arm
URDF = PKG / 'description' / 'urdf' / 'robot_arm.urdf'
MOVEIT = PKG.parent / 'robot_arm_moveit_config'


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def _axis_rot(axis, a):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def load_joints():
    root = ET.parse(URDF).getroot()
    joints = {}
    for j in root.findall('joint'):
        o = j.find('origin')
        T = np.eye(4)
        T[:3, :3] = _rpy(*[float(v) for v in o.get('rpy', '0 0 0').split()])
        T[:3, 3] = [float(v) for v in o.get('xyz', '0 0 0').split()]
        ax, lim = j.find('axis'), j.find('limit')
        joints[j.get('name')] = dict(
            type=j.get('type'), parent=j.find('parent').get('link'), child=j.find('child').get('link'), T=T,
            axis=[float(v) for v in ax.get('xyz').split()] if ax is not None else None,
            limit=None if lim is None else {k: float(v) for k, v in lim.attrib.items()})
    return joints


def link_pose(joints, q, link):
    """4x4 pose of `link` in base_link for joint values q (dict, radians)."""
    by_child = {j['child']: (n, j) for n, j in joints.items()}
    chain = []
    while link in by_child:
        n, j = by_child[link]
        chain.append((n, j))
        link = j['parent']
    T = np.eye(4)
    for n, j in reversed(chain):
        T = T @ j['T']
        if j['type'] in ('revolute', 'continuous'):
            R = np.eye(4)
            R[:3, :3] = _axis_rot(j['axis'], q.get(n, 0.0))
            T = T @ R
    return T


def tip(joints, q_base, q_shoulder, q_elbow):
    return link_pose(joints, {'base_joint': q_base, 'shoulder_joint': q_shoulder, 'elbow_joint': q_elbow},
                     'arm_tip')[:3, 3]


def limits(joints, name):
    lim = joints[name]['limit']
    return lim['lower'], lim['upper']
