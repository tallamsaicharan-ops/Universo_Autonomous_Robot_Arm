"""Pure jogging logic for the joystick teleop (no ROS imports, so it can be unit-tested anywhere).

A stick deflection in [-1, 1] becomes a joint velocity (fraction of that joint's max speed);
targets are integrated over time and kept inside the URDF limits.
"""
from dataclasses import dataclass
import math
from typing import Dict, Iterable, Optional
import xml.etree.ElementTree as ET

JOINTS = ('base_joint', 'shoulder_joint', 'elbow_joint')


@dataclass(frozen=True)
class JointSpec:
    name: str
    lower: Optional[float]      # None for continuous joints
    upper: Optional[float]
    max_velocity: float         # rad/s

    @property
    def continuous(self) -> bool:
        return self.lower is None or self.upper is None


def parse_urdf_limits(urdf_xml: str, joint_names: Iterable[str] = JOINTS,
                      velocity_caps: Optional[Dict[str, float]] = None) -> Dict[str, JointSpec]:
    """Read type, limits and velocity of the given joints from a URDF string."""
    root = ET.fromstring(urdf_xml)
    found = {j.get('name'): j for j in root.findall('joint')}
    specs = {}
    for name in joint_names:
        if name not in found:
            raise ValueError(f'joint {name} not in robot_description')
        j = found[name]
        lim = j.find('limit')
        vel = float(lim.get('velocity')) if lim is not None and lim.get('velocity') else 0.5
        if velocity_caps and name in velocity_caps:
            vel = min(vel, velocity_caps[name])
        if j.get('type') == 'continuous':
            specs[name] = JointSpec(name, None, None, vel)
        else:
            specs[name] = JointSpec(name, float(lim.get('lower')), float(lim.get('upper')), vel)
    return specs


def apply_deadzone(value: float, deadzone: float) -> float:
    """0 inside the deadzone, then rescaled so the output still reaches +/-1 at full deflection."""
    if abs(value) <= deadzone:
        return 0.0
    out = (abs(value) - deadzone) / (1.0 - deadzone)
    return math.copysign(min(out, 1.0), value)


def clamp(q: float, spec: JointSpec, margin: float = 0.0) -> float:
    if spec.continuous:
        return q
    return min(max(q, spec.lower + margin), spec.upper - margin)


def step_targets(targets: Dict[str, float], commands: Dict[str, float], specs: Dict[str, JointSpec],
                 dt: float, speed_scale: float = 1.0, margin: float = 0.0) -> Dict[str, float]:
    """Move each target by command * max_velocity * speed_scale * dt, then clamp to limits.

    commands: joint -> stick value in [-1, 1] (after deadzone and sign).
    """
    out = dict(targets)
    for name, cmd in commands.items():
        cmd = max(-1.0, min(1.0, cmd))
        spec = specs[name]
        out[name] = clamp(targets[name] + cmd * spec.max_velocity * speed_scale * dt, spec, margin)
    return out


def move_duration(current: Dict[str, float], goal: Dict[str, float], specs: Dict[str, JointSpec],
                  minimum: float = 0.5) -> float:
    """Time (s) for the slowest joint to reach the goal at its max speed, plus 20 %."""
    t = max(abs(goal[n] - current[n]) / specs[n].max_velocity for n in goal)
    return max(minimum, 1.2 * t)
