"""Load configured joint limits into the LimitManager."""

import math
from pathlib import Path

import yaml

from .limit_manager import JointLimit, LimitManager


def load_joint_limits(config_path):
    """Load joint limits from joints.yaml.

    Only joints with finite, ordered limits in radians are accepted.
    Limits marked unverified are loaded for software testing, but this
    function does not certify that they are safe for physical hardware.
    """
    path = Path(config_path)

    with path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    if not isinstance(config, dict) or not isinstance(config.get("joints"), dict):
        raise ValueError("Configuration must contain a 'joints' mapping.")

    joint_limits = {}

    for name, data in config["joints"].items():
        if not isinstance(name, str) or not name:
            raise ValueError("Joint names must be non-empty strings.")

        if not isinstance(data, dict):
            raise ValueError(f"Invalid configuration for joint '{name}'.")

        minimum = data.get("min_position_rad")
        maximum = data.get("max_position_rad")

        # Null bounds mean that the joint has no configured numeric limits.
        if minimum is None or maximum is None:
            continue

        if (
            isinstance(minimum, bool)
            or isinstance(maximum, bool)
            or not isinstance(minimum, (int, float))
            or not isinstance(maximum, (int, float))
            or not math.isfinite(minimum)
            or not math.isfinite(maximum)
            or minimum >= maximum
        ):
            raise ValueError(f"Invalid radian limits for joint '{name}'.")

        joint_limits[name] = JointLimit(
            minimum=float(minimum),
            maximum=float(maximum),
        )

    if not joint_limits:
        raise ValueError("No numeric joint limits were configured.")

    manager = LimitManager(joint_limits)

    verified = {
        name: bool(config["joints"][name].get("limits_verified", False))
        for name in joint_limits
    }

    return manager, verified
