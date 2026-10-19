"""Load joint limits and verification status from joints.yaml."""

import math
from pathlib import Path

import yaml

from .limit_manager import JointLimit, LimitManager


def load_joint_limits(config_path):
    """Return (LimitManager, verification_status).

    Continuous joints have no positional bounds.
    Bounded joints require finite, ordered limits in radians.
    This function does not certify physical hardware safety.
    """
    path = Path(config_path)

    with path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    if not isinstance(config, dict) or not isinstance(config.get("joints"), dict):
        raise ValueError("Configuration must contain a 'joints' mapping.")

    joint_limits = {}
    verified = {}

    for name, data in config["joints"].items():
        if not isinstance(name, str) or not name:
            raise ValueError("Joint names must be non-empty strings.")

        if not isinstance(data, dict):
            raise ValueError(f"Invalid configuration for joint '{name}'.")

        joint_type = data.get("type")
        limits_verified = data.get("limits_verified", False)

        if not isinstance(limits_verified, bool):
            raise ValueError(
                f"'limits_verified' for joint '{name}' must be boolean."
            )

        if joint_type == "continuous":
            if (
                data.get("min_position_rad") is not None
                or data.get("max_position_rad") is not None
            ):
                raise ValueError(
                    f"Continuous joint '{name}' must not have position bounds."
                )

            joint_limits[name] = JointLimit(continuous=True)
            verified[name] = limits_verified
            continue

        if joint_type != "revolute":
            raise ValueError(
                f"Unsupported joint type '{joint_type}' for joint '{name}'."
            )

        minimum = data.get("min_position_rad")
        maximum = data.get("max_position_rad")

        if minimum is None or maximum is None:
            raise ValueError(
                f"Bounded joint '{name}' requires numeric radian limits."
            )

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
        verified[name] = limits_verified

    if not joint_limits:
        raise ValueError("No joints were configured.")

    return LimitManager(joint_limits), verified

