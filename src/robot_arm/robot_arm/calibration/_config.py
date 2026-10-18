"""Find / read / write the calibration YAML files (config/encoders.yaml, config/calibration.yaml).

Search order: $ROBOT_ARM_CONFIG_DIR, source tree <package>/config, installed share directory.
Writes always go to the SOURCE tree when it exists (so results survive a rebuild).
"""
import os
from pathlib import Path

import yaml


def config_dir() -> Path:
    env = os.environ.get('ROBOT_ARM_CONFIG_DIR')
    if env and Path(env).is_dir():
        return Path(env)
    src = Path(__file__).resolve().parents[2] / 'config'
    if (src / 'encoders.yaml').is_file():
        return src
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('robot_arm')) / 'config'
        if (share / 'encoders.yaml').is_file():
            return share
    except Exception:
        pass
    raise FileNotFoundError('config directory with encoders.yaml not found; set ROBOT_ARM_CONFIG_DIR')


def load_yaml(name: str) -> dict:
    with (config_dir() / name).open('r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def save_yaml(name: str, data: dict) -> Path:
    path = config_dir() / name
    with path.open('w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return path
