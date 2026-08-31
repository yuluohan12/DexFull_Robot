from __future__ import annotations

import copy
import os
from pathlib import Path

import yaml


DEFAULT_HARDWARE_CONFIG = Path.home() / ".config" / "dexfull" / "hardware.yaml"


def _deep_merge(base: dict, override: dict) -> dict:
    result = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def hardware_config_path() -> Path:
    value = os.environ.get("DEXFULL_HARDWARE_CONFIG")
    return Path(value).expanduser() if value else DEFAULT_HARDWARE_CONFIG


def load_camera_config(base_path: str | os.PathLike) -> dict:
    with Path(base_path).open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    profile_path = hardware_config_path()
    if profile_path.exists():
        with profile_path.open("r", encoding="utf-8") as stream:
            profile = yaml.safe_load(stream) or {}
        # The wrapper keeps room for future hand/robot selectors in the same
        # per-robot hardware profile. Direct topic keys remain accepted.
        camera_override = profile.get("cameras", profile)
        config = _deep_merge(config, camera_override)
    return config
