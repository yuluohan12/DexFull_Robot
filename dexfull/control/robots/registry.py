from __future__ import annotations

import importlib
from dataclasses import dataclass
from typing import Dict, Iterable, Tuple


@dataclass(frozen=True)
class RobotAdapter:
    name: str
    dds_family: str
    motor_count: int
    joints: Tuple[Tuple[str, int], ...]
    controller_module: str
    controller_class: str
    ik_module: str
    ik_class: str
    pelvis_height: float
    supports_motion_mode: bool = True

    def create_controller(self, *, motion_mode: bool, simulation_mode: bool):
        module = importlib.import_module(self.controller_module)
        cls = getattr(module, self.controller_class)
        kwargs = {"simulation_mode": simulation_mode}
        if self.supports_motion_mode:
            kwargs["motion_mode"] = motion_mode
        return cls(**kwargs)

    def create_ik(self, **kwargs):
        module = importlib.import_module(self.ik_module)
        return getattr(module, self.ik_class)(**kwargs)

    @property
    def joint_names(self) -> list:
        return [name for name, _ in self.joints]

    @property
    def joint_indices(self) -> list:
        return [index for _, index in self.joints]


_REGISTRY: Dict[str, RobotAdapter] = {}


def register_robot(adapter: RobotAdapter) -> None:
    key = adapter.name.upper()
    if key in _REGISTRY:
        raise ValueError(f"robot adapter already registered: {adapter.name}")
    _REGISTRY[key] = adapter


def get_robot_adapter(name: str) -> RobotAdapter:
    _ensure_builtins()
    try:
        return _REGISTRY[str(name).upper()]
    except KeyError as exc:
        raise ValueError(
            f"unsupported robot type {name!r}; available={sorted(_REGISTRY)}"
        ) from exc


def list_robot_types() -> Iterable[str]:
    _ensure_builtins()
    return tuple(sorted(_REGISTRY))


def _ensure_builtins() -> None:
    if not _REGISTRY:
        importlib.import_module("dexfull.control.robots.unitree")
