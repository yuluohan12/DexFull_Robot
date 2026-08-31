from __future__ import annotations

import importlib
import threading
from typing import Dict, Iterable

from .base import HandPlugin


_REGISTRY: Dict[str, HandPlugin] = {}
_BUILTINS_LOADED = False
_BUILTINS_LOCK = threading.RLock()


def register_hand(plugin: HandPlugin) -> None:
    key = plugin.name.lower()
    if key in _REGISTRY:
        raise ValueError(f"hand plugin already registered: {plugin.name}")
    _REGISTRY[key] = plugin


def get_hand_plugin(name) -> HandPlugin | None:
    if name is None or str(name).lower() in ("", "none", "null"):
        return None
    _ensure_builtins()
    key = str(name).lower()
    try:
        return _REGISTRY[key]
    except KeyError as exc:
        raise ValueError(
            f"unsupported hand type {name!r}; available={sorted(_REGISTRY)}"
        ) from exc


def list_hand_types() -> Iterable[str]:
    _ensure_builtins()
    return tuple(sorted(_REGISTRY))


def _ensure_builtins() -> None:
    global _BUILTINS_LOADED
    if _BUILTINS_LOADED:
        return
    with _BUILTINS_LOCK:
        if _BUILTINS_LOADED:
            return
        importlib.import_module("dexfull.hand_drivers.brainco")
        importlib.import_module("dexfull.hand_drivers.unitree")
        importlib.import_module("dexfull.hand_drivers.inspire")
        _BUILTINS_LOADED = True
