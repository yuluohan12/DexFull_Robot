"""Small, Linux-safe CPU-affinity helpers for latency-sensitive workers."""

from __future__ import annotations

import os
import threading
from collections.abc import Iterable


def resolve_cpu_affinity(value, *, role: str = "compute") -> tuple[int, ...]:
    """Resolve an explicit CPU list or a conservative ``auto`` placement.

    ``compute`` workers use the last two CPUs in the lower half of the current
    cpuset.  DexFull's IK worker uses the upper half, so the two optimizers do
    not compete on the same cores.  ``io`` workers use another lower-half CPU
    and avoid CPU 0 when the cpuset is large enough; CPU 0 commonly handles
    kernel, IRQ and DDS housekeeping on the robot. ``camera`` reserves the
    final CPU for the RealSense SDK delivery/JPEG process.
    """
    if value is None or value is False:
        return ()
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("", "none", "off", "false", "disabled"):
            return ()
        if normalized != "auto":
            value = [part.strip() for part in value.split(",") if part.strip()]

    available = _available_cpus()
    if isinstance(value, str) and value.strip().lower() == "auto":
        if len(available) <= 2:
            return tuple(available)
        if role == "camera":
            return (available[-1],)
        lower = available[: max(1, len(available) // 2)]
        if role == "io":
            return (lower[1] if len(lower) >= 4 else lower[0],)
        if len(lower) == 2:
            return (lower[-1],)
        return tuple(lower[-min(2, len(lower)):])

    try:
        requested = sorted({int(cpu) for cpu in _iter_values(value)})
    except (TypeError, ValueError):
        return ()
    return tuple(cpu for cpu in requested if cpu in available)


def apply_current_process_affinity(value, *, role="compute") -> tuple[int, ...]:
    """Pin the current single-threaded worker process when supported."""
    return _apply_affinity(0, value, role=role)


def apply_current_thread_affinity(value, *, role="io") -> tuple[int, ...]:
    """Pin only the calling native thread when supported by the OS."""
    native_id = getattr(threading, "get_native_id", lambda: 0)()
    return _apply_affinity(native_id, value, role=role)


def _apply_affinity(target: int, value, *, role: str) -> tuple[int, ...]:
    cpus = resolve_cpu_affinity(value, role=role)
    setter = getattr(os, "sched_setaffinity", None)
    if not cpus or setter is None:
        return ()
    try:
        setter(target, set(cpus))
    except (OSError, ValueError):
        return ()
    return cpus


def _available_cpus() -> list[int]:
    getter = getattr(os, "sched_getaffinity", None)
    if getter is not None:
        try:
            cpus = sorted(int(cpu) for cpu in getter(0))
            if cpus:
                return cpus
        except (OSError, ValueError):
            pass
    return list(range(max(1, int(os.cpu_count() or 1))))


def _iter_values(value) -> Iterable:
    if isinstance(value, (int, float)):
        return (value,)
    return value
