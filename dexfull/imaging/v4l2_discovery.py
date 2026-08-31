"""Non-invasive V4L2 node discovery helpers.

Runtime recovery must never open a healthy camera merely to identify another
camera.  These helpers inspect sysfs only; an explicit camera-open validation
can still be performed by the commissioning CLI.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable, Optional


DEFAULT_SYSFS_ROOT = Path("/sys/class/video4linux")


def _read_text(path: Path) -> Optional[str]:
    try:
        return path.read_text(encoding="utf-8", errors="ignore").strip()
    except OSError:
        return None


def video_node_index(
    video_path: str,
    *,
    sysfs_root: Path = DEFAULT_SYSFS_ROOT,
) -> Optional[int]:
    value = _read_text(sysfs_root / Path(video_path).name / "index")
    try:
        return int(value) if value is not None else None
    except ValueError:
        return None


def video_node_name(
    video_path: str,
    *,
    sysfs_root: Path = DEFAULT_SYSFS_ROOT,
) -> str:
    return _read_text(sysfs_root / Path(video_path).name / "name") or ""


def is_primary_capture_node(
    video_path: str,
    *,
    sysfs_root: Path = DEFAULT_SYSFS_ROOT,
) -> bool:
    """Prefer UVC index 0 and reject explicit metadata-only nodes."""
    name = video_node_name(video_path, sysfs_root=sysfs_root).lower()
    if "metadata" in name:
        return False
    index = video_node_index(video_path, sysfs_root=sysfs_root)
    return index in (None, 0)


def select_primary_video_path(
    candidates: Iterable[str],
    *,
    sysfs_root: Path = DEFAULT_SYSFS_ROOT,
) -> Optional[str]:
    values = sorted(set(str(value) for value in candidates))
    if not values:
        return None
    preferred = [
        value
        for value in values
        if is_primary_capture_node(value, sysfs_root=sysfs_root)
    ]
    if len(preferred) == 1:
        return preferred[0]
    if not preferred and len(values) == 1:
        return values[0]
    if len(preferred) > 1:
        # Multiple index-0 nodes for one exact interface is genuinely
        # ambiguous. Refuse to guess rather than opening every active camera.
        raise ValueError(f"Multiple primary video nodes found: {preferred}")
    raise ValueError(f"Ambiguous video nodes: {values}")


def resolve_physical_video_path(
    physical_path: str,
    *,
    sysfs_root: Path = DEFAULT_SYSFS_ROOT,
) -> Optional[str]:
    """Resolve an exact sysfs device path without opening a V4L2 device."""
    target = os.path.realpath(str(physical_path))
    if not sysfs_root.exists():
        return None
    matches = []
    for node in sysfs_root.iterdir():
        if not node.name.startswith("video"):
            continue
        if os.path.realpath(str(node / "device")) == target:
            matches.append(f"/dev/{node.name}")
    return select_primary_video_path(matches, sysfs_root=sysfs_root)

