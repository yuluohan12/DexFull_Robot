"""Low-frequency device health exchange between isolated processes.

This channel intentionally carries health transitions only. Realtime images,
commands and telemetry continue to use their existing transports.
"""

from __future__ import annotations

import json
import os
import tempfile
import time
from pathlib import Path
from typing import Iterable


ONLINE = "ONLINE"
DISCONNECTED = "DISCONNECTED"
RECONNECTING = "RECONNECTING"
DISABLED = "DISABLED"


def _safe_name(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in value)


class DeviceStatusWriter:
    def __init__(self, component: str, directory: str | os.PathLike | None = None):
        root = directory or os.environ.get("DEXFULL_DEVICE_STATUS_DIR")
        self.directory = Path(root) if root else None
        self.component = component
        self._last: dict[str, tuple] = {}
        if self.directory is not None:
            self.directory.mkdir(parents=True, exist_ok=True)

    def publish(
        self,
        device: str,
        state: str,
        message: str = "",
        *,
        recoverable: bool = True,
        details: dict | None = None,
        force: bool = False,
    ) -> None:
        if self.directory is None:
            return
        signature = (state, message, recoverable, json.dumps(details or {}, sort_keys=True))
        if not force and self._last.get(device) == signature:
            return
        self._last[device] = signature
        payload = {
            "component": self.component,
            "device": device,
            "state": state,
            "message": message,
            "recoverable": bool(recoverable),
            "pid": os.getpid(),
            "updated_ms": int(time.time() * 1000),
            "details": details or {},
        }
        target = self.directory / f"{_safe_name(self.component)}--{_safe_name(device)}.json"
        fd, temp_name = tempfile.mkstemp(
            prefix=target.name + ".", suffix=".tmp", dir=str(self.directory)
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(payload, stream, ensure_ascii=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_name, target)
        finally:
            try:
                os.unlink(temp_name)
            except FileNotFoundError:
                pass


def read_device_statuses(
    directory: str | os.PathLike, *, components: Iterable[str] | None = None
) -> list[dict]:
    root = Path(directory)
    allowed = set(components or ())
    result = []
    if not root.exists():
        return result
    for path in root.glob("*.json"):
        try:
            with path.open("r", encoding="utf-8") as stream:
                item = json.load(stream)
            if not allowed or item.get("component") in allowed:
                result.append(item)
        except (OSError, ValueError, TypeError):
            # A status read must never interfere with the realtime application.
            continue
    return result
