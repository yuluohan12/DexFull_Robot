"""Thread-safe latest-value bus owned by the bridge process.

The bus intentionally has no FIFO semantics. Control producers publish the
newest immutable snapshot and the WebSocket transport samples it at its own
rate. This keeps control independent from slow clients without serializing
realtime data through an additional FIFO.
"""

from __future__ import annotations

import copy
import time
from dataclasses import dataclass
from threading import RLock
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class Snapshot:
    seq: int
    source_seq: Optional[int]
    source_timestamp: Optional[float]
    published_monotonic: float
    data: Dict[str, Any]

    def to_dict(self) -> dict:
        return {
            "seq": self.seq,
            "source_seq": self.source_seq,
            "source_timestamp": self.source_timestamp,
            "published_monotonic": self.published_monotonic,
            "data": copy.deepcopy(self.data),
        }


class LatestStateBus:
    """Latest-only state exchange inside the bridge process."""

    CHANNELS = ("vr", "robot", "hand", "health")

    def __init__(self):
        self._lock = RLock()
        self._sequences = {name: 0 for name in self.CHANNELS}
        self._values: Dict[str, Optional[Snapshot]] = {
            name: None for name in self.CHANNELS
        }

    def publish(
        self,
        channel: str,
        data: dict,
        *,
        source_seq: Optional[int] = None,
        source_timestamp: Optional[float] = None,
    ) -> Snapshot:
        if channel not in self._values:
            raise KeyError(f"unknown state channel: {channel}")
        if not isinstance(data, dict):
            raise TypeError("state bus payload must be a dict")

        with self._lock:
            self._sequences[channel] += 1
            snapshot = Snapshot(
                seq=self._sequences[channel],
                source_seq=source_seq,
                source_timestamp=source_timestamp,
                published_monotonic=time.monotonic(),
                data=copy.deepcopy(data),
            )
            self._values[channel] = snapshot
            return snapshot

    def publish_vr(self, frame: dict) -> Snapshot:
        return self.publish(
            "vr",
            frame,
            source_seq=_optional_int(frame.get("seq")),
            source_timestamp=_optional_float(frame.get("timestamp")),
        )

    def publish_robot(self, frame: dict) -> Snapshot:
        return self.publish(
            "robot",
            frame,
            source_seq=_optional_int(frame.get("seq")),
            source_timestamp=_optional_float(frame.get("timestamp")),
        )

    def publish_hand(self, frame: dict) -> Snapshot:
        return self.publish(
            "hand",
            frame,
            source_seq=_optional_int(frame.get("seq")),
            source_timestamp=_optional_float(frame.get("timestamp")),
        )

    def publish_health(self, frame: dict) -> Snapshot:
        return self.publish("health", frame)

    def get(self, channel: str) -> Optional[Snapshot]:
        if channel not in self._values:
            raise KeyError(f"unknown state channel: {channel}")
        with self._lock:
            value = self._values[channel]
            if value is None:
                return None
            return Snapshot(
                seq=value.seq,
                source_seq=value.source_seq,
                source_timestamp=value.source_timestamp,
                published_monotonic=value.published_monotonic,
                data=copy.deepcopy(value.data),
            )

    def snapshot(self) -> dict:
        with self._lock:
            return {
                name: None if value is None else value.to_dict()
                for name, value in self._values.items()
            }

    def diagnostics(self) -> dict:
        """Return timing/sequence metadata without copying telemetry payloads."""
        now = time.monotonic()
        with self._lock:
            return {
                name: (
                    None
                    if value is None
                    else {
                        "seq": value.seq,
                        "source_seq": value.source_seq,
                        "age_ms": max(0.0, (now - value.published_monotonic) * 1000.0),
                    }
                )
                for name, value in self._values.items()
            }

    def clear_realtime(self) -> None:
        with self._lock:
            for channel in ("vr", "robot", "hand"):
                self._values[channel] = None

    def clear_channel(self, channel: str) -> None:
        if channel not in self._values:
            raise KeyError(f"unknown state channel: {channel}")
        with self._lock:
            self._values[channel] = None


def _optional_int(value) -> Optional[int]:
    try:
        return None if value is None else int(value)
    except (TypeError, ValueError):
        return None


def _optional_float(value) -> Optional[float]:
    try:
        return None if value is None else float(value)
    except (TypeError, ValueError):
        return None
