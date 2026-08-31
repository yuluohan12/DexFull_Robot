"""Latest-target asynchronous IK execution for realtime control."""

from __future__ import annotations

import copy
import logging
import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable


logger = logging.getLogger("DexFull.Control.IKWorker")


@dataclass(frozen=True)
class IkResult:
    sequence: int
    q: Any
    tauff: Any
    solve_ms: float


class LatestIkWorker:
    """Run one IK solve at a time and retain only the newest pending target.

    Nonlinear IK may take longer than the XR sampling period. A FIFO would
    therefore make the robot execute stale poses. This worker deliberately
    replaces an unstarted request with the newest one.
    """

    def __init__(
        self,
        solve: Callable[..., tuple],
        name: str = "DexFullIK",
        cpu_affinity=None,
    ):
        self._solve = solve
        self._name = name
        self._requested_cpu_affinity = cpu_affinity
        self._active_cpu_affinity = None
        self._condition = threading.Condition()
        self._stop = False
        self._thread = None
        self._next_sequence = 0
        self._pending = None
        self._latest_result = None
        self._stats = self._empty_stats()

    @staticmethod
    def _empty_stats():
        return {
            "submitted": 0,
            "solved": 0,
            "replaced": 0,
            "failed": 0,
            "total_solve_ms": 0.0,
            "max_solve_ms": 0.0,
        }

    def start(self) -> None:
        with self._condition:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop = False
            self._thread = threading.Thread(
                target=self._run,
                name=self._name,
                daemon=True,
            )
            self._thread.start()

    def submit(self, left_wrist, right_wrist, current_q, current_dq) -> int:
        request = (
            self._copy(left_wrist),
            self._copy(right_wrist),
            self._copy(current_q),
            self._copy(current_dq),
        )
        with self._condition:
            if self._stop:
                return self._next_sequence
            self._next_sequence += 1
            if self._pending is not None:
                self._stats["replaced"] += 1
            self._pending = (self._next_sequence, request)
            self._stats["submitted"] += 1
            self._condition.notify()
            return self._next_sequence

    def latest_after(self, sequence: int) -> IkResult | None:
        with self._condition:
            result = self._latest_result
            if result is None or result.sequence <= sequence:
                return None
            return result

    def drain_stats(self) -> dict:
        with self._condition:
            stats = dict(self._stats)
            self._stats = self._empty_stats()
        solved = stats["solved"]
        stats["average_solve_ms"] = (
            stats["total_solve_ms"] / solved if solved else 0.0
        )
        return stats

    def close(self, timeout: float = 1.0) -> bool:
        with self._condition:
            self._stop = True
            self._pending = None
            self._condition.notify_all()
            thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(max(0.0, float(timeout)))
        return thread is None or not thread.is_alive()

    def _run(self) -> None:
        self._apply_cpu_affinity()
        while True:
            with self._condition:
                while self._pending is None and not self._stop:
                    self._condition.wait()
                if self._stop:
                    return
                sequence, request = self._pending
                self._pending = None

            started = time.perf_counter()
            try:
                q, tauff = self._solve(*request)
            except BaseException:
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                with self._condition:
                    self._stats["failed"] += 1
                    self._stats["max_solve_ms"] = max(
                        self._stats["max_solve_ms"], elapsed_ms
                    )
                logger.exception("IK solve failed")
                continue

            elapsed_ms = (time.perf_counter() - started) * 1000.0
            result = IkResult(sequence, q, tauff, elapsed_ms)
            with self._condition:
                self._latest_result = result
                self._stats["solved"] += 1
                self._stats["total_solve_ms"] += elapsed_ms
                self._stats["max_solve_ms"] = max(
                    self._stats["max_solve_ms"], elapsed_ms
                )

    @property
    def active_cpu_affinity(self):
        return self._active_cpu_affinity

    def _apply_cpu_affinity(self) -> None:
        requested = self._resolve_cpu_affinity(self._requested_cpu_affinity)
        if not requested:
            return
        setter = getattr(os, "sched_setaffinity", None)
        if setter is None:
            logger.debug("Per-thread CPU affinity is unavailable on this platform")
            return
        try:
            setter(threading.get_native_id(), set(requested))
            self._active_cpu_affinity = tuple(requested)
            logger.info("IK worker CPU affinity set to %s", list(requested))
        except (OSError, ValueError) as exc:
            logger.warning("Failed to set IK worker CPU affinity %s: %s", requested, exc)

    @staticmethod
    def _resolve_cpu_affinity(value):
        if value is None or value is False:
            return ()
        available = list(range(max(1, int(os.cpu_count() or 1))))
        getter = getattr(os, "sched_getaffinity", None)
        if getter is not None:
            try:
                available = sorted(int(cpu) for cpu in getter(0))
            except (OSError, ValueError):
                pass
        if not available:
            return ()
        if isinstance(value, str):
            normalized = value.strip().lower()
            if not normalized or normalized in {"off", "none", "false"}:
                return ()
            if normalized == "auto":
                # Keep the lower half available to the asyncio/DDS/control
                # threads and reserve the final CPU for RealSense capture.
                upper = available[len(available) // 2 :]
                return tuple(upper[:-1] or upper)             
            value = [part.strip() for part in value.split(",") if part.strip()]
        try:
            selected = sorted({int(cpu) for cpu in value})
        except (TypeError, ValueError):
            logger.warning("Ignoring invalid IK CPU affinity: %r", value)
            return ()
        allowed = set(available)
        return tuple(cpu for cpu in selected if cpu in allowed)

    @staticmethod
    def _copy(value):
        if value is None:
            return None
        copier = getattr(value, "copy", None)
        return copier() if callable(copier) else copy.deepcopy(value)
