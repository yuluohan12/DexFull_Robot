"""Independent latest-frame forwarding for the XR display."""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable


logger = logging.getLogger("DexFull.Control.ImagePump")


class LatestImagePump:
    """Forward the newest camera frame without blocking the arm control clock."""

    def __init__(
        self,
        source: Callable,
        sink: Callable,
        normalize: Callable,
        frequency: float = 30.0,
        name: str = "XRImagePump",
    ):
        self._source = source
        self._sink = sink
        self._normalize = normalize
        self._interval = 1.0 / max(1.0, float(frequency))
        self._name = name
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        self._latest = (None, 0.0)
        self._last_error_log = 0.0

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name=self._name,
            daemon=True,
        )
        self._thread.start()

    def latest(self):
        with self._lock:
            return self._latest

    def close(self, timeout: float = 1.0) -> bool:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(max(0.0, float(timeout)))
        return thread is None or not thread.is_alive()

    def _run(self) -> None:
        next_tick = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if now < next_tick:
                self._stop.wait(next_tick - now)
                continue
            next_tick += self._interval
            if next_tick < now - self._interval:
                next_tick = now + self._interval
            try:
                image, fps = self._normalize(self._source())
                if image is None:
                    continue
                with self._lock:
                    self._latest = (image, fps)
                self._sink(image)
            except Exception as exc:
                error_time = time.monotonic()
                if error_time - self._last_error_log >= 5.0:
                    logger.warning("XR image pump failed: %s", exc)
                    self._last_error_log = error_time
