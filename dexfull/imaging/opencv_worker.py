"""Killable latest-frame OpenCV capture process.

OpenCV's V4L2 ``VideoCapture.read`` has no reliable Python-side timeout and
may block indefinitely after a physical USB fault.  Keeping it in a dedicated
process lets the camera supervisor recover one camera without stopping or
blocking the Teleimager service or another camera.
"""

from __future__ import annotations

import contextlib
import multiprocessing
import queue
import time
from dataclasses import dataclass
from typing import Any, Optional


@dataclass(frozen=True)
class CapturedJpeg:
    jpeg: bytes
    sequence: int
    capture_timestamp_ms: int
    width: int
    height: int


@dataclass(frozen=True)
class CaptureEvent:
    sequence: int
    capture_timestamp_ms: int
    record_timestamp_ns: int
    record_monotonic_ns: int
    width: int
    height: int
    jpeg_bytes: int


def _put_latest(output_queue, value) -> None:
    try:
        while True:
            output_queue.get_nowait()
    except queue.Empty:
        pass
    try:
        output_queue.put_nowait(value)
    except queue.Full:
        # A concurrent consumer/producer race is harmless: bounded latency is
        # more important than preserving an obsolete video frame.
        pass


def _opencv_capture_entry(
    video_path: str,
    image_shape: tuple[int, int],
    fps: float,
    output_queue,
    capture_event_queue,
    status_connection,
    stop_event,
) -> None:
    import cv2
    import numpy as np

    cap = None
    try:
        height, width = (int(image_shape[0]), int(image_shape[1]))
        cap = cv2.VideoCapture(video_path, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open video device {video_path}")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, float(fps))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual = {
            "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": float(cap.get(cv2.CAP_PROP_FPS)),
            "fourcc": int(cap.get(cv2.CAP_PROP_FOURCC)),
        }

        first = None
        for _ in range(20):
            success, frame = cap.read()
            if success and isinstance(frame, np.ndarray) and frame.size > 0:
                first = frame
                break
            if stop_event.wait(0.05):
                return
        if first is None:
            raise RuntimeError("camera failed to produce a valid warm-up frame")
        status_connection.send(("ready", actual))

        frame = first
        failures = 0
        sequence = 0
        while not stop_event.is_set():
            capture_timestamp_ms = time.time_ns() // 1_000_000
            ok, encoded = cv2.imencode(".jpg", frame)
            if ok and encoded is not None and encoded.size > 0:
                sequence += 1
                encoded_bytes = encoded.tobytes()
                frame_height, frame_width = frame.shape[:2]
                event = CaptureEvent(
                    sequence=sequence,
                    capture_timestamp_ms=capture_timestamp_ms,
                    record_timestamp_ns=time.time_ns(),
                    record_monotonic_ns=time.monotonic_ns(),
                    width=int(frame_width),
                    height=int(frame_height),
                    jpeg_bytes=len(encoded_bytes),
                )
                try:
                    capture_event_queue.put_nowait(event)
                except queue.Full:
                    pass
                _put_latest(
                    output_queue,
                    CapturedJpeg(
                        jpeg=encoded_bytes,
                        sequence=sequence,
                        capture_timestamp_ms=capture_timestamp_ms,
                        width=int(frame_width),
                        height=int(frame_height),
                    ),
                )

            success, frame = cap.read()
            if success and isinstance(frame, np.ndarray) and frame.size > 0:
                failures = 0
                continue
            failures += 1
            if failures >= 3:
                raise RuntimeError(f"failed to read {video_path} three times")
            if stop_event.wait(0.01):
                return
    except BaseException as exc:
        with contextlib.suppress(Exception):
            status_connection.send(("error", f"{type(exc).__name__}: {exc}"))
    finally:
        if cap is not None:
            with contextlib.suppress(Exception):
                cap.release()
        with contextlib.suppress(Exception):
            status_connection.close()


class OpenCVCaptureProcess:
    """Own one V4L2 handle in a process that can be terminated on timeout."""

    def __init__(
        self,
        video_path: str,
        image_shape,
        fps: float,
        *,
        startup_timeout: float = 5.0,
        frame_timeout: float = 1.0,
        mp_context=None,
    ) -> None:
        self.video_path = str(video_path)
        self.image_shape = tuple(int(value) for value in image_shape)
        self.fps = float(fps)
        self.startup_timeout = max(0.1, float(startup_timeout))
        self.frame_timeout = max(0.1, float(frame_timeout))
        self._context = mp_context or multiprocessing.get_context("spawn")
        self._queue = self._context.Queue(maxsize=2)
        self._capture_events = self._context.Queue(maxsize=32768)
        self._stop_event = self._context.Event()
        self._status_parent, self._status_child = self._context.Pipe(duplex=False)
        self._process = self._context.Process(
            target=_opencv_capture_entry,
            args=(
                self.video_path,
                self.image_shape,
                self.fps,
                self._queue,
                self._capture_events,
                self._status_child,
                self._stop_event,
            ),
            name=f"OpenCVCapture-{self.video_path.rsplit('/', 1)[-1]}",
            daemon=True,
        )
        self.actual: dict[str, Any] = {}

    @property
    def pid(self) -> Optional[int]:
        return self._process.pid

    def start(self) -> dict[str, Any]:
        self._process.start()
        # Only the child writes status after spawn has duplicated the handle.
        with contextlib.suppress(Exception):
            self._status_child.close()
        deadline = time.monotonic() + self.startup_timeout
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            if self._status_parent.poll(min(0.1, remaining)):
                state, value = self._status_parent.recv()
                if state == "ready":
                    self.actual = dict(value)
                    return self.actual
                raise RuntimeError(value)
            if not self._process.is_alive():
                raise RuntimeError(
                    f"capture process exited during startup (code={self._process.exitcode})"
                )
        self.close()
        raise TimeoutError(
            f"camera {self.video_path} startup timed out after {self.startup_timeout:.1f}s"
        )

    def read(self, timeout: Optional[float] = None) -> CapturedJpeg:
        wait = self.frame_timeout if timeout is None else max(0.1, float(timeout))
        try:
            return self._queue.get(timeout=wait)
        except queue.Empty as exc:
            if not self._process.is_alive():
                detail = f"capture process exited (code={self._process.exitcode})"
                if self._status_parent.poll():
                    state, value = self._status_parent.recv()
                    if state == "error":
                        detail = value
                raise RuntimeError(detail) from exc
            raise TimeoutError(
                f"camera {self.video_path} produced no frame for {wait:.1f}s"
            ) from exc

    def drain_capture_events(self) -> list[CaptureEvent]:
        values = []
        while True:
            try:
                values.append(self._capture_events.get_nowait())
            except queue.Empty:
                return values

    def close(self, graceful_timeout: float = 0.3) -> None:
        self._stop_event.set()
        if self._process.pid is not None:
            self._process.join(timeout=max(0.0, float(graceful_timeout)))
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=1.0)
            if self._process.is_alive() and hasattr(self._process, "kill"):
                self._process.kill()
                self._process.join(timeout=1.0)
        with contextlib.suppress(Exception):
            self._status_parent.close()
        with contextlib.suppress(Exception):
            self._status_child.close()
        with contextlib.suppress(Exception):
            self._queue.cancel_join_thread()
            self._queue.close()
        with contextlib.suppress(Exception):
            self._capture_events.cancel_join_thread()
            self._capture_events.close()
