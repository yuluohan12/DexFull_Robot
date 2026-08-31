"""Asynchronous CSV recorder for camera capture and ZMQ send events.

The camera threads must not wait for filesystem I/O.  Producers therefore put
small metadata-only rows on a bounded queue and a dedicated writer thread owns
all CSV files.  JPEG payloads are deliberately not copied or written.
"""

from __future__ import annotations

import csv
import contextlib
import logging
import multiprocessing
import os
import queue
import re
import threading
import time
from pathlib import Path
from typing import Optional

from dexfull.common.paths import PROJECT_ROOT
from dexfull.imaging.timestamp_protocol import ZMQImageFrame


logger = logging.getLogger("DexFull.Tools.CameraFrameRecorder")

DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "tools" / "camera_frame_records"
_STOP = None

_CAPTURE_FIELDS = (
    "sequence",
    "capture_timestamp_ms",
    "record_timestamp_ms",
    "record_timestamp_ns",
    "record_monotonic_ns",
    "sensor_timestamp_ms",
    "width",
    "height",
    "jpeg_bytes",
    "source_frame_number",
    "source_frame_delta",
    "capture_interval_us",
    "sensor_interval_ms",
    "wait_duration_us",
    "jpeg_encode_duration_us",
    "handoff_duration_us",
)

_SEND_FIELDS = (
    "sequence",
    "capture_timestamp_ms",
    "send_timestamp_ms",
    "send_timestamp_ns",
    "send_monotonic_ns",
    "sensor_timestamp_ms",
    "width",
    "height",
    "jpeg_bytes",
    "wire_bytes",
    "zmq_port",
    "capture_to_send_ms",
    "encode_duration_us",
    "socket_send_duration_us",
)


def _safe_stream_name(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("._")
    return cleaned or "unknown_camera"


def _writer_process_entry(session_dir: str, event_queue) -> None:
    """Own all CSV handles in a process isolated from camera capture."""
    # Diagnostics must never preempt the realtime capture process. On Linux a
    # positive nice value is inherited only by this writer process.
    with contextlib.suppress(Exception):
        os.nice(10)
    root = Path(session_dir)
    handles: dict[tuple[str, str], object] = {}
    writers: dict[tuple[str, str], csv.DictWriter] = {}
    last_flush = time.monotonic()
    try:
        while True:
            item = event_queue.get()
            if item is _STOP:
                break
            stream, event, row = item
            key = (stream, event)
            writer = writers.get(key)
            if writer is None:
                fields = _CAPTURE_FIELDS if event == "capture" else _SEND_FIELDS
                path = root / f"{stream}_{event}.csv"
                handle = path.open("w", newline="", encoding="utf-8")
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                handles[key] = handle
                writers[key] = writer
            writer.writerow(row)

            # This flush may occasionally block on robot storage, but it now
            # occurs in a separate process and cannot pause RealSense capture.
            now = time.monotonic()
            # A one-second storage write cadence can line up with a 30 FPS
            # stream and look like a periodic camera stall on embedded flash.
            # Keep data reasonably crash-safe without creating a 1 Hz writer.
            if now - last_flush >= 10.0:
                for handle in handles.values():
                    handle.flush()
                last_flush = now
    except Exception:
        logger.exception("camera frame recorder process stopped unexpectedly")
        raise
    finally:
        for handle in handles.values():
            try:
                handle.flush()
                handle.close()
            except Exception:
                logger.exception("failed to close camera frame CSV")


class CameraFrameRecorder:
    """Write per-camera capture/send metadata without blocking realtime loops."""

    def __init__(
        self,
        output_dir: Path | str = DEFAULT_OUTPUT_DIR,
        *,
        enabled: bool = False,
        queue_size: int = 32768,
        session_name: Optional[str] = None,
    ) -> None:
        self.enabled = bool(enabled)
        self.dropped_events = 0
        self._closed = False
        self._queue = None
        self._process = None
        self.session_dir: Optional[Path] = None

        if not self.enabled:
            return

        if not session_name:
            wall_time = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
            session_name = f"camera_send_{wall_time}_{__import__('os').getpid()}"
        self.session_dir = Path(output_dir).expanduser().resolve() / session_name
        self.session_dir.mkdir(parents=True, exist_ok=False)
        context = multiprocessing.get_context("spawn")
        self._queue = context.Queue(maxsize=max(1, int(queue_size)))
        self._process = context.Process(
            target=_writer_process_entry,
            args=(str(self.session_dir), self._queue),
            name="CameraFrameCsvWriter",
            daemon=True,
        )
        self._process.start()
        logger.info(
            "camera frame recording enabled in PID=%s: %s",
            self._process.pid,
            self.session_dir,
        )

    def record_capture(self, frame: ZMQImageFrame) -> None:
        self.record_capture_metadata(
            stream=frame.stream,
            sequence=frame.sequence,
            capture_timestamp_ms=frame.capture_timestamp_ms,
            sensor_timestamp_ms=frame.sensor_timestamp_ms,
            width=frame.width,
            height=frame.height,
            jpeg_bytes=len(frame.jpeg),
        )

    def record_capture_metadata(
        self,
        *,
        stream: str,
        sequence: int,
        capture_timestamp_ms: int,
        sensor_timestamp_ms=None,
        width: int = 0,
        height: int = 0,
        jpeg_bytes: int = 0,
        record_timestamp_ns: Optional[int] = None,
        record_monotonic_ns: Optional[int] = None,
        source_frame_number: Optional[int] = None,
        source_frame_delta: Optional[int] = None,
        capture_interval_us: Optional[float] = None,
        sensor_interval_ms: Optional[float] = None,
        wait_duration_us: Optional[float] = None,
        jpeg_encode_duration_us: Optional[float] = None,
        handoff_duration_us: Optional[float] = None,
    ) -> None:
        if not self.enabled or self._closed:
            return
        now_ns = int(record_timestamp_ns or time.time_ns())
        self._submit(
            stream,
            "capture",
            {
                "sequence": int(sequence),
                "capture_timestamp_ms": int(capture_timestamp_ms),
                "record_timestamp_ms": now_ns // 1_000_000,
                "record_timestamp_ns": now_ns,
                "record_monotonic_ns": int(
                    record_monotonic_ns or time.monotonic_ns()
                ),
                "sensor_timestamp_ms": (
                    "" if sensor_timestamp_ms is None else sensor_timestamp_ms
                ),
                "width": int(width),
                "height": int(height),
                "jpeg_bytes": int(jpeg_bytes),
                "source_frame_number": (
                    "" if source_frame_number is None else int(source_frame_number)
                ),
                "source_frame_delta": (
                    "" if source_frame_delta is None else int(source_frame_delta)
                ),
                "capture_interval_us": (
                    "" if capture_interval_us is None
                    else round(float(capture_interval_us), 3)
                ),
                "sensor_interval_ms": (
                    "" if sensor_interval_ms is None
                    else round(float(sensor_interval_ms), 3)
                ),
                "wait_duration_us": (
                    "" if wait_duration_us is None else round(float(wait_duration_us), 3)
                ),
                "jpeg_encode_duration_us": (
                    "" if jpeg_encode_duration_us is None
                    else round(float(jpeg_encode_duration_us), 3)
                ),
                "handoff_duration_us": (
                    "" if handoff_duration_us is None
                    else round(float(handoff_duration_us), 3)
                ),
            },
        )

    def record_send(
        self,
        frame: ZMQImageFrame,
        *,
        port: int,
        wire_bytes: int,
        send_timestamp_ns: int,
        send_monotonic_ns: int,
        encode_duration_ns: int,
        socket_send_duration_ns: int,
    ) -> None:
        if not self.enabled or self._closed:
            return
        self._submit(
            frame.stream,
            "send",
            {
                "sequence": int(frame.sequence),
                "capture_timestamp_ms": int(frame.capture_timestamp_ms),
                "send_timestamp_ms": int(send_timestamp_ns) // 1_000_000,
                "send_timestamp_ns": int(send_timestamp_ns),
                "send_monotonic_ns": int(send_monotonic_ns),
                "sensor_timestamp_ms": (
                    "" if frame.sensor_timestamp_ms is None else frame.sensor_timestamp_ms
                ),
                "width": int(frame.width),
                "height": int(frame.height),
                "jpeg_bytes": len(frame.jpeg),
                "wire_bytes": int(wire_bytes),
                "zmq_port": int(port),
                "capture_to_send_ms": round(
                    send_timestamp_ns / 1_000_000.0 - frame.capture_timestamp_ms,
                    3,
                ),
                "encode_duration_us": round(int(encode_duration_ns) / 1_000.0, 3),
                "socket_send_duration_us": round(
                    int(socket_send_duration_ns) / 1_000.0,
                    3,
                ),
            },
        )

    def _submit(self, stream: str, event: str, row: dict) -> None:
        if self._queue is None:
            return
        try:
            self._queue.put_nowait((_safe_stream_name(stream), event, row))
        except queue.Full:
            self.dropped_events += 1
            if self.dropped_events == 1 or self.dropped_events % 1000 == 0:
                logger.warning(
                    "camera frame recorder queue full; dropped metadata events=%d",
                    self.dropped_events,
                )

    def close(self, timeout: float = 3.0) -> None:
        if not self.enabled or self._closed:
            return
        self._closed = True
        if self._queue is not None:
            try:
                self._queue.put(_STOP, timeout=max(0.1, float(timeout)))
            except queue.Full:
                logger.warning("camera recorder did not accept shutdown marker")
        if self._process is not None:
            self._process.join(timeout=max(0.1, float(timeout)))
            if self._process.is_alive():
                logger.warning("camera frame recorder did not stop before timeout")
                self._process.terminate()
                self._process.join(timeout=1.0)
            if self._process.exitcode not in (None, 0):
                logger.warning(
                    "camera frame recorder exited with code=%s",
                    self._process.exitcode,
                )
        if self._queue is not None:
            try:
                self._queue.cancel_join_thread()
                self._queue.close()
            except Exception:
                logger.exception("failed to close camera recorder queue")
        logger.info(
            "camera frame recording stopped: path=%s dropped_events=%d",
            self.session_dir,
            self.dropped_events,
        )


_global_lock = threading.Lock()
_global_recorder = CameraFrameRecorder(enabled=False)


def configure_camera_frame_recorder(
    *,
    enabled: bool,
    output_dir: Path | str = DEFAULT_OUTPUT_DIR,
) -> CameraFrameRecorder:
    """Replace the process-global recorder before camera threads are started."""
    global _global_recorder
    with _global_lock:
        _global_recorder.close()
        _global_recorder = CameraFrameRecorder(output_dir, enabled=enabled)
        return _global_recorder


def record_camera_capture(frame: ZMQImageFrame) -> None:
    _global_recorder.record_capture(frame)


def record_camera_capture_metadata(**kwargs) -> None:
    _global_recorder.record_capture_metadata(**kwargs)


def record_camera_send(frame: ZMQImageFrame, **kwargs) -> None:
    _global_recorder.record_send(frame, **kwargs)


def close_camera_frame_recorder() -> None:
    global _global_recorder
    with _global_lock:
        _global_recorder.close()
        _global_recorder = CameraFrameRecorder(enabled=False)
