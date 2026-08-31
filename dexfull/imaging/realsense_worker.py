"""Killable, latest-frame RealSense capture process.

The RealSense pipeline is deliberately owned by a child process.  This keeps
device waits, frame-wrapper destruction, cyclic GC and JPEG encoding out of the
Teleimager process that supervises cameras and publishes ZMQ/WebRTC streams.
Only the newest encoded frame is retained, so a slow consumer cannot build an
ever-growing latency backlog.
"""

from __future__ import annotations

import contextlib
import multiprocessing
import queue
import time
from dataclasses import dataclass
from typing import Any, Optional


@dataclass(frozen=True)
class CapturedRealSenseFrame:
    jpeg: bytes
    sequence: int
    capture_timestamp_ms: int
    sensor_timestamp_ms: Optional[float]
    width: int
    height: int
    depth_bytes: Optional[bytes] = None


@dataclass(frozen=True)
class RealSenseCaptureEvent:
    sequence: int
    capture_timestamp_ms: int
    sensor_timestamp_ms: Optional[float]
    record_timestamp_ns: int
    record_monotonic_ns: int
    width: int
    height: int
    jpeg_bytes: int
    source_frame_number: Optional[int] = None
    source_frame_delta: Optional[int] = None
    capture_interval_us: Optional[float] = None
    sensor_interval_ms: Optional[float] = None
    wait_duration_us: Optional[float] = None
    jpeg_encode_duration_us: Optional[float] = None
    handoff_duration_us: Optional[float] = None


def _put_latest(output_queue, value) -> None:
    """Replace any queued stale frame without blocking the capture loop."""
    try:
        while True:
            output_queue.get_nowait()
    except queue.Empty:
        pass
    try:
        output_queue.put_nowait(value)
    except queue.Full:
        # multiprocessing.Queue feeder timing can transiently report full even
        # after get_nowait().  Dropping this frame is safer than adding latency.
        pass


def _intrinsics_to_dict(intrinsics) -> dict[str, Any]:
    return {
        "width": int(intrinsics.width),
        "height": int(intrinsics.height),
        "ppx": float(intrinsics.ppx),
        "ppy": float(intrinsics.ppy),
        "fx": float(intrinsics.fx),
        "fy": float(intrinsics.fy),
        "model": str(intrinsics.model),
        "coeffs": [float(value) for value in intrinsics.coeffs],
    }


def _realsense_capture_entry(
    serial_number: str,
    image_shape: tuple[int, int],
    fps: float,
    enable_depth: bool,
    frame_timeout: float,
    disable_cyclic_gc: bool,
    cpu_affinity,
    output_queue,
    capture_event_queue,
    status_connection,
    stop_event,
) -> None:
    pipeline = None
    try:
        # Imports stay in the child: importing pyrealsense2 in the supervisor
        # must not create device/runtime state that is later inherited.
        import cv2
        import numpy as np
        import pyrealsense2 as rs
        from dexfull.common.realtime_affinity import apply_current_process_affinity

        active_cpus = apply_current_process_affinity(cpu_affinity, role="camera")

        if disable_cyclic_gc:
            # RealSense frame wrappers are reference-counted and are released
            # explicitly below.  Automatic cyclic collections in this small,
            # dedicated process only introduce count-based capture jitter.
            import gc

            gc.disable()

        height, width = (int(image_shape[0]), int(image_shape[1]))
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(str(serial_number))
        config.enable_stream(
            rs.stream.color, width, height, rs.format.bgr8, int(fps)
        )
        if enable_depth:
            config.enable_stream(
                rs.stream.depth, width, height, rs.format.z16, int(fps)
            )

        # Supplying an explicit capacity-1 SDK queue bypasses the pipeline's
        # synchronous wait/sync delivery path and makes latest-frame policy
        # unambiguous.  This is important on Jetson, where RGB delivery can
        # otherwise arrive in periodic bursts even while device timestamps are
        # advancing normally.
        try:
            sdk_frame_queue = rs.frame_queue(1, keep_frames=False)
        except TypeError:
            # Older pyrealsense2 builds expose only the capacity argument.
            sdk_frame_queue = rs.frame_queue(1)
        profile = pipeline.start(config, sdk_frame_queue)
        device = profile.get_device()
        color_sensor = None
        try:
            color_sensor = device.first_color_sensor()
        except Exception:
            for candidate in device.query_sensors():
                try:
                    if candidate.supports(rs.option.auto_exposure_priority):
                        color_sensor = candidate
                        break
                except Exception:
                    continue

        sensor_options = {}
        if color_sensor is not None:
            for option, value, name in (
                (rs.option.frames_queue_size, 1.0, "frames_queue_size"),
                # This option explicitly permits the RGB sensor to vary FPS.
                # Teleoperation requires fixed cadence, so keep it disabled.
                (rs.option.auto_exposure_priority, 0.0, "auto_exposure_priority"),
            ):
                try:
                    if color_sensor.supports(option):
                        color_sensor.set_option(option, value)
                        sensor_options[name] = float(color_sensor.get_option(option))
                except Exception as exc:
                    sensor_options[name] = f"unsupported: {exc}"
        depth_scale = None
        if enable_depth:
            depth_scale = float(device.first_depth_sensor().get_depth_scale())
        intrinsics = (
            profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )
        actual = {
            "width": width,
            "height": height,
            "fps": float(fps),
            "intrinsics": _intrinsics_to_dict(intrinsics),
            "depth_scale": depth_scale,
            "delivery": "sdk_frame_queue",
            "sdk_queue_capacity": 1,
            "sensor_options": sensor_options,
            "cpu_affinity": list(active_cpus),
        }
        status_connection.send(("ready", actual))

        align = rs.align(rs.stream.color) if enable_depth else None
        timeout_ms = max(100, int(float(frame_timeout) * 1000))
        sequence = 0
        previous_capture_ns = None
        previous_sensor_timestamp_ms = None
        previous_source_frame_number = None
        while not stop_event.is_set():
            wait_started_ns = time.monotonic_ns()
            sdk_frame = sdk_frame_queue.wait_for_frame(timeout_ms)
            wait_finished_ns = time.monotonic_ns()
            frames = sdk_frame.as_frameset()
            aligned_frames = align.process(frames) if align is not None else frames
            color_frame = aligned_frames.get_color_frame()
            if not color_frame:
                continue

            # Timestamp immediately after the SDK hands the frame to Python.
            capture_timestamp_ms = time.time_ns() // 1_000_000
            try:
                sensor_timestamp_ms = float(color_frame.get_timestamp())
            except Exception:
                sensor_timestamp_ms = None
            try:
                source_frame_number = int(color_frame.get_frame_number())
            except Exception:
                source_frame_number = None
            capture_interval_us = (
                None
                if previous_capture_ns is None
                else (wait_finished_ns - previous_capture_ns) / 1_000.0
            )
            sensor_interval_ms = (
                None
                if sensor_timestamp_ms is None or previous_sensor_timestamp_ms is None
                else sensor_timestamp_ms - previous_sensor_timestamp_ms
            )
            source_frame_delta = (
                None
                if source_frame_number is None or previous_source_frame_number is None
                else source_frame_number - previous_source_frame_number
            )

            bgr_numpy = np.asanyarray(color_frame.get_data())
            encode_started_ns = time.monotonic_ns()
            ok, encoded = cv2.imencode(".jpg", bgr_numpy)
            encode_finished_ns = time.monotonic_ns()
            if not ok or encoded is None or encoded.size == 0:
                continue

            sequence += 1
            encoded_bytes = encoded.tobytes()
            depth_bytes = None
            if enable_depth:
                depth_frame = aligned_frames.get_depth_frame()
                if depth_frame:
                    depth_bytes = np.asanyarray(depth_frame.get_data()).tobytes()

            handoff_started_ns = time.monotonic_ns()
            _put_latest(
                output_queue,
                CapturedRealSenseFrame(
                    jpeg=encoded_bytes,
                    sequence=sequence,
                    capture_timestamp_ms=capture_timestamp_ms,
                    sensor_timestamp_ms=sensor_timestamp_ms,
                    width=width,
                    height=height,
                    depth_bytes=depth_bytes,
                ),
            )
            handoff_finished_ns = time.monotonic_ns()
            event = RealSenseCaptureEvent(
                sequence=sequence,
                capture_timestamp_ms=capture_timestamp_ms,
                sensor_timestamp_ms=sensor_timestamp_ms,
                record_timestamp_ns=time.time_ns(),
                record_monotonic_ns=handoff_finished_ns,
                width=width,
                height=height,
                jpeg_bytes=len(encoded_bytes),
                source_frame_number=source_frame_number,
                source_frame_delta=source_frame_delta,
                capture_interval_us=capture_interval_us,
                sensor_interval_ms=sensor_interval_ms,
                wait_duration_us=(wait_finished_ns - wait_started_ns) / 1_000.0,
                jpeg_encode_duration_us=(
                    encode_finished_ns - encode_started_ns
                ) / 1_000.0,
                handoff_duration_us=(
                    handoff_finished_ns - handoff_started_ns
                ) / 1_000.0,
            )
            try:
                capture_event_queue.put_nowait(event)
            except queue.Full:
                pass

            previous_capture_ns = wait_finished_ns
            previous_sensor_timestamp_ms = sensor_timestamp_ms
            previous_source_frame_number = source_frame_number

            # Ensure C++ frame handles are decref'd every iteration even with
            # automatic cyclic GC disabled.
            del encoded, bgr_numpy, color_frame, aligned_frames, frames, sdk_frame
    except BaseException as exc:
        with contextlib.suppress(Exception):
            status_connection.send(("error", f"{type(exc).__name__}: {exc}"))
    finally:
        if pipeline is not None:
            with contextlib.suppress(Exception):
                pipeline.stop()
        with contextlib.suppress(Exception):
            status_connection.close()


class RealSenseCaptureProcess:
    """Own a RealSense pipeline in a process that can be killed on timeout."""

    def __init__(
        self,
        serial_number: str,
        image_shape,
        fps: float,
        *,
        enable_depth: bool = False,
        startup_timeout: float = 8.0,
        frame_timeout: float = 1.0,
        disable_cyclic_gc: bool = True,
        cpu_affinity="auto",
        mp_context=None,
    ) -> None:
        self.serial_number = str(serial_number)
        self.image_shape = tuple(int(value) for value in image_shape)
        self.fps = float(fps)
        self.enable_depth = bool(enable_depth)
        self.startup_timeout = max(0.1, float(startup_timeout))
        self.frame_timeout = max(0.1, float(frame_timeout))
        self.disable_cyclic_gc = bool(disable_cyclic_gc)
        self.cpu_affinity = cpu_affinity
        self._context = mp_context or multiprocessing.get_context("spawn")
        self._queue = self._context.Queue(maxsize=2)
        self._capture_events = self._context.Queue(maxsize=32768)
        self._stop_event = self._context.Event()
        self._status_parent, self._status_child = self._context.Pipe(duplex=False)
        self._process = self._context.Process(
            target=_realsense_capture_entry,
            args=(
                self.serial_number,
                self.image_shape,
                self.fps,
                self.enable_depth,
                self.frame_timeout,
                self.disable_cyclic_gc,
                self.cpu_affinity,
                self._queue,
                self._capture_events,
                self._status_child,
                self._stop_event,
            ),
            name=f"RealSenseCapture-{self.serial_number}",
            daemon=True,
        )
        self.actual: dict[str, Any] = {}

    @property
    def pid(self) -> Optional[int]:
        return self._process.pid

    def start(self) -> dict[str, Any]:
        self._process.start()
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
                    "RealSense capture process exited during startup "
                    f"(code={self._process.exitcode})"
                )
        self.close()
        raise TimeoutError(
            f"RealSense {self.serial_number} startup timed out after "
            f"{self.startup_timeout:.1f}s"
        )

    def read(self, timeout: Optional[float] = None) -> CapturedRealSenseFrame:
        wait = self.frame_timeout if timeout is None else max(0.1, float(timeout))
        try:
            return self._queue.get(timeout=wait)
        except queue.Empty as exc:
            if not self._process.is_alive():
                detail = (
                    "RealSense capture process exited "
                    f"(code={self._process.exitcode})"
                )
                if self._status_parent.poll():
                    state, value = self._status_parent.recv()
                    if state == "error":
                        detail = value
                raise RuntimeError(detail) from exc
            raise TimeoutError(
                f"RealSense {self.serial_number} produced no frame for {wait:.1f}s"
            ) from exc

    def drain_capture_events(self) -> list[RealSenseCaptureEvent]:
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
