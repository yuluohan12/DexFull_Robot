"""Minimal process boundary used by the isolated control runtime.

Only the latest Unity-facing VR snapshot crosses this boundary. Robot and
hand telemetry continue to be collected directly from DDS by the bridge.
"""

from __future__ import annotations

import math


_POSE_NAMES = (
    "hmd_pose",
    "left_controller_pose",
    "right_controller_pose",
)
_SIDE_BOOL_FIELDS = (
    "trigger",
    "squeeze",
    "a_button",
    "b_button",
    "thumbstick",
)
_SIDE_FLOAT_FIELDS = (
    "trigger_value",
    "squeeze_value",
)
_POSE_VALUE_COUNT = 21
_CONTROLLER_VALUE_COUNT = 19
_VALUE_COUNT = _POSE_VALUE_COUNT + _CONTROLLER_VALUE_COUNT


class LatestVrSharedState:
    """Fixed-layout, latest-only VR state shared between two processes.

    There is no FIFO and no payload serialization. A writer replaces the
    previous frame under one short process lock; the bridge samples it at its
    own WebSocket frequency.
    """

    def __init__(self, context):
        self._lock = context.Lock()
        self._sequence = context.Value("Q", 0, lock=False)
        self._timestamp = context.Value("d", 0.0, lock=False)
        self._values = context.Array("d", _VALUE_COUNT, lock=False)
        self._updated = context.Event()

    @property
    def updated_event(self):
        return self._updated

    def clear(self) -> None:
        with self._lock:
            self._sequence.value = 0
            self._timestamp.value = 0.0
            for index in range(_VALUE_COUNT):
                self._values[index] = 0.0
        self._updated.clear()

    def publish(self, frame: dict) -> bool:
        encoded = self._encode(frame)
        if encoded is None:
            return False
        sequence, timestamp, values = encoded
        with self._lock:
            self._timestamp.value = timestamp
            self._values[:] = values
            # Publish the sequence last so readers never mistake a partial
            # write for a new frame.
            self._sequence.value = sequence
        self._updated.set()
        return True

    def read_after(self, last_sequence: int = 0) -> dict | None:
        with self._lock:
            sequence = int(self._sequence.value)
            if sequence <= int(last_sequence):
                return None
            timestamp = float(self._timestamp.value)
            values = list(self._values[:])
        self._updated.clear()
        return self._decode(sequence, timestamp, values)

    @staticmethod
    def _encode(frame: dict):
        try:
            sequence = int(frame.get("seq", 0))
            timestamp = float(frame.get("timestamp", 0.0))
            if sequence <= 0 or not math.isfinite(timestamp):
                return None
            values = []
            for name in _POSE_NAMES:
                pose = [float(item) for item in frame.get(name, [])]
                if len(pose) != 7 or not all(math.isfinite(item) for item in pose):
                    return None
                values.extend(pose)

            controller = frame.get("controller_input") or {}
            if not isinstance(controller, dict):
                return None
            values.append(float(bool(controller.get("motion_data_ready", False))))
            for side_name in ("left", "right"):
                side = controller.get(side_name) or {}
                for field in _SIDE_BOOL_FIELDS:
                    values.append(float(bool(side.get(field, False))))
                for field in _SIDE_FLOAT_FIELDS:
                    value = float(side.get(field, 0.0))
                    values.append(value if math.isfinite(value) else 0.0)
                thumbstick = list(side.get("thumbstick_value", [0.0, 0.0]))
                for index in range(2):
                    value = float(thumbstick[index]) if index < len(thumbstick) else 0.0
                    values.append(value if math.isfinite(value) else 0.0)
            if len(values) != _VALUE_COUNT:
                return None
            return sequence, timestamp, values
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _decode(sequence: int, timestamp: float, values: list) -> dict:
        offset = 0
        frame = {"seq": sequence, "timestamp": timestamp}
        for name in _POSE_NAMES:
            frame[name] = values[offset : offset + 7]
            offset += 7

        controller = {"motion_data_ready": bool(values[offset])}
        offset += 1
        for side_name in ("left", "right"):
            side = {}
            for field in _SIDE_BOOL_FIELDS:
                side[field] = bool(values[offset])
                offset += 1
            for field in _SIDE_FLOAT_FIELDS:
                side[field] = float(values[offset])
                offset += 1
            side["thumbstick_value"] = [
                float(values[offset]),
                float(values[offset + 1]),
            ]
            offset += 2
            controller[side_name] = side
        frame["controller_input"] = controller
        return frame
