"""Single-stage sampler from the bridge-local state bus to WebSocket queues."""

from __future__ import annotations

import json
import logging
import math

from .telemetry import telemetry_to_unity_packet, validate_unity_packet
from ..common.state_bus import LatestStateBus, Snapshot


logger = logging.getLogger("DexFull.Bridge.Streamer")


class DirectStateStreamer:
    """Converts each new source snapshot at most once.

    Scheduling belongs to DexFullApplication. This object has no worker thread,
    which removes the old sampler -> slot -> broadcaster double scheduling.
    """

    def __init__(self, bus: LatestStateBus):
        self.bus = bus
        self._last_sequences = {"vr": 0, "robot": 0}
        self._joint_names = []
        self.stats = {
            "robot_valid": 0,
            "robot_invalid": 0,
            "vr_valid": 0,
            "vr_invalid": 0,
        }

    def next_robot_packet(self):
        snapshot = self.bus.get("robot")
        if not self.has_new("robot", snapshot):
            return None
        self._last_sequences["robot"] = snapshot.seq

        packet = telemetry_to_unity_packet(snapshot.data)
        if not validate_unity_packet(packet):
            self.stats["robot_invalid"] += 1
            return None
        packet["data"]["source"] = "direct"
        packet["data"]["ipc_online"] = False
        packet["data"]["direct_bus_online"] = True
        self._joint_names = list(packet["data"].get("record_joint_names", []))
        self.stats["robot_valid"] += 1
        return packet

    def next_vr_payload(self):
        snapshot = self.bus.get("vr")
        if not self.has_new("vr", snapshot):
            return None
        self._last_sequences["vr"] = snapshot.seq

        payload = self._extract_vr_payload(snapshot.data)
        if payload is None:
            self.stats["vr_invalid"] += 1
            return None
        self.stats["vr_valid"] += 1
        return payload

    def get_record_joint_names(self) -> list:
        return list(self._joint_names)

    @staticmethod
    def _extract_vr_payload(frame: dict):
        try:
            controller = frame.get("controller_input", {})
            if isinstance(controller, str):
                controller = json.loads(controller or "{}")
            payload = {
                "controller_input": controller,
                "hmd_pose": list(frame.get("hmd_pose", [])),
                "left_controller_pose": list(frame.get("left_controller_pose", [])),
                "right_controller_pose": list(frame.get("right_controller_pose", [])),
                "source_seq": frame.get("seq"),
                "source_timestamp": frame.get("timestamp"),
            }
            for name in ("hmd_pose", "left_controller_pose", "right_controller_pose"):
                pose = [float(value) for value in payload[name]]
                if len(pose) != 7 or not all(math.isfinite(value) for value in pose):
                    return None
                payload[name] = pose
            return payload
        except Exception:
            return None

    def has_new(self, channel: str, snapshot: Snapshot | None) -> bool:
        return snapshot is not None and snapshot.seq != self._last_sequences[channel]

    def reset(self) -> None:
        self._last_sequences = {"vr": 0, "robot": 0}
        self._joint_names = []
