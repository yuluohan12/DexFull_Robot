from __future__ import annotations

import copy
import logging
import math
import threading
import time

from dexfull.common.dds import dds_status, initialize_dds
from dexfull.control.robots import get_robot_adapter
from dexfull.control.utils.root_pose import RootPoseTransformer
from dexfull.hand_drivers import get_hand_plugin


logger = logging.getLogger("DexFull.Bridge.DDS")


class DdsTelemetryCollector:
    """Bridge-owned robot/hand DDS collection independent from the IK loop."""

    def __init__(self, bus, config: dict, runtime_status=None):
        self.bus = bus
        self.config = config
        self.control_cfg = config.get("control", {})
        self.telemetry_cfg = config.get("telemetry", {})
        self.robot = get_robot_adapter(self.control_cfg.get("robot", "G1_29"))
        self.hand = get_hand_plugin(self.control_cfg.get("hand"))
        self.runtime_status = runtime_status or (lambda: {})
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._threads = []
        self._lowstate = None
        self._root_position = []
        self._root_rotation = []
        self._seq = 0
        self._last_error_log = 0.0
        self._started_at = 0.0
        self._last_no_data_log = 0.0
        self._hand_collector = None
        self._root_transformer = RootPoseTransformer(
            mode=self.control_cfg.get("root_pose_mode", "unity_relative"),
            pelvis_height=float(
                self.control_cfg.get("root_pelvis_height") or self.robot.pelvis_height
            ),
            axis_mapping=self.control_cfg.get("root_axis_mapping", "unitree_to_unity"),
            heading_reference=self.control_cfg.get("root_heading_reference", "initial"),
            vertical_mode=self.control_cfg.get("root_vertical_mode", "filtered"),
            vertical_deadband=float(self.control_cfg.get("root_vertical_deadband", 0.01)),
            vertical_filter_alpha=float(
                self.control_cfg.get("root_vertical_filter_alpha", 0.2)
            ),
        )

    def start(self) -> None:
        if any(thread.is_alive() for thread in self._threads):
            return
        initialize_dds(
            1 if self.control_cfg.get("simulation", False) else 0,
            self.control_cfg.get("network_interface"),
        )
        self._create_subscribers()
        if self.hand is not None and self.hand.collector_factory is not None:
            self._hand_collector = self.hand.collector_factory(
                domain_id=1 if self.control_cfg.get("simulation", False) else 0,
                network_interface=self.control_cfg.get("network_interface"),
            )
            self._hand_collector.start()
        self._stop.clear()
        self._started_at = time.monotonic()
        self._threads = [
            threading.Thread(target=self._publish_loop, name="BridgeTelemetry", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._hand_collector is not None:
            self._hand_collector.stop()
        for thread in self._threads:
            if thread.is_alive():
                thread.join(timeout=1.0)
        for subscriber_name in ("_lowstate_subscriber", "_odom_subscriber"):
            subscriber = getattr(self, subscriber_name, None)
            if subscriber is not None:
                try:
                    subscriber.Close()
                except Exception as exc:
                    self._log_throttled("DDS subscriber close failed: %s", exc)
                setattr(self, subscriber_name, None)
        self._threads = []

    def status(self) -> dict:
        with self._lock:
            status = {
                "running": any(thread.is_alive() for thread in self._threads),
                "robot": self.robot.name,
                "hand": None if self.hand is None else self.hand.name,
                "robot_online": self._lowstate is not None,
                "odom_online": bool(self._root_position and self._root_rotation),
                "seq": self._seq,
            }
        if self._hand_collector is not None:
            stale_after = float(
                self.config.get("runtime", {}).get("device_stale_seconds", 2.0)
            )
            status["hand_devices"] = self._hand_collector.status(stale_after)
        return status

    def _create_subscribers(self):
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
        if self.robot.dds_family == "hg":
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        else:
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
        self._lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._lowstate_subscriber.Init(self._on_lowstate, 1)
        self._odom_subscriber = ChannelSubscriber("rt/odommodestate", SportModeState_)
        self._odom_subscriber.Init(self._on_odom, 1)

    def _on_lowstate(self, message):
        """Receive the newest robot sample without polling an empty DDS reader."""
        if message is None or self._stop.is_set():
            return
        try:
            with self._lock:
                self._lowstate = message
        except Exception as exc:
            self._log_throttled("robot DDS callback failed: %s", exc)

    def _on_odom(self, message):
        """Receive odometry on the SDK listener thread/queue."""
        if message is None or self._stop.is_set():
            return
        try:
            position = list(getattr(message, "position", []) or [])
            imu = getattr(message, "imu_state", None)
            rotation = list(getattr(imu, "quaternion", []) or [])
            if len(position) >= 3 and len(rotation) >= 4:
                position, rotation = self._root_transformer.transform(
                    position[:3], rotation[:4]
                )
                with self._lock:
                    self._root_position = position
                    self._root_rotation = rotation
        except Exception as exc:
            self._log_throttled("odom DDS callback failed: %s", exc)

    def _publish_loop(self):
        hz = max(1.0, float(self.telemetry_cfg.get("robot_hz", 30.0)))
        interval = 1.0 / hz
        next_tick = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if now < next_tick:
                self._stop.wait(next_tick - now)
                continue
            next_tick += interval
            if next_tick < now - interval:
                next_tick = now + interval
            frame = self._snapshot()
            if frame is not None:
                self.bus.publish_robot(frame)
            elif now - self._started_at >= 5.0 and now - self._last_no_data_log >= 10.0:
                interface = dds_status().get("network_interface") or "auto"
                logger.warning(
                    "No rt/lowstate DDS sample received (interface=%s). "
                    "Set control.network_interface to the robot-facing NIC if auto selection is wrong.",
                    interface,
                )
                self._last_no_data_log = now

    def _snapshot(self):
        with self._lock:
            lowstate = self._lowstate
            root_position = list(self._root_position)
            root_rotation = list(self._root_rotation)
        if lowstate is None:
            return None
        states = list(getattr(lowstate, "motor_state", []) or [])
        if not states:
            return None

        def values(attribute):
            result = []
            for index in self.robot.joint_indices:
                value = getattr(states[index], attribute, 0.0)
                try:
                    value = float(value)
                except (TypeError, ValueError):
                    value = 0.0
                result.append(value if math.isfinite(value) else 0.0)
            return result

        hand_state = (
            {"left": {"qpos": []}, "right": {"qpos": []}}
            if self._hand_collector is None
            else self._hand_collector.snapshot()
        )
        runtime = self.runtime_status() or {}
        state = str(runtime.get("state", "")).upper()
        self._seq += 1
        return {
            "version": 2,
            "seq": self._seq,
            "timestamp_ns": time.time_ns(),
            "robot": {
                "root_position": root_position,
                "root_rotation": root_rotation,
                "joint_positions": values("q"),
                "joint_velocities": values("dq"),
                "joint_torques": values("tau_est"),
                "electricity": values("vol"),
            },
            "end_effector": copy.deepcopy(hand_state),
            "metadata": {
                "arm": self.robot.name,
                "ee": "" if self.hand is None else self.hand.name,
                "joint_names": self.robot.joint_names,
                "left_hand_joint_names": [] if self.hand is None else list(self.hand.joint_names_left),
                "right_hand_joint_names": [] if self.hand is None else list(self.hand.joint_names_right),
            },
            "teleop_start": state == "RUNNING",
            "teleop_stop": state in ("PAUSED", "STOPPING", "STOPPED"),
            "ready": state in ("READY", "RUNNING", "PAUSED"),
        }

    def _log_throttled(self, message, *args):
        now = time.monotonic()
        if now - self._last_error_log >= 5.0:
            logger.warning(message, *args)
            self._last_error_log = now
