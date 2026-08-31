from __future__ import annotations

import argparse
import asyncio
import logging
import time

from . import __version__
from .common.logging_mp_config import configure_logging_mp
from .common.runtime_tuning import configure_native_threads

# Must run before importing collectors/control/hand plugins: several of those
# modules create a logging_mp logger at import time.
configure_native_threads()
configure_logging_mp()

from .bridge.collectors import DdsTelemetryCollector
from .bridge.controller import UnityController
from .bridge.process_manager import ProcessManager
from .bridge.streamer import DirectStateStreamer
from .bridge.ws_server import TeleopWebSocketServer
from .common.state_bus import LatestStateBus
from .common.device_status import ONLINE, read_device_statuses
from .common.paths import STATUS_DIR
from .common.ws_security import WebSocketTlsPolicy
from .config import CONFIG
from .control.runtime import ControlRuntime
from .hand_drivers import HandDriverManager


logger = logging.getLogger("DexFull")


class DexFullApplication:
    def __init__(self, config=None, collector=None, runtime=None):
        self.config = config or CONFIG
        self.bus = LatestStateBus()
        self.process_manager = ProcessManager(self.config.get("processes", {}))
        self.hand_drivers = HandDriverManager(
            self.process_manager,
            self.config.get("hand_drivers", {}),
            self.config["control"].get("hand"),
        )
        self.runtime = runtime or ControlRuntime(self.bus, self._build_control_args)
        self.collector = collector or DdsTelemetryCollector(
            self.bus,
            self.config,
            runtime_status=self.runtime.status,
        )
        self.streamer = DirectStateStreamer(self.bus)
        ws = self.config["ws"]
        tls_policy = WebSocketTlsPolicy.from_ws_config(
            ws,
            config_path=self.config.get("config_path"),
        )
        self.ws_server = TeleopWebSocketServer(
            ws.get("host", "0.0.0.0"),
            int(ws.get("port", 7443)),
            ws.get("path", "/ws"),
            compression=bool(ws.get("compression", False)),
            tls_policy=tls_policy,
        )
        self.controller = UnityController(
            self.process_manager,
            self.runtime,
            self.hand_drivers,
            self.collector,
            self.streamer,
            self.bus,
            self.config,
        )
        self.ws_server._bridge_call = self.controller.handle
        self.ws_server._process_manager = self.process_manager
        self._running = False
        self._tasks = []
        self._started_monotonic = 0.0
        self._device_states = {}
        self._device_first_seen = {}
        self._known_camera_devices = set()

    async def start(self):
        self._running = True
        self._started_monotonic = time.monotonic()
        self.process_manager.start_monitoring()
        self.collector.start()
        if self.config["runtime"].get("auto_start_teleimager", True):
            result = self.process_manager.start_service("teleimager")
            if result.get("status") != "ok":
                logger.warning("teleimager auto-start failed: %s", result)
        await self.ws_server.start()
        self._tasks = [
            asyncio.create_task(self._stream_loop()),
            asyncio.create_task(self._state_loop()),
            asyncio.create_task(self._device_status_loop()),
        ]
        logger.info(
            "DexFull %s ready %s://%s:%s%s (control=spawn)",
            __version__,
            self.ws_server.tls_policy.scheme,
            self.ws_server.host,
            self.ws_server.port,
            self.ws_server.path,
        )
        while self._running:
            await asyncio.sleep(1.0)

    async def shutdown(self):
        if not self._running:
            return
        self._running = False
        await asyncio.to_thread(
            self.runtime.stop_component,
            self.config["runtime"].get("stop_timeout", 15.0),
        )
        self.hand_drivers.stop()
        self.collector.stop()
        self.process_manager.stop_all()
        self.process_manager.stop_monitoring()
        await self.ws_server.stop()
        for task in self._tasks:
            if not task.done():
                task.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        self._tasks = []

    def run(self):
        async def runner():
            try:
                await self.start()
            finally:
                await self.shutdown()
        asyncio.run(runner())

    async def _stream_loop(self):
        telemetry = self.config["telemetry"]
        robot_enabled = bool(telemetry.get("enable_robot_ws", True))
        vr_enabled = bool(telemetry.get("enable_vr_ws", True))
        robot_interval = 1.0 / max(1.0, float(telemetry.get("robot_hz", 30.0)))
        vr_interval = 1.0 / max(1.0, float(telemetry.get("vr_hz", 30.0)))
        now = time.monotonic()
        next_robot = next_vr = now
        while self._running:
            now = time.monotonic()
            deadlines = []
            if robot_enabled:
                deadlines.append(next_robot)
            if vr_enabled:
                deadlines.append(next_vr)
            if not deadlines:
                await asyncio.sleep(0.25)
                continue
            deadline = min(deadlines)
            if now < deadline:
                await asyncio.sleep(deadline - now)
                continue
            try:
                if robot_enabled and now >= next_robot:
                    next_robot = self._advance(next_robot, robot_interval, now)
                    packet = self.streamer.next_robot_packet()
                    if packet is not None:
                        await self.ws_server.broadcast_robot_stream(packet)
                if vr_enabled and now >= next_vr:
                    next_vr = self._advance(next_vr, vr_interval, now)
                    payload = self.streamer.next_vr_payload()
                    if payload is not None:
                        await self.ws_server.broadcast_vr_input(payload)
            except Exception:
                logger.exception("stream iteration failed")

    async def _state_loop(self):
        previous = None
        while self._running:
            current = self.runtime.state.value
            if previous is not None and current != previous:
                await self.ws_server.broadcast_state_change(previous, current)
            previous = current
            await asyncio.sleep(0.1)

    async def _device_status_loop(self):
        runtime_cfg = self.config.get("runtime", {})
        interval = max(
            0.2, float(runtime_cfg.get("device_status_poll_seconds", 0.5))
        )
        grace = max(
            0.0, float(runtime_cfg.get("device_startup_grace_seconds", 8.0))
        )
        while self._running:
            try:
                devices = self._current_device_states()
                now = time.monotonic()
                for key, item in devices.items():
                    first_seen = self._device_first_seen.setdefault(key, now)
                    if (
                        str(item.get("state", "")).upper() == ONLINE
                        or now - first_seen >= grace
                    ):
                        await self._publish_device_transition(key, item)
            except Exception:
                logger.exception("device status iteration failed")
            await asyncio.sleep(interval)

    def _current_device_states(self):
        result = {}
        telemetry = self.collector.status()
        brainco_service = self.process_manager.services.get("brainco")
        if brainco_service is not None and brainco_service.state.name != "STOPPED":
            for side, item in telemetry.get("hand_devices", {}).items():
                device = f"{side}_hand"
                result[("brainco", device)] = {
                    "component": "brainco",
                    "device": device,
                    "state": item.get("state", "DISCONNECTED"),
                    "message": "",
                    "recoverable": True,
                    "details": {"age_seconds": item.get("age_seconds")},
                }

        teleimager = self.process_manager.services.get("teleimager")
        expected_pid = None if teleimager is None else teleimager.pid
        for item in read_device_statuses(STATUS_DIR, components=("camera",)):
            if expected_pid is None or item.get("pid") != expected_pid:
                continue
            key = (str(item.get("component")), str(item.get("device")))
            result[key] = item
            self._known_camera_devices.add(key)
        for key in self._known_camera_devices:
            if key not in result:
                result[key] = {
                    "component": key[0],
                    "device": key[1],
                    "state": "DISCONNECTED",
                    "message": "camera service is unavailable",
                    "recoverable": True,
                    "details": {},
                }

        # A child process crash is also visible immediately, before individual
        # device freshness timers expire. This reports it but never cascades a
        # stop into control or another hardware service.
        for service_name, process in self.process_manager.services.items():
            if process.stop_requested:
                continue
            state = process.state.name
            if state in ("CRASHED", "FATAL", "DEGRADED"):
                result[("service", service_name)] = {
                    "component": "service",
                    "device": service_name,
                    "state": state,
                    "message": process.last_error or "service exited unexpectedly",
                    "recoverable": bool(process.auto_restart),
                    "details": {"pid": process.pid},
                }
        return result

    async def _publish_device_transition(self, key, item):
        state = str(item.get("state", "DISCONNECTED")).upper()
        previous = self._device_states.get(key)
        if previous == state:
            return
        if state != ONLINE and self.ws_server.client_count == 0:
            # Preserve the transition until Unity connects; otherwise an
            # already-missing device would be reported before there is a
            # recipient and never be visible to that Unity session.
            return
        self._device_states[key] = state
        component = str(item.get("component", key[0]))
        device = str(item.get("device", key[1]))
        payload = {
            "code": "DEVICE_RECOVERED" if state == ONLINE else "DEVICE_DISCONNECTED",
            "component": component,
            "device": device,
            "state": state,
            "recoverable": bool(item.get("recoverable", True)),
            "message": str(item.get("message", "")),
            "details": item.get("details") or {},
        }
        if state == ONLINE:
            if previous is None:
                return
            # Recovery is informational and does not change any existing Unity
            # request method or response schema.
            await self.ws_server._broadcast_event("device_state", payload)
            logger.info("device recovered: %s/%s", component, device)
            return
        error_tip = f"设备掉线：{device}，正在等待重新连接"
        await self.ws_server.broadcast_error(error_tip, payload)
        logger.warning("%s (%s/%s: %s)", error_tip, component, device, state)

    @staticmethod
    def _advance(deadline, interval, now):
        deadline += interval
        return now + interval if deadline < now - interval else deadline

    def _build_control_args(self):
        cfg = self.config["control"]
        image_ip = self.controller._resolved_img_server_ip or cfg.get("img_server_ip")
        if self.controller._is_auto(image_ip):
            image_ip = "127.0.0.1"
        return argparse.Namespace(
            frequency=float(cfg.get("frequency", 60.0)),
            input_mode=str(cfg.get("input_mode", "hand")),
            display_mode=str(cfg.get("display_mode", "immersive")),
            arm=str(cfg.get("robot", "G1_29")),
            ee=cfg.get("hand"),
            img_server_ip=str(image_ip),
            network_interface=cfg.get("network_interface"),
            motion=bool(cfg.get("motion", False)),
            headless=bool(cfg.get("headless", True)),
            sim=bool(cfg.get("simulation", False)),
            affinity=bool(cfg.get("affinity", False)),
            record=bool(cfg.get("record", False)),
            async_ik=bool(cfg.get("async_ik", True)),
            ik_max_iterations=int(cfg.get("ik_max_iterations", 12)),
            ik_cpu_affinity=cfg.get("ik_cpu_affinity", "auto"),
            hand_retarget_cpu_affinity=cfg.get(
                "hand_retarget_cpu_affinity", "auto"
            ),
            hand_dds_cpu_affinity=cfg.get("hand_dds_cpu_affinity", "auto"),
            performance_log_interval=float(
                cfg.get("performance_log_interval", 5.0)
            ),
            task_dir=str(cfg.get("task_dir", "./data")),
            task_name=str(cfg.get("task_name", "teleop")),
            task_goal=str(cfg.get("task_goal", "")),
            task_desc=str(cfg.get("task_desc", "")),
            task_steps=str(cfg.get("task_steps", "")),
            root_pose_mode=cfg.get("root_pose_mode", "unity_relative"),
            root_pelvis_height=cfg.get("root_pelvis_height"),
            root_axis_mapping=cfg.get("root_axis_mapping", "unitree_to_unity"),
            root_heading_reference=cfg.get("root_heading_reference", "initial"),
            root_vertical_mode=cfg.get("root_vertical_mode", "filtered"),
            root_vertical_deadband=float(cfg.get("root_vertical_deadband", 0.01)),
            root_vertical_filter_alpha=float(
                cfg.get("root_vertical_filter_alpha", 0.2)
            ),
            hand_startup_wait=float(
                self.config["runtime"].get("hand_startup_wait", 15.0)
            ),
        )


def main():
    logging.basicConfig(
        level=getattr(logging, CONFIG["logging"].get("level", "INFO")),
        format=CONFIG["logging"].get("format"),
    )
    try:
        DexFullApplication().run()
    except KeyboardInterrupt:
        # asyncio.run() already executes DexFullApplication.shutdown() from
        # runner's finally block.  Keep the normal Ctrl+C path free of a
        # misleading traceback after all child processes have been reaped.
        logger.info("DexFull interrupted by user")
