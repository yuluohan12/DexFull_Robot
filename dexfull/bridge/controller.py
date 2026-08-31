from __future__ import annotations

import asyncio
import ipaddress
import socket
import threading
import time

from dexfull.control.runtime import RuntimeState
from dexfull.control.robots import get_robot_adapter
from dexfull.hand_drivers import get_hand_plugin

from .message import BasicInfos, audioobj, depthobj, imageobj


METHOD_ALIASES = {
    "getStatus": "get_status",
    "getProcessStatus": "get_process_status",
    "startImageServer": "start_image_server",
    "stopImageServer": "stop_image_server",
    "restartImageServer": "restart_image_server",
    "startTeleimager": "start_teleimager",
    "stopTeleimager": "stop_teleimager",
    "startXrTeleop": "start_xr_teleop",
    "startXRTeleop": "start_xr_teleop",
    "startXrTeleoperate": "start_xr_teleoperate",
    "stopXrTeleop": "stop_xr_teleop",
    "stopXRTeleop": "stop_xr_teleop",
    "stopXrTeleoperate": "stop_xr_teleoperate",
    "restartXrTeleop": "restart_xr_teleop",
    "restartXRTeleop": "restart_xr_teleop",
    "restartXrTeleoperate": "restart_xr_teleoperate",
    "startTeleop": "start_teleop",
    "stopTeleop": "stop_teleop",
    "pauseTeleop": "pause_teleop",
    "resumeTeleop": "resume_teleop",
    "restartTeleop": "restart_teleop",
    "startRobot": "start_teleop",
    "stopRobot": "stop_teleop",
    "startHandDriver": "start_hand_driver",
    "stopHandDriver": "stop_hand_driver",
    "restartHandDriver": "restart_hand_driver",
    "getHandDriverStatus": "get_hand_driver_status",
}


class UnityController:
    """Unity-compatible RPC adapter with a minimal control-process boundary."""

    def __init__(
        self,
        process_manager,
        runtime,
        hand_drivers,
        collector,
        streamer,
        bus,
        config,
    ):
        self.process_manager = process_manager
        self.runtime = runtime
        self.hand_drivers = hand_drivers
        self.collector = collector
        self.streamer = streamer
        self.bus = bus
        self.config = config
        self._resolved_img_server_ip = None

    async def handle(self, method: str, data: dict = None) -> dict:
        data = data or {}
        method = METHOD_ALIASES.get(method, method or "")

        if method in ("start_image_server", "start_teleimager"):
            return self.process_manager.start_service("teleimager")
        if method in ("stop_image_server", "stop_teleimager"):
            return self.process_manager.stop_service("teleimager")
        if method in ("restart_image_server", "restart_teleimager"):
            return self.process_manager.restart_service("teleimager")

        if method in ("start_xr_teleop", "start_xr_teleoperate"):
            return await self._start_xr(data)
        if method in ("stop_xr_teleop", "stop_xr_teleoperate"):
            result = await asyncio.to_thread(
                self.runtime.stop_component,
                self.config["runtime"].get("stop_timeout", 15.0),
            )
            if (
                result.get("status") == "ok"
                and self.config["runtime"].get("stop_hand_with_control", False)
            ):
                result["hand_driver"] = self.hand_drivers.stop()
            else:
                result["hand_driver"] = self.hand_drivers.status()
                result["hand_driver_kept_running"] = True
            return result
        if method in ("restart_xr_teleop", "restart_xr_teleoperate"):
            stopped = await asyncio.to_thread(
                self.runtime.stop_component,
                self.config["runtime"].get("stop_timeout", 15.0),
            )
            if stopped.get("status") != "ok":
                stopped["hand_driver"] = self.hand_drivers.status()
                stopped["hand_driver_kept_running"] = True
                return stopped
            # Restart only the XR/control domain. Camera and hand hardware
            # services have independent lifecycles and remain online.
            hand = self.hand_drivers.start()
            if hand.get("status") == "error":
                return hand
            result = await self._start_runtime(data)
            result["hand_driver"] = hand
            result["hand_driver_restarted"] = False
            if result.get("status") == "error":
                return await self._rollback_failed_start(result)
            return result

        if method == "start_teleop":
            try:
                runtime_result = (
                    self.runtime.resume()
                    if self.runtime.state == RuntimeState.PAUSED
                    else self.runtime.start_teleop()
                )
                return {
                    "state": "teleoping",
                    "zmq_url": self.build_zmq_url(),
                    "runtime": runtime_result,
                    "_response_id": self.build_zmq_url(),
                    "_enable_robot_datas_streaming": True,
                    "_enable_vr_input_streaming": True,
                }
            except Exception as exc:
                return {"status": "error", "msg": str(exc)}

        if method in ("pause_teleop", "stop_teleop"):
            try:
                result = self.runtime.pause()
                return {
                    "state": "paused",
                    "runtime": result,
                    "_disable_robot_datas_streaming": True,
                    "_disable_vr_input_streaming": True,
                }
            except Exception as exc:
                return {"status": "error", "msg": str(exc)}

        if method in ("resume_teleop", "restart_teleop"):
            try:
                result = self.runtime.resume()
                return {
                    "state": "teleoping",
                    "runtime": result,
                    "_enable_robot_datas_streaming": True,
                    "_enable_vr_input_streaming": True,
                }
            except Exception as exc:
                return {"status": "error", "msg": str(exc)}

        if method == "start_hand_driver":
            return self.hand_drivers.start()
        if method == "stop_hand_driver":
            return self.hand_drivers.stop()
        if method == "restart_hand_driver":
            return self.hand_drivers.restart()
        if method == "get_hand_driver_status":
            return self.hand_drivers.status()
        if method == "get_bridge_state":
            return {"state": self.runtime.state.value}
        if method == "get_status":
            return self.get_status()
        if method == "get_basic_infos":
            return self.build_basic_infos()
        if method == "get_process_status":
            service = data.get("service")
            if service in ("xr", "xr_teleoperate"):
                return self.runtime.status()
            return self.process_manager.get_status(service)

        return {"status": "error", "msg": f"unknown method: {method}"}

    async def _start_xr(self, data):
        # Cold retargeting/pinocchio imports can take over a minute on Jetson.
        # Preload before the runtime READY deadline and before opening hardware.
        prepared = await self._prepare_hand_driver()
        if prepared.get("status") == "error":
            return prepared
        hand = self.hand_drivers.start()
        if hand.get("status") == "error":
            return hand
        result = await self._start_runtime(data)
        result["hand_prepare"] = prepared
        if result.get("status") == "error":
            return await self._rollback_failed_start(result)
        return result

    async def _prepare_hand_driver(self):
        """Run cold imports on a daemon thread so shutdown never waits on them."""
        if getattr(self.runtime, "is_process_isolated", False):
            # Importing Pinocchio/retargeting here would load it into the bridge
            # and then load it again in the spawned control process. Let the
            # isolated process own all control-side imports.
            return {
                "status": "ok",
                "driver": getattr(self.hand_drivers, "selected_type", None),
                "prepared": False,
                "deferred_to_control_process": True,
            }
        loop = asyncio.get_running_loop()
        future = loop.create_future()

        def prepare():
            result = self.hand_drivers.prepare()
            try:
                loop.call_soon_threadsafe(self._finish_prepare, future, result)
            except RuntimeError:
                # Event loop already closed during application shutdown.
                pass

        threading.Thread(
            target=prepare,
            name="DexFullHandPreload",
            daemon=True,
        ).start()
        return await future

    @staticmethod
    def _finish_prepare(future, result):
        if not future.done():
            future.set_result(result)

    async def _rollback_failed_start(self, result):
        stopped = await asyncio.to_thread(
            self.runtime.stop_component,
            self.config["runtime"].get("stop_timeout", 15.0),
        )
        result["rollback"] = stopped
        if (
            stopped.get("status") == "ok"
            and self.config["runtime"].get("stop_hand_with_control", False)
        ):
            result["hand_driver"] = self.hand_drivers.stop()
        else:
            result["hand_driver"] = self.hand_drivers.status()
            result["hand_driver_kept_running"] = True
        return result

    async def _start_runtime(self, data):
        try:
            resolved = self.resolve_img_server_ip(data)
            self._resolved_img_server_ip = resolved
            result = self.runtime.start_component()
            result["img_server_ip"] = resolved
            result["hand_driver"] = self.hand_drivers.status()
            wait_ready = data.get("wait_ready", data.get("wait_ipc", True))
            if wait_ready:
                ready = await asyncio.to_thread(
                    self.runtime.wait_until_ready,
                    self.config["runtime"].get("startup_wait", 30.0),
                )
                result["runtime_ready"] = ready
                # Compatibility field retained for existing Unity clients.
                result["ipc_ready"] = ready
                if not ready:
                    result["status"] = "error"
                    result["msg"] = "XR component did not become ready"
            return result
        except Exception as exc:
            return {"status": "error", "msg": str(exc)}

    def get_status(self) -> dict:
        return {
            "bridge_state": self.runtime.state.value,
            "runtime": self.runtime.status(),
            "collector": self.collector.status(),
            "processes": self.process_manager.get_status(),
            "hand_driver": self.hand_drivers.status(),
            "transport": "direct_dds+latest_vr_shared_memory",
            "stream_stats": dict(self.streamer.stats),
            "data_bus": self.bus.diagnostics(),
            "ts": int(time.time() * 1000),
        }

    def build_basic_infos(self) -> dict:
        control = self.config["control"]
        basic = self.config["basic_infos"]
        robot = get_robot_adapter(control["robot"])
        hand = get_hand_plugin(control.get("hand"))
        names = list(robot.joint_names)
        if hand is not None:
            names.extend(hand.joint_names_left)
            names.extend(hand.joint_names_right)
        head_image = imageobj(
            url=self.build_zmq_url("head_port", 55555),
            width=int(basic.get("head_image_width", basic.get("image_width", 0))),
            height=int(basic.get("head_image_height", basic.get("image_height", 0))),
            fps=float(basic.get("head_image_fps", basic.get("image_fps", 0.0))),
        )
        images = [
            head_image,
            imageobj(
                url=self.build_zmq_url("right_wrist_port", 55557),
                width=int(basic.get("right_wrist_image_width", basic.get("image_width", 0))),
                height=int(basic.get("right_wrist_image_height", basic.get("image_height", 0))),
                fps=float(basic.get("right_wrist_image_fps", basic.get("image_fps", 0.0))),
            ),
            imageobj(
                url=self.build_zmq_url("left_wrist_port", 55556),
                width=int(basic.get("left_wrist_image_width", basic.get("image_width", 0))),
                height=int(basic.get("left_wrist_image_height", basic.get("image_height", 0))),
                fps=float(basic.get("left_wrist_image_fps", basic.get("image_fps", 0.0))),
            ),
        ]
        return BasicInfos(
            version=basic.get("version", "2.3.3"),
            date=basic.get("date", ""),
            author=basic.get("author", "DexFull"),
            robot_name=basic.get("robot_name") or robot.name,
            hand_name=basic.get("hand_name") or ("" if hand is None else hand.name),
            control_type=basic.get("control_type") or control.get("input_mode", ""),
            input_device_frenquency=float(
                basic.get("input_device_frequency", control.get("frequency", 0.0))
            ),
            push_data_frequency=float(
                basic.get("push_data_frequency", self.config["telemetry"].get("robot_hz", 0.0))
            ),
            image=head_image,
            images=images,
            depth=depthobj(
                width=int(basic.get("depth_width", 0)),
                height=int(basic.get("depth_height", 0)),
                fps=float(basic.get("depth_fps", 0.0)),
            ),
            audio=audioobj(
                sample_rate=int(basic.get("audio_sample_rate", 0)),
                channels=int(basic.get("audio_channels", 0)),
                format=basic.get("audio_format", ""),
                bits=int(basic.get("audio_bits", 0)),
            ),
            joint_names=names,
        ).to_dict()

    def build_zmq_url(self, port_key="head_port", default_port=55555) -> str:
        cfg = self.config["zmq"]
        host = cfg.get("host")
        if self._is_auto(host):
            host = self._resolved_img_server_ip or self.resolve_img_server_ip()
        return f"tcp://{host}:{int(cfg.get(port_key, default_port))}"

    def resolve_img_server_ip(self, data=None) -> str:
        configured = self.config["control"].get("img_server_ip")
        if not self._is_auto(configured):
            return self._valid_ipv4(configured)
        data = data or {}
        requested = data.get("_bridge_local_ip")
        if requested:
            return self._valid_ipv4(requested)
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.connect(("8.8.8.8", 80))
                return self._valid_ipv4(sock.getsockname()[0])
            finally:
                sock.close()
        except OSError:
            return self._valid_ipv4(socket.gethostbyname(socket.gethostname()))

    @staticmethod
    def _is_auto(value) -> bool:
        return str(value or "").strip().lower() in ("", "auto", "none", "null")

    @staticmethod
    def _valid_ipv4(value) -> str:
        address = ipaddress.ip_address(str(value))
        if address.version != 4 or address.is_unspecified:
            raise RuntimeError(f"invalid IPv4 address: {value}")
        return str(address)
