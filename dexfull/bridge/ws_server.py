import asyncio
import inspect
import json
import logging
import time
from typing import Any, Dict, Set

import websockets
from websockets.asyncio.server import serve as websocket_serve
from websockets.exceptions import ConnectionClosed

from ..common.ws_security import WebSocketTlsPolicy
try:
    from .message import WsEnvelope
    from .robot_data_adapter import RobotDataAdapter
except ImportError:
    from message import WsEnvelope
    from robot_data_adapter import RobotDataAdapter

logger = logging.getLogger("TeleopBridge.WS")
logging.getLogger("websockets.server").setLevel(logging.CRITICAL)


_LONG_RUNNING_METHODS = {
    "start_xr_teleop",
    "start_xr_teleoperate",
    "restart_xr_teleop",
    "restart_xr_teleoperate",
    "stop_xr_teleop",
    "stop_xr_teleoperate",
    "startXrTeleop",
    "startXRTeleop",
    "startXrTeleoperate",
    "restartXrTeleop",
    "restartXRTeleop",
    "restartXrTeleoperate",
    "stopXrTeleop",
    "stopXRTeleop",
    "stopXrTeleoperate",
}


class ClientConnection:
    def __init__(self, ws):
        self.ws = ws

        # The accepted socket's local address is the exact robot-side address
        # Unity used for this WebSocket connection. It can therefore be passed
        # internally to TeleopController when img_server_ip is configured as
        # "auto". Unity doesn't need to send or know this private field.
        self.local_ip = self._extract_ip(
            getattr(ws, "local_address", None)
        )
        self.remote_ip = self._extract_ip(
            getattr(ws, "remote_address", None)
        )

        self.last_pong = time.time()
        self.last_heartbeat_time = time.time()
        # Low-frequency control/state events. These should not be crowded out
        # by high-frequency realtime streams.
        self.queue = asyncio.Queue(maxsize=50)

        self.alive = True
        self._version = 0
        self.dropped_frames = 0
        self.robot_datas_enabled = True
        self.vr_input_enabled = True

        # Realtime streams use latest-frame semantics: never build a FIFO
        # backlog that makes Unity display stale robot/VR poses.
        self.robot_datas_queue = asyncio.Queue(maxsize=1)
        self.vr_input_queue = asyncio.Queue(maxsize=1)
        # Long XR lifecycle RPCs initialize native modules and IK in the
        # background. The receive loop must remain free for Unity heartbeat.
        self.active_long_requests = 0

    @staticmethod
    def _extract_ip(address):
        """
        Extract an IP string from websockets local_address/remote_address.

        IPv4 addresses normally arrive as (host, port); IPv6 may contain
        additional tuple elements. Returning None is safe: TeleopController
        will continue with its route/interface fallback.
        """
        if address is None:
            return None

        if isinstance(address, (tuple, list)):
            if not address:
                return None
            value = address[0]
        else:
            value = getattr(address, "host", None)
            if value is None:
                value = str(address)

        value = str(value).strip()
        return value or None

    def touch(self):
        now = time.time()
        self.last_pong = now
        self.last_heartbeat_time = now


class TeleopWebSocketServer:
    def __init__(
        self,
        host: str,
        port: int,
        path: str = "/ws",
        *,
        compression: bool = False,
        tls_policy: WebSocketTlsPolicy | None = None,
    ):
        self.host = host
        self.port = port
        self.path = path or "/ws"
        self.compression = bool(compression)
        # Direct unit users may omit a policy. The application always builds
        # one from config, whose production default is fail-closed WSS.
        self.tls_policy = tls_policy or WebSocketTlsPolicy(None)
        self._clients: Set[ClientConnection] = set()
        self._server = None
        self._running = False
        self._bridge_call = None
        self._last_robot_datas = None
        self._last_robot_state = None
        self.robot_datas_streaming = False
        self.vr_input_streaming = False
        self._last_vr_input = None
        self._vr_input_streaming_callback = None
        self._health_task = None
        self._robot_state_interval = 1.0
        self._last_robot_state_emit = 0.0
        self._last_slow_log = 0.0
        self._request_tasks = set()
        # Keep lifecycle mutations ordered even though they run outside the
        # receive loop. A stop/restart arriving during cold startup must wait
        # for that startup instead of racing the control runtime.
        self._lifecycle_request_lock = asyncio.Lock()

    async def start(self):
        logger.info(
            "WS Server starting at %s://%s:%s%s",
            self.tls_policy.scheme,
            self.host,
            self.port,
            self.path,
        )
        self._server = await websocket_serve(
            self._handler,
            self.host,
            self.port,
            # A separate health loop owns protocol pings and can suspend them
            # while a known long-running XR request is active.
            ping_interval=None,
            max_size=2**20,
            ssl=self.tls_policy.context,
            # Telemetry frames are only 1-3 KiB. Deflate costs more CPU and
            # latency on Jetson than the small bandwidth saving is worth.
            compression="deflate" if self.compression else None,
        )
        self._running = True
        self._health_task = asyncio.create_task(self._health_loop())

    async def stop(self):
        logger.info("WS Server stopping...")
        self._running = False

        if self._health_task:
            self._health_task.cancel()
            try:
                await self._health_task
            except asyncio.CancelledError:
                pass

        for task in list(self._request_tasks):
            task.cancel()
        if self._request_tasks:
            await asyncio.gather(*self._request_tasks, return_exceptions=True)
        self._request_tasks.clear()

        if self._server:
            self._server.close()
            await self._server.wait_closed()

        for c in list(self._clients):
            await self._close_client(c, code=1001, reason="server stopping")
        self._clients.clear()

    @property
    def client_count(self):
        return len(self._clients)

    async def _handler(self, ws, path=None):
        request_path = self._get_request_path(ws, path)
        if request_path != self.path:
            logger.warning("Reject websocket path %s, expected %s", request_path, self.path)
            await ws.close(code=1008, reason="invalid websocket path")
            return

        client = ClientConnection(ws)
        self._clients.add(client)
        logger.info(
            "Client connected (%s), remote=%s, local=%s",
            len(self._clients),
            client.remote_ip,
            client.local_ip,
        )

        sender_task = asyncio.create_task(self._client_sender_loop(client))
        if self.robot_datas_streaming and self._last_robot_datas is not None and client.robot_datas_enabled:
            if self._last_robot_state is not None:
                self._queue_payload(client, self._last_robot_state)
            self._queue_latest_robot_datas(client, self._last_robot_datas)
        if self.vr_input_streaming and self._last_vr_input is not None and client.vr_input_enabled:
            self._queue_latest_vr_input(client, self._last_vr_input)

        try:
            async for msg in ws:
                await self._on_message(client, msg)
        except ConnectionClosed:
            pass
        except Exception as e:
            logger.exception("WebSocket handler error: %s", e)
            await self._send_envelope(client, WsEnvelope.build_error(error_tip="internal_error"))
        finally:
            client.alive = False
            self._clients.discard(client)
            sender_task.cancel()
            await asyncio.gather(sender_task, return_exceptions=True)
            logger.info("Client disconnected (%s)", len(self._clients))

    def _get_request_path(self, ws, path=None):
        if path is not None:
            return path
        request = getattr(ws, "request", None)
        if request is not None and getattr(request, "path", None):
            return request.path
        if getattr(ws, "path", None):
            return ws.path
        return self.path

    async def _on_message(self, client: ClientConnection, msg: str):
        client.touch()

        try:
            envelope = WsEnvelope.from_json(msg)
        except ValueError as e:
            await self._send_envelope(client, WsEnvelope.build_error(error_tip=f"invalid_json: {e}"))
            return

        if envelope.type == "ping":
            await self._handle_ping(client, envelope)
            return

        if envelope.type == "pong":
            client.touch()
            return

        if envelope.type == "request":
            if envelope.method in _LONG_RUNNING_METHODS:
                self._start_background_request(client, envelope)
            else:
                await self._handle_request(client, envelope)
            return

        if envelope.type == "error":
            logger.warning("Unity client error: %s", envelope.error_tip)
            return

        await self._send_envelope(
            client,
            WsEnvelope.build_error(
                id=envelope.id,
                timestamp=envelope.timestamp,
                error_tip=f"unknown_type: {envelope.type}",
            ),
        )

    def _start_background_request(self, client, envelope):
        client.active_long_requests += 1
        task = asyncio.create_task(
            self._run_background_request(client, envelope),
            name=f"UnityRPC:{envelope.method or 'unknown'}",
        )
        self._request_tasks.add(task)

        def finished(done_task):
            client.active_long_requests = max(0, client.active_long_requests - 1)
            self._request_tasks.discard(done_task)
            if done_task.cancelled():
                return
            try:
                error = done_task.exception()
            except asyncio.CancelledError:
                return
            if error is not None:
                logger.error(
                    "Background Unity request %s failed: %s",
                    envelope.method,
                    error,
                )

        task.add_done_callback(finished)

    async def _run_background_request(self, client, envelope):
        async with self._lifecycle_request_lock:
            await self._handle_request(client, envelope)

    async def _handle_ping(self, client: ClientConnection, envelope: WsEnvelope):
        client.touch()
        await self._send_envelope(client, WsEnvelope.build_pong(envelope))

    async def _handle_request(self, client: ClientConnection, envelope: WsEnvelope):
        method = envelope.method
        data = envelope.data if isinstance(envelope.data, dict) else {}

        if not method:
            await self._send_protocol_error(client, envelope, "missing method")
            return

        if method in ("set_robot_datas", "subscribe_robot_datas"):
            enabled = bool(data.get("enabled", True))
            client.robot_datas_enabled = enabled
            if not enabled:
                self._clear_client_robot_datas_queue(client)
            data = {"robot_datas_enabled": enabled}
            await self._send_response(client, envelope, method, True, data)
            return

        if method in ("set_vr_input", "subscribe_vr_input"):
            enabled = bool(data.get("enabled", True))
            client.vr_input_enabled = enabled
            if not enabled:
                self._clear_client_vr_input_queue(client)
            data = {"vr_input_enabled": enabled}
            await self._send_response(client, envelope, method, True, data)
            return

        if method == "disconnect":
            await self._send_response(client, envelope, method, True, {"state": "disconnected"})
            await self._close_client(client, code=1000, reason="client requested disconnect")
            return

        if self._bridge_call is None:
            await self._send_response(
                client,
                envelope,
                method,
                False,
                {"state": "error"},
                error_tip="bridge call not configured",
            )
            return

        try:
            bridge_data = dict(data)

            if method in (
                "start_xr_teleop",
                "start_xr_teleoperate",
                "restart_xr_teleop",
                "restart_xr_teleoperate",
            ):
                if client.local_ip:
                    bridge_data["_bridge_local_ip"] = client.local_ip

            result = self._bridge_call(method, bridge_data)
            if inspect.isawaitable(result):
                result = await result
        except Exception as e:
            logger.exception("Bridge call failed: %s", e)
            await self._send_response(
                client,
                envelope,
                method,
                False,
                {"state": "error"},
                error_tip=str(e),
            )
            return

        if isinstance(result, dict) and result.get("status") == "error" and str(result.get("msg", "")).startswith("unknown method"):
            await self._send_protocol_error(client, envelope, f"unknown_method: {method}")
            return

        succeed = not (isinstance(result, dict) and result.get("status") == "error")
        error_tip = None if succeed else (result.get("msg") if isinstance(result, dict) else "request failed")
        await self._send_response(client, envelope, method, succeed, result, error_tip=error_tip)

    def _response_id(self, envelope: WsEnvelope, data: Any):
        if isinstance(data, dict) and data.get("_response_id"):
            return data["_response_id"]
        return envelope.id

    def _response_data(self, data: Any):
        if not isinstance(data, dict):
            return data
        return {k: v for k, v in data.items() if not k.startswith("_")}

    async def _send_protocol_error(self, client, envelope, error_tip: str):
        await self._send_envelope(
            client,
            WsEnvelope.build_error(
                id=envelope.id,
                method=envelope.method,
                timestamp=envelope.timestamp,
                error_tip=error_tip,
            ),
        )

    async def _send_response(
        self,
        client,
        envelope: WsEnvelope,
        method: str,
        succeed: bool,
        data: Any,
        error_tip: str = None,
    ):
        await self._send_envelope(
            client,
            WsEnvelope.build_response(
                id=self._response_id(envelope, data),
                method=method,
                succeed=succeed,
                timestamp=envelope.timestamp,
                data=self._response_data(data),
                error_tip=error_tip,
            ),
        )
        if succeed:
            await self._apply_response_side_effects(data)

    async def _apply_response_side_effects(self, data: Any):
        if not isinstance(data, dict):
            return
        if data.get("_enable_robot_datas_streaming"):
            await self.set_robot_datas_streaming(True)
        if data.get("_disable_robot_datas_streaming"):
            await self.set_robot_datas_streaming(False)
        if data.get("_enable_vr_input_streaming"):
            await self.set_vr_input_streaming(True)
        if data.get("_disable_vr_input_streaming"):
            await self.set_vr_input_streaming(False)

    async def _send(self, client: ClientConnection, payload: Dict[str, Any]):
        try:
            await client.ws.send(json.dumps(payload, ensure_ascii=False))
        except ConnectionClosed:
            client.alive = False
        except Exception as e:
            logger.warning("send failed: %s", e)
            client.alive = False

    async def _send_envelope(self, client: ClientConnection, envelope: WsEnvelope):
        try:
            await client.ws.send(envelope.to_json())
        except ConnectionClosed:
            client.alive = False
        except Exception as e:
            logger.warning("send envelope failed: %s", e)
            client.alive = False

    async def _close_client(self, client: ClientConnection, code=1000, reason=""):
        client.alive = False
        try:
            await client.ws.close(code=code, reason=reason)
        except TypeError:
            try:
                await client.ws.close()
            except Exception:
                pass
        except Exception:
            pass
        self._clients.discard(client)

    async def broadcast_state(self, state: Dict[str, Any]):
        await self._broadcast_event("state", state)

    async def broadcast_state_change(self, old, new):
        await self._broadcast_event("state_change", {"from": old, "to": new})

    async def broadcast_process_state(self, name, state):
        await self._broadcast_event("process_state", {"service": name, "state": state})

    async def broadcast_error(self, error_tip: str, data: Dict[str, Any] | None = None):
        """Push an asynchronous Unity-compatible ``type:error`` envelope."""
        await self._broadcast(
            WsEnvelope.build_error(
                error_tip=error_tip,
                timestamp=int(time.time() * 1000),
                data=data,
            ).to_dict()
        )

    async def _broadcast_event(self, event_name: str, data: Dict[str, Any]):
        await self._broadcast(
            WsEnvelope.build_event(
                eventName=event_name,
                timestamp=int(time.time()*1000),
                data=data,
            ).to_dict()
        )

    async def _broadcast(self, payload: Dict[str, Any]):
        if not self._clients:
            return

        dead = []
        msg = json.dumps(payload, ensure_ascii=False)
        for c in self._clients:
            if not c.alive:
                dead.append(c)
                continue
            try:
                c.queue.put_nowait(msg)
            except asyncio.QueueFull:
                c.dropped_frames += 1
            except Exception:
                dead.append(c)

        for d in dead:
            self._clients.discard(d)

    async def _health_loop(self):
        while self._running:
            dead = []
            for c in list(self._clients):
                if c.active_long_requests:
                    # Cold XR startup can legitimately take tens of seconds.
                    # Its request task is bounded by runtime.startup_wait.
                    c.touch()
                    continue
                try:
                    pong_waiter = await c.ws.ping()
                    await asyncio.wait_for(pong_waiter, timeout=5)
                    c.touch()
                except asyncio.CancelledError:
                    raise
                except Exception:
                    dead.append(c)

            for d in dead:
                await self._close_client(d, code=1001, reason="heartbeat timeout")
                logger.warning("Client removed due to timeout")

            await asyncio.sleep(5)

    async def _client_sender_loop(self, client: ClientConnection):
        while client.alive:
            try:
                # Priority 1: control/state messages.
                msg = self._try_get_nowait(client.queue)

                # Priority 2: latest VR frame.
                if msg is None:
                    msg = self._try_get_nowait(client.vr_input_queue)

                # Priority 3: latest robot frame.
                if msg is None:
                    msg = self._try_get_nowait(client.robot_datas_queue)

                if msg is None:
                    msg = await self._wait_next_client_message(client)

                await self._send_sender_loop_message(client, msg)
            except ConnectionClosed:
                break
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error("send loop error: %s", e)
                client.alive = False
                break

    async def _wait_next_client_message(self, client: ClientConnection):
        control_task = asyncio.create_task(client.queue.get())
        vr_task = asyncio.create_task(client.vr_input_queue.get())
        robot_task = asyncio.create_task(client.robot_datas_queue.get())
        tasks = {control_task, vr_task, robot_task}
        try:
            done, _ = await asyncio.wait(
                tasks,
                return_when=asyncio.FIRST_COMPLETED,
            )

            # If more than one queue becomes ready in the same event-loop turn,
            # preserve deterministic priority: control > VR > robot.
            if control_task in done:
                msg = control_task.result()

                if vr_task in done:
                    self._requeue_latest_best_effort(
                        client.vr_input_queue,
                        vr_task.result(),
                    )
                if robot_task in done:
                    self._requeue_latest_best_effort(
                        client.robot_datas_queue,
                        robot_task.result(),
                    )

            elif vr_task in done:
                msg = vr_task.result()

                if robot_task in done:
                    self._requeue_latest_best_effort(
                        client.robot_datas_queue,
                        robot_task.result(),
                    )

            else:
                msg = robot_task.result()

            return msg
        finally:
            pending = [task for task in tasks if not task.done()]
            for task in pending:
                task.cancel()
            if pending:
                await asyncio.gather(*pending, return_exceptions=True)

    async def _send_sender_loop_message(self, client: ClientConnection, msg: str):
        start = time.perf_counter()
        await client.ws.send(msg)
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        if elapsed_ms > 10.0:
            self._log_slow(
                "WS send slow elapsed=%.2fms bytes=%s queue=%s vr_queue=%s robot_queue=%s",
                elapsed_ms,
                len(msg),
                client.queue.qsize(),
                client.vr_input_queue.qsize(),
                client.robot_datas_queue.qsize(),
            )

    @staticmethod
    def _try_get_nowait(queue: asyncio.Queue):
        try:
            return queue.get_nowait()
        except asyncio.QueueEmpty:
            return None

    @staticmethod
    def _requeue_latest_best_effort(queue: asyncio.Queue, msg: str):
        while True:
            try:
                queue.get_nowait()
            except asyncio.QueueEmpty:
                break

        try:
            queue.put_nowait(msg)
        except asyncio.QueueFull:
            pass

    def _queue_payload(self, client: ClientConnection, payload: Dict[str, Any]):
        try:
            client.queue.put_nowait(json.dumps(payload, ensure_ascii=False))
        except asyncio.QueueFull:
            client.dropped_frames += 1

    async def broadcast_robot_stream(self, packet: dict):
        start = time.perf_counter()
        self._last_robot_datas = RobotDataAdapter.to_robot_datas_event(packet)
        now = time.time()
        include_state = now - self._last_robot_state_emit >= self._robot_state_interval
        if include_state:
            self._last_robot_state = RobotDataAdapter.to_robot_state_event(packet)
            self._last_robot_state_emit = now
        if not self.robot_datas_streaming:
            return
        await self._broadcast_robot_stream(include_state=include_state)
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        if elapsed_ms > 10.0:
            clients = sum(1 for c in self._clients if c.alive and c.robot_datas_enabled)
            self._log_slow("WS robot stream slow elapsed=%.2fms clients=%s", elapsed_ms, clients)

    async def _broadcast_robot_stream(self, include_state: bool = False):
        if not self.robot_datas_streaming:
            return
        if not self._clients:
            return
        if self._last_robot_datas is None:
            return

        dead = []

        for c in self._clients:
            if not c.alive:
                dead.append(c)
                continue
            if not c.robot_datas_enabled:
                continue

            # robot_state is low frequency and belongs to the control queue.
            if include_state and self._last_robot_state is not None:
                self._queue_payload(c, self._last_robot_state)

            # robot_datas is high frequency and must use latest-frame semantics.
            self._queue_latest_robot_datas(c, self._last_robot_datas)

        for d in dead:
            self._clients.discard(d)

    async def set_robot_datas_streaming(self, enabled: bool):
        self.robot_datas_streaming = bool(enabled)
        if not self.robot_datas_streaming:
            self._last_robot_datas = None
            self._last_robot_state = None
            for c in list(self._clients):
                self._clear_client_robot_datas_queue(c)
            return
        if self._last_robot_datas is not None:
            await self._broadcast_robot_stream(include_state=True)

    async def broadcast_vr_input(self, data: dict):
        envelope = WsEnvelope.build_event(
            eventName="vr_input",
            timestamp=int(time.time()*1000),
            data=data,
        )
        payload = envelope.to_dict()
        self._last_vr_input = payload
        if not self.vr_input_streaming:
            return
        if not self._clients:
            return

        dead = []
        for c in self._clients:
            if not c.alive:
                dead.append(c)
                continue
            if not c.vr_input_enabled:
                continue
            self._queue_latest_vr_input(c, payload)

        for d in dead:
            self._clients.discard(d)

    async def set_vr_input_streaming(self, enabled: bool):
        enabled = bool(enabled)
        self.vr_input_streaming = enabled
        if not enabled:
            self._last_vr_input = None
            for c in list(self._clients):
                self._clear_client_vr_input_queue(c)
        callback = self._vr_input_streaming_callback
        if callable(callback):
            try:
                callback(enabled)
            except Exception as e:
                logger.warning("VR input streaming callback failed: %s", e)

    def _queue_latest_robot_datas(self, client: ClientConnection, payload: Dict[str, Any]):
        msg = json.dumps(payload, ensure_ascii=False)
        self._clear_client_robot_datas_queue(client)
        try:
            client.robot_datas_queue.put_nowait(msg)
        except asyncio.QueueFull:
            client.dropped_frames += 1
            self._clear_client_robot_datas_queue(client)
            try:
                client.robot_datas_queue.put_nowait(msg)
            except asyncio.QueueFull:
                client.dropped_frames += 1

    @staticmethod
    def _clear_client_robot_datas_queue(client: ClientConnection):
        while True:
            try:
                client.robot_datas_queue.get_nowait()
            except asyncio.QueueEmpty:
                break

    def _queue_latest_vr_input(self, client: ClientConnection, payload: Dict[str, Any]):
        msg = json.dumps(payload, ensure_ascii=False)
        self._clear_client_vr_input_queue(client)
        try:
            client.vr_input_queue.put_nowait(msg)
        except asyncio.QueueFull:
            client.dropped_frames += 1
            self._clear_client_vr_input_queue(client)
            try:
                client.vr_input_queue.put_nowait(msg)
            except asyncio.QueueFull:
                client.dropped_frames += 1

    @staticmethod
    def _clear_client_vr_input_queue(client: ClientConnection):
        while True:
            try:
                client.vr_input_queue.get_nowait()
            except asyncio.QueueEmpty:
                break

    def _log_slow(self, msg, *args):
        now = time.time()
        if now - self._last_slow_log > 1.0:
            logger.warning(msg, *args)
            self._last_slow_log = now
