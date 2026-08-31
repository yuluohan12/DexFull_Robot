from __future__ import annotations

import argparse
import importlib
import logging
import multiprocessing
import os
import threading
import time
from enum import Enum
from typing import Callable, Optional

from dexfull.control.process_channel import LatestVrSharedState


logger = logging.getLogger("DexFull.Control.Runtime")


def _configure_nested_process_start_method() -> str:
    """Restore the process model expected by the Linux XR/vendor stack.

    The bridge deliberately spawns ``DexFullControl`` so it does not inherit
    the bridge's DDS and WebSocket threads.  TeleVuer and several hand plugins
    then create their own small workers using ``multiprocessing.Process``.
    Their upstream implementations are fork-oriented and include objects
    (notably Vuer's ``FrozenList``) that cannot be pickled by a second spawn.

    Selecting fork *inside the freshly spawned control process* preserves the
    safe bridge/control boundary while keeping those nested workers compatible.
    """
    if os.name == "posix" and "fork" in multiprocessing.get_all_start_methods():
        multiprocessing.set_start_method("fork", force=True)
    return multiprocessing.get_start_method()


class RuntimeState(str, Enum):
    STOPPED = "STOPPED"
    STARTING = "STARTING"
    READY = "READY"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    STOPPING = "STOPPING"
    ERROR = "ERROR"


class ProcessRuntimeHooks:
    """Hooks used inside the isolated control process."""

    def __init__(self, vr_state, connection, send_lock, stop_event, heartbeat):
        self._vr_state = vr_state
        self._connection = connection
        self._send_lock = send_lock
        self.stop_event = stop_event
        self._heartbeat = heartbeat
        self._last_vr_seq = None
        self.current_state = "STARTING"

    def publish_vr(self, frame: dict) -> None:
        if not frame:
            return
        seq = frame.get("seq")
        if seq is not None and seq == self._last_vr_seq:
            return
        if self._vr_state.publish(frame):
            self._last_vr_seq = seq

    def publish_telemetry(self, frame: dict) -> None:
        # Bridge owns robot/hand DDS collection in the parent process.
        return None

    def publish_hand(self, frame: dict) -> None:
        # Hand state is collected directly from DDS by the bridge.
        return None

    def set_state(self, state: str) -> None:
        self.current_state = str(state).upper()
        self.touch()
        self._send(("state", self.current_state, time.time()))

    def send_error(self, message: str) -> None:
        self._send(("error", str(message), time.time()))

    def touch(self) -> None:
        self._heartbeat.value = time.monotonic()

    def _send(self, message) -> None:
        try:
            with self._send_lock:
                self._connection.send(message)
        except (BrokenPipeError, EOFError, OSError):
            self.stop_event.set()


def _control_process_entry(args, connection, vr_state, heartbeat) -> None:
    """Spawn-safe control process entry point.

    Heavy XR, CasADi, Pinocchio and hand-control imports happen only here, so
    they cannot contend for the bridge process GIL.
    """

    nested_start_method = _configure_nested_process_start_method()

    from dexfull.common.logging_mp_config import configure_control_process_logging
    from dexfull.common.runtime_tuning import configure_native_threads
    import logging_mp

    configure_native_threads()
    configure_control_process_logging(logging_mp)
    logging_mp.getLogger("DexFull.Control.Worker").info(
        "Nested XR/hand workers use start_method=%s",
        nested_start_method,
    )

    stop_event = threading.Event()
    command_stop = threading.Event()
    module_lock = threading.RLock()
    send_lock = threading.Lock()
    module_holder = {"module": None}
    pending_commands = []
    hooks = ProcessRuntimeHooks(
        vr_state,
        connection,
        send_lock,
        stop_event,
        heartbeat,
    )

    def apply_command(command, module):
        if command == "start":
            module.request_start()
            hooks.set_state("RUNNING")
        elif command == "pause":
            module.request_pause()
            hooks.set_state("PAUSED")
        elif command == "resume":
            module.request_resume()
            hooks.set_state("RUNNING")
        elif command == "stop":
            hooks.set_state("STOPPING")
            module.request_stop()

    def command_loop():
        while not command_stop.is_set():
            try:
                command = connection.recv() if connection.poll(0.1) else None
            except (EOFError, OSError):
                stop_event.set()
                return

            if command is not None:
                command = str(command).strip().lower()
                if command == "stop":
                    stop_event.set()
                    pending_commands.clear()
                elif command in ("start", "pause", "resume"):
                    pending_commands.append(command)

            with module_lock:
                module = module_holder["module"]
            if module is None:
                continue

            try:
                if stop_event.is_set():
                    apply_command("stop", module)
                    return
                if hooks.current_state == "STARTING":
                    continue
                while pending_commands:
                    apply_command(pending_commands.pop(0), module)
            except Exception as exc:
                hooks.send_error(f"control command {command!r} failed: {exc}")

    command_thread = threading.Thread(
        target=command_loop,
        name="DexFullControlCommands",
        daemon=True,
    )
    command_thread.start()
    hooks.set_state("STARTING")

    try:
        module = importlib.import_module("dexfull.control.session")
        with module_lock:
            module_holder["module"] = module
        if stop_event.is_set():
            hooks.set_state("STOPPED")
            return
        module.run(args, hooks)
    except KeyboardInterrupt:
        stop_event.set()
    except SystemExit as exc:
        if exc.code not in (None, 0):
            hooks.send_error(f"SystemExit: {exc.code}")
            hooks.set_state("ERROR")
    except BaseException as exc:
        logger.exception("isolated control process exited with error")
        hooks.send_error(f"{type(exc).__name__}: {exc}")
        hooks.set_state("ERROR")
    finally:
        stop_event.set()
        command_stop.set()
        with module_lock:
            module_holder["module"] = None
        command_thread.join(timeout=0.5)
        if hooks.current_state != "ERROR":
            hooks.set_state("STOPPED")
        try:
            connection.close()
        except OSError:
            pass


class ControlRuntime:
    """Supervise XR/IK/robot control in an isolated spawned process."""

    is_process_isolated = True

    def __init__(
        self,
        bus,
        args_factory: Callable[[], argparse.Namespace],
        *,
        worker_entry=None,
        mp_context=None,
        stall_timeout=2.0,
    ):
        self.bus = bus
        self._args_factory = args_factory
        self._worker_entry = worker_entry or _control_process_entry
        self._context = mp_context or multiprocessing.get_context("spawn")
        self._vr_state = LatestVrSharedState(self._context)
        self._heartbeat = self._context.Value("d", 0.0, lock=False)
        self._lock = threading.RLock()
        self._send_lock = threading.Lock()
        self._state = RuntimeState.STOPPED
        self._process = None
        self._connection = None
        self._monitor_thread = None
        self._monitor_stop = threading.Event()
        self._last_error = None
        self._last_exitcode = None
        self._state_changed_at = time.time()
        self._stall_timeout = max(0.5, float(stall_timeout))
        self._control_loop_stalled = False

    @property
    def state(self):
        with self._lock:
            return self._state

    def status(self) -> dict:
        with self._lock:
            process = self._process
            alive = bool(process and process.is_alive())
            pid = process.pid if process is not None else None
            exitcode = process.exitcode if process is not None else self._last_exitcode
            heartbeat = float(self._heartbeat.value)
            heartbeat_age_ms = (
                None
                if heartbeat <= 0.0
                else max(0.0, (time.monotonic() - heartbeat) * 1000.0)
            )
            return {
                "state": self._state.value,
                "alive": alive,
                "pid": pid,
                "exitcode": exitcode,
                "last_error": self._last_error,
                "state_changed_at": self._state_changed_at,
                "heartbeat_age_ms": heartbeat_age_ms,
                "control_loop_stalled": self._control_loop_stalled,
                "transport": "control_pipe+latest_vr_shared_memory",
            }

    def start_component(self) -> dict:
        with self._lock:
            if self._process is not None and self._process.is_alive():
                return {"status": "ok", **self.status(), "already_running": True}
            self._dispose_finished_locked()
            self._last_error = None
            self._last_exitcode = None
            self._control_loop_stalled = False
            self._heartbeat.value = 0.0
            self._vr_state.clear()
            self.bus.clear_channel("vr")
            parent_connection, child_connection = self._context.Pipe(duplex=True)
            process = self._context.Process(
                target=self._worker_entry,
                args=(
                    self._args_factory(),
                    child_connection,
                    self._vr_state,
                    self._heartbeat,
                ),
                name="DexFullControl",
                daemon=False,
            )
            self._connection = parent_connection
            self._process = process
            self._monitor_stop.clear()
            self._set_state_locked(RuntimeState.STARTING)
            try:
                process.start()
            except Exception as exc:
                parent_connection.close()
                child_connection.close()
                self._connection = None
                self._process = None
                self._last_error = f"{type(exc).__name__}: {exc}"
                self._set_state_locked(RuntimeState.ERROR)
                return {"status": "error", **self.status()}
            child_connection.close()
            self._monitor_thread = threading.Thread(
                target=self._monitor,
                args=(process, parent_connection),
                name="DexFullControlMonitor",
                daemon=True,
            )
            self._monitor_thread.start()
            logger.info("isolated control process started, PID=%s", process.pid)
            return {"status": "ok", **self.status()}

    def start_teleop(self) -> dict:
        self._send_command("start", RuntimeState.RUNNING)
        return {"status": "ok", **self.status()}

    def pause(self) -> dict:
        self._send_command("pause", RuntimeState.PAUSED)
        return {"status": "ok", **self.status()}

    def resume(self) -> dict:
        self._send_command("resume", RuntimeState.RUNNING)
        return {"status": "ok", **self.status()}

    def stop_component(self, timeout=15.0) -> dict:
        with self._lock:
            process = self._process
            if process is None or not process.is_alive():
                self._dispose_finished_locked()
                self._set_state_locked(RuntimeState.STOPPED)
                return {"status": "ok", **self.status()}
            self._set_state_locked(RuntimeState.STOPPING)
        # The child may have failed between the liveness check above and this
        # send.  Shutdown must still join/reap it instead of escaping with a
        # broken-pipe exception.
        try:
            self._send_raw("stop")
        except RuntimeError as exc:
            logger.warning("control stop command could not be delivered: %s", exc)
        process.join(max(0.0, float(timeout)))
        forced = False
        if process.is_alive():
            forced = True
            self._last_error = "control process did not stop before timeout; terminated"
            logger.warning(self._last_error)
            self._terminate_process_tree(process)
            process.join(2.0)

        self._monitor_stop.set()
        monitor = self._monitor_thread
        if monitor is not None and monitor is not threading.current_thread():
            monitor.join(timeout=1.0)
        with self._lock:
            if process.is_alive():
                self._set_state_locked(RuntimeState.ERROR)
                return {"status": "error", "forced": forced, **self.status()}
            self._last_exitcode = process.exitcode
            self._dispose_finished_locked()
            self._set_state_locked(RuntimeState.STOPPED)
            return {"status": "ok", "forced": forced, **self.status()}

    def wait_until_ready(self, timeout=30.0) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout))
        while time.monotonic() < deadline:
            state = self.state
            if state in (RuntimeState.READY, RuntimeState.RUNNING, RuntimeState.PAUSED):
                return True
            if state in (RuntimeState.ERROR, RuntimeState.STOPPED):
                return False
            time.sleep(0.05)
        return False

    def _send_command(self, command: str, state: RuntimeState) -> None:
        self._require_process()
        self._send_raw(command)
        self._set_state(state)

    def _send_raw(self, command: str) -> None:
        with self._send_lock:
            with self._lock:
                connection = self._connection
            if connection is None:
                raise RuntimeError("control component is not running")
            try:
                connection.send(command)
            except (BrokenPipeError, EOFError, OSError) as exc:
                with self._lock:
                    self._last_error = f"control command send failed: {exc}"
                    self._set_state_locked(RuntimeState.ERROR)
                raise RuntimeError(self._last_error) from exc

    def _monitor(self, process, connection) -> None:
        """Monitor exactly one process generation.

        Restart can create a new child before an old monitor thread has fully
        unwound.  Keeping the process/pipe as generation-local arguments
        prevents an old monitor from consuming or changing the new child's
        state.
        """
        last_vr_sequence = 0
        last_stall_check = 0.0
        while not self._monitor_stop.is_set():
            if connection is not None:
                try:
                    while connection.poll(0.0):
                        self._handle_message(connection.recv())
                except (EOFError, OSError):
                    pass

            frame = self._vr_state.read_after(last_vr_sequence)
            if frame is not None:
                last_vr_sequence = int(frame["seq"])
                self.bus.publish_vr(frame)

            if not process.is_alive():
                break
            now = time.monotonic()
            if now - last_stall_check >= 0.5:
                last_stall_check = now
                heartbeat = float(self._heartbeat.value)
                with self._lock:
                    running = self._state == RuntimeState.RUNNING
                    stalled = bool(
                        running
                        and heartbeat > 0.0
                        and now - heartbeat >= self._stall_timeout
                    )
                    changed = stalled != self._control_loop_stalled
                    self._control_loop_stalled = stalled
                if changed:
                    if stalled:
                        logger.error(
                            "control loop heartbeat stalled for %.1fs; "
                            "XR input may still be online but robot commands are not advancing",
                            now - heartbeat,
                        )
                    else:
                        logger.info("control loop heartbeat recovered")
                    self.bus.publish_health(self.status())
            self._vr_state.updated_event.wait(0.01)

        if connection is not None:
            try:
                # Give the multiprocessing feeder a short final opportunity
                # to flush the terminal ERROR/STOPPED message.
                while connection.poll(0.05):
                    self._handle_message(connection.recv())
            except (EOFError, OSError):
                pass
        frame = self._vr_state.read_after(last_vr_sequence)
        if frame is not None:
            self.bus.publish_vr(frame)
        with self._lock:
            if self._process is not process:
                return
            self._last_exitcode = process.exitcode
            if self._state not in (
                RuntimeState.ERROR,
                RuntimeState.STOPPED,
                RuntimeState.STOPPING,
            ):
                self._last_error = (
                    f"control process exited unexpectedly (exitcode={self._last_exitcode})"
                )
                self._set_state_locked(RuntimeState.ERROR)

    def _handle_message(self, message) -> None:
        if not isinstance(message, (tuple, list)) or not message:
            return
        kind = message[0]
        if kind == "state" and len(message) >= 2:
            mapped = RuntimeState.__members__.get(str(message[1]).upper())
            if mapped is not None:
                self._set_state(mapped)
        elif kind == "error" and len(message) >= 2:
            with self._lock:
                self._last_error = str(message[1])
                self._set_state_locked(RuntimeState.ERROR)

    def _require_process(self) -> None:
        with self._lock:
            if self._process is None or not self._process.is_alive():
                raise RuntimeError("control component is not running")

    def _set_state(self, state) -> None:
        with self._lock:
            self._set_state_locked(state)

    def _set_state_locked(self, state) -> None:
        self._state = state
        self._state_changed_at = time.time()
        self.bus.publish_health(self.status())

    def _dispose_finished_locked(self) -> None:
        process = self._process
        if process is not None and process.is_alive():
            return
        connection = self._connection
        self._connection = None
        if connection is not None:
            try:
                connection.close()
            except OSError:
                pass
        if process is not None:
            self._last_exitcode = process.exitcode
            try:
                process.close()
            except (OSError, ValueError):
                pass
        self._process = None
        if self._monitor_thread is threading.current_thread() or not (
            self._monitor_thread and self._monitor_thread.is_alive()
        ):
            self._monitor_thread = None

    @staticmethod
    def _terminate_process_tree(process) -> None:
        try:
            import psutil

            root = psutil.Process(process.pid)
            descendants = root.children(recursive=True)
            for child in reversed(descendants):
                try:
                    child.terminate()
                except psutil.Error:
                    pass
            process.terminate()
            _, alive = psutil.wait_procs(descendants, timeout=1.0)
            for child in alive:
                try:
                    child.kill()
                except psutil.Error:
                    pass
        except Exception:
            process.terminate()


class RuntimeHooks:
    """In-process hooks retained for standalone tests and developer tools."""

    def __init__(self, bus, state_callback, stop_event=None):
        self.bus = bus
        self._state_callback = state_callback
        self.stop_event = stop_event or threading.Event()
        self._last_vr_seq = None

    def publish_vr(self, frame: dict) -> None:
        if not frame:
            return
        seq = frame.get("seq")
        if seq is not None and seq == self._last_vr_seq:
            return
        self._last_vr_seq = seq
        self.bus.publish_vr(frame)

    def publish_telemetry(self, frame: dict) -> None:
        return None

    def publish_hand(self, frame: dict) -> None:
        if frame:
            self.bus.publish_hand(frame)

    def set_state(self, state: str) -> None:
        self._state_callback(str(state).upper())


class InProcessControlRuntime:
    """Legacy in-process runtime used only by deterministic unit tests."""

    is_process_isolated = False

    def __init__(self, bus, args_factory, module_loader: Optional[Callable] = None):
        self.bus = bus
        self._args_factory = args_factory
        self._module_loader = module_loader or (
            lambda: importlib.import_module("dexfull.control.session")
        )
        self._lock = threading.RLock()
        self._state = RuntimeState.STOPPED
        self._thread = None
        self._module = None
        self._stop_requested = threading.Event()
        self._last_error = None
        self._state_changed_at = time.time()

    @property
    def state(self):
        with self._lock:
            return self._state

    def status(self) -> dict:
        with self._lock:
            return {
                "state": self._state.value,
                "alive": bool(self._thread and self._thread.is_alive()),
                "last_error": self._last_error,
                "state_changed_at": self._state_changed_at,
                "transport": "in_process_test",
            }

    def start_component(self) -> dict:
        with self._lock:
            if self._thread and self._thread.is_alive():
                return {"status": "ok", **self.status(), "already_running": True}
            self._last_error = None
            self._stop_requested.clear()
            self.bus.clear_channel("vr")
            self._set_state_locked(RuntimeState.STARTING)
            self._thread = threading.Thread(
                target=self._run, name="DexFullControlTest", daemon=True
            )
            self._thread.start()
            return {"status": "ok", **self.status()}

    def start_teleop(self) -> dict:
        module = self._require_module()
        module.request_start()
        self._set_state(RuntimeState.RUNNING)
        return {"status": "ok", **self.status()}

    def pause(self) -> dict:
        module = self._require_module()
        module.request_pause()
        self._set_state(RuntimeState.PAUSED)
        return {"status": "ok", **self.status()}

    def resume(self) -> dict:
        module = self._require_module()
        module.request_resume()
        self._set_state(RuntimeState.RUNNING)
        return {"status": "ok", **self.status()}

    def stop_component(self, timeout=15.0) -> dict:
        with self._lock:
            thread, module = self._thread, self._module
            if not thread or not thread.is_alive():
                self._set_state_locked(RuntimeState.STOPPED)
                return {"status": "ok", **self.status()}
            self._stop_requested.set()
            self._set_state_locked(RuntimeState.STOPPING)
        if module is not None and hasattr(module, "request_stop"):
            module.request_stop()
        thread.join(max(0.0, float(timeout)))
        with self._lock:
            if thread.is_alive():
                self._last_error = "control component did not stop before timeout"
                self._set_state_locked(RuntimeState.ERROR)
                return {"status": "error", **self.status()}
            self._set_state_locked(RuntimeState.STOPPED)
            return {"status": "ok", **self.status()}

    def wait_until_ready(self, timeout=30.0) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout))
        while time.monotonic() < deadline:
            state = self.state
            if state in (RuntimeState.READY, RuntimeState.RUNNING, RuntimeState.PAUSED):
                return True
            if state in (RuntimeState.ERROR, RuntimeState.STOPPED):
                return False
            time.sleep(0.05)
        return False

    def _run(self):
        try:
            module = self._module_loader()
            with self._lock:
                self._module = module
            if self._stop_requested.is_set():
                self._set_state(RuntimeState.STOPPED)
                return
            module.run(
                self._args_factory(),
                RuntimeHooks(self.bus, self._on_hook_state, self._stop_requested),
            )
        except BaseException as exc:
            logger.exception("in-process control component exited with error")
            with self._lock:
                self._last_error = f"{type(exc).__name__}: {exc}"
                self._set_state_locked(RuntimeState.ERROR)
        finally:
            with self._lock:
                self._module = None
                if self._state not in (RuntimeState.ERROR, RuntimeState.STOPPED):
                    self._set_state_locked(RuntimeState.STOPPED)

    def _on_hook_state(self, state: str):
        mapped = RuntimeState.__members__.get(state)
        if mapped is not None:
            self._set_state(mapped)

    def _require_module(self):
        with self._lock:
            if not self._thread or not self._thread.is_alive() or self._module is None:
                raise RuntimeError("control component is not running")
            return self._module

    def _set_state(self, state):
        with self._lock:
            self._set_state_locked(state)

    def _set_state_locked(self, state):
        self._state = state
        self._state_changed_at = time.time()
        self.bus.publish_health(self.status())
