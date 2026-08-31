from __future__ import annotations

import copy
import logging
import multiprocessing
import signal
import threading
import time


logger = logging.getLogger("DexFull.Hand.BrainCoCollector")

_MOTORS_PER_HAND = 6
_FIELDS_PER_HAND = _MOTORS_PER_HAND * 3  # qpos, qvel, current
_SIDES = {"left": 0, "right": 1}


def _motor_values(message) -> dict:
    states = list(getattr(message, "states", []) or [])[:_MOTORS_PER_HAND]
    return {
        "qpos": [float(getattr(state, "q", 0.0)) for state in states],
        "qvel": [float(getattr(state, "dq", 0.0)) for state in states],
        # The native service stores measured motor current in tau_est.
        "current": [float(getattr(state, "tau_est", 0.0)) for state in states],
    }


def _write_shared_sample(
    side: str,
    message,
    values,
    sequences,
    timestamps_ns,
    monotonic_ns,
    changes,
    lock,
) -> None:
    """Copy one DDS sample into fixed-size latest-value shared memory."""

    if message is None:
        return
    decoded = _motor_values(message)
    if len(decoded["qpos"]) != _MOTORS_PER_HAND:
        return

    side_index = _SIDES[side]
    offset = side_index * _FIELDS_PER_HAND
    flattened = decoded["qpos"] + decoded["qvel"] + decoded["current"]
    with lock:
        previous = list(values[offset:offset + _MOTORS_PER_HAND])
        if int(sequences[side_index]) == 0 or previous != decoded["qpos"]:
            changes[side_index] += 1
        values[offset:offset + _FIELDS_PER_HAND] = flattened
        sequences[side_index] += 1
        timestamps_ns[side_index] = time.time_ns()
        monotonic_ns[side_index] = time.monotonic_ns()


def _brainco_collector_process_entry(
    domain_id,
    network_interface,
    values,
    sequences,
    timestamps_ns,
    monotonic_ns,
    changes,
    lock,
    stop_event,
    ready_connection,
) -> None:
    """Own BrainCo DDS readers outside the WS/JSON bridge process.

    Unitree's subscriber callbacks execute Python code. Keeping those
    callbacks in the bridge process made their rate collapse when websocket
    serialization occupied that process's GIL. This worker publishes only a
    fixed-size latest snapshot, so backlog can never accumulate.
    """

    subscribers = []
    try:
        if hasattr(signal, "SIGINT"):
            signal.signal(signal.SIGINT, signal.SIG_IGN)

        from dexfull.common.dds import initialize_dds
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_

        initialize_dds(domain_id, network_interface)

        def on_left(message):
            _write_shared_sample(
                "left", message, values, sequences, timestamps_ns,
                monotonic_ns, changes, lock,
            )

        def on_right(message):
            _write_shared_sample(
                "right", message, values, sequences, timestamps_ns,
                monotonic_ns, changes, lock,
            )

        subscribers = [
            ChannelSubscriber("rt/brainco/left/state", MotorStates_),
            ChannelSubscriber("rt/brainco/right/state", MotorStates_),
        ]
        subscribers[0].Init(on_left, 1)
        subscribers[1].Init(on_right, 1)
        ready_connection.send(("ready", None))
        while not stop_event.wait(0.25):
            pass
    except BaseException as exc:
        try:
            ready_connection.send(("error", f"{type(exc).__name__}: {exc}"))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        for subscriber in subscribers:
            try:
                subscriber.Close()
            except Exception:
                pass
        try:
            ready_connection.close()
        except OSError:
            pass


class BrainCoStateCollector:
    """Latest BrainCo state collected in a dedicated DDS process.

    The transport is deliberately narrow: two fixed-size latest samples and
    counters. It is not the former general command/data IPC channel and has no
    FIFO semantics.
    """

    def __init__(self, domain_id: int = 0, network_interface=None, mp_context=None):
        self.domain_id = int(domain_id)
        self.network_interface = network_interface
        self._context = mp_context or multiprocessing.get_context("spawn")
        self._lock = self._context.Lock()
        self._values = self._context.Array("d", _FIELDS_PER_HAND * 2, lock=False)
        self._sequences = self._context.Array("Q", 2, lock=False)
        self._timestamps_ns = self._context.Array("Q", 2, lock=False)
        self._monotonic_ns = self._context.Array("Q", 2, lock=False)
        self._changes = self._context.Array("Q", 2, lock=False)
        self._stop = self._context.Event()
        self._process = None
        self._process_lock = threading.RLock()
        self._started = False
        # Kept for direct callback tests and backwards-compatible status probes.
        self._last_update = {"left": None, "right": None}
        self._diag_started = time.monotonic()
        self._diag_sequences = [0, 0]
        self._diag_changes = [0, 0]

    def start(self):
        with self._process_lock:
            if self._process is not None and self._process.is_alive():
                return
            self._clear_shared()
            self._stop.clear()
            parent_connection, child_connection = self._context.Pipe(duplex=False)
            process = self._context.Process(
                target=_brainco_collector_process_entry,
                args=(
                    self.domain_id,
                    self.network_interface,
                    self._values,
                    self._sequences,
                    self._timestamps_ns,
                    self._monotonic_ns,
                    self._changes,
                    self._lock,
                    self._stop,
                    child_connection,
                ),
                name="DexFullBrainCoCollector",
                daemon=True,
            )
            process.start()
            child_connection.close()
            self._process = process
            self._started = True

        if parent_connection.poll(5.0):
            kind, detail = parent_connection.recv()
            parent_connection.close()
            if kind == "ready":
                logger.info(
                    "BrainCo telemetry collector process started, PID=%s",
                    process.pid,
                )
                return
            self.stop()
            raise RuntimeError(f"BrainCo telemetry collector failed: {detail}")

        parent_connection.close()
        self.stop()
        raise TimeoutError("BrainCo telemetry collector did not initialize in 5s")

    def stop(self):
        with self._process_lock:
            self._started = False
            self._stop.set()
            process = self._process
        if process is not None:
            process.join(timeout=2.0)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1.0)
        with self._process_lock:
            if self._process is process:
                self._process = None

    def snapshot(self) -> dict:
        result = {}
        with self._lock:
            for side, side_index in _SIDES.items():
                result[side] = self._read_side_locked(side_index)
            sequences = [int(value) for value in self._sequences]
            changes = [int(value) for value in self._changes]
        self._report_diagnostics(sequences, changes)
        return copy.deepcopy(result)

    def status(self, stale_after: float = 2.0) -> dict:
        now_ns = time.monotonic_ns()
        process = self._process
        process_alive = bool(process and process.is_alive())
        result = {}
        with self._lock:
            for side, side_index in _SIDES.items():
                updated_ns = int(self._monotonic_ns[side_index])
                if updated_ns:
                    age = max(0.0, (now_ns - updated_ns) / 1_000_000_000.0)
                else:
                    legacy_update = self._last_update[side]
                    age = (
                        None if legacy_update is None
                        else max(0.0, time.monotonic() - legacy_update)
                    )
                result[side] = {
                    "state": (
                        "ONLINE"
                        if age is not None and age <= stale_after
                        else "DISCONNECTED"
                    ),
                    "age_seconds": age,
                    "sequence": int(self._sequences[side_index]),
                    "collector_process_alive": process_alive,
                }
        return result

    # Direct callbacks remain useful for deterministic unit tests. Production
    # DDS callbacks call the same shared-memory writer in the child process.
    def _on_left(self, message):
        self._update("left", message)

    def _on_right(self, message):
        self._update("right", message)

    def _update(self, side: str, message):
        if message is None or self._stop.is_set():
            return
        _write_shared_sample(
            side,
            message,
            self._values,
            self._sequences,
            self._timestamps_ns,
            self._monotonic_ns,
            self._changes,
            self._lock,
        )
        self._last_update[side] = time.monotonic()

    def _read_side_locked(self, side_index: int) -> dict:
        sequence = int(self._sequences[side_index])
        if sequence <= 0:
            return self._empty_state()
        offset = side_index * _FIELDS_PER_HAND
        raw = list(self._values[offset:offset + _FIELDS_PER_HAND])
        return {
            "qpos": raw[0:6],
            "qvel": raw[6:12],
            "current": raw[12:18],
            "sequence": sequence,
            "timestamp_ns": int(self._timestamps_ns[side_index]),
            "monotonic_ns": int(self._monotonic_ns[side_index]),
        }

    def _clear_shared(self) -> None:
        with self._lock:
            self._values[:] = [0.0] * (_FIELDS_PER_HAND * 2)
            self._sequences[:] = [0, 0]
            self._timestamps_ns[:] = [0, 0]
            self._monotonic_ns[:] = [0, 0]
            self._changes[:] = [0, 0]
        self._last_update = {"left": None, "right": None}
        self._diag_started = time.monotonic()
        self._diag_sequences = [0, 0]
        self._diag_changes = [0, 0]

    def _report_diagnostics(self, sequences, changes) -> None:
        now = time.monotonic()
        elapsed = now - self._diag_started
        if elapsed < 5.0:
            return
        for side, side_index in _SIDES.items():
            logger.info(
                "BrainCo isolated collector stats: side=%s receive_hz=%.1f "
                "value_change_hz=%.1f sequence=%d",
                side,
                (sequences[side_index] - self._diag_sequences[side_index]) / elapsed,
                (changes[side_index] - self._diag_changes[side_index]) / elapsed,
                sequences[side_index],
            )
        self._diag_started = now
        self._diag_sequences = sequences
        self._diag_changes = changes

    @staticmethod
    def _motor_values(message) -> dict:
        return _motor_values(message)

    @staticmethod
    def _empty_state() -> dict:
        return {
            "qpos": [],
            "qvel": [],
            "current": [],
            "sequence": 0,
            "timestamp_ns": 0,
            "monotonic_ns": 0,
        }
