from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber # dds
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_                           # idl
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_

from dexfull.hand_drivers.retargeting import HandRetargeting, HandType
import numpy as np
from enum import IntEnum
import threading
import time
from multiprocessing import Array, Process, Value

from dexfull.common.realtime_affinity import (
    apply_current_process_affinity,
    apply_current_thread_affinity,
    resolve_cpu_affinity,
)

import logging_mp
logger_mp = logging_mp.getLogger(__name__)

brainco_Num_Motors = 6
kTopicbraincoLeftCommand = "rt/brainco/left/cmd"
kTopicbraincoLeftState = "rt/brainco/left/state"
kTopicbraincoRightCommand = "rt/brainco/right/cmd"
kTopicbraincoRightState = "rt/brainco/right/state"


class _BraincoDdsLifecycle:
    """Callback-based, cancellable readiness shared by both BrainCo modes."""

    def _init_lifecycle(self, stop_event=None, startup_timeout=15.0):
        self._runtime_stop_event = stop_event
        self._close_event = threading.Event()
        self._hand_ready_event = threading.Event()
        self._left_ready = threading.Event()
        self._right_ready = threading.Event()
        self._startup_timeout = max(0.1, float(startup_timeout))
        self.running = False

    def _stop_requested(self):
        return self._close_event.is_set() or (
            self._runtime_stop_event is not None
            and self._runtime_stop_event.is_set()
        )

    def _on_left_hand_state(self, message):
        if message is None or self._stop_requested():
            return
        states = getattr(message, "states", None)
        if states is None or len(states) < brainco_Num_Motors:
            return
        for idx, motor_id in enumerate(Brainco_Left_Hand_JointIndex):
            self.left_hand_state_array[idx] = states[motor_id].q
        self._left_ready.set()
        self._update_ready()

    def _on_right_hand_state(self, message):
        if message is None or self._stop_requested():
            return
        states = getattr(message, "states", None)
        if states is None or len(states) < brainco_Num_Motors:
            return
        for idx, motor_id in enumerate(Brainco_Right_Hand_JointIndex):
            self.right_hand_state_array[idx] = states[motor_id].q
        self._right_ready.set()
        self._update_ready()

    def _update_ready(self):
        if self._left_ready.is_set() and self._right_ready.is_set():
            self.hand_sub_ready = True
            self._hand_ready_event.set()

    def _wait_until_dds_ready(self, label, required=False):
        deadline = time.monotonic() + self._startup_timeout
        next_log = 0.0
        while not self._hand_ready_event.is_set():
            if self._stop_requested():
                raise RuntimeError(f"{label} DDS startup cancelled")
            now = time.monotonic()
            if now >= deadline:
                missing = []
                if not self._left_ready.is_set():
                    missing.append(kTopicbraincoLeftState)
                if not self._right_ready.is_set():
                    missing.append(kTopicbraincoRightState)
                message = (
                    f"{label} DDS state timeout after {self._startup_timeout:.1f}s; "
                    f"missing={','.join(missing)}"
                )
                if required:
                    raise TimeoutError(message)
                logger_mp.warning(
                    "%s. Continuing in degraded mode; the hand service will reconnect independently.",
                    message,
                )
                return False
            if now >= next_log:
                logger_mp.warning(
                    "[%s] Waiting for BrainCo DDS state (left=%s right=%s)...",
                    label,
                    self._left_ready.is_set(),
                    self._right_ready.is_set(),
                )
                next_log = now + 1.0
            self._hand_ready_event.wait(min(0.1, deadline - now))
        return True

    def _close_dds(self):
        self._close_event.set()
        for name in (
            "LeftHandState_subscriber",
            "RightHandState_subscriber",
            "LeftHandCmb_publisher",
            "RightHandCmb_publisher",
        ):
            endpoint = getattr(self, name, None)
            if endpoint is None:
                continue
            close = getattr(endpoint, "Close", None) or getattr(endpoint, "close", None)
            if close is not None:
                try:
                    close()
                except Exception as exc:
                    logger_mp.warning("Failed to close %s: %s", name, exc)


class Brainco_Controller_ctrl(_BraincoDdsLifecycle):
    def __init__(self, left_gripper_trigger_in, left_gripper_squeeze_in, right_gripper_trigger_in, right_gripper_squeeze_in,
                       dual_hand_data_lock = None, dual_hand_state_array = None, dual_hand_action_array = None, fps = 50.0, Unit_Test = False, simulation_mode = False,
                       xr_motion_data_ready_in = None, stop_event=None, startup_timeout=15.0,
                       worker_cpu_affinity="auto", performance_log_interval=5.0):
        logger_mp.info("Initialize Brainco_Controller_ctrl...")
        self.fps = fps
        self.hand_sub_ready = False
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        self.hand_control_thread = None
        self.worker_cpu_affinity = worker_cpu_affinity
        self.performance_log_interval = max(1.0, float(performance_log_interval))
        self._init_lifecycle(stop_event, startup_timeout)

        if not self.Unit_Test:
            self.hand_retargeting = HandRetargeting(HandType.BRAINCO_HAND)
        else:
            self.hand_retargeting = HandRetargeting(HandType.BRAINCO_HAND_Unit_Test)

        # initialize handcmd publisher and handstate subscriber
        self.LeftHandCmb_publisher = ChannelPublisher(kTopicbraincoLeftCommand, MotorCmds_)
        self.LeftHandCmb_publisher.Init()
        self.RightHandCmb_publisher = ChannelPublisher(kTopicbraincoRightCommand, MotorCmds_)
        self.RightHandCmb_publisher.Init()

        # initialize brainco hand's cmd msg (in parent process, not forked)
        self.left_hand_msg  = MotorCmds_()
        self.left_hand_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(len(Brainco_Left_Hand_JointIndex))]
        self.right_hand_msg = MotorCmds_()
        self.right_hand_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(len(Brainco_Right_Hand_JointIndex))]
        for idx, id in enumerate(Brainco_Left_Hand_JointIndex):
            self.left_hand_msg.cmds[id].q = 0.0
            self.left_hand_msg.cmds[id].dq = 1.0
        for idx, id in enumerate(Brainco_Right_Hand_JointIndex):
            self.right_hand_msg.cmds[id].q = 0.0
            self.right_hand_msg.cmds[id].dq = 1.0

        # Shared Arrays for hand states
        self.left_hand_state_array  = Array('d', brainco_Num_Motors, lock=True)  
        self.right_hand_state_array = Array('d', brainco_Num_Motors, lock=True)

        self.LeftHandState_subscriber = ChannelSubscriber(kTopicbraincoLeftState, MotorStates_)
        self.LeftHandState_subscriber.Init(self._on_left_hand_state, 1)
        self.RightHandState_subscriber = ChannelSubscriber(kTopicbraincoRightState, MotorStates_)
        self.RightHandState_subscriber.Init(self._on_right_hand_state, 1)

        if self._wait_until_dds_ready("Brainco_Controller_ctrl"):
            logger_mp.info("[Brainco_Controller_ctrl] Subscribe dds ok.")

        self.hand_control_thread = threading.Thread(target=self.control_process, args=(left_gripper_trigger_in, left_gripper_squeeze_in, right_gripper_trigger_in, right_gripper_squeeze_in,
                                                                          self.left_hand_state_array, self.right_hand_state_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array,
                                                                          xr_motion_data_ready_in), daemon=True)
        self.hand_control_thread.start()

        logger_mp.info("Initialize Brainco_Controller_ctrl OK!\n")

    def ctrl_dual_hand(self, left_q_target, right_q_target):
        """
        Set current left, right hand motor state target q
        """
        for idx, id in enumerate(Brainco_Left_Hand_JointIndex):             
            self.left_hand_msg.cmds[id].q = left_q_target[idx]
        for idx, id in enumerate(Brainco_Right_Hand_JointIndex):             
            self.right_hand_msg.cmds[id].q = right_q_target[idx] 

        self.LeftHandCmb_publisher.Write(self.left_hand_msg)
        self.RightHandCmb_publisher.Write(self.right_hand_msg)
        # logger_mp.debug("hand ctrl publish ok.")
    
    def control_process(self, left_gripper_trigger_in, left_gripper_squeeze_in, right_gripper_trigger_in, right_gripper_squeeze_in,
                              left_hand_state_array, right_hand_state_array, dual_hand_data_lock = None, dual_hand_state_array = None, dual_hand_action_array = None,
                              xr_motion_data_ready_in = None):
        self.running = True
        active_cpus = apply_current_thread_affinity(
            self.worker_cpu_affinity, role="compute"
        )
        logger_mp.info(
            "Brainco_Controller_ctrl control_process started (cpu_affinity=%s).",
            list(active_cpus) if active_cpus else "default",
        )

        left_q_target  = np.full(brainco_Num_Motors, 0.0, dtype=float)
        right_q_target = np.full(brainco_Num_Motors, 0.0, dtype=float)

        # Diagnostic: track DDS Write time and loop jitter
        _diag_interval = self.performance_log_interval
        _diag_last_log = time.monotonic()
        _diag_max_loop_ms = 0.0
        _diag_max_write_ms = 0.0
        _diag_loop_count = 0

        try:
            while self.running and not self._stop_requested():
                start_time = time.time()
                # trigger value range: [10.0, 0.0], 10.0 means no press, 0.0 means full press
                # squeeze value range: [0.0, 1.0],   0.0 means no press, 1.0 means full press
                with left_gripper_trigger_in.get_lock():
                    left_trigger_value = left_gripper_trigger_in.value
                with left_gripper_squeeze_in.get_lock():
                    left_squeeze_value = left_gripper_squeeze_in.value
                with right_gripper_trigger_in.get_lock():
                    right_trigger_value = right_gripper_trigger_in.value
                with right_gripper_squeeze_in.get_lock():
                    right_squeeze_value = right_gripper_squeeze_in.value
                if xr_motion_data_ready_in is not None:
                    with xr_motion_data_ready_in.get_lock():
                        xr_motion_data_ready = xr_motion_data_ready_in.value
                else:
                    xr_motion_data_ready = True

                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))

                if xr_motion_data_ready:
                    # In the official document, the angles are in the range [0, 1] ==> 0.0: fully open  1.0: fully closed
                    left_triger_value = (10.0 - left_trigger_value) / 10.0
                    left_q_target[0]  = np.clip((left_triger_value - 0.5) / 0.5, 0.0, 0.98) # thumb-aux
                    left_q_target[1]  = np.clip(left_triger_value / 0.5, 0.0, 0.7) # thumb
                    left_q_target[2]  = np.clip(left_squeeze_value, 0.0, 0.98)                   # index
                    left_q_target[3]  = np.clip(left_triger_value, 0.0, 0.98)   # middle
                    left_q_target[4]  = np.clip(left_triger_value, 0.0, 0.98)   # ring
                    left_q_target[5]  = np.clip(left_triger_value, 0.0, 0.98)   # pinky

                    right_triger_value = (10.0 - right_trigger_value) / 10.0
                    right_q_target[0] = np.clip((right_triger_value - 0.5) / 0.5, 0.0, 0.98)
                    right_q_target[1] = np.clip(right_triger_value / 0.5, 0.0, 0.7)
                    right_q_target[2] = np.clip(right_squeeze_value, 0.0, 0.98)                  # index
                    right_q_target[3] = np.clip(right_triger_value, 0.0, 0.98)  # middle
                    right_q_target[4] = np.clip(right_triger_value, 0.0, 0.98)  # ring
                    right_q_target[5] = np.clip(right_triger_value, 0.0, 0.98)  # pinky

                # get dual hand state
                action_data = np.concatenate((left_q_target, right_q_target))
                if dual_hand_state_array and dual_hand_action_array:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                try:
                    t_write_start = time.perf_counter()
                    self.ctrl_dual_hand(left_q_target, right_q_target)
                    write_ms = (time.perf_counter() - t_write_start) * 1000.0
                    _diag_max_write_ms = max(_diag_max_write_ms, write_ms)
                except Exception as e:
                    logger_mp.error(f"[Brainco_Controller_ctrl] ctrl_dual_hand failed: {e}")
                current_time = time.time()
                time_elapsed = current_time - start_time
                loop_ms = time_elapsed * 1000.0
                _diag_max_loop_ms = max(_diag_max_loop_ms, loop_ms)
                _diag_loop_count += 1

                # Periodic diagnostic log
                now = time.monotonic()
                if now - _diag_last_log >= _diag_interval:
                    elapsed = max(1e-6, now - _diag_last_log)
                    logger_mp.info(
                        "[Brainco_Controller_ctrl] control_process cycle stats: "
                        "loop_hz=%.1f max_loop_ms=%.1f max_write_ms=%.1f",
                        _diag_loop_count / elapsed,
                        _diag_max_loop_ms,
                        _diag_max_write_ms,
                    )
                    _diag_last_log = now
                    _diag_max_loop_ms = 0.0
                    _diag_max_write_ms = 0.0
                    _diag_loop_count = 0

                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                self._close_event.wait(sleep_time)
        except Exception as e:
            logger_mp.error(f"[Brainco_Controller_ctrl] control_process crashed: {e}", exc_info=True)
        finally:
            logger_mp.info("Brainco_Controller_ctrl has been closed.")

    def close(self):
        self.running = False
        self._close_event.set()
        thread = self.hand_control_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._close_dds()


def _brainco_hand_retarget_side_fn(
    side,
    hand_array,
    xr_motion_data_ready_in,
    retarget_q_target,
    retarget_ready,
    retarget_sequence,
    running_flag,
    unit_test,
    cpu_affinity="auto",
    performance_log_interval=5.0,
):
    """Compute one hand in its own process and publish its latest target.

    Left and right retargeters are independent. Running them sequentially made
    every output update pay for two nonlinear solves and reduced moving-hand
    targets to roughly 13--30 Hz on the robot.
    """
    import numpy as np

    side_index = 0 if side == "left" else 1
    output_offset = side_index * brainco_Num_Motors
    active_cpus = apply_current_process_affinity(cpu_affinity, role="compute")
    logger_mp.info(
        "BrainCo %s retarget worker started (requested_affinity=%s "
        "cpu_affinity=%s)",
        side,
        cpu_affinity,
        list(active_cpus) if active_cpus else "default",
    )

    # Import and construct inside the child. No DDS object crosses the fork.
    from dexfull.hand_drivers.retargeting import HandRetargeting, HandType

    hand_retargeting = HandRetargeting(
        HandType.BRAINCO_HAND_Unit_Test if unit_test else HandType.BRAINCO_HAND
    )
    if side == "left":
        indices = hand_retargeting.left_indices
        retargeter = hand_retargeting.left_retargeting
        hardware_order = hand_retargeting.left_dex_retargeting_to_hardware
    else:
        indices = hand_retargeting.right_indices
        retargeter = hand_retargeting.right_retargeting
        hardware_order = hand_retargeting.right_dex_retargeting_to_hardware

    diag_interval = max(1.0, float(performance_log_interval))
    diag_started = time.monotonic()
    diag_solves = 0
    diag_skipped = 0
    diag_max_solve_ms = 0.0
    last_input = None
    joint_maximums = (1.52, 1.05, 1.47, 1.47, 1.47, 1.47)

    while running_flag.value:
        solved = False
        try:
            with hand_array.get_lock():
                hand_raw = tuple(hand_array[:])

            if xr_motion_data_ready_in is not None:
                with xr_motion_data_ready_in.get_lock():
                    xr_motion_data_ready = bool(xr_motion_data_ready_in.value)
            else:
                xr_motion_data_ready = True

            # Re-solving an identical XR frame used a full core while idle and
            # stole time from arm IK, DDS and WebSocket work under load.
            if xr_motion_data_ready and hand_raw != last_input:
                solve_started = time.perf_counter()
                hand_data = np.asarray(hand_raw).reshape(25, 3)
                reference = hand_data[indices[1, :]] - hand_data[indices[0, :]]
                q_target = retargeter.retarget(reference)[hardware_order]
                for idx, maximum in enumerate(joint_maximums):
                    q_target[idx] = np.clip(q_target[idx] / maximum, 0.0, 1.0)

                with retarget_q_target.get_lock():
                    retarget_q_target[
                        output_offset:output_offset + brainco_Num_Motors
                    ] = list(q_target)
                retarget_ready[side_index] = 1
                retarget_sequence[side_index] += 1
                last_input = hand_raw
                solved = True
                diag_solves += 1
                diag_max_solve_ms = max(
                    diag_max_solve_ms,
                    (time.perf_counter() - solve_started) * 1000.0,
                )
            else:
                diag_skipped += 1
        except Exception as exc:
            # Survive a malformed/transient XR sample and retry the latest one.
            logger_mp.debug("BrainCo %s retarget iteration failed: %s", side, exc)

        now = time.monotonic()
        if now - diag_started >= diag_interval:
            elapsed = max(1e-6, now - diag_started)
            logger_mp.info(
                "BrainCo retarget stats: side=%s solve_hz=%.1f "
                "max_solve_ms=%.1f duplicate_skips=%d",
                side,
                diag_solves / elapsed,
                diag_max_solve_ms,
                diag_skipped,
            )
            diag_started = now
            diag_solves = 0
            diag_skipped = 0
            diag_max_solve_ms = 0.0

        if not solved:
            time.sleep(0.001)


class Brainco_Controller_hand(_BraincoDdsLifecycle):
    def __init__(self, left_hand_array, right_hand_array, dual_hand_data_lock = None, dual_hand_state_array = None,
                       dual_hand_action_array = None, fps = 50.0, Unit_Test = False, simulation_mode = False,
                       xr_motion_data_ready_in = None, stop_event=None, startup_timeout=15.0,
                       retarget_cpu_affinity="auto", dds_cpu_affinity="auto",
                       performance_log_interval=5.0):
        logger_mp.info("Initialize Brainco_Controller_hand...")
        self.fps = fps
        self.hand_sub_ready = False
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        self.retarget_q_target = None
        self.retarget_ready = None
        self.retarget_sequence = None
        self.retarget_running = None
        self.retarget_process = None
        self.retarget_processes = []
        self.dds_write_thread = None
        self.retarget_cpu_affinity = retarget_cpu_affinity
        self.dds_cpu_affinity = dds_cpu_affinity
        self.performance_log_interval = max(1.0, float(performance_log_interval))
        self._init_lifecycle(stop_event, startup_timeout)

        # HandRetargeting is now in a separate Process (no DDS, fork-safe)
        # — no HandRetargeting initialization here.

        # initialize handcmd publisher and handstate subscriber
        self.LeftHandCmb_publisher = ChannelPublisher(kTopicbraincoLeftCommand, MotorCmds_)
        self.LeftHandCmb_publisher.Init()
        self.RightHandCmb_publisher = ChannelPublisher(kTopicbraincoRightCommand, MotorCmds_)
        self.RightHandCmb_publisher.Init()

        # initialize brainco hand's cmd msg (in parent process, not forked)
        self.left_hand_msg  = MotorCmds_()
        self.left_hand_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(len(Brainco_Left_Hand_JointIndex))]
        self.right_hand_msg = MotorCmds_()
        self.right_hand_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(len(Brainco_Right_Hand_JointIndex))]
        for idx, id in enumerate(Brainco_Left_Hand_JointIndex):
            self.left_hand_msg.cmds[id].q = 0.0
            self.left_hand_msg.cmds[id].dq = 1.0
        for idx, id in enumerate(Brainco_Right_Hand_JointIndex):
            self.right_hand_msg.cmds[id].q = 0.0
            self.right_hand_msg.cmds[id].dq = 1.0

        # Shared Arrays for hand states
        self.left_hand_state_array  = Array('d', brainco_Num_Motors, lock=True)
        self.right_hand_state_array = Array('d', brainco_Num_Motors, lock=True)

        self.LeftHandState_subscriber = ChannelSubscriber(kTopicbraincoLeftState, MotorStates_)
        self.LeftHandState_subscriber.Init(self._on_left_hand_state, 1)
        self.RightHandState_subscriber = ChannelSubscriber(kTopicbraincoRightState, MotorStates_)
        self.RightHandState_subscriber.Init(self._on_right_hand_state, 1)

        if self._wait_until_dds_ready("Brainco_Controller_hand"):
            logger_mp.info("[Brainco_Controller_hand] Subscribe dds ok.")

        # ---- Per-hand retargeting processes (NO DDS objects — fork-safe) ----
        # Shared output: retarget_q_target = [left_6, right_6] floats
        self.retarget_q_target = Array('d', brainco_Num_Motors * 2, lock=True)
        self.retarget_ready = Array('B', 2, lock=False)
        self.retarget_sequence = Array('Q', 2, lock=False)
        self.retarget_running = Value('i', 1)

        retarget_cpus = resolve_cpu_affinity(
            self.retarget_cpu_affinity, role="compute"
        )
        side_affinities = (
            [str(retarget_cpus[index % len(retarget_cpus)]) for index in range(2)]
            if retarget_cpus
            else [self.retarget_cpu_affinity, self.retarget_cpu_affinity]
        )
        for side_index, (side, hand_array) in enumerate((
            ("left", left_hand_array),
            ("right", right_hand_array),
        )):
            process = Process(
                target=_brainco_hand_retarget_side_fn,
                args=(side, hand_array, xr_motion_data_ready_in,
                      self.retarget_q_target, self.retarget_ready,
                      self.retarget_sequence, self.retarget_running, Unit_Test,
                      side_affinities[side_index],
                      self.performance_log_interval),
                daemon=True,
            )
            process.start()
            self.retarget_processes.append(process)
            logger_mp.info(
                "[Brainco_Controller_hand] %s retarget process started (pid=%d).",
                side,
                process.pid,
            )
        # Legacy attribute retained for callers that only inspect liveness.
        self.retarget_process = self.retarget_processes[0]

        # ---- DDS Write Thread (isolated control process, 50Hz) ----
        self.dds_write_thread = threading.Thread(
            target=self._dds_write_loop,
            args=(self.left_hand_state_array, self.right_hand_state_array,
                  dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array),
            daemon=True,
        )
        self.dds_write_thread.start()

        logger_mp.info("Initialize Brainco_Controller_hand OK!")

    def ctrl_dual_hand(self, left_q_target, right_q_target):
        """
        Set current left, right hand motor state target q
        """
        for idx, id in enumerate(Brainco_Left_Hand_JointIndex):             
            self.left_hand_msg.cmds[id].q = left_q_target[idx]
        for idx, id in enumerate(Brainco_Right_Hand_JointIndex):             
            self.right_hand_msg.cmds[id].q = right_q_target[idx] 

        self.LeftHandCmb_publisher.Write(self.left_hand_msg)
        self.RightHandCmb_publisher.Write(self.right_hand_msg)
        # logger_mp.debug("hand ctrl publish ok.")
    
    def _dds_write_loop(self, left_hand_state_array, right_hand_state_array,
                         dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array):
        """
        Lightweight DDS write thread in the isolated control process.
        Reads latest retarget result from shared Array, publishes at self.fps.
        No heavy computation — GIL-friendly.
        """
        self.running = True
        active_cpus = apply_current_thread_affinity(
            self.dds_cpu_affinity, role="io"
        )
        logger_mp.info(
            "Brainco_Controller_hand DDS write loop started "
            "(fps=%.1f cpu_affinity=%s).",
            self.fps,
            list(active_cpus) if active_cpus else "default",
        )

        left_q_target  = np.full(brainco_Num_Motors, 0.0, dtype=float)
        right_q_target = np.full(brainco_Num_Motors, 0.0, dtype=float)

        # Diagnostic: track loop time and DDS Write time
        _diag_interval = self.performance_log_interval
        _diag_last_log = time.monotonic()
        _diag_max_loop_ms = 0.0
        _diag_max_write_ms = 0.0
        _diag_loop_count = 0
        _diag_target_updates = 0
        _diag_side_updates = [0, 0]
        _last_retarget_sequence = [-1, -1]

        try:
            while self.running and not self._stop_requested():
                start_time = time.time()

                # Read latest retarget result (non-blocking — just reads shared Array)
                ready = [bool(value) for value in self.retarget_ready]
                if any(ready):
                    sequences = [int(value) for value in self.retarget_sequence]
                    with self.retarget_q_target.get_lock():
                        raw = list(self.retarget_q_target[:])
                    if ready[0]:
                        left_q_target = np.array(raw[:brainco_Num_Motors])
                    if ready[1]:
                        right_q_target = np.array(raw[brainco_Num_Motors:])
                    changed = False
                    for side_index, sequence in enumerate(sequences):
                        if sequence != _last_retarget_sequence[side_index]:
                            _diag_side_updates[side_index] += 1
                            _last_retarget_sequence[side_index] = sequence
                            changed = True
                    if changed:
                        _diag_target_updates += 1

                # Build state/action shared with the local control loop.
                state_data = np.concatenate((
                    np.array(left_hand_state_array[:]),
                    np.array(right_hand_state_array[:]),
                ))
                action_data = np.concatenate((left_q_target, right_q_target))
                if dual_hand_state_array is not None and dual_hand_action_array is not None:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                try:
                    t_write_start = time.perf_counter()
                    self.ctrl_dual_hand(left_q_target, right_q_target)
                    write_ms = (time.perf_counter() - t_write_start) * 1000.0
                    _diag_max_write_ms = max(_diag_max_write_ms, write_ms)
                except Exception as e:
                    logger_mp.error("[Brainco_Controller_hand] ctrl_dual_hand failed: %s", e)

                time_elapsed = time.time() - start_time
                loop_ms = time_elapsed * 1000.0
                _diag_max_loop_ms = max(_diag_max_loop_ms, loop_ms)
                _diag_loop_count += 1

                now = time.monotonic()
                if now - _diag_last_log >= _diag_interval:
                    elapsed = max(1e-6, now - _diag_last_log)
                    logger_mp.info(
                        "[Brainco_Controller_hand] command stats: "
                        "write_hz=%.1f target_hz=%.1f "
                        "left_target_hz=%.1f right_target_hz=%.1f "
                        "max_loop_ms=%.1f max_write_ms=%.1f",
                        _diag_loop_count / elapsed,
                        _diag_target_updates / elapsed,
                        _diag_side_updates[0] / elapsed,
                        _diag_side_updates[1] / elapsed,
                        _diag_max_loop_ms,
                        _diag_max_write_ms,
                    )
                    _diag_last_log = now
                    _diag_max_loop_ms = 0.0
                    _diag_max_write_ms = 0.0
                    _diag_loop_count = 0
                    _diag_target_updates = 0
                    _diag_side_updates = [0, 0]

                sleep_time = max(0, (1.0 / self.fps) - time_elapsed)
                self._close_event.wait(sleep_time)
        except Exception as e:
            logger_mp.error("[Brainco_Controller_hand] dds_write_loop crashed: %s", e, exc_info=True)
        finally:
            logger_mp.info("Brainco_Controller_hand DDS write loop has been closed.")

    def close(self):
        self.running = False
        self._close_event.set()
        if self.retarget_running is not None:
            self.retarget_running.value = 0

        thread = self.dds_write_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)

        for process in self.retarget_processes:
            if process is None or not process.is_alive():
                continue
            process.join(timeout=1.0)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1.0)

        self._close_dds()

# according to the official documentation, https://www.brainco-hz.com/docs/revolimb-hand/product/parameters.html
# the motor sequence is as shown in the table below
# ┌──────┬───────┬────────────┬────────┬────────┬────────┬────────┐
# │ Id   │   0   │     1      │   2    │   3    │   4    │   5    │
# ├──────┼───────┼────────────┼────────┼────────┼────────┼────────┤
# │Joint │ thumb │ thumb-aux  |  index │ middle │  ring  │  pinky │
# └──────┴───────┴────────────┴────────┴────────┴────────┴────────┘
class Brainco_Right_Hand_JointIndex(IntEnum):
    kRightHandThumbAux = 1
    kRightHandThumb = 0
    kRightHandIndex = 2
    kRightHandMiddle = 3
    kRightHandRing = 4
    kRightHandPinky = 5

class Brainco_Left_Hand_JointIndex(IntEnum):
    kLeftHandThumbAux = 1
    kLeftHandThumb = 0
    kLeftHandIndex = 2
    kLeftHandMiddle = 3
    kLeftHandRing = 4
    kLeftHandPinky = 5
