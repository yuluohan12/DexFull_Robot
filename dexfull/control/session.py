import time
import argparse
import math
import copy
import threading
import logging_mp
from dexfull.common.logging_mp_config import configure_logging_mp

configure_logging_mp(logging_mp)
logger_mp = logging_mp.getLogger(__name__)

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from unitree_sdk2py.core.channel import ChannelSubscriber # dds
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from dexfull.xr import TeleVuerWrapper
from dexfull.common.dds import initialize_dds
from dexfull.control.image_pump import LatestImagePump
from dexfull.control.ik_worker import LatestIkWorker
from dexfull.control.robots import get_robot_adapter, list_robot_types
from dexfull.hand_drivers import get_hand_plugin, list_hand_types
from dexfull.imaging.image_client import ImageClient
from dexfull.control.utils.episode_writer import EpisodeWriter
from dexfull.control.utils.motion_switcher import MotionSwitcher, LocoClientWrapper
from dexfull.control.utils.root_pose import RootPoseTransformer
from sshkeyboard import listen_keyboard, stop_listening

# for simulation
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
def publish_reset_category(category: int, publisher): # Scene Reset signal
    msg = String_(data=str(category))
    publisher.Write(msg)
    logger_mp.info(f"published reset category: {category}")

# state transition
START          = False  # Enable to start robot following VR user motion
STOP           = False  # Enable to begin system exit procedure
PAUSED         = False  # Keep component alive while robot following is suspended
READY          = False  # Ready to (1) enter START state, (2) enter RECORD_RUNNING state
RECORD_RUNNING = False  # True if [Recording]
RECORD_TOGGLE  = False  # Toggle recording state
VR_INPUT_STATE = {"available": False}
EE_STATE = {"available": False}

BRAINCO_LEFT_JOINT_NAMES = [
    "kLeftHandThumb",
    "kLeftHandThumbAux",
    "kLeftHandIndex",
    "kLeftHandMiddle",
    "kLeftHandRing",
    "kLeftHandPinky",
]

BRAINCO_RIGHT_JOINT_NAMES = [
    "kRightHandThumb",
    "kRightHandThumbAux",
    "kRightHandIndex",
    "kRightHandMiddle",
    "kRightHandRing",
    "kRightHandPinky",
]

latest_vr_input = {}
latest_vr_input_lock = threading.Lock()
latest_vr_input_seq = 0
_last_vr_input_error_log = 0.0


def _camera_bgr_and_fps(frame):
    """Return ``(bgr, fps)`` for TeleImage and legacy image clients."""
    if frame is None:
        return None, 0.0
    if hasattr(frame, "bgr") and hasattr(frame, "fps"):
        return frame.bgr, float(frame.fps)
    if isinstance(frame, (tuple, list)) and len(frame) >= 2:
        return frame[0], float(frame[1])
    raise TypeError(f"unsupported image client frame type: {type(frame).__name__}")


class LatestTelemetryBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._seq = 0
        self._frame = {}

    def update(self, *, robot=None, end_effector=None, vr=None, action=None, metadata=None):
        with self._lock:
            self._seq += 1
            self._frame = {
                "version": 1,
                "seq": self._seq,
                "timestamp_ns": time.time_ns(),
                "robot": copy.deepcopy(robot or {}),
                "end_effector": copy.deepcopy(end_effector or {}),
                "vr": copy.deepcopy(vr or {}),
                "action": copy.deepcopy(action or {}),
                "metadata": copy.deepcopy(metadata or {}),
            }

    def snapshot(self):
        with self._lock:
            return copy.deepcopy(self._frame)


class RootOdomCache:
    def __init__(
        self,
        pose_mode="unity_relative",
        pelvis_height=0.793,
        axis_mapping="unitree_to_unity",
        heading_reference="initial",
        vertical_mode="filtered",
        vertical_deadband=0.01,
        vertical_filter_alpha=0.2,
    ):
        self._lock = threading.Lock()
        self._sub = None
        self._running = False
        self._thread = None
        self._pose_transformer = RootPoseTransformer(
            mode=pose_mode,
            pelvis_height=pelvis_height,
            axis_mapping=axis_mapping,
            heading_reference=heading_reference,
            vertical_mode=vertical_mode,
            vertical_deadband=vertical_deadband,
            vertical_filter_alpha=vertical_filter_alpha,
        )
        self._last_root_position = []
        self._last_root_rotation = []
        self._last_error_log = 0.0

    def start(self):
        try:
            with self._lock:
                self._pose_transformer.reset()
                self._last_root_position = []
                self._last_root_rotation = []
            self._sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
            self._running = True
            self._sub.Init(self._on_message, 1)
        except Exception as e:
            self._log_throttled("[telemetry] root odom unavailable: %s", e)

    def stop(self):
        self._running = False
        if self._sub is not None:
            try:
                self._sub.Close()
            except Exception as e:
                self._log_throttled("[telemetry] root odom close failed: %s", e)
            self._sub = None

    def snapshot(self):
        with self._lock:
            return {
                "root_position": list(self._last_root_position),
                "root_rotation": list(self._last_root_rotation),
            }

    def _on_message(self, msg):
        if not self._running or msg is None:
            return
        self._update_from_msg(msg)

    def _update_from_msg(self, msg):
        try:
            position = list(getattr(msg, "position", []))
            imu_state = getattr(msg, "imu_state", None)
            quaternion = list(getattr(imu_state, "quaternion", [])) if imu_state is not None else []
            if len(position) < 3 or len(quaternion) < 4:
                return
            root_position = [float(position[0]), float(position[1]), float(position[2])]
            qw, qx, qy, qz = [float(v) for v in quaternion[:4]]
            norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
            if norm <= 0.0 or not math.isfinite(norm):
                return
            root_rotation = [qw / norm, qx / norm, qy / norm, qz / norm]
            if not all(math.isfinite(v) for v in root_position + root_rotation):
                return
            with self._lock:
                output_position, output_rotation = self._pose_transformer.transform(
                    root_position,
                    root_rotation,
                )
                self._last_root_position = output_position
                self._last_root_rotation = output_rotation
        except Exception as e:
            self._log_throttled("[telemetry] root odom parse failed: %s", e)

    def _log_throttled(self, msg, *args):
        now = time.time()
        if now - self._last_error_log > 5.0:
            logger_mp.warning(msg, *args)
            self._last_error_log = now


latest_telemetry = LatestTelemetryBuffer()
#  -------        ---------                -----------                -----------            ---------
#   state          [Ready]      ==>        [Recording]     ==>         [AutoSave]     -->     [Ready]
#  -------        ---------      |         -----------      |         -----------      |     ---------
#   START           True         |manual      True          |manual      True          |        True
#   READY           True         |set         False         |set         False         |auto    True
#   RECORD_RUNNING  False        |to          True          |to          False         |        False
#                                ∨                          ∨                          ∨
#   RECORD_TOGGLE   False       True          False        True          False                  False
#  -------        ---------                -----------                 -----------            ---------
#  ==> manual: when READY is True, set RECORD_TOGGLE=True to transition.
#  --> auto  : Auto-transition after saving data.

def on_press(key):
    global STOP, START, PAUSED, RECORD_TOGGLE
    if key == 'r':
        START = True
        PAUSED = False
    elif key == 'q':
        START = False
        PAUSED = False
        STOP = True
    elif key == 's' and START == True:
        RECORD_TOGGLE = True
    else:
        logger_mp.warning(f"[on_press] {key} was pressed, but no action is defined for this key.")


def request_start():
    """Start robot following through the runtime lifecycle API."""
    global START, STOP, PAUSED
    STOP = False
    START = True
    PAUSED = False


def request_pause():
    """Suspend following without tearing down XR and robot resources."""
    global PAUSED
    PAUSED = True


def request_resume():
    global START, PAUSED
    START = True
    PAUSED = False


def request_stop():
    """Request full component shutdown for stop/restart operations."""
    global START, STOP, PAUSED
    START = False
    PAUSED = False
    STOP = True

def get_state() -> dict:
    """Return current heartbeat state"""
    global START, STOP, RECORD_RUNNING, READY, VR_INPUT_STATE, EE_STATE
    vr_input = get_latest_vr_input()
    return {
        "START": START,
        "STOP": STOP,
        "READY": READY,
        "RECORD_RUNNING": RECORD_RUNNING,
        "vr_input": vr_input,
        "ee_state": EE_STATE,
        "telemetry": latest_telemetry.snapshot(),
    }

def _to_plain(value):
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, dict):
        return {k: _to_plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_plain(v) for v in value]
    return value

def _clamp01(value):
    try:
        return max(0.0, min(1.0, float(value)))
    except Exception:
        return 0.0

def _normalized_inverse(value, max_value=10.0):
    try:
        return _clamp01(1.0 - float(value) / float(max_value))
    except Exception:
        return 0.0

def _hand_tracking_input(tele_data, side):
    pinch = bool(getattr(tele_data, f"{side}_hand_pinch", False))
    pinch_value = float(getattr(tele_data, f"{side}_hand_pinchValue", 10.0))
    squeeze = bool(getattr(tele_data, f"{side}_hand_squeeze", False))
    squeeze_value = float(getattr(tele_data, f"{side}_hand_squeezeValue", 0.0))
    return {
        "mode": "hand",
        "trigger": _normalized_inverse(pinch_value, 15.0),
        "grip": _clamp01(squeeze_value),
        "joystick": [0.0, 0.0],
        "primary_button": pinch,
        "secondary_button": squeeze,
        "menu_button": False,
        "pinch": pinch,
        "pinch_value": pinch_value,
        "squeeze": squeeze,
        "squeeze_value": squeeze_value,
    }

def _controller_input(tele_data, side):
    thumbstick = getattr(tele_data, f"{side}_ctrl_thumbstickValue", [0.0, 0.0])
    return {
        "mode": "controller",
        "trigger": _normalized_inverse(getattr(tele_data, f"{side}_ctrl_triggerValue", 10.0), 10.0),
        "grip": _clamp01(getattr(tele_data, f"{side}_ctrl_squeezeValue", 0.0)),
        "joystick": _to_plain(thumbstick),
        "primary_button": bool(getattr(tele_data, f"{side}_ctrl_aButton", False)),
        "secondary_button": bool(getattr(tele_data, f"{side}_ctrl_bButton", False)),
        "menu_button": bool(getattr(tele_data, f"{side}_ctrl_thumbstick", False)),
        "trigger_pressed": bool(getattr(tele_data, f"{side}_ctrl_trigger", False)),
        "grip_pressed": bool(getattr(tele_data, f"{side}_ctrl_squeeze", False)),
        "thumbstick_pressed": bool(getattr(tele_data, f"{side}_ctrl_thumbstick", False)),
    }

def build_vr_input_state(tele_data, input_mode):
    if tele_data is None:
        return {"available": False}
    if input_mode == "hand":
        left_hand = _hand_tracking_input(tele_data, "left")
        right_hand = _hand_tracking_input(tele_data, "right")
    else:
        left_hand = _controller_input(tele_data, "left")
        right_hand = _controller_input(tele_data, "right")
    return {
        "available": True,
        "source": "dexfull_control",
        "mode": input_mode,
        "hmd_pose": _matrix_to_xyz_wxyz(tele_data.head_pose),
        "left_controller_pose": _matrix_to_xyz_wxyz(tele_data.left_wrist_pose),
        "right_controller_pose": _matrix_to_xyz_wxyz(tele_data.right_wrist_pose),
        "left_hand": left_hand,
        "right_hand": right_hand,
    }

def _log_vr_input_error(msg, *args):
    global _last_vr_input_error_log
    now = time.time()
    if now - _last_vr_input_error_log > 5.0:
        logger_mp.warning(msg, *args)
        _last_vr_input_error_log = now

def _valid_pose_matrix(matrix) -> bool:
    if matrix is None or not hasattr(matrix, "astype") or not hasattr(matrix, "shape"):
        return False
    try:
        if tuple(matrix.shape) != (4, 4):
            return False
        values = matrix.astype(float).reshape(-1).tolist()
        return len(values) == 16 and all(math.isfinite(float(v)) for v in values)
    except Exception:
        return False

def _matrix_to_xyz_wxyz(matrix) -> list:
    m = matrix.astype(float)
    px, py, pz = float(m[0, 3]), float(m[1, 3]), float(m[2, 3])
    r00, r01, r02 = float(m[0, 0]), float(m[0, 1]), float(m[0, 2])
    r10, r11, r12 = float(m[1, 0]), float(m[1, 1]), float(m[1, 2])
    r20, r21, r22 = float(m[2, 0]), float(m[2, 1]), float(m[2, 2])
    trace = r00 + r11 + r22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r21 - r12) / s
        qy = (r02 - r20) / s
        qz = (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(max(0.0, 1.0 + r00 - r11 - r22)) * 2.0
        qw = (r21 - r12) / s
        qx = 0.25 * s
        qy = (r01 + r10) / s
        qz = (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(max(0.0, 1.0 + r11 - r00 - r22)) * 2.0
        qw = (r02 - r20) / s
        qx = (r01 + r10) / s
        qy = 0.25 * s
        qz = (r12 + r21) / s
    else:
        s = math.sqrt(max(0.0, 1.0 + r22 - r00 - r11)) * 2.0
        qw = (r10 - r01) / s
        qx = (r02 + r20) / s
        qy = (r12 + r21) / s
        qz = 0.25 * s
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError("invalid quaternion norm")
    pose = [px, py, pz, qw / norm, qx / norm, qy / norm, qz / norm]
    if len(pose) != 7 or not all(math.isfinite(v) for v in pose):
        raise ValueError("invalid xyz,wxyz pose")
    return pose

def _safe_bool(tele_data, name: str) -> bool:
    try:
        return bool(getattr(tele_data, name, False))
    except Exception:
        return False

def _safe_float(tele_data, name: str) -> float:
    try:
        value = float(getattr(tele_data, name, 0.0))
        return value if math.isfinite(value) else 0.0
    except Exception:
        return 0.0

def _safe_thumbstick(tele_data, name: str) -> list:
    try:
        value = getattr(tele_data, name, [0.0, 0.0])
        values = value.tolist() if hasattr(value, "tolist") else list(value)
        x = float(values[0]) if len(values) > 0 and math.isfinite(float(values[0])) else 0.0
        y = float(values[1]) if len(values) > 1 and math.isfinite(float(values[1])) else 0.0
        return [x, y]
    except Exception:
        return [0.0, 0.0]

def _build_controller_side(tele_data, side: str) -> dict:
    return {
        "trigger": _safe_bool(tele_data, f"{side}_ctrl_trigger"),
        "trigger_value": _safe_float(tele_data, f"{side}_ctrl_triggerValue"),
        "squeeze": _safe_bool(tele_data, f"{side}_ctrl_squeeze"),
        "squeeze_value": _safe_float(tele_data, f"{side}_ctrl_squeezeValue"),
        "a_button": _safe_bool(tele_data, f"{side}_ctrl_aButton"),
        "b_button": _safe_bool(tele_data, f"{side}_ctrl_bButton"),
        "thumbstick": _safe_bool(tele_data, f"{side}_ctrl_thumbstick"),
        "thumbstick_value": _safe_thumbstick(tele_data, f"{side}_ctrl_thumbstickValue"),
    }

def _build_controller_input(tele_data) -> dict:
    return {
        "motion_data_ready": _safe_bool(tele_data, "motion_data_ready"),
        "left": _build_controller_side(tele_data, "left"),
        "right": _build_controller_side(tele_data, "right"),
    }

def update_latest_vr_input(tele_data) -> None:
    global latest_vr_input, latest_vr_input_seq, VR_INPUT_STATE
    if tele_data is None:
        return
    try:
        head = getattr(tele_data, "head_pose", None)
        left = getattr(tele_data, "left_wrist_pose", None)
        right = getattr(tele_data, "right_wrist_pose", None)
        if not (_valid_pose_matrix(head) and _valid_pose_matrix(left) and _valid_pose_matrix(right)):
            return

        frame = {
            "seq": latest_vr_input_seq + 1,
            "timestamp": time.time(),
            "controller_input": _build_controller_input(tele_data),
            "hmd_pose": _matrix_to_xyz_wxyz(head),
            "left_controller_pose": _matrix_to_xyz_wxyz(left),
            "right_controller_pose": _matrix_to_xyz_wxyz(right),
        }
        with latest_vr_input_lock:
            latest_vr_input_seq += 1
            frame["seq"] = latest_vr_input_seq
            latest_vr_input = frame
            VR_INPUT_STATE = dict(frame)
    except Exception as e:
        _log_vr_input_error("[vr_input] failed to update latest frame: %s", e)

def get_latest_vr_input() -> dict:
    with latest_vr_input_lock:
        return copy.deepcopy(latest_vr_input)

def _split_dual_hand_values(values, left_count):
    vals = [float(v) for v in values]
    return vals[:left_count], vals[left_count:]

def build_ee_state(ee_type, dual_hand_data_lock=None, dual_hand_state_array=None, dual_hand_action_array=None):
    if not ee_type or dual_hand_state_array is None:
        return {"available": False}

    count_map = {
        "dex3": 7,
        "inspire_dfx": 6,
        "inspire_ftp": 6,
        "brainco": 6,
    }
    left_count = count_map.get(ee_type)
    if left_count is None:
        return {"available": False, "type": ee_type}

    try:
        if dual_hand_data_lock is not None:
            with dual_hand_data_lock:
                state_values = list(dual_hand_state_array[:])
                action_values = list(dual_hand_action_array[:]) if dual_hand_action_array is not None else []
        else:
            state_values = list(dual_hand_state_array[:])
            action_values = list(dual_hand_action_array[:]) if dual_hand_action_array is not None else []

        left_qpos, right_qpos = _split_dual_hand_values(state_values, left_count)
        left_action, right_action = _split_dual_hand_values(action_values, left_count) if action_values else ([], [])
        return {
            "available": True,
            "source": "dexfull_control",
            "type": ee_type,
            "left": {
                "qpos": left_qpos,
                "action": left_action,
            },
            "right": {
                "qpos": right_qpos,
                "action": right_action,
            },
        }
    except Exception as e:
        logger_mp.warning(f"[ee_state] failed to build ee_state: {e}")
        return {"available": False, "type": ee_type}

def _arm_joint_indices(arm_type: str):
    adapter = get_robot_adapter(arm_type)
    return [
        type("JointRef", (), {"name": name, "value": index})
        for name, index in adapter.joints
    ]

def get_robot_state_snapshot(arm_ctrl, arm_type: str, root_cache: RootOdomCache) -> dict:
    empty = {
        "root_position": [],
        "root_rotation": [],
        "joint_positions": [],
        "joint_velocities": [],
        "joint_torques": [],
        "electricity": [],
    }
    try:
        lowstate = getattr(arm_ctrl, "lowstate_buffer", None).GetData()
        if lowstate is None:
            return empty
        indices = _arm_joint_indices(arm_type)
        root = root_cache.snapshot() if root_cache is not None else {}

        def finite_or_zero(value):
            try:
                value = float(value)
                return value if math.isfinite(value) else 0.0
            except Exception:
                return 0.0

        return {
            "root_position": list(root.get("root_position", [])),
            "root_rotation": list(root.get("root_rotation", [])),
            "joint_positions": [finite_or_zero(lowstate.motor_state[idx.value].q) for idx in indices],
            "joint_velocities": [finite_or_zero(lowstate.motor_state[idx.value].dq) for idx in indices],
            "joint_torques": [finite_or_zero(getattr(lowstate.motor_state[idx.value], "tau_est", 0.0)) for idx in indices],
            "electricity": [finite_or_zero(getattr(lowstate.motor_state[idx.value], "vol", 0.0)) for idx in indices],
        }
    except Exception as e:
        logger_mp.warning("[telemetry] failed to build robot state snapshot: %s", e)
        return empty

def build_end_effector_telemetry(ee_type, dual_hand_data_lock=None, dual_hand_state_array=None):
    if not ee_type or dual_hand_state_array is None:
        return {"left": {"qpos": []}, "right": {"qpos": []}}
    count_map = {
        "dex3": 7,
        "inspire_dfx": 6,
        "inspire_ftp": 6,
        "brainco": 6,
    }
    left_count = count_map.get(ee_type)
    if left_count is None:
        return {"left": {"qpos": []}, "right": {"qpos": []}}
    try:
        with dual_hand_data_lock:
            values = [float(v) for v in list(dual_hand_state_array[:])]
        return {
            "left": {"qpos": values[:left_count]},
            "right": {"qpos": values[left_count:]},
        }
    except Exception as e:
        logger_mp.warning("[telemetry] failed to build end-effector telemetry: %s", e)
        return {"left": {"qpos": []}, "right": {"qpos": []}}

def build_telemetry_metadata(
    arm_type,
    ee_type,
):
    joint_names = [
        idx.name
        for idx in _arm_joint_indices(arm_type)
    ]

    ee_type = ee_type or ""

    plugin = get_hand_plugin(ee_type)
    left_hand_joint_names = [] if plugin is None else list(plugin.joint_names_left)
    right_hand_joint_names = [] if plugin is None else list(plugin.joint_names_right)

    return {
        "arm": arm_type,
        "ee": ee_type,
        "joint_names": joint_names,
        "left_hand_joint_names":
            left_hand_joint_names,
        "right_hand_joint_names":
            right_hand_joint_names,
    }

def run(args=None, runtime_hooks=None):
    """Run XR control as a reusable component.

    ``runtime_hooks`` provides lifecycle control and latest VR publication.
    In DexFull it is backed by the isolated control-process boundary; the
    standalone keyboard listener is not started.
    """
    global START, STOP, PAUSED, READY, RECORD_RUNNING, RECORD_TOGGLE
    global VR_INPUT_STATE, EE_STATE, latest_vr_input, latest_vr_input_seq

    START = False
    STOP = False
    PAUSED = False
    READY = False
    RECORD_RUNNING = False
    RECORD_TOGGLE = False
    VR_INPUT_STATE = {"available": False}
    EE_STATE = {"available": False}
    with latest_vr_input_lock:
        latest_vr_input = {}
        latest_vr_input_seq = 0

    # Initialize optional workers before hardware setup so partial-startup
    # failures can always execute the common cleanup path safely.
    input_sampler_stop = None
    input_sampler_thread = None
    image_pump = None
    ik_worker = None
    latest_direct_tele_data = None
    latest_direct_tele_data_lock = threading.Lock()
    listen_keyboard_thread = None
    img_client = None
    tv_wrapper = None
    arm_ctrl = None
    hand_ctrl = None
    gripper_ctrl = None
    root_odom_cache = None
    sim_state_subscriber = None
    recorder = None

    if runtime_hooks is not None:
        runtime_hooks.set_state("STARTING")
        if runtime_hooks.stop_event.is_set():
            runtime_hooks.set_state("STOPPED")
            return

    parser = argparse.ArgumentParser()
    # basic control parameters
    parser.add_argument('--frequency', type = float, default = 30.0, help = 'control and record \'s frequency')
    parser.add_argument('--input-mode', type=str, choices=['hand', 'controller'], default='hand', help='Select XR device input tracking source')
    parser.add_argument('--display-mode', type=str, choices=['immersive', 'ego', 'pass-through'], default='immersive', help='Select XR device display mode')
    parser.add_argument('--arm', type=str, choices=list(list_robot_types()), default='G1_29', help='Select robot adapter')
    parser.add_argument('--ee', type=str, choices=list(list_hand_types()), help='Select hand driver plugin')
    parser.add_argument('--img-server-ip', type=str, default='192.168.123.164', help='IP address of image server, used by teleimager and televuer')
    parser.add_argument('--network-interface', type=str, default=None, help='Network interface for dds communication, e.g., eth0, wlan0. If None, use default interface.')
    parser.add_argument(
        '--root-pose-mode',
        choices=['unity_relative', 'absolute'],
        default='unity_relative',
        help='unity_relative anchors odom to the Unity standing pelvis; absolute preserves raw odom pose.',
    )
    parser.add_argument(
        '--root-pelvis-height',
        type=float,
        default=None,
        help='Unity standing pelvis height in metres. Defaults by robot model.',
    )
    parser.add_argument(
        '--root-axis-mapping',
        choices=['unitree_to_unity', 'raw'],
        default='unitree_to_unity',
        help='Map Unitree X-forward/Y-left translation to the existing Unity transport axes.',
    )
    parser.add_argument(
        '--root-heading-reference',
        choices=['initial', 'odom_world'],
        default='initial',
        help='initial aligns odom translation with the robot heading captured at startup.',
    )
    parser.add_argument(
        '--root-vertical-mode',
        choices=['filtered', 'fixed', 'relative'],
        default='filtered',
        help='filtered suppresses small odom Z jitter while preserving crouching height changes.',
    )
    parser.add_argument(
        '--root-vertical-deadband',
        type=float,
        default=0.01,
        help='Odom Z deadband in metres used by filtered vertical mode.',
    )
    parser.add_argument(
        '--root-vertical-filter-alpha',
        type=float,
        default=0.2,
        help='EMA alpha in (0,1] used by filtered vertical mode.',
    )
    # mode flags
    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')
    parser.add_argument('--headless', action='store_true', help='Enable headless mode (no display)')
    parser.add_argument('--sim', action = 'store_true', help = 'Enable isaac simulation mode')
    parser.add_argument('--affinity', action = 'store_true', help = 'Enable high priority and set CPU affinity mode')
    # record mode and task info
    parser.add_argument('--record', action = 'store_true', help = 'Enable data recording mode')
    parser.add_argument(
        '--sync-ik',
        action='store_false',
        dest='async_ik',
        default=True,
        help='Disable latest-target background IK (diagnostic fallback only)',
    )
    parser.add_argument('--ik-max-iterations', type=int, default=12)
    parser.add_argument('--ik-cpu-affinity', default='auto')
    parser.add_argument('--hand-retarget-cpu-affinity', default='auto')
    parser.add_argument('--hand-dds-cpu-affinity', default='auto')
    parser.add_argument('--performance-log-interval', type=float, default=5.0)
    parser.add_argument('--task-dir', type = str, default = './utils/data/', help = 'path to save data')
    parser.add_argument('--task-name', type = str, default = 'pick cube', help = 'task file name for recording')
    parser.add_argument('--task-goal', type = str, default = 'pick up cube.', help = 'task goal for recording at json file')
    parser.add_argument('--task-desc', type = str, default = 'task description', help = 'task description for recording at json file')
    parser.add_argument('--task-steps', type = str, default = 'step1: do this; step2: do that;', help = 'task steps for recording at json file')

    if args is None:
        args = parser.parse_args()
    logger_mp.info(f"args: {args}")

    try:
        # setup dds communication domains id
        initialize_dds(
            1 if args.sim else 0,
            network_interface=args.network_interface,
        )

        # Standalone developer mode keeps keyboard controls. Production
        # lifecycle is always called directly by DexFull RuntimeHooks.
        if runtime_hooks is None:
            listen_keyboard_thread = threading.Thread(target=listen_keyboard, 
                                                      kwargs={"on_press": on_press, "until": None, "sequential": False,}, 
                                                      daemon=True)
            listen_keyboard_thread.start()

        # image client
        img_client = ImageClient(
            host=args.img_server_ip,
            auto_subscribe=False,
        )
        camera_config = img_client.get_cam_config()
        logger_mp.debug(f"Camera config: {camera_config}")
        xr_need_local_img = not (args.display_mode == 'pass-through' or camera_config['head_camera']['enable_webrtc'])
        decode_cameras = set()
        subscribe_cameras = set()
        if xr_need_local_img:
            decode_cameras.add('head_camera')
            subscribe_cameras.add('head_camera')
        if args.record:
            decode_cameras.update({
                'head_camera',
                'left_wrist_camera',
                'right_wrist_camera',
            })
            subscribe_cameras.update(decode_cameras)
        img_client.subscribe_enabled(
            request_bgr_cameras=decode_cameras,
            camera_names=subscribe_cameras,
        )

        # televuer_wrapper: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
        tv_wrapper = TeleVuerWrapper(use_hand_tracking=args.input_mode == "hand", 
                                     binocular=camera_config['head_camera']['binocular'],
                                     img_shape=camera_config['head_camera']['image_shape'],
                                     # maybe should decrease fps for better performance?
                                     # https://github.com/unitreerobotics/xr_teleoperate/issues/172
                                     # display_fps=camera_config['head_camera']['fps'] ? args.frequency? 30.0?
                                     display_mode=args.display_mode,
                                     zmq=camera_config['head_camera']['enable_zmq'],
                                     webrtc=camera_config['head_camera']['enable_webrtc'],
                                      webrtc_url=f"https://{args.img_server_ip}:{camera_config['head_camera']['webrtc_port']}/offer",
                                      )

        # Image decoding/copying used to run in the 60 Hz arm loop. Under a
        # merged Python process it can wait behind IK/native callbacks for the
        # GIL and turn a nominal 16.7 ms period into 25-100 ms. Keep the camera
        # at its own display clock and expose only its latest frame to recording.
        if (
            runtime_hooks is not None
            and camera_config['head_camera']['enable_zmq']
            and xr_need_local_img
        ):
            image_pump = LatestImagePump(
                img_client.get_head_frame,
                tv_wrapper.render_to_xr,
                _camera_bgr_and_fps,
                frequency=float(camera_config['head_camera'].get('fps', 30.0)),
            )
            image_pump.start()
            logger_mp.info("XR image forwarding moved to independent latest-frame pump.")

        # In unified mode XR sampling has its own clock. IK, image reads and
        # recording can no longer reduce the VR frame production frequency.
        if runtime_hooks is not None:
            input_sampler_stop = threading.Event()

            def sample_xr_input():
                nonlocal latest_direct_tele_data
                sample_hz = max(1.0, float(args.frequency))
                interval = 1.0 / sample_hz
                next_tick = time.monotonic()
                last_error_log = 0.0
                while not input_sampler_stop.is_set():
                    now = time.monotonic()
                    if now < next_tick:
                        input_sampler_stop.wait(next_tick - now)
                        continue
                    next_tick += interval
                    if next_tick < now - interval:
                        next_tick = now + interval
                    try:
                        tele_data = tv_wrapper.get_tele_data()
                        with latest_direct_tele_data_lock:
                            latest_direct_tele_data = tele_data
                        update_latest_vr_input(tele_data)
                        runtime_hooks.publish_vr(get_latest_vr_input())
                    except Exception as exc:
                        error_time = time.monotonic()
                        if error_time - last_error_log >= 5.0:
                            logger_mp.warning("XR input sampler failed: %s", exc)
                            last_error_log = error_time
                        input_sampler_stop.wait(min(interval, 0.1))

            input_sampler_thread = threading.Thread(
                target=sample_xr_input,
                name="XRInputSampler",
                daemon=True,
            )
            input_sampler_thread.start()
        
        # motion mode (G1: Regular mode R1+X, not Running mode R2+A)
        if args.motion:
            if args.input_mode == "controller":
                loco_wrapper = LocoClientWrapper()
        else:
            motion_switcher = MotionSwitcher()
            status, result = motion_switcher.Enter_Debug_Mode()
            logger_mp.info(f"Enter debug mode: {'Success' if status == 0 else 'Failed'}")

        robot_adapter = get_robot_adapter(args.arm)
        ik_options = {}
        if str(args.arm).upper() == "G1_29":
            ik_options["max_iterations"] = max(
                3, int(getattr(args, "ik_max_iterations", 12))
            )
        arm_ik = robot_adapter.create_ik(**ik_options)
        arm_ctrl = robot_adapter.create_controller(
            motion_mode=args.motion,
            simulation_mode=args.sim,
        )

        root_pelvis_height = getattr(args, 'root_pelvis_height', None)
        if root_pelvis_height is None:
            root_pelvis_height = robot_adapter.pelvis_height
        root_odom_cache = None
        if runtime_hooks is None:
            root_odom_cache = RootOdomCache(
                pose_mode=getattr(args, 'root_pose_mode', 'unity_relative'),
                pelvis_height=root_pelvis_height,
                axis_mapping=getattr(args, 'root_axis_mapping', 'unitree_to_unity'),
                heading_reference=getattr(args, 'root_heading_reference', 'initial'),
                vertical_mode=getattr(args, 'root_vertical_mode', 'filtered'),
                vertical_deadband=getattr(args, 'root_vertical_deadband', 0.01),
                vertical_filter_alpha=getattr(args, 'root_vertical_filter_alpha', 0.2),
            )
            root_odom_cache.start()

        # Hand hardware is selected through a plugin descriptor. New hand
        # models register a factory without changing this control session.
        hand_plugin = get_hand_plugin(args.ee)
        hand_context = (
            None
            if hand_plugin is None
            else hand_plugin.control_factory(
                args.input_mode,
                args.sim,
                stop_event=(
                    None
                    if runtime_hooks is None
                    else runtime_hooks.stop_event
                ),
                startup_timeout=float(
                    getattr(args, "hand_startup_wait", 15.0)
                ),
                retarget_cpu_affinity=getattr(
                    args, "hand_retarget_cpu_affinity", "auto"
                ),
                dds_cpu_affinity=getattr(
                    args, "hand_dds_cpu_affinity", "auto"
                ),
                performance_log_interval=float(
                    getattr(args, "performance_log_interval", 5.0)
                ),
            )
        )
        hand_value = hand_context.get if hand_context is not None else lambda name, default=None: default
        dual_hand_data_lock = hand_value("dual_hand_data_lock")
        dual_hand_state_array = hand_value("dual_hand_state_array")
        dual_hand_action_array = hand_value("dual_hand_action_array")
        left_hand_pos_array = hand_value("left_hand_pos_array")
        right_hand_pos_array = hand_value("right_hand_pos_array")
        hand_ctrl = hand_value("hand_ctrl")
        left_gripper_value = hand_value("left_gripper_value")
        right_gripper_value = hand_value("right_gripper_value")
        dual_gripper_data_lock = hand_value("dual_gripper_data_lock")
        dual_gripper_state_array = hand_value("dual_gripper_state_array")
        dual_gripper_action_array = hand_value("dual_gripper_action_array")
        gripper_ctrl = hand_value("gripper_ctrl")
        left_gripper_trigger_value = hand_value("left_gripper_trigger_value")
        left_gripper_squeeze_value = hand_value("left_gripper_squeeze_value")
        right_gripper_trigger_value = hand_value("right_gripper_trigger_value")
        right_gripper_squeeze_value = hand_value("right_gripper_squeeze_value")
        
        # affinity mode (if you dont know what it is, then you probably don't need it)
        if args.affinity:
            import psutil
            p = psutil.Process(os.getpid())
            p.cpu_affinity([0,1,2,3]) # Set CPU affinity to cores 0-3
            try:
                p.nice(-20)           # Set highest priority
                logger_mp.info("Set high priority successfully.")
            except psutil.AccessDenied:
                logger_mp.warning("Failed to set high priority. Please run as root.")
                
            for child in p.children(recursive=True):
                try:
                    # logger_mp.info(f"Child process {child.pid} name: {child.name()}")
                    child.cpu_affinity([5,6])
                    child.nice(-20)
                except psutil.AccessDenied:
                    pass

        # simulation mode
        if args.sim:
            reset_pose_publisher = ChannelPublisher("rt/reset_pose/cmd", String_)
            reset_pose_publisher.Init()
            from dexfull.control.utils.sim_state_topic import start_sim_state_subscribe
            sim_state_subscriber = start_sim_state_subscribe()

        # record + headless / non-headless mode
        if args.record:
            recorder = EpisodeWriter(task_dir = os.path.join(args.task_dir, args.task_name),
                                     task_goal = args.task_goal,
                                     task_desc = args.task_desc,
                                     task_steps = args.task_steps,
                                     frequency = args.frequency, 
                                     rerun_log = not args.headless)

        logger_mp.info("----------------------------------------------------------------")
        logger_mp.info("🟢  Press [r] to start syncing the robot with your movements.")
        if args.record:
            logger_mp.info("🟡  Press [s] to START or SAVE recording (toggle cycle).")
        else:
            logger_mp.info("🔵  Recording is DISABLED (run with --record to enable).")
        logger_mp.info("🔴  Press [q] to stop and exit the program.")
        logger_mp.info("⚠️  IMPORTANT: Please keep your distance and stay safe.")
        READY = True                  # now ready to (1) enter START state
        if runtime_hooks is not None:
            runtime_hooks.set_state("READY")
        while not START and not STOP: # wait for start or stop signal.
            time.sleep(0.033)
            if (
                image_pump is None
                and camera_config['head_camera']['enable_zmq']
                and xr_need_local_img
            ):
                head_img, _ = _camera_bgr_and_fps(img_client.get_head_frame())
                tv_wrapper.render_to_xr(head_img)

        logger_mp.info("---------------------🚀start Tracking🚀-------------------------")
        if runtime_hooks is not None:
            runtime_hooks.set_state("RUNNING")
        arm_ctrl.speed_gradual_max()

        async_ik = bool(
            runtime_hooks is not None and getattr(args, "async_ik", True)
        )
        if async_ik:
            ik_worker = LatestIkWorker(
                arm_ik.solve_ik,
                name=f"{args.arm}LatestIK",
                cpu_affinity=getattr(args, "ik_cpu_affinity", "auto"),
            )
            ik_worker.start()
            logger_mp.info(
                "Realtime IK worker enabled (latest-target, max_iter=%d).",
                int(getattr(args, "ik_max_iterations", 12)),
            )

        ik_result_sequence = 0
        performance_interval = max(
            1.0, float(getattr(args, "performance_log_interval", 5.0))
        )
        performance_started = time.monotonic()
        performance_loops = 0
        sync_ik_count = 0
        sync_ik_total_ms = 0.0
        sync_ik_max_ms = 0.0

        # First-loop timing diagnostics.
        # Only logs the first control iteration and does not change control behavior.
        first_loop_diag = True
        head_img = None
        head_img_fps = 0.0
        left_wrist_img = None
        right_wrist_img = None

        # main loop. robot start to follow VR user's motion
        while not STOP:
            start_time = time.time()

            if PAUSED:
                time.sleep(min(0.05, 1.0 / max(float(args.frequency), 1.0)))
                continue

            performance_loops += 1

            if first_loop_diag:
                diag_t0 = time.perf_counter()
                logger_mp.info("[FIRST_LOOP_DIAG] loop begin")
            # get image
            if camera_config['head_camera']['enable_zmq']:
                if image_pump is not None:
                    head_img, head_img_fps = image_pump.latest()
                elif args.record or xr_need_local_img:
                    head_img, head_img_fps = _camera_bgr_and_fps(
                        img_client.get_head_frame()
                    )
                if xr_need_local_img and image_pump is None:
                    tv_wrapper.render_to_xr(head_img)
            if camera_config['left_wrist_camera']['enable_zmq']:
                if args.record:
                    left_wrist_img, _ = _camera_bgr_and_fps(
                        img_client.get_left_wrist_frame()
                    )
            if camera_config['right_wrist_camera']['enable_zmq']:
                if args.record:
                    right_wrist_img, _ = _camera_bgr_and_fps(
                        img_client.get_right_wrist_frame()
                    )

            # record mode
            if args.record and RECORD_TOGGLE:
                RECORD_TOGGLE = False
                if not RECORD_RUNNING:
                    if recorder.create_episode():
                        RECORD_RUNNING = True
                    else:
                        logger_mp.error("Failed to create episode. Recording not started.")
                else:
                    RECORD_RUNNING = False
                    recorder.save_episode()
                    if args.sim:
                        publish_reset_category(1, reset_pose_publisher)

            # Get the latest XR sample. Unified mode owns a dedicated sampler;
            # legacy mode keeps the original inline behavior.
            if runtime_hooks is not None:
                with latest_direct_tele_data_lock:
                    tele_data = latest_direct_tele_data
                if tele_data is None:
                    time.sleep(min(0.01, 1.0 / max(float(args.frequency), 1.0)))
                    continue
            else:
                tele_data = tv_wrapper.get_tele_data()
                update_latest_vr_input(tele_data)

            if first_loop_diag:
                diag_t1 = time.perf_counter()
                logger_mp.info(
                    "[FIRST_LOOP_DIAG] get_tele_data done elapsed=%.3fs",
                    diag_t1 - diag_t0,
                )

            if first_loop_diag:
                diag_t2 = time.perf_counter()
            if args.ee in ("dex3", "inspire_dfx", "inspire_ftp", "brainco") and args.input_mode in ("hand", "controller"):
                EE_STATE = build_ee_state(args.ee, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array)
            else:
                EE_STATE = {"available": False, "type": args.ee or ""}
            if runtime_hooks is None:
                latest_telemetry.update(
                    robot=get_robot_state_snapshot(arm_ctrl, args.arm, root_odom_cache),
                    end_effector=build_end_effector_telemetry(args.ee, dual_hand_data_lock, dual_hand_state_array),
                    vr=get_latest_vr_input(),
                    action={},
                    metadata=build_telemetry_metadata(args.arm, args.ee),
                )

            if first_loop_diag:
                diag_t3 = time.perf_counter()
                logger_mp.info(
                    "[FIRST_LOOP_DIAG] telemetry update done "
                    "vr=%.3fs telemetry=%.3fs",
                    diag_t2 - diag_t1,
                    diag_t3 - diag_t2,
                )
            if (args.ee == "dex3" or args.ee == "inspire_dfx" or args.ee == "inspire_ftp" or args.ee == "brainco") and args.input_mode == "hand":
                with left_hand_pos_array.get_lock():
                    left_hand_pos_array[:] = tele_data.left_hand_pos.flatten()
                with right_hand_pos_array.get_lock():
                    right_hand_pos_array[:] = tele_data.right_hand_pos.flatten()
            elif args.ee == "brainco" and args.input_mode == "controller":
                with left_gripper_trigger_value.get_lock():
                    left_gripper_trigger_value.value = tele_data.left_ctrl_triggerValue
                with left_gripper_squeeze_value.get_lock():
                    left_gripper_squeeze_value.value = tele_data.left_ctrl_squeezeValue
                with right_gripper_trigger_value.get_lock():
                    right_gripper_trigger_value.value = tele_data.right_ctrl_triggerValue
                with right_gripper_squeeze_value.get_lock():
                    right_gripper_squeeze_value.value = tele_data.right_ctrl_squeezeValue
            elif args.ee == "dex1" and args.input_mode == "controller":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_ctrl_triggerValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_ctrl_triggerValue
            elif args.ee == "dex1" and args.input_mode == "hand":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_hand_pinchValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_hand_pinchValue
            else:
                pass
            
            # high level control
            if args.input_mode == "controller" and args.motion:
                # quit teleoperate
                if tele_data.right_ctrl_aButton:
                    START = False
                    STOP = True
                # command robot to enter damping mode. soft emergency stop function
                if tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick:
                    loco_wrapper.Damp()
                # https://github.com/unitreerobotics/xr_teleoperate/issues/135, control, limit velocity to within 0.3
                loco_wrapper.Move(-tele_data.left_ctrl_thumbstickValue[1] * 0.3,
                                  -tele_data.left_ctrl_thumbstickValue[0] * 0.3,
                                  -tele_data.right_ctrl_thumbstickValue[0]* 0.3)

            # get current robot state data.
            current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
            current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

            if first_loop_diag:
                diag_t4 = time.perf_counter()
                logger_mp.info(
                    "[FIRST_LOOP_DIAG] before solve_ik "
                    "post_telemetry_pre_ik=%.3fs",
                    diag_t4 - diag_t3,
                )

            # Nonlinear IK is slower than the 60 Hz XR clock on Jetson. In
            # unified mode queue only the latest target, keeping hand/XR input
            # fresh instead of blocking the whole loop behind IPOPT.
            time_ik_start = time.perf_counter()
            sol_q = current_lr_arm_q
            if ik_worker is not None:
                ik_worker.submit(
                    tele_data.left_wrist_pose,
                    tele_data.right_wrist_pose,
                    current_lr_arm_q,
                    current_lr_arm_dq,
                )
                ik_result = ik_worker.latest_after(ik_result_sequence)
                if ik_result is not None:
                    ik_result_sequence = ik_result.sequence
                    sol_q, sol_tauff = ik_result.q, ik_result.tauff
                    arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)
            else:
                sol_q, sol_tauff = arm_ik.solve_ik(
                    tele_data.left_wrist_pose,
                    tele_data.right_wrist_pose,
                    current_lr_arm_q,
                    current_lr_arm_dq,
                )
                solve_ms = (time.perf_counter() - time_ik_start) * 1000.0
                sync_ik_count += 1
                sync_ik_total_ms += solve_ms
                sync_ik_max_ms = max(sync_ik_max_ms, solve_ms)
                arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)
            time_ik_end = time.perf_counter()
            logger_mp.debug("ik dispatch:\t%.6f", time_ik_end - time_ik_start)

            if first_loop_diag:
                diag_t5 = time.perf_counter()
                logger_mp.info(
                    "[FIRST_LOOP_DIAG] ik %s "
                    "dispatch=%.3fs total=%.3fs",
                    "queued" if ik_worker is not None else "solved",
                    diag_t5 - diag_t4,
                    diag_t5 - diag_t0,
                )
                first_loop_diag = False

            # record data
            if args.record:
                READY = recorder.is_ready() # now ready to (2) enter RECORD_RUNNING state
                # dex hand or gripper
                if args.ee == "dex3" and args.input_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:7]
                        right_ee_state = dual_hand_state_array[-7:]
                        left_hand_action = dual_hand_action_array[:7]
                        right_hand_action = dual_hand_action_array[-7:]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.input_mode == "hand":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.input_mode == "controller":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = arm_ctrl.get_current_motor_q().tolist()
                        current_body_action = [-tele_data.left_ctrl_thumbstickValue[1]  * 0.3,
                                               -tele_data.left_ctrl_thumbstickValue[0]  * 0.3,
                                               -tele_data.right_ctrl_thumbstickValue[0] * 0.3]
                elif (args.ee == "inspire_dfx" or args.ee == "inspire_ftp" or args.ee == "brainco") and args.input_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:6]
                        right_ee_state = dual_hand_state_array[-6:]
                        left_hand_action = dual_hand_action_array[:6]
                        right_hand_action = dual_hand_action_array[-6:]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "brainco" and args.input_mode == "controller":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:6]
                        right_ee_state = dual_hand_state_array[-6:]
                        left_hand_action = dual_hand_action_array[:6]
                        right_hand_action = dual_hand_action_array[-6:]
                        current_body_state = []
                        current_body_action = []
                else:
                    left_ee_state = []
                    right_ee_state = []
                    left_hand_action = []
                    right_hand_action = []
                    current_body_state = []
                    current_body_action = []

                # arm state and action
                left_arm_state  = current_lr_arm_q[:7]
                right_arm_state = current_lr_arm_q[-7:]
                left_arm_action = sol_q[:7]
                right_arm_action = sol_q[-7:]
                if RECORD_RUNNING:
                    colors = {}
                    depths = {}
                    if camera_config['head_camera']['binocular']:
                        if head_img is not None:
                            colors[f"color_{0}"] = head_img[:, :camera_config['head_camera']['image_shape'][1]//2]
                            colors[f"color_{1}"] = head_img[:, camera_config['head_camera']['image_shape'][1]//2:]
                        else:
                            logger_mp.warning("Head image is None!")
                        if camera_config['left_wrist_camera']['enable_zmq']:
                            if left_wrist_img is not None:
                                colors[f"color_{2}"] = left_wrist_img
                            else:
                                logger_mp.warning("Left wrist image is None!")
                        if camera_config['right_wrist_camera']['enable_zmq']:
                            if right_wrist_img is not None:
                                colors[f"color_{3}"] = right_wrist_img
                            else:
                                logger_mp.warning("Right wrist image is None!")
                    else:
                        if head_img is not None:
                            colors[f"color_{0}"] = head_img
                        else:
                            logger_mp.warning("Head image is None!")
                        if camera_config['left_wrist_camera']['enable_zmq']:
                            if left_wrist_img is not None:
                                colors[f"color_{1}"] = left_wrist_img
                            else:
                                logger_mp.warning("Left wrist image is None!")
                        if camera_config['right_wrist_camera']['enable_zmq']:
                            if right_wrist_img is not None:
                                colors[f"color_{2}"] = right_wrist_img
                            else:
                                logger_mp.warning("Right wrist image is None!")
                    states = {
                        "left_arm": {                                                                    
                            "qpos":   left_arm_state.tolist(),    # numpy.array -> list
                            "qvel":   [],                          
                            "torque": [],                        
                        }, 
                        "right_arm": {                                                                    
                            "qpos":   right_arm_state.tolist(),       
                            "qvel":   [],                          
                            "torque": [],                         
                        },                        
                        "left_ee": {                                                                    
                            "qpos":   left_ee_state,           
                            "qvel":   [],                           
                            "torque": [],                          
                        }, 
                        "right_ee": {                                                                    
                            "qpos":   right_ee_state,       
                            "qvel":   [],                           
                            "torque": [],  
                        }, 
                        "body": {
                            "qpos": current_body_state,
                        }, 
                    }
                    actions = {
                        "left_arm": {                                   
                            "qpos":   left_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],      
                        }, 
                        "right_arm": {                                   
                            "qpos":   right_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],       
                        },                         
                        "left_ee": {                                   
                            "qpos":   left_hand_action,       
                            "qvel":   [],       
                            "torque": [],       
                        }, 
                        "right_ee": {                                   
                            "qpos":   right_hand_action,       
                            "qvel":   [],       
                            "torque": [], 
                        }, 
                        "body": {
                            "qpos": current_body_action,
                        }, 
                    }
                    if args.sim:
                        sim_state = sim_state_subscriber.read_data()            
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions, sim_state=sim_state)
                    else:
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions)

            # This heartbeat represents completion of the actual control
            # iteration. XR sampling runs on another thread and must not mask a
            # blocked arm/IK/control loop as healthy.
            if runtime_hooks is not None:
                runtime_hooks.touch()

            performance_now = time.monotonic()
            performance_elapsed = performance_now - performance_started
            if performance_elapsed >= performance_interval:
                if ik_worker is not None:
                    ik_stats = ik_worker.drain_stats()
                    ik_count = ik_stats["solved"]
                    ik_average_ms = ik_stats["average_solve_ms"]
                    ik_max_ms = ik_stats["max_solve_ms"]
                    ik_replaced = ik_stats["replaced"]
                    ik_failed = ik_stats["failed"]
                else:
                    ik_count = sync_ik_count
                    ik_average_ms = (
                        sync_ik_total_ms / sync_ik_count
                        if sync_ik_count else 0.0
                    )
                    ik_max_ms = sync_ik_max_ms
                    ik_replaced = 0
                    ik_failed = 0
                logger_mp.info(
                    "[CONTROL_PERF] input_hz=%.1f ik_hz=%.1f "
                    "ik_avg=%.1fms ik_max=%.1fms replaced=%d failed=%d",
                    performance_loops / performance_elapsed,
                    ik_count / performance_elapsed,
                    ik_average_ms,
                    ik_max_ms,
                    ik_replaced,
                    ik_failed,
                )
                performance_started = performance_now
                performance_loops = 0
                sync_ik_count = 0
                sync_ik_total_ms = 0.0
                sync_ik_max_ms = 0.0

            current_time = time.time()
            time_elapsed = current_time - start_time
            sleep_time = max(0, (1 / args.frequency) - time_elapsed)
            time.sleep(sleep_time)
            logger_mp.debug(f"main process sleep: {sleep_time}")

    except KeyboardInterrupt:
        logger_mp.info("⛔ KeyboardInterrupt, exiting program...")
    except Exception:
        import traceback
        logger_mp.error(traceback.format_exc())
        if runtime_hooks is not None:
            runtime_hooks.set_state("ERROR")
            raise
    finally:
        if runtime_hooks is not None:
            runtime_hooks.set_state("STOPPING")
        try:
            if ik_worker is not None and not ik_worker.close(timeout=1.0):
                logger_mp.warning(
                    "IK worker is still finishing the current bounded solve; "
                    "shutdown will continue."
                )
        except Exception as e:
            logger_mp.error(f"Failed to stop IK worker: {e}")
        try:
            if input_sampler_stop is not None:
                input_sampler_stop.set()
            if input_sampler_thread is not None and input_sampler_thread.is_alive():
                input_sampler_thread.join(timeout=1.0)
        except Exception as e:
            logger_mp.error(f"Failed to stop XR input sampler: {e}")
        try:
            if image_pump is not None and not image_pump.close(timeout=1.0):
                logger_mp.warning("XR image pump did not stop before timeout.")
        except Exception as e:
            logger_mp.error(f"Failed to stop XR image pump: {e}")
        try:
            if hand_ctrl is not None and hasattr(hand_ctrl, "close"):
                hand_ctrl.close()
            if gripper_ctrl is not None and hasattr(gripper_ctrl, "close"):
                gripper_ctrl.close()
        except Exception as e:
            logger_mp.error(f"Failed to close hand controller: {e}")
        try:
            if arm_ctrl is not None:
                arm_ctrl.ctrl_dual_arm_go_home()
        except Exception as e:
            logger_mp.error(f"Failed to ctrl_dual_arm_go_home: {e}")
        
        try:
            if runtime_hooks is None and listen_keyboard_thread is not None:
                stop_listening()
                listen_keyboard_thread.join()
        except Exception as e:
            logger_mp.error(f"Failed to stop keyboard listener: {e}")
        
        try:
            if img_client is not None:
                img_client.close()
        except Exception as e:
            logger_mp.error(f"Failed to close image client: {e}")

        try:
            if tv_wrapper is not None:
                tv_wrapper.close()
        except Exception as e:
            logger_mp.error(f"Failed to close televuer wrapper: {e}")

        try:
            if root_odom_cache is not None:
                root_odom_cache.stop()
        except Exception as e:
            logger_mp.error(f"Failed to stop root odom cache: {e}")

        try:
            if not args.motion:
                pass
                # status, result = motion_switcher.Exit_Debug_Mode()
                # logger_mp.info(f"Exit debug mode: {'Success' if status == 3104 else 'Failed'}")
        except Exception as e:
            logger_mp.error(f"Failed to exit debug mode: {e}")

        try:
            if args.sim and sim_state_subscriber is not None:
                sim_state_subscriber.stop_subscribe()
        except Exception as e:
            logger_mp.error(f"Failed to stop sim state subscriber: {e}")
        
        try:
            if args.record and recorder is not None:
                recorder.close()
        except Exception as e:
            logger_mp.error(f"Failed to close recorder: {e}")
        logger_mp.info("✅ Finally, exiting program.")
        if runtime_hooks is not None:
            runtime_hooks.set_state("STOPPED")
        else:
            raise SystemExit(0)


if __name__ == '__main__':
    run()
