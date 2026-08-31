from multiprocessing import Array, Lock, Value

from ..base import HandControlContext, HandPlugin
from ..registry import register_hand
from .collector import BrainCoStateCollector


LEFT_JOINTS = (
    "kLeftHandThumb", "kLeftHandThumbAux", "kLeftHandIndex",
    "kLeftHandMiddle", "kLeftHandRing", "kLeftHandPinky",
)
RIGHT_JOINTS = (
    "kRightHandThumb", "kRightHandThumbAux", "kRightHandIndex",
    "kRightHandMiddle", "kRightHandRing", "kRightHandPinky",
)


def _prepare_control():
    # Importing retargeting/pinocchio can be slow on the Jetson. Do it before
    # ControlRuntime starts its READY deadline and before hardware is opened.
    from . import controller
    return controller


def _create_control(
    input_mode: str,
    simulation: bool,
    stop_event=None,
    startup_timeout: float = 15.0,
    retarget_cpu_affinity="auto",
    dds_cpu_affinity="auto",
    performance_log_interval: float = 5.0,
) -> HandControlContext:
    from .controller import Brainco_Controller_ctrl, Brainco_Controller_hand

    lock = Lock()
    state = Array("d", 12, lock=False)
    action = Array("d", 12, lock=False)
    values = {
        "dual_hand_data_lock": lock,
        "dual_hand_state_array": state,
        "dual_hand_action_array": action,
    }
    if input_mode == "hand":
        left = Array("d", 75, lock=True)
        right = Array("d", 75, lock=True)
        values.update(left_hand_pos_array=left, right_hand_pos_array=right)
        values["hand_ctrl"] = Brainco_Controller_hand(
            left, right, lock, state, action,
            simulation_mode=simulation,
            stop_event=stop_event,
            startup_timeout=startup_timeout,
            retarget_cpu_affinity=retarget_cpu_affinity,
            dds_cpu_affinity=dds_cpu_affinity,
            performance_log_interval=performance_log_interval,
        )
    else:
        trigger_l = Value("d", 0.0, lock=True)
        squeeze_l = Value("d", 0.0, lock=True)
        trigger_r = Value("d", 0.0, lock=True)
        squeeze_r = Value("d", 0.0, lock=True)
        values.update(
            left_gripper_trigger_value=trigger_l,
            left_gripper_squeeze_value=squeeze_l,
            right_gripper_trigger_value=trigger_r,
            right_gripper_squeeze_value=squeeze_r,
        )
        values["hand_ctrl"] = Brainco_Controller_ctrl(
            trigger_l, squeeze_l, trigger_r, squeeze_r,
            lock, state, action, simulation_mode=simulation,
            stop_event=stop_event,
            startup_timeout=startup_timeout,
            worker_cpu_affinity=retarget_cpu_affinity,
            performance_log_interval=performance_log_interval,
        )
    return HandControlContext(values)


register_hand(HandPlugin(
    name="brainco",
    joint_names_left=LEFT_JOINTS,
    joint_names_right=RIGHT_JOINTS,
    isolation="external_process",
    service_name="brainco",
    control_factory=_create_control,
    collector_factory=BrainCoStateCollector,
    prepare_factory=_prepare_control,
))
