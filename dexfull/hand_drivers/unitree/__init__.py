from multiprocessing import Array, Lock, Value

from ..base import HandControlContext, HandPlugin
from ..registry import register_hand


def _prepare_control():
    from . import controller
    return controller


def _create_dex3(input_mode: str, simulation: bool, **_lifecycle) -> HandControlContext:
    from .controller import Dex3_1_Controller
    left, right = Array("d", 75, lock=True), Array("d", 75, lock=True)
    lock = Lock()
    state, action = Array("d", 14, lock=False), Array("d", 14, lock=False)
    controller = Dex3_1_Controller(
        left, right, lock, state, action, simulation_mode=simulation
    )
    return HandControlContext({
        "left_hand_pos_array": left, "right_hand_pos_array": right,
        "dual_hand_data_lock": lock, "dual_hand_state_array": state,
        "dual_hand_action_array": action, "hand_ctrl": controller,
    })


def _create_dex1(input_mode: str, simulation: bool, **_lifecycle) -> HandControlContext:
    from .controller import Dex1_1_Gripper_Controller
    left, right = Value("d", 0.0, lock=True), Value("d", 0.0, lock=True)
    lock = Lock()
    state, action = Array("d", 2, lock=False), Array("d", 2, lock=False)
    controller = Dex1_1_Gripper_Controller(
        left, right, lock, state, action, simulation_mode=simulation
    )
    return HandControlContext({
        "left_gripper_value": left, "right_gripper_value": right,
        "dual_gripper_data_lock": lock, "dual_gripper_state_array": state,
        "dual_gripper_action_array": action, "gripper_ctrl": controller,
    })


register_hand(HandPlugin(
    name="dex1", joint_names_left=("left_gripper",),
    joint_names_right=("right_gripper",), isolation="in_process",
    control_factory=_create_dex1,
    prepare_factory=_prepare_control,
))
register_hand(HandPlugin(
    name="dex3", joint_names_left=tuple(f"left_hand_{i}" for i in range(7)),
    joint_names_right=tuple(f"right_hand_{i}" for i in range(7)),
    isolation="in_process", control_factory=_create_dex3,
    prepare_factory=_prepare_control,
))
