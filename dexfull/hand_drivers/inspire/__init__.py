from multiprocessing import Array, Lock

from ..base import HandControlContext, HandPlugin
from ..registry import register_hand


def _prepare_control():
    from . import controller
    return controller


def _factory(controller_name):
    def create(input_mode: str, simulation: bool, **_lifecycle) -> HandControlContext:
        from . import controller as module
        cls = getattr(module, controller_name)
        left, right = Array("d", 75, lock=True), Array("d", 75, lock=True)
        lock = Lock()
        state, action = Array("d", 12, lock=False), Array("d", 12, lock=False)
        controller = cls(left, right, lock, state, action, simulation_mode=simulation)
        return HandControlContext({
            "left_hand_pos_array": left, "right_hand_pos_array": right,
            "dual_hand_data_lock": lock, "dual_hand_state_array": state,
            "dual_hand_action_array": action, "hand_ctrl": controller,
        })
    return create


for name, class_name in (
    ("inspire_dfx", "Inspire_Controller_DFX"),
    ("inspire_ftp", "Inspire_Controller_FTP"),
):
    register_hand(HandPlugin(
        name=name,
        joint_names_left=tuple(f"left_hand_{i}" for i in range(6)),
        joint_names_right=tuple(f"right_hand_{i}" for i in range(6)),
        isolation="in_process",
        control_factory=_factory(class_name),
        prepare_factory=_prepare_control,
    ))
