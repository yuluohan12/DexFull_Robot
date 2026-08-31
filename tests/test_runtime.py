import argparse
import threading

from dexfull.common.state_bus import LatestStateBus
from dexfull.control.runtime import ControlRuntime, RuntimeState


class FakeControl:
    def __init__(self):
        self.stopped = threading.Event()

    def run(self, args, hooks):
        hooks.set_state("READY")
        self.stopped.wait(1.0)
        hooks.set_state("STOPPED")

    def request_start(self):
        pass

    def request_pause(self):
        pass

    def request_resume(self):
        pass

    def request_stop(self):
        self.stopped.set()


def test_control_lifecycle_is_direct_and_ipc_free():
    control = FakeControl()
    runtime = ControlRuntime(
        LatestStateBus(),
        args_factory=lambda: argparse.Namespace(),
        module_loader=lambda: control,
    )
    runtime.start_component()
    assert runtime.wait_until_ready(1.0)
    assert runtime.start_teleop()["state"] == RuntimeState.RUNNING.value
    assert runtime.pause()["state"] == RuntimeState.PAUSED.value
    assert runtime.resume()["state"] == RuntimeState.RUNNING.value
    assert runtime.stop_component(1.0)["state"] == RuntimeState.STOPPED.value

