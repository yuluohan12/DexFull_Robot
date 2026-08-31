from types import SimpleNamespace

from dexfull.bridge.collectors.dds import DdsTelemetryCollector
from dexfull.common.state_bus import LatestStateBus
from dexfull.hand_drivers.brainco.collector import BrainCoStateCollector


def _state(index):
    return SimpleNamespace(q=index + 0.1, dq=index + 0.2, tau_est=index + 0.3, vol=48.0)


def test_bridge_builds_robot_frame_without_control_loop():
    config = {
        "control": {
            "robot": "G1_29",
            "hand": None,
            "root_pose_mode": "absolute",
        },
        "telemetry": {"robot_hz": 30.0},
    }
    collector = DdsTelemetryCollector(
        LatestStateBus(), config, runtime_status=lambda: {"state": "RUNNING"}
    )
    collector._lowstate = SimpleNamespace(motor_state=[_state(i) for i in range(35)])
    collector._root_position = [1.0, 2.0, 3.0]
    collector._root_rotation = [1.0, 0.0, 0.0, 0.0]

    frame = collector._snapshot()

    assert frame["metadata"]["arm"] == "G1_29"
    assert len(frame["robot"]["joint_positions"]) == 29
    assert frame["robot"]["root_position"] == [1.0, 2.0, 3.0]
    assert frame["teleop_start"] is True
    assert frame["end_effector"] == {
        "left": {"qpos": []},
        "right": {"qpos": []},
    }


def test_bridge_dds_callbacks_update_latest_state_without_polling():
    collector = DdsTelemetryCollector(
        LatestStateBus(),
        {
            "control": {
                "robot": "G1_29",
                "hand": None,
                "root_pose_mode": "absolute",
            },
            "telemetry": {"robot_hz": 30.0},
        },
    )
    lowstate = SimpleNamespace(motor_state=[_state(i) for i in range(35)])
    odom = SimpleNamespace(
        position=[1.0, 2.0, 3.0],
        imu_state=SimpleNamespace(quaternion=[1.0, 0.0, 0.0, 0.0]),
    )

    collector._on_lowstate(lowstate)
    collector._on_odom(odom)

    assert collector._lowstate is lowstate
    assert collector._root_position == [1.0, 2.0, 3.0]
    assert not hasattr(collector, "_robot_loop")
    assert not hasattr(collector, "_odom_loop")


def test_brainco_dds_callbacks_keep_latest_hand_state_without_polling():
    collector = BrainCoStateCollector()
    message = SimpleNamespace(
        states=[SimpleNamespace(q=float(index)) for index in range(8)]
    )

    collector._on_left(message)

    assert collector.snapshot()["left"]["qpos"] == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    assert not hasattr(collector, "_read_loop")
