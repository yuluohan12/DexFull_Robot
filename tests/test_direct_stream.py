from dexfull.bridge.streamer import DirectStateStreamer
from dexfull.common.state_bus import LatestStateBus


def test_robot_snapshot_is_converted_once_without_ipc():
    bus = LatestStateBus()
    stream = DirectStateStreamer(bus)
    bus.publish_robot({
        "seq": 1,
        "timestamp_ns": 1_000_000,
        "robot": {
            "root_position": [0, 0, 0.793],
            "root_rotation": [1, 0, 0, 0],
            "joint_positions": [0.1],
            "joint_velocities": [0.0],
            "joint_torques": [0.0],
            "electricity": [1.0],
        },
        "end_effector": {"left": {"qpos": []}, "right": {"qpos": []}},
        "metadata": {
            "arm": "test", "ee": "", "joint_names": ["joint"],
            "left_hand_joint_names": [], "right_hand_joint_names": [],
        },
    })
    packet = stream.next_robot_packet()
    assert packet["data"]["ipc_online"] is False
    assert packet["data"]["source"] == "direct"
    assert stream.next_robot_packet() is None

