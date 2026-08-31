from __future__ import annotations

import math


def _finite_list(value):
    try:
        result = [float(item) for item in (value or [])]
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(item) for item in result) else None


def telemetry_to_unity_packet(frame: dict) -> dict:
    frame = frame if isinstance(frame, dict) else {}
    robot = frame.get("robot", {}) if isinstance(frame.get("robot"), dict) else {}
    hand = frame.get("end_effector", {}) if isinstance(frame.get("end_effector"), dict) else {}
    metadata = frame.get("metadata", {}) if isinstance(frame.get("metadata"), dict) else {}
    left = hand.get("left", {}) if isinstance(hand.get("left"), dict) else {}
    right = hand.get("right", {}) if isinstance(hand.get("right"), dict) else {}
    timestamp_ns = int(frame.get("timestamp_ns", 0) or 0)
    joint_names = list(metadata.get("joint_names", []) or [])
    left_names = list(metadata.get("left_hand_joint_names", []) or [])
    right_names = list(metadata.get("right_hand_joint_names", []) or [])
    return {
        "type": "robot_stream",
        "ts": timestamp_ns // 1_000_000 if timestamp_ns else None,
        "data": {
            "source": "direct_dds",
            "dds_online": bool(robot.get("joint_positions")),
            "odom_online": bool(robot.get("root_position")) and bool(robot.get("root_rotation")),
            "fk_online": False,
            "ipc_online": False,
            "direct_bus_online": True,
            "teleop_start": bool(frame.get("teleop_start", False)),
            "teleop_stop": bool(frame.get("teleop_stop", False)),
            "ready": bool(frame.get("ready", False)),
            "heartbeat": {},
            "robot": {"arm": metadata.get("arm", ""), "ee": metadata.get("ee", "")},
            "joint_names": joint_names,
            "record_joint_names": joint_names + left_names + right_names,
            "root_position": list(robot.get("root_position", []) or []),
            "root_rotation": list(robot.get("root_rotation", []) or []),
            "joint_positions": list(robot.get("joint_positions", []) or []),
            "joint_velocities": list(robot.get("joint_velocities", []) or []),
            "joint_torques": list(robot.get("joint_torques", []) or []),
            "electricity": list(robot.get("electricity", []) or []),
            "left_ee": _hand_state(left),
            "right_ee": _hand_state(right),
        },
    }


def _hand_state(value: dict) -> dict:
    """Preserve qpos while forwarding measured state and sample identity."""
    return {
        "qpos": list(value.get("qpos", []) or []),
        "qvel": list(value.get("qvel", []) or []),
        "current": list(value.get("current", []) or []),
        "sequence": int(value.get("sequence", 0) or 0),
        "timestamp_ns": int(value.get("timestamp_ns", 0) or 0),
    }


def validate_unity_packet(packet: dict) -> bool:
    if not isinstance(packet, dict) or not isinstance(packet.get("data"), dict):
        return False
    data = packet["data"]
    root_position = _finite_list(data.get("root_position"))
    root_rotation = _finite_list(data.get("root_rotation"))
    values = [
        _finite_list(data.get("joint_positions")),
        _finite_list(data.get("joint_velocities")),
        _finite_list(data.get("joint_torques")),
        _finite_list(data.get("electricity")),
    ]
    if root_position is None or len(root_position) not in (0, 3):
        return False
    if root_rotation is None or len(root_rotation) not in (0, 4):
        return False
    if any(value is None for value in values):
        return False
    if len({len(value) for value in values}) != 1:
        return False
    left = _finite_list(data.get("left_ee", {}).get("qpos", []))
    right = _finite_list(data.get("right_ee", {}).get("qpos", []))
    return left is not None and right is not None
