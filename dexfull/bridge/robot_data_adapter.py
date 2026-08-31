try:
    from .message import WsEnvelope
except ImportError:
    from message import WsEnvelope


class RobotDataAdapter:
    """
    Convert internal robot realtime packets into Unity realtime events.

    The input packet is produced by RobotDataStreamer. The output is split into
    robot_state and robot_datas events sent to Unity over WebSocket.
    """

    @classmethod
    def to_robot_datas_event(cls, packet: dict) -> dict:
        data = packet.get("data", {}) if isinstance(packet, dict) else {}
        ts = packet.get("ts") if isinstance(packet, dict) else None
        joint_positions = list(data.get("joint_positions", []))
        joint_velocities = list(data.get("joint_velocities", []))
        joint_torques = list(data.get("joint_torques", []))
        electricity = list(data.get("electricity", []))
        left_ee = data.get("left_ee", {}) if isinstance(data.get("left_ee", {}), dict) else {}
        right_ee = data.get("right_ee", {}) if isinstance(data.get("right_ee", {}), dict) else {}
        left_ee_qpos = list(left_ee.get("qpos", []))
        right_ee_qpos = list(right_ee.get("qpos", []))
        hand_qpos = left_ee_qpos + right_ee_qpos
        hand_qvel = cls._sized_values(
            left_ee.get("qvel", []), len(left_ee_qpos)
        ) + cls._sized_values(right_ee.get("qvel", []), len(right_ee_qpos))
        hand_current = cls._sized_values(
            left_ee.get("current", []), len(left_ee_qpos)
        ) + cls._sized_values(right_ee.get("current", []), len(right_ee_qpos))


        payload = {
            # Unix epoch milliseconds, same unit as Teleimager image metadata.
            "robot_timestamp": ts,
            "robot_timestamp_ms": ts,
            "positions": data.get("root_position", []),
            "rotations": data.get("root_rotation", []),
            "velocities": joint_velocities + hand_qvel,
            # BrainCo exposes measured motor current, not calibrated torque.
            "torques": joint_torques + [0.0] * len(hand_qpos),
            "angles": joint_positions + hand_qpos,
            "electricity": electricity + hand_current,
            "left_ee_pose": data.get("left_ee_pose", []),
            "right_ee_pose": data.get("right_ee_pose", []),
        }
        return WsEnvelope.build_event(eventName="robot_datas", timestamp=ts, data=payload).to_dict()

    @staticmethod
    def _sized_values(value, size: int) -> list:
        values = list(value or [])
        if len(values) < size:
            values.extend([0.0] * (size - len(values)))
        return values[:size]
        
    @classmethod
    def to_robot_state_event(cls, packet: dict) -> dict:
        data = packet.get("data", {}) if isinstance(packet, dict) else {}
        ts = packet.get("ts") if isinstance(packet, dict) else None

        payload = {
            "robot_timestamp": ts,
            "robot_timestamp_ms": ts,
            "robot": data.get("robot", {}),
            "source": data.get("source"),
            "dds_online": data.get("dds_online", False),
            "odom_online": data.get("odom_online", False),
            "fk_online": data.get("fk_online", False),
            "ipc_online": data.get("ipc_online", False),
            "teleop_start": data.get("teleop_start", False),
            "teleop_stop": data.get("teleop_stop", False),
            "ready": data.get("ready", False),
            "heartbeat": data.get("heartbeat", {}),
        }
        return WsEnvelope.build_event(eventName="robot_state", timestamp=ts, data=payload).to_dict()
