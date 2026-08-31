import json
import time
from dataclasses import asdict, dataclass, field, is_dataclass
from typing import Any, Optional


def _now() -> float:
    return time.time()


def _to_plain(value: Any) -> Any:
    if is_dataclass(value):
        return asdict(value)
    return value


def _drop_none(payload: dict) -> dict:
    return {k: v for k, v in payload.items() if v is not None}


@dataclass
class WsEnvelope:
    """
    Unity WebSocket protocol envelope.

    Field names intentionally match the Unity handoff document exactly:
    type, id, method, succeed, eventName, error_tip, timestamp, data.
    """

    type: str
    id: Optional[str] = None
    method: Optional[str] = None
    succeed: Optional[bool] = None
    eventName: Optional[str] = None
    error_tip: Optional[str] = None
    timestamp: Optional[float] = None
    data: Any = None

    @classmethod
    def from_json(cls, json_str: str) -> "WsEnvelope":
        try:
            raw = json.loads(json_str)
        except json.JSONDecodeError as e:
            raise ValueError(f"invalid_json: {e}") from e
        if not isinstance(raw, dict):
            raise ValueError("message_not_object")
        return cls(
            type=raw.get("type", ""),
            id=raw.get("id"),
            method=raw.get("method"),
            succeed=raw.get("succeed"),
            eventName=raw.get("eventName"),
            error_tip=raw.get("error_tip"),
            timestamp=raw.get("timestamp"),
            data=raw.get("data"),
        )

    def to_dict(self, include_none: bool = False) -> dict:
        payload = {
            "type": self.type,
            "id": self.id,
            "method": self.method,
            "succeed": self.succeed,
            "eventName": self.eventName,
            "error_tip": self.error_tip,
            "timestamp": self.timestamp if self.timestamp is not None else _now(),
            "data": _to_plain(self.data),
        }
        return payload if include_none else _drop_none(payload)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)

    @classmethod
    def build_pong(cls, ping: "WsEnvelope") -> "WsEnvelope":
        return cls(
            type="pong",
            id=ping.id,
            timestamp=ping.timestamp,
        )

    @classmethod
    def build_response(
        cls,
        *,
        method: Optional[str],
        succeed: bool,
        id: Optional[str] = None,
        timestamp: Optional[float] = None,
        data: Any = None,
        error_tip: Optional[str] = None,
    ) -> "WsEnvelope":
        return cls(
            type="response",
            id=id,
            method=method,
            succeed=succeed,
            error_tip=error_tip,
            timestamp=timestamp,
            data=data,
        )

    @classmethod
    def build_event(
        cls,
        *,
        eventName: str,
        data: Any,
        id: Optional[str] = None,
        timestamp: Optional[float] = None,
    ) -> "WsEnvelope":
        return cls(
            type="event",
            id=id,
            eventName=eventName,
            timestamp=timestamp,
            data=data,
        )

    @classmethod
    def build_error(
        cls,
        *,
        error_tip: str = "",
        id: Optional[str] = None,
        timestamp: Optional[float] = None,
        method: Optional[str] = None,
        data: Any = None,
    ) -> "WsEnvelope":
        return cls(
            type="error",
            id=id,
            method=method,
            error_tip=error_tip,
            timestamp=timestamp,
            data=data,
        )


@dataclass
class RobotDatas:
    # Unity-facing robot_datas core fields. Extended fields are added by RobotDataAdapter.
    positions: list = field(default_factory=list)
    rotations: list = field(default_factory=list)
    velocities: list = field(default_factory=list)
    torques: list = field(default_factory=list)
    angles: list = field(default_factory=list)
    electricity: list = field(default_factory=list)

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class VRInputDatas:
    controller_input: str = ""
    hmd_pose: list = field(default_factory=list)
    left_controller_pose: list = field(default_factory=list)
    right_controller_pose: list = field(default_factory=list)

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class imageobj:
    url: str=""
    width: int = 0
    height: int = 0
    fps: float = 0.0


@dataclass
class depthobj:
    width: int = 0
    height: int = 0
    fps: float = 0.0


@dataclass
class audioobj:
    sample_rate: int = 0
    channels: int = 0
    format: str = ""
    bits: int = 0


@dataclass
class BasicInfos:
    version: str = "1.0.0"
    date: str = ""
    author: str = "unitree"
    robot_name: str = ""
    hand_name: str = ""
    control_type: str = ""
    # Keep the field spelling used by the robot-side 1.1 Unity contract.
    input_device_frenquency: float = 0.0
    push_data_frequency: float = 0.0
    image: imageobj = field(default_factory=imageobj)
    images: list[imageobj] = field(default_factory=list)
    depth: depthobj = field(default_factory=depthobj)
    audio: audioobj = field(default_factory=audioobj)
    joint_names: list = field(default_factory=list)

    def to_dict(self) -> dict:
        return asdict(self)


def parse_message(json_str: str) -> dict:
    try:
        msg = json.loads(json_str)
        if not isinstance(msg, dict):
            return {"type": "error", "error": "msg_not_dict"}
        return msg
    except json.JSONDecodeError as e:
        return {"type": "error", "error": f"invalid_json: {e}"}


def is_request(msg: dict) -> bool:
    return msg.get("type") == "request"


def is_response(msg: dict) -> bool:
    return msg.get("type") == "response"


def is_event(msg: dict) -> bool:
    return msg.get("type") == "event"


def is_stream(msg: dict) -> bool:
    return msg.get("type") == "stream"


def validate_request(msg: dict) -> Optional[str]:
    if not is_request(msg):
        return "message type is not request"
    if "id" not in msg:
        return "missing id"
    if "method" not in msg:
        return "missing method"
    return None
