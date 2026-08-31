"""Timestamp metadata embedded in otherwise standard JPEG messages."""

import json
from dataclasses import dataclass
from typing import Any, Dict, Optional

_JPEG_SOI = b"\xff\xd8"
_JPEG_APP15 = b"\xff\xef"
_TIMESTAMP_METADATA_MAGIC = b"TELEIMAGER\x00"
_TIMESTAMP_PROTOCOL = "teleimager-jpeg-v2"


@dataclass(frozen=True)
class ZMQImageFrame:
    """A JPEG and the source metadata captured atomically with it."""

    jpeg: bytes
    stream: str
    sequence: int
    capture_timestamp_ms: int
    width: int = 0
    height: int = 0
    sensor_timestamp_ms: Optional[float] = None


def encode_timestamped_jpeg(frame: ZMQImageFrame) -> bytes:
    """Embed timing metadata in JPEG APP15 while preserving normal decoding."""
    jpeg = bytes(frame.jpeg)
    if not jpeg.startswith(_JPEG_SOI):
        raise ValueError("ZMQImageFrame payload is not a JPEG")

    metadata = {
        "protocol": _TIMESTAMP_PROTOCOL,
        "version": 2,
        "stream": frame.stream,
        "sequence": int(frame.sequence),
        "capture_timestamp_ms": int(frame.capture_timestamp_ms),
        "width": int(frame.width),
        "height": int(frame.height),
        "codec": "jpeg",
    }
    if frame.sensor_timestamp_ms is not None:
        metadata["sensor_timestamp_ms"] = float(frame.sensor_timestamp_ms)

    payload = _TIMESTAMP_METADATA_MAGIC + json.dumps(
        metadata,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("utf-8")
    segment_length = len(payload) + 2
    if segment_length > 0xFFFF:
        raise ValueError("JPEG timestamp metadata is too large")

    app15_segment = _JPEG_APP15 + segment_length.to_bytes(2, "big") + payload
    return jpeg[:2] + app15_segment + jpeg[2:]


def extract_timestamp_metadata(jpeg: bytes) -> Dict[str, Any]:
    """Read Teleimager APP15 metadata; return an empty dict for legacy JPEGs."""
    if not jpeg or not jpeg.startswith(_JPEG_SOI):
        return {}

    offset = 2
    size = len(jpeg)
    while offset + 4 <= size and jpeg[offset] == 0xFF:
        marker = jpeg[offset + 1]
        if marker in (0xD8, 0xD9):
            offset += 2
            continue
        if marker == 0xDA:
            break

        segment_length = int.from_bytes(jpeg[offset + 2:offset + 4], "big")
        if segment_length < 2 or offset + 2 + segment_length > size:
            break
        if marker == 0xEF:
            payload = jpeg[offset + 4:offset + 2 + segment_length]
            if payload.startswith(_TIMESTAMP_METADATA_MAGIC):
                try:
                    metadata = json.loads(
                        payload[len(_TIMESTAMP_METADATA_MAGIC):].decode("utf-8")
                    )
                    return metadata if isinstance(metadata, dict) else {}
                except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
                    return {}
        offset += 2 + segment_length
    return {}
