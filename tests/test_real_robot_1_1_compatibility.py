import math

import pytest
import yaml

from dexfull.bridge.controller import UnityController
from dexfull.control.utils.root_pose import RootPoseTransformer
from dexfull.imaging.timestamp_protocol import (
    ZMQImageFrame,
    encode_timestamped_jpeg,
    extract_timestamp_metadata,
)


def test_initial_heading_maps_robot_forward_to_unity_forward():
    transformer = RootPoseTransformer(
        mode="unity_relative",
        pelvis_height=0.76,
        vertical_mode="fixed",
    )
    half_yaw = math.pi / 4.0
    facing_world_y = [math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)]
    transformer.transform([10.0, 20.0, 0.70], facing_world_y)

    transport_position, _ = transformer.transform(
        [10.0, 20.4, 0.70], facing_world_y
    )

    assert transport_position == pytest.approx([0.0, 0.4, 0.76])


def test_quaternion_is_precompensated_for_existing_unity_converter():
    transformer = RootPoseTransformer(mode="unity_relative", pelvis_height=0.76)
    transformer.transform([0.0, 0.0, 0.70], [1.0, 0.0, 0.0, 0.0])

    _, transport = transformer.transform(
        [0.0, 0.0, 0.70], [0.5, 0.5, 0.5, 0.5]
    )

    assert transport == pytest.approx([0.5, -0.5, -0.5, -0.5])


def test_filtered_height_has_no_permanent_deadband_bias():
    transformer = RootPoseTransformer(
        mode="unity_relative",
        pelvis_height=0.76,
        vertical_mode="filtered",
        vertical_deadband=0.01,
        vertical_filter_alpha=1.0,
    )
    transformer.transform([0.0, 0.0, 0.70], [1.0, 0.0, 0.0, 0.0])
    position, _ = transformer.transform(
        [0.0, 0.0, 0.50], [1.0, 0.0, 0.0, 0.0]
    )
    assert position[2] == pytest.approx(0.56)


def test_basic_infos_is_a_superset_of_robot_1_1_unity_contract():
    config = {
        "control": {"robot": "G1_29", "hand": None, "input_mode": "hand"},
        "telemetry": {"robot_hz": 30.0},
        "basic_infos": {
            "version": "2.0.0",
            "input_device_frequency": 60.0,
            "push_data_frequency": 30.0,
            "image_width": 640,
            "image_height": 480,
            "image_fps": 30.0,
        },
        "zmq": {
            "host": "192.168.123.164",
            "head_port": 55555,
            "left_wrist_port": 55556,
            "right_wrist_port": 55557,
        },
    }
    controller = UnityController(None, None, None, None, None, None, config)

    result = controller.build_basic_infos()

    assert result["input_device_frenquency"] == 60.0
    assert result["push_data_frequency"] == 30.0
    assert [image["url"] for image in result["images"]] == [
        "tcp://192.168.123.164:55555",
        "tcp://192.168.123.164:55557",
        "tcp://192.168.123.164:55556",
    ]
    assert result["image"] == result["images"][0]


def test_camera_config_matches_robot_1_1_devices():
    with open("config/cam_config_server.yaml", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    assert config["head_camera"]["serial_number"] == 254322073111
    assert config["left_wrist_camera"]["type"] == "opencv"
    assert config["right_wrist_camera"]["enable_zmq"] is True


def test_timestamp_metadata_matches_robot_1_1_wire_format():
    encoded = encode_timestamped_jpeg(
        ZMQImageFrame(
            jpeg=b"\xff\xd8\xff\xd9",
            stream="head",
            sequence=7,
            capture_timestamp_ms=123456,
            width=640,
            height=480,
        )
    )

    metadata = extract_timestamp_metadata(encoded)

    assert metadata["capture_timestamp_ms"] == 123456
    assert "publish_timestamp_ms" not in metadata
