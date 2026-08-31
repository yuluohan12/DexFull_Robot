from __future__ import annotations

import copy
import os
import sys
from pathlib import Path

import yaml

from .common.paths import CONFIG_DIR, LOG_DIR, PROJECT_ROOT, STATUS_DIR, ensure_runtime_dirs


DEFAULT_CONFIG = {
    "ws": {
        "host": "0.0.0.0",
        "port": 7443,
        "path": "/ws",
        "compression": False,
        # Production fails closed if TLS is disabled or certificate files are
        # missing. This override exists only for isolated development.
        "allow_insecure_transport": False,
        "tls": {
            "enabled": True,
            "cert_file": "~/.config/dexfull/ws-tls/server-cert.pem",
            "key_file": "~/.config/dexfull/ws-tls/server-key.pem",
        },
    },
    "runtime": {
        # Spawned control process owns its cold Pinocchio/CasADi/retargeting
        # imports. First boot on Jetson can exceed one minute.
        "startup_wait": 120.0,
        "hand_startup_wait": 15.0,
        "auto_start_teleimager": True,
        "stop_timeout": 15.0,
        # XR/control restart is independent from hand hardware service restart.
        "stop_hand_with_control": False,
        "device_status_poll_seconds": 0.5,
        "device_stale_seconds": 2.0,
        # XR input sampling runs independently, so supervise progress of the
        # actual arm/control loop instead of treating input activity as health.
        "control_stall_seconds": 2.0,
        "device_startup_grace_seconds": 8.0,
    },
    "control": {
        "frequency": 60.0,
        "input_mode": "hand",
        "display_mode": "immersive",
        "robot": "G1_29",
        "hand": "brainco",
        "img_server_ip": "auto",
        "network_interface": None,
        "motion": True,
        # Process-wide pinning from the split XR service is unsafe after merge.
        "affinity": False,
        "headless": True,
        "simulation": False,
        "record": False,
        # Keep XR/hand sampling at the requested rate even when nonlinear IK
        # needs longer than one control period.
        "async_ik": True,
        # Pin only the IK worker thread. Unlike the legacy process-wide mode,
        # this does not constrain DDS, WebSocket, camera or hand processes.
        "ik_cpu_affinity": "auto",
        # Keep hand retargeting away from the upper-half IK cores.  The small
        # DDS writer is pinned separately so command publication is not starved
        # while both nonlinear optimizers are busy.
        "hand_retarget_cpu_affinity": "auto",
        "hand_dds_cpu_affinity": "auto",
        # Realtime teleoperation favours bounded latency over solving an
        # already obsolete pose to very high precision.
        "ik_max_iterations": 12,
        "performance_log_interval": 5.0,
        "root_pose_mode": "unity_relative",
        "root_pelvis_height": 0.76,
        "root_axis_mapping": "unitree_to_unity",
        "root_heading_reference": "initial",
        "root_vertical_mode": "filtered",
        "root_vertical_deadband": 0.01,
        "root_vertical_filter_alpha": 0.2,
    },
    "telemetry": {
        "robot_hz": 30.0,
        "vr_hz": 30.0,
        "enable_robot_ws": True,
        "enable_vr_ws": True,
    },
    "teleimager": {
        "enabled": True,
        "realsense": True,
        "affinity": False,
        "config": "cam_config_server.yaml",
        "auto_restart": True,
        "record_camera_frames": True,
        "camera_record_dir": "tools/camera_frame_records",
    },
    "hand_drivers": {
        "brainco": {
            "enabled": True,
            "isolation": "external_process",
            "service": "brainco",
            "auto_restart": True,
        },
        "dex1": {"enabled": True, "isolation": "in_process"},
        "dex3": {"enabled": True, "isolation": "in_process"},
        "inspire_dfx": {"enabled": True, "isolation": "in_process"},
        "inspire_ftp": {"enabled": True, "isolation": "in_process"},
    },
    "basic_infos": {
        "version": "2.3.3",
        "date": "",
        "author": "unitree",
        "robot_name": None,
        "hand_name": None,
        "control_type": None,
        "image_width": 640,
        "image_height": 480,
        "image_fps": 30.0,
        "input_device_frequency": 60.0,
        "push_data_frequency": 30.0,
        "depth_width": 0,
        "depth_height": 0,
        "depth_fps": 0.0,
        "audio_sample_rate": 0,
        "audio_channels": 0,
        "audio_format": "",
        "audio_bits": 0,
    },
    "zmq": {
        "host": "auto",
        "head_port": 55555,
        "left_wrist_port": 55556,
        "right_wrist_port": 55557,
    },
    "logging": {
        "level": "INFO",
        "format": "%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    },
}


def _deep_merge(base: dict, override: dict) -> dict:
    result = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _config_path() -> Path:
    value = os.environ.get("DEXFULL_CONFIG")
    return Path(value).expanduser() if value else CONFIG_DIR / "dexfull.yaml"


def _processes(config: dict) -> dict:
    tele_cfg = config["teleimager"]
    tele_cmd = [sys.executable, "-m", "dexfull.imaging.image_server"]
    if tele_cfg.get("realsense", True):
        tele_cmd.append("--rs")
    if not tele_cfg.get("affinity", False):
        tele_cmd.append("--no-affinity")
    if tele_cfg.get("record_camera_frames", False):
        tele_cmd.append("--record-camera-frames")

    camera_record_dir = Path(
        str(tele_cfg.get("camera_record_dir", "tools/camera_frame_records"))
    )
    if not camera_record_dir.is_absolute():
        camera_record_dir = PROJECT_ROOT / camera_record_dir
    tele_cmd.extend(["--camera-record-dir", str(camera_record_dir)])

    camera_config = Path(str(tele_cfg.get("config", "cam_config_server.yaml")))
    if not camera_config.is_absolute():
        camera_config = CONFIG_DIR / camera_config

    brainco_root = (
        PROJECT_ROOT
        / "dexfull"
        / "hand_drivers"
        / "brainco"
        / "native"
    )
    brainco_cfg = config["hand_drivers"]["brainco"]
    brainco_binary = brainco_root / "bin" / "brainco_hand_server"
    brainco_cmd = [str(brainco_binary)]
    network_interface = config["control"].get("network_interface")
    if network_interface:
        brainco_cmd.extend(["--network-interface", str(network_interface)])

    return {
        "teleimager": {
            "cmd": tele_cmd,
            "cwd": str(PROJECT_ROOT),
            "env": {
                "CAM_CONFIG_PATH": str(camera_config),
                "DEXFULL_DEVICE_STATUS_DIR": str(STATUS_DIR),
                "PYTHONUNBUFFERED": "1",
            },
            "log_path": str(LOG_DIR / "teleimager.log"),
            "auto_restart": bool(tele_cfg.get("auto_restart", True)),
            "max_restarts": 5,
            "restart_window_seconds": 60.0,
            "stable_reset_seconds": 60.0,
            "degraded_retry_seconds": 30.0,
            "start_timeout": 15.0,
            "cpu_affinity": None,
        },
        "brainco": {
            "cmd": brainco_cmd,
            "cwd": str(brainco_root),
            "env": {},
            "log_path": str(LOG_DIR / "brainco.log"),
            "auto_restart": bool(
                brainco_cfg.get("auto_restart", True)
            ),
            "max_restarts": 5,
            "restart_window_seconds": 60.0,
            "stable_reset_seconds": 60.0,
            "degraded_retry_seconds": 30.0,
            "start_timeout": 10.0,
            "cpu_affinity": None,
        },
    }


def load_config() -> dict:
    ensure_runtime_dirs()
    path = _config_path()
    file_config = {}
    if path.exists():
        with path.open("r", encoding="utf-8") as stream:
            file_config = yaml.safe_load(stream) or {}
    config = _deep_merge(DEFAULT_CONFIG, file_config)
    config["processes"] = _processes(config)
    config["config_path"] = str(path)
    return config


CONFIG = load_config()
