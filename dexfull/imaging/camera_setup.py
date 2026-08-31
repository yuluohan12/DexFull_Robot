"""Discover cameras and write a per-robot DexFull hardware profile."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from .camera_config import DEFAULT_HARDWARE_CONFIG
from .image_server import CameraFinder


def _inventory(finder: CameraFinder) -> dict:
    return {
        "realsense_serials": [str(value) for value in finder.rs_serial_numbers],
        "video_devices": [
            {
                "video_path": path,
                "video_id": item.get("video_id"),
                "serial_number": item.get("serial_number"),
                "physical_path": item.get("physical_path"),
                "uid": item.get("uid"),
            }
            for path, item in finder.uvc_rgb_cameras.items()
        ],
    }


def _selector(finder: CameraFinder, video_path: str) -> dict:
    item = finder.uvc_rgb_cameras.get(video_path)
    if item is None:
        raise SystemExit(f"Camera {video_path} is not an available RGB device")
    # Hardware serial is portable across USB ports. Many wrist cameras expose
    # no serial; in that case retain physical USB topology and rediscover the
    # volatile /dev/videoN on every reconnect.
    if item.get("serial_number"):
        return {
            "serial_number": str(item["serial_number"]),
            "physical_path": None,
            "video_id": None,
        }
    return {
        "serial_number": None,
        "physical_path": item.get("physical_path"),
        "video_id": None,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Scan cameras or commission a per-robot hardware profile"
    )
    parser.add_argument("--scan", action="store_true")
    parser.add_argument("--commission", action="store_true")
    parser.add_argument("--rs", action="store_true", help="discover RealSense")
    parser.add_argument("--head-serial")
    parser.add_argument("--left-video", help="e.g. /dev/video2")
    parser.add_argument("--right-video", help="e.g. /dev/video4")
    parser.add_argument("--output", default=str(DEFAULT_HARDWARE_CONFIG))
    args = parser.parse_args()
    if not args.scan and not args.commission:
        parser.error("choose --scan or --commission")

    finder = CameraFinder(args.rs, verbose=False, reload_driver=False)
    inventory = _inventory(finder)
    if args.scan:
        print(json.dumps(inventory, ensure_ascii=False, indent=2))
        if not args.commission:
            return

    if not args.left_video or not args.right_video:
        parser.error("--commission requires --left-video and --right-video")
    head_serial = args.head_serial
    if head_serial is None:
        if len(finder.rs_serial_numbers) != 1:
            parser.error(
                "--head-serial is required unless exactly one RealSense is connected"
            )
        head_serial = str(finder.rs_serial_numbers[0])
    if str(head_serial) not in [str(value) for value in finder.rs_serial_numbers]:
        parser.error(f"RealSense serial {head_serial} is not connected")

    profile = {
        "cameras": {
            "head_camera": {
                "serial_number": str(head_serial),
                "physical_path": None,
                "video_id": None,
            },
            "left_wrist_camera": _selector(finder, args.left_video),
            "right_wrist_camera": _selector(finder, args.right_video),
        }
    }
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(profile, stream, sort_keys=False, allow_unicode=True)
    print(f"Hardware profile written to {output}")


if __name__ == "__main__":
    main()
