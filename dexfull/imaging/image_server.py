# Copyright 2025 YuShu TECHNOLOGY CO.,LTD ("Unitree Robotics")
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import logging_mp
from dexfull.common.logging_mp_config import configure_logging_mp

configure_logging_mp(logging_mp)
logger_mp = logging_mp.getLogger(__name__)
import os
import argparse
import glob
import cv2
import numpy as np
# uvc will be imported when needed
import yaml
import time
import threading
import signal
import functools
import subprocess
import platform
from .image_client import (
    TripleRingBuffer,
    ZMQ_PublisherManager,
    ZMQ_Responser,
)
from .timestamp_protocol import ZMQImageFrame
from .camera_config import load_camera_config
from .opencv_worker import OpenCVCaptureProcess
from .realsense_worker import RealSenseCaptureProcess
from .v4l2_discovery import (
    is_primary_capture_node,
    resolve_physical_video_path,
)
from dexfull.tools import camera_frame_recorder
from dexfull.common.device_status import (
    DeviceStatusWriter,
    DISCONNECTED,
    ONLINE,
    RECONNECTING,
)
# webrtc dependencies
import asyncio
import json
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from aiortc.rtcrtpsender import RTCRtpSender
from aiortc.contrib.media import MediaRelay
from aiortc.codecs import h264
import av
import ssl
from pathlib import Path
import queue
import fractions
from typing import Dict, Optional, Tuple, Any

# ========================================================
# cam_config_server.yaml path
# ========================================================
from pathlib import Path
CONFIG_PATH = os.path.normpath(
    os.environ.get(
        "CAM_CONFIG_PATH",
        str(Path(__file__).resolve().parents[2] / "config" / "cam_config_server.yaml"),
    )
)

# ========================================================
# certificate and key paths
# ========================================================
module_dir = Path(__file__).resolve().parent.parent.parent
default_cert = module_dir / "cert.pem"
default_key = module_dir / "key.pem"
env_cert = os.getenv("XR_TELEOP_CERT")
env_key = os.getenv("XR_TELEOP_KEY")
user_config_dir = Path.home() / ".config" / "dexfull"
user_cert = user_config_dir / "cert.pem"
user_key = user_config_dir / "key.pem"
CERT_PEM_PATH = Path(env_cert or (user_cert if user_cert.exists() else default_cert))
KEY_PEM_PATH = Path(env_key or (user_key if user_key.exists() else default_key))
CERT_PEM_PATH = CERT_PEM_PATH.resolve()
KEY_PEM_PATH = KEY_PEM_PATH.resolve()

# ========================================================
# libx264 for Jetson (Patch h264 Encoder)
# ========================================================
def jetson_software_encode_frame(self, frame: av.VideoFrame, force_keyframe: bool):
    if self.codec and (frame.width != self.codec.width or frame.height != self.codec.height):
        self.codec = None

    if self.codec is None:
        try:
            self.codec = av.CodecContext.create("libx264", "w")
            self.codec.width = frame.width
            self.codec.height = frame.height
            self.codec.bit_rate = self.target_bitrate
            self.codec.pix_fmt = "yuv420p"
            self.codec.framerate = fractions.Fraction(30, 1)
            self.codec.time_base = fractions.Fraction(1, 30)
        
            self.codec.options = {
                "preset": "ultrafast",
                "tune": "zerolatency",
                "threads": "1",
                "g": "60",
            }
            self.frame_count = 0
            force_keyframe = True
        except Exception as e:
            logger_mp.error(f"[H264 Patch] Initialization failed: {e}")
            return

    if not force_keyframe and hasattr(self, "frame_count") and self.frame_count % 60 == 0:
        force_keyframe = True
    
    self.frame_count = self.frame_count + 1 if hasattr(self, "frame_count") else 1
    frame.pict_type = av.video.frame.PictureType.I if force_keyframe else av.video.frame.PictureType.NONE

    try:
        for packet in self.codec.encode(frame):
            data = bytes(packet)
            if data:
                yield from self._split_bitstream(data)
    except Exception as e:
        logger_mp.warning(f"[H264 Patch] Encode error: {e}")

h264.H264Encoder._encode_frame = jetson_software_encode_frame

# ========================================================
# Embed HTML and JS directly
# ========================================================
INDEX_HTML = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8"/>
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>WebRTC Stream</title>
    <style>
    body { 
        font-family: sans-serif; 
        background: #fff; 
        color: #000; 
        text-align: center; 
    }
    button { padding: 10px 20px; font-size: 16px; cursor: pointer; }
    video { width: 100%; max-width: 1280px; background: #000; margin-top: 10px; }
    
    /* Title link style */
    h1 a {
        text-decoration: none;
        color: #000;
    }
    h1 a:hover {
        color: #555;
    }
    </style>
</head>
<body>
sah
    <button id="start" onclick="start()">Start</button>
    <button id="stop" style="display: none" onclick="stop()">Stop</button>
    
    <div id="media">
        <video id="video" autoplay playsinline muted></video>
        <audio id="audio" autoplay></audio>
    </div>
    
    <script src="client.js"></script>
</body>
</html>
"""

CLIENT_JS = """
var pc = null;

function negotiate() {
    pc.addTransceiver('video', { direction: 'recvonly' });
    return pc.createOffer().then((offer) => {
        return pc.setLocalDescription(offer);
    }).then(() => {
        return new Promise((resolve) => {
            if (pc.iceGatheringState === 'complete') {
                resolve();
            } else {
                const checkState = () => {
                    if (pc.iceGatheringState === 'complete') {
                        pc.removeEventListener('icegatheringstatechange', checkState);
                        resolve();
                    }
                };
                pc.addEventListener('icegatheringstatechange', checkState);
            }
        });
    }).then(() => {
        var offer = pc.localDescription;
        return fetch('/offer', {
            body: JSON.stringify({
                sdp: offer.sdp,
                type: offer.type,
            }),
            headers: {
                'Content-Type': 'application/json'
            },
            method: 'POST'
        });
    }).then((response) => {
        return response.json();
    }).then((answer) => {
        return pc.setRemoteDescription(answer);
    }).catch((e) => {
        alert(e);
    });
}

function start() {
    var config = {
        sdpSemantics: 'unified-plan'
    };

    // Removed STUN server check logic completely

    pc = new RTCPeerConnection(config);

    pc.addEventListener('track', (evt) => {
        if (evt.track.kind == 'video') {
            document.getElementById('video').srcObject = evt.streams[0];
        } else {
            document.getElementById('audio').srcObject = evt.streams[0];
        }
    });

    document.getElementById('start').style.display = 'none';
    negotiate();
    document.getElementById('stop').style.display = 'inline-block';
}

function stop() {
    document.getElementById('stop').style.display = 'none';
    document.getElementById('start').style.display = 'inline-block';
    if (pc) {
        pc.close();
        pc = null;
    }
}
"""

# ========================================================
# WebRTC publish
# ========================================================
class BGRArrayVideoStreamTrack(MediaStreamTrack):
    """MediaStreamTrack exposing BGR ndarrays as av.VideoFrame (latest-frame semantics)."""
    kind = "video"

    def __init__(self):
        super().__init__()
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=1)
        self._start_time = None
        self._pts = 0

    async def recv(self) -> av.VideoFrame:
        # This will suspend execution until a frame is available
        # preventing CPU busy-waiting
        frame = await self._queue.get()
        return frame

    def push_frame(self, bgr_numpy: np.ndarray, loop: Optional[asyncio.AbstractEventLoop] = None):
        if bgr_numpy is None:
            return

        # 1. Convert and calculate PTS immediately
        # MediaRelay requires consistent PTS to function correctly
        try:
            video_frame = av.VideoFrame.from_ndarray(bgr_numpy, format="bgr24")
            
            if self._start_time is None:
                self._start_time = time.time()
                self._pts = 0
            else:
                # 90000 is the standard RTP clock rate for video
                # This ensures smooth playback
                self._pts = int((time.time() - self._start_time) * 90000)
            
            video_frame.pts = self._pts
            video_frame.time_base = fractions.Fraction(1, 90000)
            
        except Exception as e:
            logger_mp.debug(f"Conversion failed: {e}")
            return

        # 2. Push to queue thread-safely
        target_loop = loop or asyncio.get_event_loop()
        if target_loop.is_closed():
            return
            
        def _put():
            try:
                # Drop old frame if queue is full (Low Latency strategy)
                if self._queue.full():
                    self._queue.get_nowait()
                self._queue.put_nowait(video_frame)
            except Exception:
                pass

        target_loop.call_soon_threadsafe(_put)


class WebRTC_PublisherThread(threading.Thread):
    """
    Runs aiohttp + aiortc in a separate THREAD (not Process).
    This enables shared memory and removes Pickling overhead.
    """
    def __init__(self, port: int, host: str = "0.0.0.0", codec_pref: str = None):
        super().__init__(daemon=True)
        self._host = host
        self._port = port
        self._codec_pref = codec_pref
        self._app = web.Application()
        self._runner: Optional[web.AppRunner] = None
        self._pcs = set()
        self._start_event = threading.Event()
        self._stop_event = threading.Event()
        self._frame_queue = queue.Queue(maxsize=1)

        self._bgr_track: Optional[BGRArrayVideoStreamTrack] = None
        self._relay: Optional[MediaRelay] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None

        # register routes
        self._app.router.add_get("/", self._index)
        self._app.router.add_get("/client.js", self._javascript)
        self._app.router.add_post("/offer", self._offer)

        self._app.router.add_options("/", self._options)
        self._app.router.add_options("/client.js", self._options)
        self._app.router.add_options("/offer", self._options)

    async def _index(self, request: web.Request) -> web.Response:
        return web.Response(content_type="text/html", text=INDEX_HTML)
    
    async def _javascript(self, request: web.Request) -> web.Response:
        return web.Response(content_type="application/javascript", text=CLIENT_JS)

    async def _options(self, request):
        return web.Response(
            status=200,
            headers={
                "Access-Control-Allow-Origin": "*",
                "Access-Control-Allow-Methods": "POST, GET, OPTIONS",
                "Access-Control-Allow-Headers": "Content-Type",
            }
        )

    async def _offer(self, request: web.Request) -> web.Response:
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        pc = RTCPeerConnection()
        self._pcs.add(pc)

        # CORE LOGIC: Use MediaRelay to subscribe
        # This ensures encoding happens only once globally
        if self._bgr_track and self._relay:
            try:
                relayed_track = self._relay.subscribe(self._bgr_track)
                transceiver = pc.addTransceiver(relayed_track, direction="sendonly")
                capabilities = RTCRtpSender.getCapabilities("video")
                pref = (self._codec_pref or "h264").lower()

                if pref == "h264":
                    h264_codecs = [c for c in capabilities.codecs if c.mimeType == "video/H264"]
                    if h264_codecs:
                        transceiver.setCodecPreferences(h264_codecs)
                        logger_mp.info(f"[WebRTC] Preferred H264 for port:{self._port}")
                    else:
                        logger_mp.warning(f"[WebRTC] H264 preferred but not found, using auto-negotiation for port:{self._port}")
                        
                elif pref == "vp8":
                    vp8_codecs = [c for c in capabilities.codecs if c.mimeType == "video/VP8"]
                    if vp8_codecs:
                        transceiver.setCodecPreferences(vp8_codecs)
                        logger_mp.info(f"[WebRTC] Preferred VP8 for port:{self._port}")
                    else:
                        logger_mp.warning(f"[WebRTC] VP8 preferred but not found, using auto-negotiation for port:{self._port}")
                
                else:
                    h264_codecs = [c for c in capabilities.codecs if c.mimeType == "video/H264"]
                    if h264_codecs:
                        transceiver.setCodecPreferences(h264_codecs)
                        logger_mp.info(f"[WebRTC] Preferred codec '{pref}' not found, falling back to H264 for port:{self._port}")
                    else:
                        logger_mp.warning(f"[WebRTC] Preferred codec '{pref}' not found, using auto-negotiation for port:{self._port}")
                    
            except Exception as e:
                logger_mp.error(f"Relay subscription failed: {e}")

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            if pc.connectionState in ["failed", "closed"]:
                await self._cleanup_pc(pc)

        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}),
            headers={
                "Access-Control-Allow-Origin": "*",
                "Access-Control-Allow-Methods": "POST, OPTIONS",
                "Access-Control-Allow-Headers": "Content-Type",
            }
        )

    async def _cleanup_pc(self, pc):
        self._pcs.discard(pc)
        try:
            await pc.close()
        except: pass

    def wait_for_start(self, timeout=1.0):
        return self._start_event.wait(timeout=timeout)

    def run(self):
        # Create a new Event Loop for this thread
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        
        async def _main():
            self._runner = web.AppRunner(self._app)
            await self._runner.setup()
            
            # Init Track and Relay inside the loop
            self._bgr_track = BGRArrayVideoStreamTrack()
            self._relay = MediaRelay()

            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_context.load_cert_chain(CERT_PEM_PATH, KEY_PEM_PATH)
            site = web.TCPSite(self._runner, self._host, self._port, ssl_context=ssl_context)
            await site.start()
            self._start_event.set()
            
            # Frame Pushing Loop
            while not self._stop_event.is_set():
                try:
                    # Non-blocking check for new frames
                    if not self._frame_queue.empty():
                        # Get frame (no pickling overhead in Threads!)
                        frame = self._frame_queue.get_nowait()
                        self._bgr_track.push_frame(frame, loop=self._loop)
                    
                    # CRITICAL: Yield control to asyncio loop to handle WebRTC packets
                    await asyncio.sleep(0.005)
                except Exception:
                    await asyncio.sleep(0.005)

        try:
            self._loop.run_until_complete(_main())
        except Exception as e:
            logger_mp.error(f"WebRTC Thread Error: {e}")
        finally:
            if self._loop: self._loop.close()

    def send(self, data: np.ndarray):
        """Send data to the processing thread."""
        # Simple drop-frame logic if queue is full
        if not self._frame_queue.full():
            self._frame_queue.put(data)
        else:
            try:
                self._frame_queue.get_nowait()
                self._frame_queue.put(data)
            except: pass

    def stop(self):
        self._stop_event.set()
        self.join(timeout=1.0)


# ========================================================
# WebRTC Manager
# ========================================================
class WebRTC_PublisherManager:
    """Manages WebRTC_PublisherThreads."""
    _instance: Optional["WebRTC_PublisherManager"] = None
    _publisher_threads: Dict[Tuple[str, int], WebRTC_PublisherThread] = {}
    _lock = threading.Lock()
    _running = True

    def __init__(self):
        pass

    @classmethod
    def get_instance(cls) -> "WebRTC_PublisherManager":
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance

    def _create_publisher(self, port: int, host: str, codec_pref: str):
        t = WebRTC_PublisherThread(port, host, codec_pref)
        t.start()
        if not t.wait_for_start(timeout=10.0):  # Increase timeout to 10 seconds
             raise ConnectionError("Publisher failed to start (Timeout)")
        return t

    def _get_publisher(self, port, host, codec_pref):
        key = (host, port)
        with self._lock:
            if key not in self._publisher_threads:
                self._publisher_threads[key] = self._create_publisher(port, host, codec_pref)
            return self._publisher_threads[key]

    def publish(self, data: Any, port: int, host: str = "0.0.0.0", codec_pref: str = None) -> None:
        if not self._running: return
        try:
            pub = self._get_publisher(port, host, codec_pref)
            pub.send(data)
        except Exception as e:
            logger_mp.error(f"Unexpected error in publish: {e}")
            pass

    def close(self) -> None:
        self._running = False
        with self._lock:
            for key, pub in list(self._publisher_threads.items()):
                try:
                    pub.stop()
                except Exception: pass
            self._publisher_threads.clear()

# ========================================================
# UVC driver reload
# ========================================================
def reload_uvc_driver():
    """Reload uvcvideo only when the process already has root privileges.

    The packaged Teleimager runs as the unprivileged ``unitree`` user.
    It must never invoke interactive sudo from a systemd child process.
    """
    if os.geteuid() != 0:
        logger_mp.info(
            "Skipping UVC driver reload: root privileges are unavailable."
        )
        return False

    try:
        subprocess.run(
            ["modprobe", "-r", "uvcvideo"],
            check=True,
        )
        time.sleep(1)

        subprocess.run(
            ["modprobe", "uvcvideo", "debug=0"],
            check=True,
        )
        time.sleep(1)

        logger_mp.info("UVC driver reloaded successfully.")
        return True

    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        logger_mp.warning(f"Failed to reload UVC driver: {e}")
        return False

# ========================================================
# camera finder and cameras
# ========================================================
class CameraFinder:
    """
    Discover connected cameras and their properties.
    vpath: /dev/videoX
    ppath: physical path in /sys/class/video4linux, e.g. /sys/devices/pci0000:00/0000:00:14.0/usb1/1-11/1-11.2/1-11.2:1.0
    uid: USB unique ID, e.g. "001:002"
    dev_info: extra info from uvc
    sn: serial number of the camera
    """
    def __init__(self, realsense_enable=False, verbose=False, reload_driver=True,
                 probe_video_frames=True):
        self.verbose = verbose
        self._probe_video_frames = bool(probe_video_frames)
        # uvc
        if reload_driver:
            reload_uvc_driver()
        import uvc
        self.uvc_devices = uvc.device_list()
        self.uid_map = {dev["uid"]: dev for dev in self.uvc_devices}
        # all video devices
        self.video_paths = self._list_video_paths()
        # realsense
        if realsense_enable:
            self.rs_serial_numbers = self._list_realsense_serial_numbers()
            self.rs_video_paths = self._list_realsense_video_paths()
            if self._probe_video_frames:
                self.rs_rgb_video_paths = [
                    p for p in self.rs_video_paths if self._is_like_rgb(p)
                ]
            else:
                self.rs_rgb_video_paths = [
                    p for p in self.rs_video_paths if is_primary_capture_node(p)
                ]
        else:
            self.rs_serial_numbers = []
            self.rs_video_paths = []
            self.rs_rgb_video_paths = []
        # rgb & uvc
        self.uvc_rgb_video_paths = self._list_uvc_rgb_video_paths()
        self.uvc_rgb_video_ids = [int(v.replace("/dev/video", "")) for v in self.uvc_rgb_video_paths]
        self.uvc_rgb_physical_paths = [self._get_ppath_from_vpath(v) for v in self.uvc_rgb_video_paths]
        self.uvc_rgb_uids = [self._get_uid_from_ppath(p) for p in self.uvc_rgb_physical_paths]
        self.uvc_rgb_dev_info = [self.uid_map.get(uid) for uid in self.uvc_rgb_uids]
        self.uvc_rgb_serial_numbers = [dev_info.get("serialNumber") if dev_info else None for dev_info in self.uvc_rgb_dev_info]
        # all uvc cameras
        self.uvc_rgb_cameras = {}
        for vpath, vid, ppath, uid, dev_info, sn in zip(
            self.uvc_rgb_video_paths,
            self.uvc_rgb_video_ids,
            self.uvc_rgb_physical_paths,
            self.uvc_rgb_uids,
            self.uvc_rgb_dev_info,
            self.uvc_rgb_serial_numbers,
        ):
            self.uvc_rgb_cameras[vpath] = {
                "video_id": vid,
                "physical_path": ppath,
                "uid": uid,
                "dev_info": dev_info,
                "serial_number": sn
            }
        if self.verbose:
            self.info()

    # utils
    def _list_video_paths(self):
        base = "/sys/class/video4linux/"
        if not os.path.exists(base):
            return []
        return [f"/dev/{x}" for x in sorted(os.listdir(base)) if x.startswith("video")]

    def _list_uvc_rgb_video_paths(self):
        candidates = [p for p in self.video_paths if p not in self.rs_video_paths]
        if self._probe_video_frames:
            return [p for p in candidates if self._is_like_rgb(p)]
        return [p for p in candidates if is_primary_capture_node(p)]

    def _list_realsense_video_paths(self):
        def _read_text(path):
            try:
                with open(path, "r", encoding="utf-8", errors="ignore") as f:
                    return f.read().strip()
            except Exception:
                return None

        def _parent_usb_device_sysdir(video_sysdir):
            d = os.path.realpath(os.path.join(video_sysdir, "device"))
            for _ in range(10):
                if d is None or d == "/" or not os.path.isdir(d):
                    break
                id_vendor = _read_text(os.path.join(d, "idVendor"))
                id_product = _read_text(os.path.join(d, "idProduct"))
                if id_vendor and id_product:
                    return d
                d_next = os.path.dirname(d)
                if d_next == d:
                    break
                d = d_next
            return None

        ports = []
        for devnode in sorted(glob.glob("/dev/video*")):
            sysdir = f"/sys/class/video4linux/{os.path.basename(devnode)}"
            name = _read_text(os.path.join(sysdir, "name"))
            usb_dir = _parent_usb_device_sysdir(sysdir)
            vendor_id = _read_text(os.path.join(usb_dir, "idVendor")) if usb_dir else None

            # Match RealSense by name and Intel vendor ID
            if name and "realsense" in name.lower() and (vendor_id or "").lower() in ("8086", "32902"):
                ports.append(devnode)

        return ports
    
    def get_realsense_module(self) -> object:
        try:
            import pyrealsense2 as rs
            return rs
        except ImportError:
            arch = platform.machine()
            system = platform.system()
            print(f"[RealSense] Platform: {system} / {arch}")

            if system == "Linux" and arch.startswith("aarch64"):
                # Jetson NX / arm64
                msg = (
                    "[RealSense] pyrealsense2 not installed. please build from source:\n"
                    "    cd ~\n"
                    "    git clone https://github.com/IntelRealSense/librealsense.git\n"
                    "    cd librealsense\n"
                    "    git checkout v2.50.0\n"
                    "    mkdir build && cd build\n"
                    "    cmake .. -DBUILD_PYTHON_BINDINGS=ON -DPYTHON_EXECUTABLE=$(which python3)\n"
                    "    make -j$(nproc)\n"
                    "    sudo make install\n"
                )
            else:
                # x86/x64
                msg = (
                    "[RealSense] pyrealsense2 not installed. You can try:\n"
                    "    pip install pyrealsense2\n"
                )
            raise RuntimeError(msg)

    def _list_realsense_serial_numbers(self):
        rs = self.get_realsense_module()
        ctx = rs.context()
        devices = ctx.query_devices()
        serials = []
        for dev in devices:
            try:
                serials.append(dev.get_info(rs.camera_info.serial_number))
            except Exception:
                continue
        return serials

    def _get_ppath_from_vpath(self, video_path):
        sysfs_path = f"/sys/class/video4linux/{os.path.basename(video_path)}/device"
        return os.path.realpath(sysfs_path)

    def _get_uid_from_ppath(self, physical_path):
        def read_file(path):
            return open(path).read().strip() if os.path.exists(path) else None

        busnum_file = os.path.join(physical_path, "busnum")
        devnum_file = os.path.join(physical_path, "devnum")

        if not (os.path.exists(busnum_file) and os.path.exists(devnum_file)):
            parent = os.path.dirname(physical_path)
            busnum_file = os.path.join(parent, "busnum")
            devnum_file = os.path.join(parent, "devnum")

        if os.path.exists(busnum_file) and os.path.exists(devnum_file):
            bus = read_file(busnum_file)
            dev = read_file(devnum_file)
            return f"{bus}:{dev}"
        return None

    def _is_like_rgb(self, video_path):
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            return False
        ret, frame = cap.read()
        cap.release()
        return ret and frame is not None and frame.ndim == 3 and frame.shape[2] == 3

    # --------------------------------------------------------
    # public api
    # --------------------------------------------------------
    def is_rs_serial_exist(self, serial_number):
        return str(serial_number) in self.rs_serial_numbers

    def is_vpath_exist(self, vpath):
        return vpath in self.video_paths
    
    def is_ppath_exist(self, physical_path):
        for cam in self.uvc_rgb_cameras.values():
            if cam.get("physical_path") == physical_path:
                return True
        return False
    
    def get_uid_by_sn(self, serial_number):
        matches = [
            cam for cam in self.uvc_rgb_cameras.values()
            if cam.get("serial_number") == str(serial_number)
        ]
        if not matches:
            return None
        if len(matches) > 1:
            raise ValueError(f"Multiple cameras found with serial number {serial_number}")
        return matches[0].get("uid")

    def get_uid_by_ppath(self, physical_path):
        for cam in self.uvc_rgb_cameras.values():
            if cam.get("physical_path") == physical_path:
                return cam.get("uid")
        return None
    
    def get_uid_by_vpath(self, video_path):
        cam = self.uvc_rgb_cameras.get(video_path)
        if cam:
            return cam.get("uid")
        return None
    
    def get_vpath_by_sn(self, serial_number):
        matches = []
        for cam in self.uvc_rgb_cameras.values():
            if cam.get("serial_number") == str(serial_number):
                vpath = f"/dev/video{cam.get('video_id')}"
                matches.append(vpath)
        if not matches:
            return None
        if len(matches) > 1:
            raise ValueError(f"Multiple video devices found for serial number {serial_number}: {matches}. ")
        return matches[0]

    def get_vpath_by_ppath(self, physical_path):
        vpath = resolve_physical_video_path(physical_path)
        if vpath is None:
            return None
        # Commissioning mode may validate the selected target. Runtime recovery
        # intentionally performs no VideoCapture open/read probes.
        if self._probe_video_frames and not self._is_like_rgb(vpath):
            return None
        return vpath
    

    def info(self):
        logger_mp.info("======================= Camera Discovery Start ==================================")
        logger_mp.info("Found video devices: %s", self.video_paths)
        logger_mp.info("Found RGB video devices: %s", self.uvc_rgb_video_paths)

        if self.rs_serial_numbers:
            logger_mp.info("----------------------- Realsense Cameras ----------------------------------")
            logger_mp.info(f"RealSense serial numbers: {self.rs_serial_numbers}")
            logger_mp.info(f"RealSense video paths: {self.rs_video_paths}")
            logger_mp.info(f"RealSense RGB-like video paths: {self.rs_rgb_video_paths}")

        for idx, (vpath, cam) in enumerate(self.uvc_rgb_cameras.items(), start=1):
            logger_mp.info("----------------------- OpenCV / UVC Camera %d -----------------------------", idx)
            logger_mp.info("video_path    : %s", vpath)
            logger_mp.info("video_id      : %s", cam.get("video_id"))
            logger_mp.info("serial_number : %s", cam.get("serial_number") or "unknown")
            logger_mp.info("physical_path : %s", cam.get("physical_path"))
            logger_mp.info("extra_info:")

            dev_info = cam.get("dev_info")
            uid = cam.get("uid")

            if dev_info:
                for k, v in dev_info.items():
                    logger_mp.info("    %s: %s", k, v)
                try:
                    import uvc
                    cap = uvc.Capture(uid)
                    for fmt in cap.available_modes:
                        logger_mp.info("    format: %dx%d@%d %s", fmt.height, fmt.width, fmt.fps, fmt.format_name)
                    cap.close()
                    cap = None
                except Exception as e:
                    logger_mp.warning("    failed to get formats: %s", e)
            else:
                logger_mp.info("    no uvc extra info available")

        logger_mp.info("=========================== Camera Discovery End ================================")

class BaseCamera:
    def __init__(self, cam_topic, img_shape, fps, 
                 enable_zmq=True, zmq_port=55555, enable_webrtc=False, webrtc_port=66666, webrtc_codec=None):
        self._ready = threading.Event()
        self._cam_topic = cam_topic
        self._img_shape = img_shape # (H, W)
        self._fps = fps
        self._sequence = 0
        self._enable_zmq = enable_zmq
        self._zmq_port = zmq_port
        if self._enable_zmq:
            self._zmq_buffer = TripleRingBuffer()
        else:
            self._zmq_buffer = None

        self._enable_webrtc = enable_webrtc
        self._webrtc_port = webrtc_port
        self._webrtc_codec = webrtc_codec
        if self._enable_webrtc:
            self._webrtc_buffer = TripleRingBuffer()
        else:
            self._webrtc_buffer = None

    def __str__(self):
        raise NotImplementedError
    
    def __repr__(self):
        return self.__str__()

    def _update_frame(self):
        """Return a jepg frame as bytes, and a bgr frame as numpy array"""
        raise NotImplementedError
    
    def wait_until_ready(self, timeout=None):
        """Block until the camera is ready (first frame is available) or timeout occurs."""
        return self._ready.wait(timeout=timeout)

    def enable_webrtc(self):
        return self._enable_webrtc
    
    def enable_zmq(self):
        return self._enable_zmq

    def _write_zmq_frame(self, jpeg_bytes, capture_timestamp_ms=None,
                         sensor_timestamp_ms=None, source_sequence=None,
                         record_capture=True):
        """Bind source timing to a newly captured JPEG before another thread sees it."""
        if not self._enable_zmq or self._zmq_buffer is None or jpeg_bytes is None:
            return
        if source_sequence is None:
            self._sequence += 1
        else:
            self._sequence = int(source_sequence)
        frame = ZMQImageFrame(
            jpeg=bytes(jpeg_bytes),
            stream=self._cam_topic,
            sequence=self._sequence,
            capture_timestamp_ms=int(
                capture_timestamp_ms or time.time_ns() // 1_000_000
            ),
            width=int(self._img_shape[1]),
            height=int(self._img_shape[0]),
            sensor_timestamp_ms=sensor_timestamp_ms,
        )
        self._zmq_buffer.write(frame)
        if record_capture:
            camera_frame_recorder.record_camera_capture(frame)

    def get_zmq_frame(self):
        return self._zmq_buffer.read() if self._enable_zmq and self._zmq_buffer else None

    def get_jpeg_bytes(self):
        """Compatibility accessor for callers that only need JPEG bytes."""
        frame = self.get_zmq_frame()
        return frame.jpeg if isinstance(frame, ZMQImageFrame) else frame

    def get_bgr_frame(self):
        bgr_numpy = self._webrtc_buffer.read() if self._enable_webrtc and self._webrtc_buffer else None
        return bgr_numpy

    def get_depth_frame(self):
        """Return a depth frame as bytes, or None if not supported. 
           Before call this function, must first call get_frame() to update the latest depth data."""
        return None

    def get_zmq_port(self):
        """Return the zmq port number the camera is serving on."""
        return self._zmq_port
    
    def get_webrtc_port(self):
        """Return the webrtc port number the camera is serving on."""
        return self._webrtc_port
    
    def get_webrtc_codec(self):
        """Return the webrtc codec setting."""
        return self._webrtc_codec

    def get_fps(self):
        """Return the camera FPS setting."""
        return self._fps

    def release(self):
        """Release camera resources."""
        raise NotImplementedError

class RealSenseCamera(BaseCamera):
    def __init__(self, cam_topic, serial_number, img_shape, fps, 
                 enable_zmq=True, zmq_port = 55555, enable_webrtc=False,
                 webrtc_port=66666, webrtc_codec=None, enable_depth=False,
                 isolate_capture_process=False, frame_timeout_seconds=1.0,
                 startup_timeout_seconds=8.0, disable_cyclic_gc=True,
                 capture_cpu_affinity="auto"):
        super().__init__(cam_topic, img_shape, fps, enable_zmq, zmq_port, enable_webrtc, webrtc_port, webrtc_codec)
        self._serial_number = serial_number
        self._enable_depth = enable_depth
        self._latest_depth = None
        self._isolate_capture_process = bool(isolate_capture_process)
        self._frame_timeout_seconds = max(0.1, float(frame_timeout_seconds))
        self._capture_process = None
        self.pipeline = None
        self.align = None
        try:
            if self._isolate_capture_process:
                self._capture_process = RealSenseCaptureProcess(
                    self._serial_number,
                    self._img_shape,
                    self._fps,
                    enable_depth=self._enable_depth,
                    startup_timeout=startup_timeout_seconds,
                    frame_timeout=self._frame_timeout_seconds,
                    disable_cyclic_gc=disable_cyclic_gc,
                    cpu_affinity=capture_cpu_affinity,
                )
                actual = self._capture_process.start()
                self.intrinsics = actual.get("intrinsics")
                self.g_depth_scale = actual.get("depth_scale")
                logger_mp.info(
                    "[RealSenseCamera: %s] capture isolated in PID=%s, "
                    "cyclic_gc=%s, delivery=%s, cpu_affinity=%s, sensor_options=%s",
                    self._cam_topic,
                    self._capture_process.pid,
                    "disabled" if disable_cyclic_gc else "enabled",
                    actual.get("delivery", "unknown"),
                    actual.get("cpu_affinity", "default"),
                    actual.get("sensor_options", {}),
                )
                logger_mp.info(str(self))
                return

            rs = self.check_pyrealsense2_install()
            # Alignment is useful only when depth is enabled. Running it for a
            # color-only stream adds work to the same thread that must drain
            # the RealSense queue at 30 Hz.
            self.align = rs.align(rs.stream.color) if self._enable_depth else None
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(self._serial_number)

            config.enable_stream(rs.stream.color, self._img_shape[1], self._img_shape[0], rs.format.bgr8, self._fps)
            if self._enable_depth:
                config.enable_stream(rs.stream.depth, self._img_shape[1], self._img_shape[0], rs.format.z16, self._fps)

            profile = self.pipeline.start(config)
            self._device = profile.get_device()
            if self._device is None:
                logger_mp.error('[RealSenseCamera] pipe_profile.get_device() is None .')
            if self._enable_depth:
                assert self._device is not None
                depth_sensor = self._device.first_depth_sensor()
                self.g_depth_scale = depth_sensor.get_depth_scale()

            self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            logger_mp.info(str(self))
        except Exception as e:
            self.release()
            raise RuntimeError(f"[RealSenseCamera] Failed to initialize RealSense camera {self._serial_number}: {e}")

    def __str__(self):
        return (
            f"[RealSenseCamera: {self._cam_topic}] initialized with "
            f"{self._img_shape[0]}x{self._img_shape[1]} @ {self._fps} FPS.\n"
            f"ZMQ: {'enabled, zmq_port=' + str(self._zmq_port) if self._enable_zmq else 'disabled'}; "
            f"WebRTC: {'enabled, webrtc_port=' + str(self._webrtc_port) if self._enable_webrtc else 'disabled'}"
        )

    def check_pyrealsense2_install(self):
        try:
            import pyrealsense2 as rs
            return rs
        except Exception as e:
            raise ImportError(
                "pyrealsense2 not installed. Install Intel RealSense SDK and pyrealsense2 Python bindings."
            ) from e
    
    def _update_frame(self):
        if self._capture_process is not None:
            self._record_isolated_capture_events()
            try:
                captured = self._capture_process.read(
                    self._frame_timeout_seconds
                )
            finally:
                self._record_isolated_capture_events()
            self._latest_depth = captured.depth_bytes
            if self._enable_zmq:
                self._write_zmq_frame(
                    captured.jpeg,
                    capture_timestamp_ms=captured.capture_timestamp_ms,
                    sensor_timestamp_ms=captured.sensor_timestamp_ms,
                    source_sequence=captured.sequence,
                    record_capture=False,
                )
            if self._enable_webrtc:
                encoded = np.frombuffer(captured.jpeg, dtype=np.uint8)
                bgr_numpy = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                if bgr_numpy is not None:
                    self._webrtc_buffer.write(bgr_numpy)
            if not self._ready.is_set():
                self._ready.set()
            return

        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames) if self.align is not None else frames
        color_frame = aligned_frames.get_color_frame()
        if not color_frame:
            return None
        capture_timestamp_ms = time.time_ns() // 1_000_000
        sensor_timestamp_ms = None
        try:
            # RealSense timestamps are device-clock values in milliseconds. Keep
            # them as metadata, but use host capture time for cross-stream sync.
            sensor_timestamp_ms = float(color_frame.get_timestamp())
        except Exception:
            pass

        if self._enable_depth:   
            depth_frame = aligned_frames.get_depth_frame()
            if depth_frame:
                self._latest_depth = np.asanyarray(depth_frame.get_data())
            else:
                self._latest_depth = None

        bgr_numpy = np.asanyarray(color_frame.get_data())

        if self._enable_webrtc:
            self._webrtc_buffer.write(bgr_numpy)

        if self._enable_zmq:
            ok, buf = cv2.imencode(".jpg", bgr_numpy)
            if ok:
                self._write_zmq_frame(
                    buf.tobytes(),
                    capture_timestamp_ms=capture_timestamp_ms,
                    sensor_timestamp_ms=sensor_timestamp_ms,
                )
        
        if not self._ready.is_set():
            self._ready.set()
    
    def get_depth_frame(self):
        if self._latest_depth is None:
            return None
        if isinstance(self._latest_depth, bytes):
            return self._latest_depth
        return self._latest_depth.tobytes()

    def _record_isolated_capture_events(self):
        if self._capture_process is None:
            return
        for event in self._capture_process.drain_capture_events():
            camera_frame_recorder.record_camera_capture_metadata(
                stream=self._cam_topic,
                sequence=event.sequence,
                capture_timestamp_ms=event.capture_timestamp_ms,
                sensor_timestamp_ms=event.sensor_timestamp_ms,
                width=event.width,
                height=event.height,
                jpeg_bytes=event.jpeg_bytes,
                record_timestamp_ns=event.record_timestamp_ns,
                record_monotonic_ns=event.record_monotonic_ns,
                source_frame_number=event.source_frame_number,
                source_frame_delta=event.source_frame_delta,
                capture_interval_us=event.capture_interval_us,
                sensor_interval_ms=event.sensor_interval_ms,
                wait_duration_us=event.wait_duration_us,
                jpeg_encode_duration_us=event.jpeg_encode_duration_us,
                handoff_duration_us=event.handoff_duration_us,
            )

    def release(self):
        if self._capture_process is not None:
            try:
                self._record_isolated_capture_events()
                self._capture_process.close()
            except Exception as exc:
                logger_mp.warning(
                    "[RealSenseCamera: %s] capture process release failed: %s",
                    self._cam_topic,
                    exc,
                )
            self._capture_process = None
        if self.pipeline is not None and hasattr(self.pipeline, "stop"):
            try:
                self.pipeline.stop()
            except Exception as e:
                logger_mp.warning(f"[RealSenseCamera] pipeline.stop() failed: {e}")
        self.pipeline = None
        logger_mp.info(f"[RealSenseCamera] Released {self._cam_topic}")

class UVCCamera(BaseCamera):
    def __init__(self, cam_topic, uid, img_shape, fps, 
                 enable_zmq=True, zmq_port=55555, enable_webrtc=False, webrtc_port=66666, webrtc_codec=None):
        super().__init__(cam_topic, img_shape, fps, enable_zmq, zmq_port, enable_webrtc, webrtc_port, webrtc_codec)
        import uvc
        self.uid = uid
        self.cap = None
        try:
            self.cap = uvc.Capture(self.uid)
        except Exception as e:
            self.cap = None
            raise RuntimeError(f"[UVCCamera] Failed to open camera {self._cam_topic}: {e}")

        try:
            self.cap.frame_mode = self._choose_mode(self.cap, width=self._img_shape[1], height=self._img_shape[0], fps=self._fps)
            logger_mp.info(str(self))
        except Exception as e:
            self.cap = None
            raise RuntimeError(f"[UVCCamera] Failed to set mode for {self._cam_topic}: {e}")

    def __str__(self):
        return (
            f"[UVCCamera: {self._cam_topic}] initialized with "
            f"{self._img_shape[0]}x{self._img_shape[1]} @ {self._fps} FPS, MJPG.\n"
            f"ZMQ: {'enabled, zmq port=' + str(self._zmq_port) if self._enable_zmq else 'disabled'}; "
            f"WebRTC: {'enabled, webrtc port=' + str(self._webrtc_port) if self._enable_webrtc else 'disabled'}"
        )

    def _choose_mode(self, cap, width=None, height=None, fps=None):
        for m in cap.available_modes:
            if m.width == width and m.height == height and m.fps == fps and m.format_name == "MJPG":
                return m
        raise ValueError("[UVCCamera] No matching uvc mode found")

    def _update_frame(self):
        if self.cap is not None:
            frame = self.cap.get_frame_robust() # get_frame(timeout=500)
            if frame is not None:
                capture_timestamp_ms = time.time_ns() // 1_000_000
                if self._enable_zmq:
                    if frame.jpeg_buffer is not None:
                        self._write_zmq_frame(
                            frame.jpeg_buffer,
                            capture_timestamp_ms=capture_timestamp_ms,
                        )

                if self._enable_webrtc:
                    if frame.bgr is not None:
                        self._webrtc_buffer.write(frame.bgr)

                if not self._ready.is_set():
                    self._ready.set()
            else:
                raise RuntimeError

    def release(self):
        # if usbhub is plugged out, calling stop_streaming and close may hang forever.
        # try:
        #     self.cap.stop_streaming()
        # except Exception:
        #     pass
        # try:
        #     self.cap.close()
        # except Exception:
        #     pass
        # self.cap = None
        logger_mp.info(f"[UVCCamera] Released {self._cam_topic}")

class OpenCVCamera(BaseCamera):
    """OpenCV/V4L2 backend with optional killable capture isolation."""

    def __init__(self, cam_topic, video_path, img_shape, fps,
                 enable_zmq=True, zmq_port=55555, enable_webrtc=False,
                 webrtc_port=66666, webrtc_codec=None,
                 isolate_capture_process=False, frame_timeout_seconds=1.0,
                 startup_timeout_seconds=5.0):
        super().__init__(
            cam_topic, img_shape, fps, enable_zmq, zmq_port,
            enable_webrtc, webrtc_port, webrtc_codec,
        )
        self._video_path = video_path
        self._isolate_capture_process = bool(isolate_capture_process)
        self._frame_timeout_seconds = max(0.1, float(frame_timeout_seconds))
        self.cap = None
        self._capture_process = None
        try:
            if self._isolate_capture_process:
                self._capture_process = OpenCVCaptureProcess(
                    self._video_path,
                    self._img_shape,
                    self._fps,
                    startup_timeout=startup_timeout_seconds,
                    frame_timeout=self._frame_timeout_seconds,
                )
                actual = self._capture_process.start()
            else:
                self.cap = cv2.VideoCapture(self._video_path, cv2.CAP_V4L2)
                if not self.cap.isOpened():
                    raise RuntimeError(f"Failed to open video device {self._video_path}")
                # FOURCC must be negotiated before resolution and FPS on the
                # real robot's Gemini 305 wrist cameras.
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._img_shape[1])
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._img_shape[0])
                self.cap.set(cv2.CAP_PROP_FPS, self._fps)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                actual = {
                    "width": int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    "height": int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                    "fps": float(self.cap.get(cv2.CAP_PROP_FPS)),
                    "fourcc": int(self.cap.get(cv2.CAP_PROP_FOURCC)),
                }
                if not self._can_read_frame():
                    raise RuntimeError(
                        f"Camera {self._cam_topic} failed to produce a valid frame after warm-up"
                    )
            self._log_negotiated_mode(actual)
            logger_mp.info(str(self))
        except Exception:
            self.release()
            raise

    def _log_negotiated_mode(self, actual):
        actual_width = int(actual.get("width", 0))
        actual_height = int(actual.get("height", 0))
        actual_fps = float(actual.get("fps", 0.0))
        fourcc_value = int(actual.get("fourcc", 0))
        actual_fourcc = "".join(
            chr((fourcc_value >> (8 * index)) & 0xFF) for index in range(4)
        )
        logger_mp.info(
            "[OpenCVCamera: %s] requested=%dx%d @ %.1f FPS, "
            "actual=%dx%d @ %.1f FPS, FOURCC=%s, device=%s, isolated=%s",
            self._cam_topic, self._img_shape[1], self._img_shape[0],
            float(self._fps), actual_width, actual_height, actual_fps,
            actual_fourcc, self._video_path, self._isolate_capture_process,
        )
        if actual_width != int(self._img_shape[1]) or actual_height != int(self._img_shape[0]):
            logger_mp.warning(
                "[OpenCVCamera: %s] resolution mismatch: configured=%dx%d actual=%dx%d",
                self._cam_topic, self._img_shape[1], self._img_shape[0],
                actual_width, actual_height,
            )
        if actual_fourcc.strip("\x00") != "MJPG":
            logger_mp.warning(
                "[OpenCVCamera: %s] requested MJPG but camera selected %s",
                self._cam_topic, actual_fourcc,
            )

    def __str__(self):
        return (
            f"[OpenCVCamera: {self._cam_topic}] initialized with "
            f"{self._img_shape[0]}x{self._img_shape[1]} @ {self._fps} FPS.\n"
            f"ZMQ: {'enabled, zmq port=' + str(self._zmq_port) if self._enable_zmq else 'disabled'}; "
            f"WebRTC: {'enabled, webrtc port=' + str(self._webrtc_port) if self._enable_webrtc else 'disabled'}"
        )
        
    def _read_frame_with_retry(self, max_attempts=3, retry_interval=0.01,
                               log_warning=False):
        if self.cap is None or not self.cap.isOpened():
            return False, None
        last_error = None
        for attempt in range(max_attempts):
            try:
                success, frame = self.cap.read()
                if success and isinstance(frame, np.ndarray) and frame.size > 0:
                    return True, frame
            except Exception as exc:
                last_error = exc
                if log_warning:
                    logger_mp.warning(
                        "[OpenCVCamera: %s] frame read failed (%d/%d): %s",
                        self._cam_topic, attempt + 1, max_attempts, exc,
                    )
            if attempt + 1 < max_attempts:
                time.sleep(retry_interval)
        if log_warning and last_error is None:
            logger_mp.warning(
                "[OpenCVCamera: %s] no valid frame after %d attempts",
                self._cam_topic, max_attempts,
            )
        return False, None

    def _can_read_frame(self):
        for attempt in range(20):
            success, frame = self._read_frame_with_retry(
                max_attempts=1, retry_interval=0.0,
            )
            if success:
                logger_mp.info(
                    "[OpenCVCamera: %s] warm-up completed after %d attempts, shape=%s",
                    self._cam_topic, attempt + 1, frame.shape,
                )
                return True
            time.sleep(0.05)
        return False
    
    def _update_frame(self):
        if self._capture_process is not None:
            self._record_isolated_capture_events()
            try:
                captured = self._capture_process.read(self._frame_timeout_seconds)
            finally:
                self._record_isolated_capture_events()
            if self._enable_zmq:
                self._write_zmq_frame(
                    captured.jpeg,
                    capture_timestamp_ms=captured.capture_timestamp_ms,
                    source_sequence=captured.sequence,
                    record_capture=False,
                )
            if self._enable_webrtc:
                encoded = np.frombuffer(captured.jpeg, dtype=np.uint8)
                bgr_numpy = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                if bgr_numpy is not None:
                    self._webrtc_buffer.write(bgr_numpy)
            if not self._ready.is_set():
                self._ready.set()
            return

        ret, bgr_numpy = self._read_frame_with_retry(
            max_attempts=3, retry_interval=0.01, log_warning=True,
        )
        if not ret:
            raise RuntimeError(
                f"[OpenCVCamera: {self._cam_topic}] failed to read {self._video_path}"
            )
        capture_timestamp_ms = time.time_ns() // 1_000_000
        if self._enable_webrtc:
            self._webrtc_buffer.write(bgr_numpy)
        if self._enable_zmq:
            ok, buf = cv2.imencode(".jpg", bgr_numpy)
            if ok and buf is not None and buf.size > 0:
                self._write_zmq_frame(
                    buf.tobytes(), capture_timestamp_ms=capture_timestamp_ms,
                )
            else:
                logger_mp.warning(
                    "[OpenCVCamera: %s] failed to encode JPEG", self._cam_topic
                )
        if not self._ready.is_set():
            self._ready.set()

    def _record_isolated_capture_events(self):
        if self._capture_process is None:
            return
        for event in self._capture_process.drain_capture_events():
            camera_frame_recorder.record_camera_capture_metadata(
                stream=self._cam_topic,
                sequence=event.sequence,
                capture_timestamp_ms=event.capture_timestamp_ms,
                width=event.width,
                height=event.height,
                jpeg_bytes=event.jpeg_bytes,
                record_timestamp_ns=event.record_timestamp_ns,
                record_monotonic_ns=event.record_monotonic_ns,
            )

    def release(self):
        if self._capture_process is not None:
            try:
                self._record_isolated_capture_events()
                self._capture_process.close()
            except Exception as exc:
                logger_mp.warning(
                    "[OpenCVCamera: %s] capture process release failed: %s",
                    self._cam_topic, exc,
                )
            self._capture_process = None
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception as exc:
                logger_mp.warning(
                    "[OpenCVCamera: %s] release failed: %s", self._cam_topic, exc
                )
        self.cap = None
        logger_mp.info(f"[OpenCVCamera] Released {self._cam_topic}")

class IsaacSimCamera(BaseCamera):
    def __init__(self, cam_topic, img_shape, fps,
                 enable_zmq=True, zmq_port=55555, enable_webrtc=False, webrtc_port=66666, webrtc_codec=None,
                 image_source="head", binocular=False):
        """
        IsaacSim camera that reads from shared memory.

        Args:
            cam_topic: camera topic name
            img_shape: image shape [height, width]
            fps: frames per second
            enable_zmq: enable ZMQ publishing
            zmq_port: ZMQ port
            enable_webrtc: enable WebRTC publishing
            webrtc_port: WebRTC port
            webrtc_codec: WebRTC codec preference
            image_source: which image to read from shared memory ("head", "left", "right")
            binocular: if True and image_source=="head", concatenate left+right for binocular vision
        """
        super().__init__(cam_topic, img_shape, fps, enable_zmq, zmq_port, enable_webrtc, webrtc_port, webrtc_codec)
        from tools.shared_memory_utils import MultiImageReader # https://github.com/unitreerobotics/unitree_sim_isaaclab/tree/main/tools
        self.multi_image_reader = MultiImageReader()
        self._image_source = image_source  # "head", "left", or "right"
        self._binocular = binocular
        # For IsaacSim cameras, set ready immediately since the camera object is initialized
        # and will wait for shared memory data in _update_frame
        self._ready.set()
        logger_mp.info(str(self))

    def __str__(self):
        mode = "binocular" if self._binocular else "monocular"
        return (
            f"[IsaacSimCamera: {self._cam_topic}] initialized with "
            f"{self._img_shape[0]}x{self._img_shape[1]} @ {self._fps} FPS, source='{self._image_source}', mode='{mode}'.\n"
            f"ZMQ: {'enabled, zmq port=' + str(self._zmq_port) if self._enable_zmq else 'disabled'}; "
            f"WebRTC: {'enabled, webrtc port=' + str(self._webrtc_port) if self._enable_webrtc else 'disabled'}"
        )

    def _update_frame(self):
        # Get the image data based on source and binocular settings
        frame_data = None
        if self._binocular:
            # For binocular cameras: concatenate left + right images
            left_img = self.multi_image_reader.read_single_image('left')
            right_img = self.multi_image_reader.read_single_image('right')
            logger_mp.debug(f"[IsaacSimCamera] {self._cam_topic} - left: {left_img is not None}, right: {right_img is not None}")

            if left_img is not None and right_img is not None:
                frame_data = cv2.hconcat([left_img, right_img])
                logger_mp.debug(f"[IsaacSimCamera] {self._cam_topic} - concatenated binocular frame: {frame_data.shape}")
        else:
            # For monocular cameras: use the specified source directly
            frame_data = self.multi_image_reader.read_single_image(self._image_source)
            if frame_data is None:
                logger_mp.debug(f"[IsaacSimCamera] {self._cam_topic} - no data for source '{self._image_source}'")

        # Publish the frame data only if we have valid data
        if frame_data is not None:
            capture_timestamp_ms = time.time_ns() // 1_000_000
            # For ZMQ: encode to JPEG bytes
            if self._enable_zmq:
                ok, buf = cv2.imencode(".jpg", frame_data)
                if ok:
                    self._write_zmq_frame(
                        buf.tobytes(),
                        capture_timestamp_ms=capture_timestamp_ms,
                    )
                else:
                    logger_mp.warning(f"[IsaacSimCamera] Failed to encode to JPEG for {self._cam_topic}")

            # For WebRTC: use BGR frames directly
            if self._enable_webrtc:
                self._webrtc_buffer.write(frame_data)
            else:
                logger_mp.warning(f"[IsaacSimCamera] Failed to encode to WebRTC for {self._cam_topic}")
            if not self._ready.is_set():
                self._ready.set()
        else:
            logger_mp.debug(f"[IsaacSimCamera] No data available for {self._cam_topic}, frame_data is None")
        # If no data is available, just return silently and wait for next frame

    def release(self):
        if hasattr(self, 'multi_image_reader') and self.multi_image_reader is not None:
            self.multi_image_reader.close()
        self.multi_image_reader = None
        logger_mp.info(f"[IsaacSimCamera] Released {self._cam_topic}")
# ========================================================
# image server
# ========================================================
class ImageServer:
    """Long-lived image service with one reconnect supervisor per camera."""

    # Retry forever, but keep recovery latency bounded after a physical device
    # is reinserted. Runtime discovery is sysfs-only, so a 2 s cap does not
    # repeatedly open healthy cameras.
    DEFAULT_BACKOFF = (0.5, 1.0, 2.0, 2.0)

    def __init__(self, cam_config, realsense_enable=False,
                 camera_finder_verbose=False, isaacsim_enable=False,
                 camera_factory=None, finder_factory=None, status_writer=None):
        self._cam_config = cam_config
        self._realsense_enable = realsense_enable
        self._finder_verbose = camera_finder_verbose
        self._isaacsim_enable = isaacsim_enable
        self._camera_factory = camera_factory
        self._finder_factory = finder_factory or CameraFinder
        self._status = status_writer or DeviceStatusWriter("camera")
        self._stop_event = threading.Event()
        self._cameras: dict[str, BaseCamera | None] = {}
        self._camera_lock = threading.RLock()
        self._discovery_lock = threading.Lock()
        self._threads = []
        self._cleaned = False
        # The configuration endpoint remains alive even if every camera is
        # currently unplugged, allowing XR to start and devices to recover.
        self._responser = ZMQ_Responser(self._cam_config)
        self._zmq_publisher_manager = ZMQ_PublisherManager.get_instance()
        self._webrtc_publisher_manager = WebRTC_PublisherManager.get_instance()
        logger_mp.info(
            "[Image Server] Service started; unavailable cameras will reconnect indefinitely."
        )

    def _discover(self, *, include_realsense=False):
        with self._discovery_lock:
            return self._finder_factory(
                bool(include_realsense and self._realsense_enable),
                self._finder_verbose,
                reload_driver=False,
                probe_video_frames=False,
            )

    def _create_camera(self, cam_topic: str, cam_cfg: dict) -> BaseCamera:
        if self._camera_factory is not None:
            return self._camera_factory(cam_topic, cam_cfg)

        cam_type = str(cam_cfg.get("type", "uvc")).lower()
        if self._isaacsim_enable:
            cam_type = "isaacsim"
        common = (
            cam_topic,
            cam_cfg.get("image_shape"),
            cam_cfg.get("fps", 30),
            cam_cfg.get("enable_zmq", False),
            cam_cfg.get("zmq_port"),
            cam_cfg.get("enable_webrtc", False),
            cam_cfg.get("webrtc_port"),
            cam_cfg.get("webrtc_codec"),
        )
        if cam_type == "isaacsim":
            binocular = bool(cam_cfg.get("binocular", False))
            source = "left" if "left" in cam_topic.lower() else (
                "right" if "right" in cam_topic.lower() else "head"
            )
            return IsaacSimCamera(*common, image_source=source, binocular=binocular)

        # Wrist-camera recovery does not query the live RealSense device. It
        # only needs V4L2 sysfs data for its own configured physical path.
        finder = self._discover(include_realsense=(cam_type == "realsense"))
        serial = cam_cfg.get("serial_number")
        serial = None if serial in (None, "") else str(serial)
        physical = cam_cfg.get("physical_path")
        physical = None if physical in (None, "") else str(physical)
        video_id = cam_cfg.get("video_id")
        video_path = None if video_id in (None, "") else f"/dev/video{video_id}"

        if cam_type == "realsense":
            if not self._realsense_enable:
                raise RuntimeError("RealSense support requires --rs")
            if serial is None:
                if len(finder.rs_serial_numbers) != 1:
                    raise RuntimeError(
                        "RealSense serial is not configured and discovery is ambiguous"
                    )
                serial = str(finder.rs_serial_numbers[0])
            if not finder.is_rs_serial_exist(serial):
                raise RuntimeError(f"RealSense serial {serial} was not found")
            return RealSenseCamera(
                common[0], serial, *common[1:],
                isolate_capture_process=bool(
                    cam_cfg.get("isolate_capture_process", False)
                ),
                frame_timeout_seconds=float(
                    cam_cfg.get("frame_timeout_seconds", 1.0)
                ),
                startup_timeout_seconds=float(
                    cam_cfg.get("startup_timeout_seconds", 8.0)
                ),
                disable_cyclic_gc=bool(
                    cam_cfg.get("disable_cyclic_gc", True)
                ),
                capture_cpu_affinity=cam_cfg.get(
                    "capture_cpu_affinity", "auto"
                ),
            )

        if cam_type == "opencv":
            path = None
            if physical is not None:
                path = finder.get_vpath_by_ppath(physical)
            elif serial is not None:
                path = finder.get_vpath_by_sn(serial)
            elif video_path is not None and finder.is_vpath_exist(video_path):
                path = video_path
            if path is None:
                raise RuntimeError(
                    f"OpenCV camera selector not found (physical_path={physical}, "
                    f"serial_number={serial}, video_id={video_id})"
                )
            return OpenCVCamera(
                common[0], path, *common[1:],
                isolate_capture_process=bool(
                    cam_cfg.get("isolate_capture_process", False)
                ),
                frame_timeout_seconds=float(
                    cam_cfg.get("frame_timeout_seconds", 1.0)
                ),
                startup_timeout_seconds=float(
                    cam_cfg.get("startup_timeout_seconds", 5.0)
                ),
            )

        if cam_type == "uvc":
            uid = None
            if physical is not None:
                uid = finder.get_uid_by_ppath(physical)
            elif serial is not None:
                uid = finder.get_uid_by_sn(serial)
            elif video_path is not None:
                uid = finder.get_uid_by_vpath(video_path)
            if uid is None:
                raise RuntimeError(
                    f"UVC camera selector not found (physical_path={physical}, "
                    f"serial_number={serial}, video_id={video_id})"
                )
            return UVCCamera(common[0], uid, *common[1:])
        raise ValueError(f"Unknown camera type {cam_type} for {cam_topic}")

    def _camera_loop(self, cam_topic: str, cam_cfg: dict):
        configured = cam_cfg.get("reconnect_backoff_seconds", self.DEFAULT_BACKOFF)
        backoff = tuple(float(value) for value in configured) or self.DEFAULT_BACKOFF
        attempt = 0
        was_online = False
        while not self._stop_event.is_set():
            camera = None
            try:
                state = DISCONNECTED if attempt == 0 else RECONNECTING
                self._status.publish(
                    cam_topic,
                    state,
                    "camera is unavailable; waiting for device",
                    details={"attempt": attempt + 1},
                )
                camera = self._create_camera(cam_topic, cam_cfg)
                with self._camera_lock:
                    self._cameras[cam_topic] = camera
                interval = 1.0 / max(1.0, float(camera.get_fps()))
                next_frame = time.monotonic()
                ready_deadline = next_frame + (
                    15.0 if self._isaacsim_enable else 5.0
                )
                while not self._stop_event.is_set():
                    camera._update_frame()
                    if not camera._ready.is_set() and time.monotonic() >= ready_deadline:
                        raise TimeoutError("first frame timeout")
                    if camera._ready.is_set():
                        if not was_online:
                            self._status.publish(
                                cam_topic,
                                ONLINE,
                                "camera stream is online",
                                details={"attempt": attempt + 1},
                            )
                            logger_mp.info("[Image Server] %s is online.", cam_topic)
                            was_online = True
                            attempt = 0
                        if camera.enable_zmq():
                            frame = camera.get_zmq_frame()
                            if frame is not None:
                                self._zmq_publisher_manager.publish(
                                    frame, camera.get_zmq_port()
                                )
                        if camera.enable_webrtc():
                            frame = camera.get_bgr_frame()
                            if frame is not None:
                                self._webrtc_publisher_manager.publish(
                                    frame,
                                    camera.get_webrtc_port(),
                                    codec_pref=camera.get_webrtc_codec(),
                                )
                    next_frame += interval
                    delay = next_frame - time.monotonic()
                    if delay > 0:
                        self._stop_event.wait(delay)
                    else:
                        next_frame = time.monotonic()
            except Exception as exc:
                was_online = False
                delay = backoff[min(attempt, len(backoff) - 1)]
                attempt += 1
                self._status.publish(
                    cam_topic,
                    RECONNECTING,
                    str(exc),
                    details={"attempt": attempt, "retry_in_seconds": delay},
                    force=True,
                )
                logger_mp.warning(
                    "[Image Server] %s disconnected: %s; retry in %.1fs",
                    cam_topic, exc, delay,
                )
                self._stop_event.wait(delay)
            finally:
                with self._camera_lock:
                    self._cameras[cam_topic] = None
                if camera is not None:
                    try:
                        camera.release()
                    except Exception as exc:
                        logger_mp.warning(
                            "[Image Server] Failed to release %s: %s", cam_topic, exc
                        )

    def _clean_up(self):
        if self._cleaned:
            return
        self._cleaned = True
        self._responser.stop()
        for thread in self._threads:
            if thread.is_alive() and thread is not threading.current_thread():
                thread.join(timeout=2.0)
        self._threads.clear()
        try:
            self._zmq_publisher_manager.close()
        except Exception:
            pass
        try:
            self._webrtc_publisher_manager.close()
        except Exception:
            pass
        camera_frame_recorder.close_camera_frame_recorder()
        logger_mp.info("[Image Server] Clean up completed. Server stopped.")

    def start(self):
        self._stop_event.clear()
        for cam_topic, cam_cfg in self._cam_config.items():
            if not cam_cfg.get("enable_zmq", False) and not cam_cfg.get("enable_webrtc", False):
                continue
            thread = threading.Thread(
                target=self._camera_loop,
                args=(cam_topic, cam_cfg),
                name=f"CameraSupervisor-{cam_topic}",
                daemon=True,
            )
            thread.start()
            self._threads.append(thread)

    def wait(self):
        self._stop_event.wait()
        self._clean_up()

    def stop(self):
        self._stop_event.set()

# ========================================================
# utility functions
# ========================================================
def signal_handler(server, signum, frame):
    logger_mp.info(f"[Image Server] Received signal {signum}, initiating graceful shutdown...")
    server.stop()

def set_performance_mode(cores=[0, 1, 2]):
    import psutil
    try:
        p = psutil.Process(os.getpid())
        
        # Set CPU affinity for the process and all its threads
        p.cpu_affinity(cores)
        logger_mp.info(f"[Performance] CPU Affinity locked to: {cores}")

    except psutil.AccessDenied:
        logger_mp.warning("[Performance] Access Denied: Run as sudo for full optimization")
    except Exception as e:
        logger_mp.error(f"[Performance] Error: {e}")

def run_isaacsim_server():
    # Load config file, start image server
    try:
        cam_config = load_camera_config(CONFIG_PATH)
    except Exception as e:
        logger_mp.error(f"Failed to load configuration file at {CONFIG_PATH}: {e}")
        raise SystemExit(1)
    # start image server
    server = ImageServer(cam_config, realsense_enable=False, camera_finder_verbose=False, isaacsim_enable=True)
    server.start()
    return server

def main():
    logger_mp.info(
        "\n====================== Image Server Startup Guide ======================\n"
        "Please first read this repo's README.md to learn how to configure and use the teleimager.\n"
        "To discover connected cameras, run the following command:\n"
        "\n"
        "    teleimager-server --cf\n"
        "\n"
        "The '--cf' flag means 'camera find'.\n"
        "This will list all detected cameras and their details (video paths, serial numbers and physical path etc.).\n"
        "Use that information to fill in your 'cam_config_server.yaml' file.\n"
        "Once configured, you can start the image server with:\n"
        "\n"
        "    teleimager-server\n"
        "\n"
        "Note:\n"
        " - If you have RealSense cameras, add the '--rs' flag to enable RealSense support.\n"
        " - Make sure you have proper permissions to access the camera devices (e.g., run with sudo or set udev rules).\n"
        "=========================================================================="
    )

    # command line args
    parser = argparse.ArgumentParser()
    parser.add_argument('--cf', action = 'store_true', help = 'Enable camera found mode, print all connected cameras info')
    parser.add_argument('--rs', action = 'store_true', help = 'Enable RealSense camera mode. Otherwise only find UVC/OpenCV cameras.')
    parser.add_argument('--no-affinity', action='store_false', dest='affinity', help='Disable CPU affinity setting for performance optimization.')
    parser.add_argument(
        '--record-camera-frames',
        action='store_true',
        help='Record camera capture/send metadata as CSV files.',
    )
    parser.add_argument(
        '--camera-record-dir',
        default=str(camera_frame_recorder.DEFAULT_OUTPUT_DIR),
        help='Directory for camera capture/send CSV sessions.',
    )
    args = parser.parse_args()

    if args.affinity:
        set_performance_mode(cores=[0, 1, 2])

    # if enable camera finder mode, just print cameras info and exit
    if args.cf:
        CameraFinder(realsense_enable=args.rs, verbose=True)
        raise SystemExit(0)

    # Load config file, start image server
    try:
        cam_config = load_camera_config(CONFIG_PATH)
    except Exception as e:
        logger_mp.error(f"Failed to load configuration file at {CONFIG_PATH}: {e}")
        raise SystemExit(1)

    camera_frame_recorder.configure_camera_frame_recorder(
        enabled=args.record_camera_frames,
        output_dir=args.camera_record_dir,
    )

    # start image server
    server = ImageServer(cam_config, realsense_enable=args.rs, camera_finder_verbose=False)
    server.start()

    # graceful shutdown handling
    signal.signal(signal.SIGINT, functools.partial(signal_handler, server))
    signal.signal(signal.SIGTERM, functools.partial(signal_handler, server))

    logger_mp.info("[Image Server] Running... Press Ctrl+C to exit.")
    server.wait()

if __name__ == "__main__":
    main()
