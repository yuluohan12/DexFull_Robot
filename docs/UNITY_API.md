# Unity Integration Guide

This document describes the Unity protocol preserved by DexFull. Existing Unity
method names, request envelopes, response envelopes, event names and data-field
names remain compatible; Unity does not need an interface change.

## Architecture

```text
Unity
  <-> WSS encrypted WebSocket JSON
DexFullApplication
  <-> direct DDS state bus
  <-> minimal lifecycle pipe + latest VR shared memory
isolated control process / teleimager / robot and hand DDS
```

The robot side only provides realtime control and realtime data streaming.
Unity is responsible for recording and saving datasets.

## WebSocket

Default endpoint:

```text
wss://<robot_ip>:7443/ws
```

The endpoint comes from `config/dexfull.yaml` under `ws.host`, `ws.port` and
`ws.path`. The JSON request/response protocol below is unchanged. Unity must
pin the SHA-256 fingerprint of the common server certificate; no Unity client
certificate is required. See `docs/WSS_DEPLOYMENT.md`.

## Request Format

Use Unity request/response messages:

```json
{
  "type": "request",
  "id": "status-1",
  "method": "get_status",
  "timestamp": 1780000000.0,
  "data": {}
}
```

Responses include the same `id`:

```json
{
  "type": "response",
  "id": "status-1",
  "method": "get_status",
  "succeed": true,
  "timestamp": 1780000000.0,
  "data": {}
}
```

## Supported Methods

Unity method names currently supported:

```text
get_basic_infos
getStatus
getProcessStatus
startImageServer
stopImageServer
restartImageServer
startXrTeleop / startXRTeleop
stopXrTeleop / stopXRTeleop
restartXrTeleop / restartXRTeleop
startTeleop
stopTeleop
pauseTeleop
resumeTeleop
startHandDriver
stopHandDriver
restartHandDriver
getHandDriverStatus
startRobot
stopRobot
```

### Basic Infos

```json
{
  "type": "request",
  "id": "basic-1",
  "method": "get_basic_infos",
  "timestamp": 1780000000.0,
  "data": {}
}
```

Response:

```json
{
  "type": "response",
  "id": "basic-1",
  "method": "get_basic_infos",
  "succeed": true,
  "timestamp": 1780000000.0,
  "data": {
    "version": "2.3.1",
    "date": "",
    "author": "DexFull",
    "robot_name": "G1_29",
    "hand_name": "brainco",
    "control_type": "hand",
    "input_device_frenquency": 60.0,
    "push_data_frequency": 30.0,
    "image": {"url": "tcp://<robot_ip>:55555", "width": 640, "height": 480, "fps": 30.0},
    "images": [
      {"url": "tcp://<robot_ip>:55555", "width": 640, "height": 480, "fps": 30.0},
      {"url": "tcp://<robot_ip>:55557", "width": 640, "height": 480, "fps": 30.0},
      {"url": "tcp://<robot_ip>:55556", "width": 640, "height": 480, "fps": 30.0}
    ],
    "depth": {"width": 0, "height": 0, "fps": 0.0},
    "audio": {"sample_rate": 0, "channels": 0, "format": "", "bits": 0},
    "joint_names": []
  }
}
```

`images` follows the robot 1.1 order: head, right wrist, left wrist. `image`
is retained as an older-client alias of `images[0]`. The misspelled
`input_device_frenquency` field is intentional and matches the deployed Unity
contract.

### Process Control

Start image server:

```json
{
  "type": "request",
  "id": "start-img-1",
  "method": "start_image_server",
  "timestamp": 1780000000.0,
  "data": {}
}
```

Stop image server:

```json
{
  "type": "request",
  "id": "stop-img-1",
  "method": "stop_image_server",
  "timestamp": 1780000000.0,
  "data": {}
}
```

Start the isolated XR/control component (and ensure the selected hand service):

```json
{
  "type": "request",
  "id": "start-xr-1",
  "method": "start_xr_teleop",
  "timestamp": 1780000000.0,
  "data": {
    "wait_ready": true
  }
}
```

XR start/stop/restart may take tens of seconds on the robot while native control,
IK and hand-driver modules initialize. DexFull processes these lifecycle requests
in the background so the same WebSocket can continue handling Unity `ping`
messages. The lifecycle response is still sent with the original request `id`
after the operation completes; no Unity method or envelope change is required.

Fully stop the XR component:

```json
{
  "type": "request",
  "id": "stop-xr-1",
  "method": "stop_xr_teleop",
  "timestamp": 1780000000.0,
  "data": {}
}
```

### Teleoperation Control

Start robot following:

```json
{
  "type": "request",
  "id": "teleop-start-1",
  "method": "start_teleop",
  "timestamp": 1780000000.0,
  "data": {}
}
```

Response:

```json
{
  "type": "response",
  "id": "tcp://<robot_ip>:55555",
  "method": "start_teleop",
  "succeed": true,
  "timestamp": 1780000000.0,
  "data": {
    "state": "teleoping",
    "zmq_url": "tcp://<robot_ip>:55555"
  }
}
```

Bridge sends this response first. After the response is sent successfully,
Bridge enables realtime `robot_state` and `robot_datas` events on the same WebSocket connection.
The `id` field intentionally carries the head-camera ZMQ URL required by the
Unity protocol.

Stop robot following:

```json
{
  "type": "request",
  "id": "teleop-stop-1",
  "method": "stop_teleop",
  "timestamp": 1780000000.0,
  "data": {}
}
```

After a successful `stop_teleop` response, Bridge stops pushing `robot_state` and `robot_datas`.
Robot state sampling may continue internally, but it is only cached and is not
sent to Unity until the next successful `start_teleop`.

### Status

Get full Bridge status:

```json
{
  "type": "request",
  "id": "status-1",
  "method": "get_status",
  "timestamp": 1780000000.0,
  "data": {}
}
```

Get one process status:

```json
{
  "type": "request",
  "id": "proc-img-1",
  "method": "get_process_status",
  "timestamp": 1780000000.0,
  "data": {
    "service": "teleimager"
  }
}
```

Supported services:

```text
teleimager
xr (the isolated DexFull control runtime; `xr_teleoperate` remains an alias)
brainco
```

### Test Client Stream Control

For WebSocket test websites, `robot_state` and `robot_datas` start only after successful
`start_teleop`. If the stream is noisy while testing other commands, disable it
for the current client:

```json
{
  "type": "request",
  "id": "stream-off-1",
  "method": "set_robot_datas",
  "timestamp": 1780000000.0,
  "data": {
    "enabled": false
  }
}
```

Enable it again:

```json
{
  "type": "request",
  "id": "stream-on-1",
  "method": "set_robot_datas",
  "timestamp": 1780000000.0,
  "data": {
    "enabled": true
  }
}
```

This only affects the current WebSocket connection.

## Removed Recording Methods

The following robot-side recording methods are removed:

```text
toggle_record
start_xr_record
stop_xr_record
start_record
stop_record
```

The control component is not started with `--record` by Bridge.
Bridge no longer writes CSV files on the robot.

If Unity sends any of these methods, Bridge returns:

```json
{
  "type": "error",
  "method": "start_record",
  "error_tip": "unknown_method: start_record",
  "timestamp": 1780000000.0
}
```

## Unity-Side Recording

Unity should record these realtime streams locally:

```text
WebSocket robot_state   -> robot/source/online/teleop readiness state
WebSocket robot_datas   -> root odom pose, robot joints, arm states, FK end-effector poses
ZMQ/WebRTC camera       -> head and wrist camera frames
Unity operator inputs   -> headset, controllers, task labels, timestamps
```

Recommended timestamp policy:

```text
Use Unix epoch milliseconds as the shared dataset timeline.
Align image capture_timestamp_ms with robot_timestamp_ms; do not align by frame index.
Keep camera sensor_timestamp_ms only as device-clock diagnostic metadata.
```

## Realtime Robot Stream

After successful `start_teleop`, Bridge continuously pushes Unity protocol events:

```json
{
  "type": "event",
  "eventName": "robot_state",
  "timestamp": 1780000000123,
  "data": {
    "robot": {"arm": "G1_29", "ee": null},
    "source": "direct",
    "dds_online": true,
    "odom_online": true,
    "fk_online": false,
    "ipc_online": false,
    "teleop_start": true,
    "teleop_stop": false,
    "ready": true,
    "heartbeat": {},
    "robot_timestamp": 1780000000123,
    "robot_timestamp_ms": 1780000000123
  }
}
```

`robot_datas.positions` is the robot root global position from
`rt/odommodestate` `SportModeState_.position`.
`robot_datas.rotations` keeps the Unitree/Unity integration contract in
`[w, x, y, z]` order. Bridge does not reorder these four values; Unity's
coordinate conversion layer is responsible for constructing its native
quaternion representation.

For Unity driving, the default robot-side `root_pose_mode` is
`unity_relative`: the first valid odom pose becomes the session origin, the
initial output height is the configured `root_pelvis_height`, and later frames
preserve odom horizontal translation and rotation deltas. The generic G1 model
fallback is `0.793 m`; this project's calibrated Unity value is `0.76 m`. Set
the mode to `absolute` to restore the previous raw odom output.

The current G1 Unity profile uses `root_pelvis_height: 0.76`,
`root_axis_mapping: unitree_to_unity`, and `root_vertical_mode: filtered`.
The axis mapping converts Unitree X-forward/Y-left translation into the
transport order consumed by the existing Unity coordinate converter. Filtered
vertical mode applies a `0.01 m` deadband and a low-latency EMA, suppressing
small estimator noise while retaining real crouching and standing motion.

```json
{
  "type": "event",
  "eventName": "robot_datas",
  "timestamp": 1780000000123,
  "data": {
    "positions": [0.0, 0.0, 0.0],
    "rotations": [0.0, 0.0, 0.0, 1.0],
    "velocities": [],
    "torques": [],
    "angles": [],
    "electricity": [],
    "left_arm": {},
    "right_arm": {},
    "left_ee_pose": [],
    "right_ee_pose": [],
    "robot_timestamp": 1780000000123,
    "robot_timestamp_ms": 1780000000123
  }
}
```

For a robot without dexterous hands:

```text
robot.ee      = null
left_ee       = {}
right_ee      = {}
left_ee_pose  = arm FK pose
right_ee_pose = arm FK pose
```

## Camera Streams

teleimager ports:

```text
head_camera        ZMQ tcp://<robot_ip>:55555
left_wrist_camera  ZMQ tcp://<robot_ip>:55556
right_wrist_camera ZMQ tcp://<robot_ip>:55557
config responder   tcp://<robot_ip>:60000
head WebRTC        https://<robot_ip>:60001
```

For a RealSense head camera, set `teleimager.realsense: true` in
`config/dexfull.yaml`; DexFull then starts teleimager with `--rs`.

## Device disconnect notification

Physical camera or hand disconnects are pushed asynchronously without changing
any existing Unity request method:

```json
{
  "type": "error",
  "error_tip": "设备掉线：left_hand，正在等待重新连接",
  "timestamp": 1780000000123,
  "data": {
    "code": "DEVICE_DISCONNECTED",
    "component": "brainco",
    "device": "left_hand",
    "state": "DISCONNECTED",
    "recoverable": true,
    "message": "",
    "details": {"age_seconds": 2.4}
  }
}
```

Recovery is sent as the existing event envelope with
`eventName: "device_state"` and `data.code: "DEVICE_RECOVERED"`. A device
disconnect does not imply that XR/control or another device has stopped.

## Recommended Unity Startup Flow

```text
1. Connect wss://<robot_ip>:7443/ws and verify the pinned server certificate
2. start_image_server
3. Wait until teleimager state is RUNNING
4. Start ZMQ/WebRTC image receiver
5. start_xr_teleop
6. Wait for the start response `runtime_ready: true` (`ipc_ready` remains only
   as a compatibility response field; Unity does not connect to an IPC service)
7. start_teleop when the operator is ready and the robot area is safe
8. Unity records robot_datas, images, and operator inputs locally
9. stop_teleop
10. stop_xr_teleop / stop_image_server when finished
```
