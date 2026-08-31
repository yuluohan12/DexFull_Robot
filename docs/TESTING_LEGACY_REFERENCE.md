# DexFull Testing Guide

## Offline Tests

Run on a development machine or on the robot:

```bash
cd ~/ws/DexFull
pytest -q
```

These tests use fakes and mocks where needed. They do not start robot motion.

## Real Robot Tests

Start DexFull first:

```bash
cd ~/ws/DexFull
python3 -m dexfull
```

The current automated tests are offline and do not command a robot. For a real
robot smoke test, connect Unity and start components in this order:

```text
teleimager
XR component in the Bridge process
selected external hand-driver service (BrainCo when `ee: brainco`)
```

It does not call `start_teleop`.

## Recording Policy

Robot-side recording is removed.

Removed RPC methods:

```text
toggle_record
start_xr_record
stop_xr_record
start_record
stop_record
```

DexFull does not start the control component with `--record`.
Bridge no longer writes CSV files on the robot.

Unity should record:

```text
WebSocket robot_state
WebSocket robot_datas
ZMQ/WebRTC camera frames
Unity-side headset/controller/task data
```

## WebSocket Test Tip

Realtime `robot_state` and `robot_datas` events are globally gated by `start_teleop` and
`stop_teleop`. Before `start_teleop`, Bridge may cache sampled robot data
internally but does not push it to Unity.

Disable realtime stream for a WebSocket test website:

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

Then send normal requests such as:

```json
{
  "type": "request",
  "id": "status-1",
  "method": "get_status",
  "timestamp": 1780000000.0,
  "data": {}
}
```

## ZMQ Video Check

Run:

```bash
python tools/zmq_viewer.py --host 127.0.0.1 --camera head --no-display
```

Or with GUI:

```bash
python tools/zmq_viewer.py --host 127.0.0.1 --camera head
```
