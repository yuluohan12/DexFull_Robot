# 实机 1.1 适配与验收

## 基线

DexFull 的实机行为以以下目录为准：

- `teleop_bridge_1.1`
- `xr_teleoperate_1.1`
- `brainco_hand_service`

旧目录只作为比对和回退基线，DexFull 不从中导入 Python 模块。

## 已迁入的实机差异

- 控制频率 60 Hz、Unity 推送频率 30 Hz；
- G1_29、BrainCo、`motion: true`；
- 站立骨盆高度 0.76 m；
- 初始机器人朝向对齐和 Unity 四元数预补偿；
- 头部 RealSense `254322073111`；
- 左右腕部 Gemini/OpenCV 相机的 Jetson USB physical path；
- MJPG 模式协商、相机预热和坏帧重试；
- ImageClient 按需订阅，只为 XR/录制需要的相机执行 BGR 解码；
- Unity 1.1 `get_basic_infos` 三路 `images`、
  `input_device_frenquency` 和 `push_data_frequency` 字段；
- BrainCo 原生服务由 DexFull 管理，但仍保持独立进程隔离。

为了兼容旧 Unity，`get_basic_infos.image` 仍等于 `images[0]`。IPC 状态字段
也仍作为协议占位存在。控制进程边界只传生命周期、健康状态和最新 VR
共享帧；机器人及灵巧手状态仍由 Bridge 直接订阅 DDS。

## 安装和启动

在机器人已有的 Python 3.10/vendor SDK 环境中执行：

```bash
cd ~/ws/DexFull
python3 -m pip install -e ".[control,imaging]"
chmod +x scripts/run.sh scripts/build_brainco.sh
./scripts/build_brainco.sh     # 已有匹配架构二进制时可跳过
./scripts/run.sh
```

DexFull 会自动启动 teleimager。选择 BrainCo 时，启动 XR 组件会同时启动
`dexfull/hand_drivers/brainco/native/bin/brainco_hand_server`。

## 启动前检查

```bash
test -x dexfull/hand_drivers/brainco/native/bin/brainco_hand_server
test -e /sys/devices/platform/bus@0/3610000.usb/usb2/2-2/2-2.3/2-2.3:1.4
test -e /sys/devices/platform/bus@0/3610000.usb/usb2/2-2/2-2.2/2-2.2:1.4
python3 -c "import unitree_sdk2py, pinocchio, casadi"
```

如果 DDS 使用指定网卡，在 `config/dexfull.yaml` 设置
`control.network_interface`。不要同时运行旧 Bridge、旧 XR 或旧 BrainCo
服务，否则会发生端口、相机或 DDS 命令发布冲突。

建议在实机上明确指定 DDS 网卡，避免 Jetson 上的有线、无线、Docker 等
多个接口导致 SDK 自动选择错误：

```bash
ip -br link
export DEXFULL_DDS_INTERFACE=eth0  # 替换为实际连接机器人 DDS 的接口
python3 -m dexfull
```

DexFull 的状态订阅使用 Unitree SDK 官方推荐的回调模式。未启动 BrainCo 或
暂时没有里程计数据时不会再通过空轮询持续输出
`[Reader] take sample error`。如果启动 5 秒后仍未收到 `rt/lowstate`，日志会
每 10 秒给出一次带网卡名称的诊断提示；此时应检查网卡，而不是忽略提示。

### Teleimager 配置发现

`control.img_server_ip: auto` 会使用 Unity 连接对应的实机网卡地址访问
Teleimager，Unity 的 ZMQ 地址也保持该网卡地址。若 60000 配置端口尚未就绪，XR 会回退读取
`config/cam_config_client.yaml`，其次读取 `config/cam_config_server.yaml`。
显式设置 `control.img_server_ip` 时仍可连接独立部署的远端 Teleimager。

如果需要检查相机服务为何没有响应：

```bash
cd ~/ws/dexfull_robot
tail -n 200 log/teleimager.log
ss -ltnp | grep -E '60000|55555|55556|55557'
```

新机器人首次部署时用下面两步生成机器专属配置，避免修改安装包内的默认文件：

```bash
python3 -m dexfull.imaging.camera_setup --scan --rs
python3 -m dexfull.imaging.camera_setup --commission --rs \
  --left-video /dev/videoX --right-video /dev/videoY
```

结果写入 `~/.config/dexfull/hardware.yaml`。也可通过
`DEXFULL_HARDWARE_CONFIG` 指向其他配置。设备重插后即使 `/dev/videoN`
变化，服务也会按序列号或 USB 物理拓扑重新发现；若腕部相机没有硬件序列号，
首次标定仍必须明确左右关系，防止两路图像被静默交换。

DexFull 会在读取服务器配置后立即开放 60000 应答端口，再执行可能耗时几十秒
的 `/dev/video*` 探测。客户端仍保留 8 秒等待和本地配置回退，以兼容远端
Teleimager 以及较慢的旧版本服务。

### XR TLS 证书

证书查找顺序如下：

1. 构造参数或 `XR_TELEOP_CERT`、`XR_TELEOP_KEY`。
2. `~/.config/dexfull/cert.pem` 和 `key.pem`。
3. 兼容旧实机目录 `~/.config/xr_teleoperate/`。
4. DexFull 项目根目录。

全部缺失时，DexFull 会调用 `openssl` 自动生成一组自签名证书到
`~/.config/dexfull/`。如果系统没有 `openssl`，控制启动响应会直接给出缺少
证书的明确错误，不再让 Vuer 子进程以模糊的 `Errno 2` 静默退出。

### Unity 心跳与 XR 启动

XR 的首次启动会加载原生控制、IK 和手部驱动模块，实机上可能需要二十秒以上。
DexFull 2.0.2 起，XR 的启动、停止和重启请求在后台执行，同一 WebSocket 的
收包循环会继续响应 Unity 应用层 `ping`；服务器健康检查也不会在已知的生命周期
操作期间误删连接。最终 XR 响应仍使用原请求 `id`，并继续等待 `runtime_ready`，
Unity 方法名和消息结构均未改变。

若仍出现心跳超时，先确认机器人日志至少含 `DexFull 2.3.1 ready`，再对照时间戳检查
Unity 连接是否在 XR 初始化完成前被移除。正常情况下，初始化期间不应出现
`Client removed (timeout)`，连接关闭时也不应再出现未回收的 `Queue.get` 任务。

### BrainCo 启动、DDS 与退出

DexFull 2.0.3 起，选中手型的控制模块会在启动 BrainCo 硬件和 XR READY
计时之前预加载。BrainCo 左右手控制状态使用 DDS 回调，不再对空 reader 高频
调用 `Read()`，因此不会再由控制器持续刷出 `[Reader] take sample error`。

左右手状态默认最多等待 `runtime.hand_startup_wait`（15 秒）。该等待可被
XR stop 和 Ctrl+C 立即取消；超时会列出缺失的 DDS topic 并以降级模式继续，
不会停止 XR、机器人或相机。控制 session 退出时
会关闭手部 DDS endpoint、写线程和重定向子进程。启动失败回滚只有在控制线程
确认停止后才关闭原生 hand driver，避免运行中的控制线程失去 DDS 发布端。

DexFull 2.3.1 起，左右 BrainCo 和三路相机分别重连。物理掉线不会触发全局
安全联锁：未掉线的部件与摇操主循环继续运行。设备重试采用 1、2、5、10、30 秒
退避，达到 30 秒后无限重试。BrainCo 恢复时会从实测关节位置用约 1 秒渐变到
最新目标，避免执行掉线期间积累的旧目标而突跳。

## 安全验收顺序

1. 在机器人断开执行器或处于安全支撑状态时启动 DexFull。
2. Unity 调用 `get_basic_infos`，确认三路 URL 顺序为头、右腕、左腕。
3. 启动图像服务，确认 55555、55556、55557 都有帧且无连续 MJPG 错误。
4. 启动 XR 组件但暂不调用 `start_teleop`，确认 DDS 与 BrainCo 状态在线。
5. 检查 `robot_datas` 为约 30 Hz，关节数为 29 + 6 + 6。
6. 在安全区域调用 `start_teleop`，验证前进方向、朝向和骨盆高度。
7. 测试 pause/resume/stop/restart，确认不会遗留 BrainCo 或 teleimager 进程。

Windows 离线测试不能替代上述相机、DDS、BrainCo 和机器人运动测试。

物理插拔压力测试可在另一终端运行：

```bash
python3 scripts/device_recovery_stress.py \
  --url wss://10.2.5.18:7443/ws \
  --server-cert ~/.config/dexfull/ws-tls/server-cert.pem --duration 180 \
  --expect left_hand --expect head_camera
```

测试期间依次拔出、等待掉线消息、再插回设备。脚本必须同时收到
`DEVICE_DISCONNECTED` 和 `DEVICE_RECOVERED` 才通过；同时观察机器人控制与未拔出的
相机/灵巧手是否持续运行。
