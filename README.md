# DexFull

DexFull 是统一的 XR 机器人遥操作工程。它将原来的 `teleop_bridge`、
`xr_teleoperate` 和 `brainco_hand_service` 重组为一个项目、一个配置入口和
一个生命周期，同时保留 Unity 现有 WebSocket 方法与消息格式。

当前实机适配基线为工作区中的 `teleop_bridge_1.1` 与
`xr_teleoperate_1.1`。DexFull 不在运行时导入这两个目录，相关实机行为已
迁入自身模块。

## 模块边界

```text
Unity WebSocket
       │
       ▼
dexfull.bridge
  ├─ 独立 DDS 机器人/灵巧手状态采集
  ├─ latest-state bus
  └─ robot_state / robot_datas / vr_input 传输

dexfull.control
  ├─ XR 输入
  ├─ 遥操作状态机
  ├─ IK 与控制命令
  └─ dexfull.control.robots 注册式机器人适配

dexfull.hand_drivers
  ├─ brainco（做法 B：统一管理、原生子进程隔离）
  ├─ unitree dex1/dex3
  └─ inspire dfx/ftp
```

Bridge 直接订阅 `rt/lowstate`、`rt/odommodestate` 和选中灵巧手的状态
主题。其 30 Hz 数据时钟不依赖 IK 控制循环，因此控制耗时不会改变 Unity
收到的状态采样节奏。XR、坐标转换、IK 和机器人控制运行在独立的
`DexFullControl` 子进程；Bridge 与控制进程之间只传递低频生命周期/健康状态，
以及固定共享内存中的最新 VR 帧，不再通过旧 XR-Bridge IPC 转发所有控制和
DDS 数据。重启 XR 默认不会重启 Teleimager 或 BrainCo 服务。

## 启动

```bash
cd DexFull
python3 -m pip install -e ".[control,imaging]"
./scripts/build_brainco.sh       # 仅 BrainCo 首次构建需要
python3 -m dexfull
```

主配置为 `config/dexfull.yaml`。也可以设置：

```bash
export DEXFULL_CONFIG=/path/to/dexfull.yaml
```

当前 `cam_config_server.yaml` 已同步实机 1.1 的设备标识：头部 RealSense
序列号 `254322073111`，左右腕部 OpenCV 相机使用 Jetson USB physical path。
更换相机或 USB 口后必须同步修改这三个标识。

实机部署与验收步骤见 `docs/REAL_ROBOT_1_1.md`；BrainCo 灵巧手的频率诊断与
CPU 配置见 `docs/HAND_PERFORMANCE.md`。

## 扩展机器人

实现控制器和 IK 类，然后创建 `RobotAdapter` 并调用 `register_robot()`。
Bridge 会从适配器读取 DDS 消息族、关节编号和名称；Control 通过同一适配器
创建控制器和 IK，不需要在主循环增加新的型号分支。

## 扩展灵巧手

每种灵巧手提供一个 `HandPlugin`：

- `control_factory`：创建控制所需共享对象和控制器；
- `collector_factory`：可选的 Bridge 独立状态采集器；
- `service_name`：厂商 SDK 需要隔离进程时填写；
- 左右手关节名称：用于保持 Unity 数据顺序稳定。

BrainCo 是完整示例，原生源码、头文件、库和二进制都位于
`dexfull/hand_drivers/brainco/native`，不再是顶层独立项目。

## 兼容与回退

原三个目录仍保留，不参与 DexFull 运行时导入。现场验证完成前，可以继续使用
原启动方式回退。Unity 接口清单见 `docs/UNITY_API.md`。
