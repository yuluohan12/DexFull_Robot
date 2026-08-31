# DexFull 架构与迁移说明

## 运行边界

DexFull 是一个项目和一个启动入口，但按实时性与故障域划分进程：

```text
DexFullApplication（主进程）
├─ Unity WebSocket
├─ DdsTelemetryCollector
├─ latest-state bus
├─ ControlRuntime 进程监管
├─ Teleimager 服务监管
└─ Hand Driver 服务监管
        │
        ├─ 低频 Pipe：start/pause/resume/stop、状态和错误
        └─ 定长共享内存：最新 Unity VR 帧（无 FIFO、无 JSON/pickle）
                    │
DexFullControl（spawn 控制进程）
├─ XR/Vuer 输入
├─ 坐标转换
├─ IK
├─ 机器人控制器
└─ 灵巧手控制插件
```

实时控制链 `XR -> IK -> robot controller` 完全位于控制进程内部。机器人与
灵巧手状态由主进程直接订阅 DDS，不从控制进程转发。这样 CasADi/IPOPT 的
GIL 和 CPU 峰值不会阻塞 Unity 心跳、WebSocket 发送或 Bridge DDS 采集。

## 最小进程通信契约

跨控制进程只允许三类通用信息：

1. 生命周期命令：`start`、`pause`、`resume`、`stop`。
2. 状态与错误：`STARTING`、`READY`、`RUNNING`、`PAUSED`、`STOPPING`、
   `STOPPED`、`ERROR` 和心跳时间。
3. Unity 需要的最新 VR 位姿与控制器输入。使用固定字段共享数组，写入会覆盖
   上一帧，不允许形成待处理队列。

该契约不包含机器人型号、IK 类型、灵巧手型号、DDS 消息或相机配置。新增
`RobotAdapter` 或 `HandPlugin` 不需要修改进程通信协议。

## 生命周期

- 重启 XR/控制域只重启 `DexFullControl`。
- Teleimager 默认保持运行。
- BrainCo 等外部 hand service 默认保持运行；只有显式 hand-driver 命令或
  DexFull 整体退出才停止。
- 控制进程异常不会关闭主进程的 Unity WebSocket；Unity 可以继续查询状态并
  发起重启。
- 控制进程采用 `spawn`，避免在已启动 DDS/线程的主进程上执行不安全的 fork。

## 依赖规则

- `bridge` 只依赖机器人/手的只读适配描述，不持有 IK 或控制器实例。
- `control` 通过注册器构造机器人与手控制器，不管理 Unity 连接。
- `hand_drivers` 不依赖 Bridge；插件可以声明独立状态采集器和外部硬件服务。
- `common` 不依赖业务模块。
- DexFull 不从保留的旧项目目录导入运行时代码。

## 旧项目迁移映射

| 原项目 | DexFull 位置 |
| --- | --- |
| `teleop_bridge` | `dexfull/bridge`、`dexfull/common` |
| `xr_teleoperate/teleop_hand_and_arm.py` | `dexfull/control/session.py` |
| `xr_teleoperate/robot_control/robot_arm*` | `dexfull/control/robots/unitree` |
| `xr_teleoperate/robot_hand_*` | `dexfull/hand_drivers/*` |
| `teleimager` | `dexfull/imaging` |
| `televuer` | `dexfull/xr` |
| `brainco_hand_service` | `dexfull/hand_drivers/brainco/native` |

旧目录继续作为实机回退基线保留，但不参与 DexFull 运行。
