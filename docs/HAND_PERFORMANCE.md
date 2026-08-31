# BrainCo 灵巧手频率优化与诊断

DexFull 2.3.1 将 BrainCo 手部链路拆成三个互不阻塞的执行单元，同时保持
Unity 的方法名、消息格式和 `robot_datas` 数据结构不变：

1. 手部重定向在独立进程中计算，`auto` 默认使用 IK 核心之外的低半区计算核。
2. DDS 命令发布在线程中保持 50 Hz，`auto` 默认避开 CPU 0 和手部计算核。
3. 原生串口服务先写最新命令，再执行可能阻塞的状态读取；重连保护使用 1 秒
   实际时间渐变，不再因串口循环变慢而把 50 帧渐变拉长到数秒。

## 实机部署

原生服务代码有变化，复制工程后必须在机器人上重新编译，否则启动的仍是旧的
`native/bin/brainco_hand_server`：

```bash
cd ~/ws/dexfull_robot
./scripts/build_brainco.sh
python3 -m dexfull
```

启动日志应包含 `DexFull 2.3.1 ready`。每 5 秒会输出三层频率：

- `BrainCo retarget stats`: `solve_hz` 是 XR 关键点到手指目标的计算频率。
- `command stats`: `target_hz` 是新目标到达频率，`write_hz` 是 DDS 命令发布频率。
- `hand serial stats`: `loop_hz` 是原生写命令循环频率，`state_hz` 是硬件反馈频率。

正常情况下 `write_hz` 和 `loop_hz` 应接近 50 Hz。若 `solve_hz` 很低，问题位于
重定向计算或 CPU 调度；若只有 `write_hz` 低，问题位于 Python DDS 发布；若前两者
正常而 `loop_hz/state_hz` 低，问题位于 USB、串口或 BrainCo SDK。

`teleop_data/robot_state/angle_data.csv` 保存的是实际关节反馈，不是命令目标。
反馈以 0.001 为单位量化，因此手指移动很慢时相邻帧相同是正常现象；判断控制卡顿
应同时观察上述三层频率，不能只按 CSV 中数值变化的行数推断命令频率。

## CPU 配置

默认配置通常适合 8 核实机：

```yaml
control:
  hand_retarget_cpu_affinity: auto
  hand_dds_cpu_affinity: auto
```

若机器人 cpuset 或核心负载布局不同，可显式填写逗号分隔的核心，例如
`hand_retarget_cpu_affinity: "2,3"` 和 `hand_dds_cpu_affinity: "1"`；设置为
`off` 可禁用对应绑定。