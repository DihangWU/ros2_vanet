# ROS2 Jazzy：车辆状态与制动闭环

SUMO 是车辆运动真值源。ROS2 读取车辆状态、接收警告并发出后车制动命令，Gazebo 和 RViz2 同步展示。默认接入 Veins / OMNeT++ / INET 后车实际收包事件；人工警告保留为可选学习入口。

## 构建与运行

默认 `scenario:=traffic`：400 m 双车道直路、A/B 和 bg_01～bg_06 六辆背景车，时长 30 秒。初始车辆 ID 从 SUMO 路线文件读取，全部车辆发布 `/<id>/odom`、`/<id>/path` 和 TF。道路 Marker 长度读取 SUMO 节点定义，与 Gazebo 道路同步。旧场景入口为 `scenario:=two_cars duration:=15.0`。

从项目根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/runtime"
ros2 launch cosim_bridge demo.launch.py
```

默认先打开 Gazebo 和 RViz2，8 秒后打开 SUMO。点击 SUMO 的绿色“开始 / Play”按钮才开始运行；约第 5 秒前车急刹，30 秒结束后保留最终画面，Ctrl+C 退出。不要单独点击 Gazebo 的播放按钮。

无界面运行时无需点击按钮，会自动实时运行：

```bash
ros2 launch cosim_bridge demo.launch.py sumo_gui:=false gazebo:=false rviz:=false
```

关闭测试警告，观察普通跟车基线：

```bash
ros2 launch cosim_bridge demo.launch.py network:=false test_warning:=false
```

`network:=false` 恢复人工警告模式，此时 `warning_delay:=0.1` 控制测试警告延迟，可取 0 到 0.4 秒。网络模式不启动测试节点，`test_warning` 和 `warning_delay` 不影响实际收包。`network_gui:=true` 可打开 Qtenv，演示请用 F6 Fast Run，普通 Run 的包动画可能让联合时间暂停。`playback_rate:=0.5` 为半速；`duration` 默认 30 秒，由 ROS2、Veins 和 SUMO 统一使用（当前最少 15 秒）；`startup_delay` 仅控制 SUMO 窗口打开时间。从其他目录启动需指定 `project_root`。
每次启动前关闭上一组演示，避免同时发布两个 `/clock`。

## 默认网络闭环

Veins 是唯一 SUMO 步进者。`network_backend.py` 在 TCP 9998 接收车辆状态和 INET 实际收包事件，经 `sumo_bridge` 发布 `/v2v_warning`，`simulated=false`。控制器发布 `/brake_cmd` 后，ROS2 在下一次应答中把停车目标交给 Veins 执行。ROS2 在此模式不调用 TraCI `simulationStep()`。

收包桥接当前集成在 `sumo_bridge`，没有独立的 `v2v_receiver` 节点。每步 JSONL 应答用于同步进程，不替代车辆间无线网络。网络构建依赖与安装路径见 [OMNeT++ README](../omnet/README.md)。

## 可选人工警告数据流（network:=false）

```text
sumo_bridge：前车在 t=5 急刹
  │ /demo/front_brake（真实交通事件，不是收包事件）
  ▼
test_warning_publisher（可关闭，人工延迟 0.1 秒）
  │ /v2v_warning，simulated=true
  ▼
brake_controller
  │ /brake_cmd
  ▼
sumo_bridge：下一次步进前调用 TraCI setSpeed(car_b)，按实时净间距调整速度
  │ 车辆状态、TF、Path、/clock、Marker
  ├──> RViz2
  └──> gazebo_sync → Gazebo
```

测试输入显示为 `TEST WARNING`，其延迟不等于无线网络延迟。默认实际收包输入显示为 `WARNING RECEIVED`，复用相同控制接口。

## 文件阅读顺序

| 文件 | 学习内容 |
| --- | --- |
| `src/cosim_interfaces/msg/V2VWarning.msg` | 事件 ID、发送车、接收车、事件时间、位置、速度和模拟标记 |
| `src/cosim_interfaces/msg/BrakeCommand.msg` | 命令时间、关联事件 ID、目标车、目标速度和目标停车净间距 |
| `src/cosim_bridge/cosim_bridge/test_warning_publisher.py` | 真实交通事件如何变成测试警告 |
| `src/cosim_bridge/cosim_bridge/network_backend.py` | 实际收包事件、锁步 JSONL 接口、网络进程管理 |
| `src/cosim_bridge/cosim_bridge/communication_visuals.py` | 通信状态、文本、范围圆、收发位置与包回放 |
| `src/cosim_bridge/cosim_bridge/brake_controller.py` | 警告校验、去重和生成命令 |
| `src/cosim_bridge/cosim_bridge/sumo_bridge.py` | SUMO 步进、执行命令、响应记录和状态发布 |
| `src/cosim_bridge/cosim_bridge/event_log.py` | 各节点独立 JSONL 日志 |
| `src/cosim_bridge/cosim_bridge/gazebo_sync.py` | 姿态服务和 Gazebo 定步时间同步 |
| `src/cosim_bridge/cosim_bridge/car_visuals.py` | RViz 三维车体部件 |
| `src/cosim_bridge/launch/demo.launch.py` | 启动顺序和参数 |

自定义消息放在独立的 `ament_cmake` 包，Python 节点放在 `ament_python` 包。

## Topic 与接口

| Topic | 类型 | 含义 |
| --- | --- | --- |
| `/demo/front_brake` | `cosim_interfaces/msg/V2VWarning` | 人工警告模式的前车交通急刹事件；网络模式记入日志 |
| `/v2v_warning` | `cosim_interfaces/msg/V2VWarning` | 默认为后车 INET 实际收包警告；可选人工测试输入 |
| `/brake_cmd` | `cosim_interfaces/msg/BrakeCommand` | 后车最终降至 0 m/s，目标停车净间距 2.5 m |
| `/car_a/odom`、`/car_b/odom` | `nav_msgs/msg/Odometry` | 车辆中心位置、朝向和前进速度 |
| `/car_a/path`、`/car_b/path` | `nav_msgs/msg/Path` | 行驶轨迹 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `map` 到车辆 `base_link` |
| `/clock` | `rosgraph_msgs/msg/Clock` | SUMO 仿真时间 |
| `/demo/markers` | `visualization_msgs/msg/MarkerArray` | 三维小车、道路、警告线、速度与状态 |
| `/v2v/markers` | `visualization_msgs/msg/MarkerArray` | 收发时间、无线延迟、制动时间与通信示意 |

查看字段和消息：

```bash
ros2 interface show cosim_interfaces/msg/V2VWarning
ros2 interface show cosim_interfaces/msg/BrakeCommand
ros2 topic echo /brake_cmd
```

控制器接受 `car_a → car_b` 的 `EMERGENCY_BRAKE`，检查坐标系、速度和事件时效，并按事件 ID 去重；当前最大警告年龄 0.5 仿真秒。桥接节点再次检查目标车、时间和停车净间距，只接受一次停车命令。订阅回调先保存命令，由唯一 SUMO 步进者在下一步前执行。收到停车目标后，每步根据前车速度和实时净间距更新后车目标速度；靠近目标且两车速度足够低时保持后车零速。

## 时间、坐标与控制边界

SUMO 内部步长 0.01 秒，ROS2 每 0.05 仿真秒取样，默认 20 Hz。控制器和测试发布器使用 `/clock`；步进定时器使用稳态时钟。事件记录精度受 0.05 秒取样周期及 ROS2 调度影响。

`map` 保留 SUMO 平面坐标。两种模式均读取实际车头位置和角度，转换为 ROS yaw，再沿朝向减去半车长作为车体中心。双车道右车道中心 y=-4.8，左车道中心 y=-1.6。状态映射已支持不同车道；A/B 停车策略仍假定两车在同一条直路同一车道。

SUMO 原有跟车安全规则仍启用，因此“后车降速”本身不能证明是 ROS2 命令导致；需结合收包和 `brake_command_applied` 事件，以及普通跟车或无收包基线判断。当前为单次警告学习演示，不是统计性的无线性能评估。

Gazebo 从暂停状态按 SUMO 时间定步推进，不发布另一个 ROS2 时钟。两个三维窗口不自动重置相机；操作见 [Gazebo](../gazebo/README.md) 和 [RViz2](../rviz/README.md)。

通信显示独立发布到 `/v2v/markers`，发包/收包来自网络帧中的 `network_events`，命令发布时间来自 `/brake_cmd` 时间戳，执行时间来自 Veins 确认。范围圆可通过 `communication_range_m` 调整，仅作距离参照；动画使用现实时间播放 1.5 秒，不影响 `/clock`。网络连接断开后停止步进，继续发布最后状态及红色断连提示。详见 [RViz2 通信显示](../rviz/README.md)。

## 日志与验证

输出到 `ros/log/events/`，每次运行覆盖相应节点文件，已被 Git 忽略：

- `test_warning_publisher.jsonl`：测试警告发布。
- `brake_controller.jsonl`：警告接收、命令发布和拒绝原因。
- `sumo_bridge.jsonl`：前车急刹、命令接收与执行、首次后车速度响应及结束摘要。
- `trajectory.csv`：两车中心位置、速度和净间距。
- `traffic_trajectory.csv`：全部车辆每步的 ID、中心 x/y、yaw 和速度。
- `summary.json`：是否执行 ROS2 制动、最小间距和最终速度。
- 默认网络模式摘要还包含 `scenario`、`vehicle_count` 和 `collision_steps`；碰撞检查失败时中止推进，不生成正常完成摘要。
- `network_process.log`：默认模式的 SUMO / OMNeT++ 进程输出；网络失败时优先查看。

网络精确发包、收包与控制应用时间另存于 `omnet/results/network_events.jsonl`，统计量为同目录 `.sca/.vec`。日志中的 `network_delay_s` 是无线发送到接收延迟，不能与 0.05 秒的 ROS2 桥接周期混为一谈。

JSONL 同时记录 SUMO 仿真时间和本机单调时钟。测试节点关闭时，不会生成新的测试警告日志；旧文件不代表本轮事件。

默认网络模式已验证：5.00 秒急刹、5.001 秒发包、5.001152 秒实际收包、5.05 秒执行制动；30 秒两车均停止，净间距约 2.548 m。极低发射功率对照无警告、无 ROS2 制动，最终间距约 2.521 m。原人工警告模式仍保留，其警告及命令约在 5.10 秒出现。这些结果对应当前初始间距和驾驶参数。

控制器边界检查（从项目根目录执行，先 source 环境）：

```bash
python3 ros/tests/test_brake_controller.py
python3 ros/tests/test_communication_visuals.py
# 默认八车网络演示完成后检查真实输出
python3 ros/tests/check_traffic_run.py
```

已验证有效警告、重复警告、过期警告、错误目标车辆和非法速度的处理。实际 INET 收包闭环也已完成联调。

## 靠近停车策略

`BrakeCommand.desired_gap_m` 默认 2.5 m，替代原来的固定减速时长字段。净间距为前车车尾到后车车头的距离。桥接节点用“前车速度 + 净间距误差”计算后车目标速度，限制在 0 到 15 m/s，并保留 SUMO 安全跟车约束。当前比例系数为 1/s。当前车基本停下、净间距不超过 2.55 m 且后车速度低于 0.05 m/s 时，保持后车零速。

这使车辆收到警告后可以继续收拢间距，而非立即在远处停下。该策略用于当前低规模学习场景，不代表高速紧急制动系统的安全设计。接口字段已修改，重新运行前需构建并重新 source 工作空间。
