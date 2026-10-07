# ROS2 Jazzy：车辆状态与制动闭环

SUMO 是车辆运动真值源。ROS2 读取车辆状态、接收警告并发出后车制动命令，Gazebo 和 RViz2 同步展示。当前警告来自测试节点，尚未接入 Veins / OMNeT++ / INET 的无线收包事件。

## 构建与运行

从项目根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/runtime"
ros2 launch cosim_bridge demo.launch.py
```

默认先打开 Gazebo 和 RViz2，8 秒后打开 SUMO。点击 SUMO 的绿色“开始 / Play”按钮才开始运行；约第 5 秒前车急刹，15 秒结束后保留最终画面，Ctrl+C 退出。不要单独点击 Gazebo 的播放按钮。

无界面运行时无需点击按钮，会自动实时运行：

```bash
ros2 launch cosim_bridge demo.launch.py sumo_gui:=false gazebo:=false rviz:=false
```

关闭测试警告，观察普通跟车基线：

```bash
ros2 launch cosim_bridge demo.launch.py test_warning:=false
```

`warning_delay:=0.1` 默认延迟 0.1 仿真秒发布测试警告，可取 0 到 0.4 秒。`playback_rate:=0.5` 为半速；`duration` 默认 15 秒；`startup_delay` 仅控制 SUMO 窗口打开时间。从其他目录启动需指定 `project_root`。
每次启动前关闭上一组演示，避免同时发布两个 `/clock`。

## 节点与数据流

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

测试输入显示为 `TEST WARNING`，其延迟不等于无线网络延迟。未来替换测试节点，让真实后车收包事件发布 `/v2v_warning` 即可复用控制接口。

## 文件阅读顺序

| 文件 | 学习内容 |
| --- | --- |
| `src/cosim_interfaces/msg/V2VWarning.msg` | 事件 ID、发送车、接收车、事件时间、位置、速度和模拟标记 |
| `src/cosim_interfaces/msg/BrakeCommand.msg` | 命令时间、关联事件 ID、目标车、目标速度和目标停车净间距 |
| `src/cosim_bridge/cosim_bridge/test_warning_publisher.py` | 真实交通事件如何变成测试警告 |
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
| `/demo/front_brake` | `cosim_interfaces/msg/V2VWarning` | 前车交通急刹事件 |
| `/v2v_warning` | `cosim_interfaces/msg/V2VWarning` | 后车收到的警告；当前由测试节点模拟 |
| `/brake_cmd` | `cosim_interfaces/msg/BrakeCommand` | 后车最终降至 0 m/s，目标停车净间距 2.5 m |
| `/car_a/odom`、`/car_b/odom` | `nav_msgs/msg/Odometry` | 车辆中心位置、朝向和前进速度 |
| `/car_a/path`、`/car_b/path` | `nav_msgs/msg/Path` | 行驶轨迹 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `map` 到车辆 `base_link` |
| `/clock` | `rosgraph_msgs/msg/Clock` | SUMO 仿真时间 |
| `/demo/markers` | `visualization_msgs/msg/MarkerArray` | 三维小车、道路、警告线、速度与状态 |

查看字段和消息：

```bash
ros2 interface show cosim_interfaces/msg/V2VWarning
ros2 interface show cosim_interfaces/msg/BrakeCommand
ros2 topic echo /brake_cmd
```

控制器接受 `car_a → car_b` 的 `EMERGENCY_BRAKE`，检查坐标系、速度和事件时效，并按事件 ID 去重；当前最大警告年龄 0.5 仿真秒。桥接节点再次检查目标车、时间和停车净间距，只接受一次停车命令。订阅回调先保存命令，由唯一 SUMO 步进者在下一步前执行。收到停车目标后，每步根据前车速度和实时净间距更新后车目标速度；靠近目标且两车速度足够低时保持后车零速。

## 时间、坐标与控制边界

SUMO 内部步长 0.01 秒，ROS2 每 0.05 仿真秒取样，默认 20 Hz。控制器和测试发布器使用 `/clock`；步进定时器使用稳态时钟。事件记录精度受 0.05 秒取样周期及 ROS2 调度影响。

`map` 保留 SUMO 平面坐标。SUMO 车头位置减去半个车长后作为 ROS 车体中心；SUMO 从正北顺时针的角度转换为 ROS 从正东逆时针的 yaw。车体 x 轴朝前。

SUMO 原有跟车安全规则仍启用，因此“后车降速”本身不能证明是 ROS2 命令导致；需结合 `brake_command_applied` 事件和关闭测试警告的基线判断。当前场景比较的是固定测试输入下的行为，不是无线网络效果评估。

Gazebo 从暂停状态按 SUMO 时间定步推进，不发布另一个 ROS2 时钟。两个三维窗口不自动重置相机；操作见 [Gazebo](../gazebo/README.md) 和 [RViz2](../rviz/README.md)。

## 日志与验证

输出到 `ros/log/events/`，每次运行覆盖相应节点文件，已被 Git 忽略：

- `test_warning_publisher.jsonl`：测试警告发布。
- `brake_controller.jsonl`：警告接收、命令发布和拒绝原因。
- `sumo_bridge.jsonl`：前车急刹、命令接收与执行、首次后车速度响应及结束摘要。
- `trajectory.csv`：两车中心位置、速度和净间距。
- `summary.json`：是否执行 ROS2 制动、最小间距和最终速度。

JSONL 同时记录 SUMO 仿真时间和本机单调时钟。测试节点关闭时，不会生成新的测试警告日志；旧文件不代表本轮事件。

已验证：5.00 秒前车急刹，5.10 秒发布警告及执行后车制动，6.60 秒首次记录到后车降速；最终两车均停止，净间距约 2.548 m。关闭测试警告时，未执行 ROS2 制动，最小净间距约 2.521 m。这些结果对应当前初始间距和驾驶参数。

控制器边界检查（从项目根目录执行，先 source 环境）：

```bash
python3 ros/tests/test_brake_controller.py
```

已验证有效警告、重复警告、过期警告、错误目标车辆和非法速度的处理。当前闭环由测试警告驱动，下一步接入真实 VANET 收包事件。

## 靠近停车策略

`BrakeCommand.desired_gap_m` 默认 2.5 m，替代原来的固定减速时长字段。净间距为前车车尾到后车车头的距离。桥接节点用“前车速度 + 净间距误差”计算后车目标速度，限制在 0 到 15 m/s，并保留 SUMO 安全跟车约束。当前比例系数为 1/s。当前车基本停下、净间距不超过 2.55 m 且后车速度低于 0.05 m/s 时，保持后车零速。

这使车辆收到警告后可以继续收拢间距，而非立即在远处停下。该策略用于当前低规模学习场景，不代表高速紧急制动系统的安全设计。接口字段已修改，重新运行前需构建并重新 source 工作空间。
