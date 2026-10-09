# ROS2 Jazzy：状态桥接与雷达制动节点

默认从 Veins / INET 实际收包事件发布 V2V 警告，使用 Gazebo 原始点云反馈后车车距。SUMO 负责车辆运动和自车速度；ROS2 负责消息包装、同步及控制命令。纯算法独立放在 [Algorithm/LidarBrake](../Algorithm/LidarBrake/README.md)。

## 构建与启动

### 独立传感器预览

完成下方构建后，从项目根目录运行：

```bash
source /opt/ros/jazzy/setup.bash
source ros/install/setup.bash
ros2 launch cosim_bridge sensor_preview.launch.py
```

入口 `launch/sensor_preview.launch.py` 启动 Gazebo、传感器桥接、`sensor_preview` 静态展示节点和 RViz。不启动 SUMO、网络仿真、`gazebo_sync`、交通桥接或制动控制器；不读取 SUMO 路线，不需要 SUMO 安装。车辆初始姿态和道路几何直接读取 `gazebo/worlds/{traffic,two_cars}.sdf`。传感器扫描入口 `sensors.launch.py` 也直接读取同一个世界中的车辆模型引用。

Gazebo 是预览模式唯一的 `/clock` 来源。启动时为世界生成临时副本，将 `real_time_factor` 设为 `playback_rate`（默认 1），使用 `-r` 自行播放；原联合仿真世界不修改，退出时清理临时文件。可以使用 Gazebo Play / Pause 控制传感器采样。车辆保持静止，静态 `map → <车辆ID>/base_link` TF 与车体 / 传感器 / 道路 Marker 独立发布；没有 Odom、运动轨迹、V2V 或制动结果。移动 GUI 相机不会改变车辆；此预览不跟踪通过 Gazebo 编辑工具手动挪动车辆的姿态。

```bash
# 旧两车静态世界
ros2 launch cosim_bridge sensor_preview.launch.py scenario:=two_cars
# Gazebo 后台采样，仅打开 RViz
ros2 launch cosim_bridge sensor_preview.launch.py gui:=false
# 不开窗口，保留 ROS2 传感器输出
ros2 launch cosim_bridge sensor_preview.launch.py gui:=false rviz:=false
# 半速传感器时间；实际速度也受渲染负载限制
ros2 launch cosim_bridge sensor_preview.launch.py playback_rate:=0.5
```

运行 `python3 ros/tests/check_sensor_preview_run.py` 可验证默认十一车预览的单一时钟、静态 TF / Marker、六路拼图和五路点云；`check_surround_camera_run.py` 验证六颗独立相机的标定及拼图。预览模式与完整 `demo.launch.py` 使用相同话题、坐标系和世界名称，应先 Ctrl+C 退出上一组再切换，不可直接将交通启动叠加到预览上。当前两种入口分别运行，未实现运行中切换时钟或 SUMO 接管。

### 完整交通演示

`surround_camera_display` 为独立侧后相机包装节点，自动读取 [侧后模组](../gazebo/models/surround_camera_rig/README.md) 的安装与光学参数，建立完整 roll/pitch/yaw 安装 TF 和 optical TF（左右相机向下俯视 10°）。新增 `/car_b/camera/{rear,left,right}/{image_raw,camera_info}`；原 `/car_b/camera/{tele,standard,wide}/...` 独立话题保留。六路图像同时间戳时发布 `/car_b/camera/surround/image_raw`（1280×1176），两列三行：左列长焦 / 主摄 / 广角，右列后 / 左 / 右；拼图不发布 CameraInfo。独立通道使用传感器 Best Effort QoS，拼图使用 Reliable QoS；任何一路缺帧只影响该组预览，不阻止独立图像发布。缓存上限十组。只引用侧后模组时预览自动退为后 / 左 / 右三行一列。桥接扫描与车顶、角雷达模块分别识别，其他车辆一行 include 即可复用。

运行中可执行 `python3 ros/tests/check_surround_camera_run.py` 检查六路真实图像、独立标定、光轴 TF、同步时间戳和拼图排列。

从项目根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/runtime"
ros2 launch cosim_bridge demo.launch.py
```

默认三车道十一车场景，时长 30 秒。Gazebo、RViz2 先打开，8 秒后打开 SUMO，点击 SUMO Play 才开始；第 5 秒前车急刹，结束后保留画面，Ctrl+C 退出。不要单独点击 Gazebo Play。运行前关闭上一组演示，避免端口和 /clock 冲突。

| 参数 | 功能 |
| --- | --- |
| `control_mode:=lidar` | 默认真实雷达反馈，需要开启 Gazebo |
| `control_mode:=sumo` | 旧前车真值反馈对照，可关闭 Gazebo |
| `network:=false` | ROS2 唯一步进 SUMO，改用人工警告 |
| `test_warning:=false` | 人工模式关闭警告；雷达模式仍会依据点云减速 |
| `network_gui:=true` | 打开 Qtenv，演示使用 F6 Fast Run |
| `sumo_gui:=false` | 自动开始，无需点击 SUMO Play |
| `rviz:=false` | 关闭 RViz |
| `show_background_labels:=true` | 显示背景车头顶标签，默认透明隐藏，保留标签生成逻辑 |
| `playback_rate:=0.5` | 半速，渲染负载可能进一步限制实际速度 |
| `duration:=30.0` | 仿真时长，最少 15 秒 |
| `startup_delay:=15.0` | 延迟打开 SUMO，不自动开始 |
| `scenario:=two_cars duration:=15.0` | 旧两车场景 |
| `project_root:=...` | 从其他目录启动时指定资源根目录 |

不显示 SUMO 窗口，但保留真实传感器：

```bash
ros2 launch cosim_bridge demo.launch.py sumo_gui:=false
# 无 Gazebo 的旧真值控制对照
ros2 launch cosim_bridge demo.launch.py control_mode:=sumo sumo_gui:=false gazebo:=false rviz:=false
# SUMO 自带跟车基线
ros2 launch cosim_bridge demo.launch.py control_mode:=sumo network:=false test_warning:=false
```

## 雷达闭环

```text
Gazebo 三维雷达 → /car_b/lidar/points ──┐
后车 Odom 的速度 → /car_b/odom ──────┼→ lidar_brake → Algorithm/LidarBrake
INET 实际收包 → /v2v_warning ────────┘                      │
                                                 /lidar_brake/cmd
                                                         ↓
                  network_backend → Veins → TraCI → SUMO 后车
                  或 network:=false 时 sumo_bridge 直接执行
```

`lidar_brake_node.py` 是 ROS2 节点包装，不是 Gazebo 插件。点云转换到本车 base_link 后送入纯算法；转换和保险杠位置读取本车模型几何。只读取后车 Odom 的速度，不读取其位置来测距，不订阅前车 Odom；V2V 只触发提前制动，不使用报文内前车位置或速度计算距离。

算法筛除地面、自车和邻道点，体素聚类得到前方有效障碍表面，滤波估计间距与相对速度，用时间车距 PD 生成加速度，并添加 TTC 保护、jerk 限制和停车保持。当前适用于固定直路有前车的演示，未实现目标身份关联。参数见 `Algorithm/LidarBrake/config.yaml`。

`/brake_cmd` 保留 V2V 事件与停车目标，供通信日志与显示使用。雷达模式真正执行的连续目标速度来自 `/lidar_brake/cmd`，不会执行旧真值跟车策略。SUMO 后车 `speedMode=30` 关闭安全跟车速度覆盖，保留加减速度及交通规则限制；命令失效时执行者依据自车速度减速。

`control_mode:=sumo` 才根据前车速度和真值净间距收拢到 2.5 m，并保留 SUMO 原有安全跟车约束。该对照结果不能用来证明雷达控制生效。

## 唯一步进与时间

网络模式 Veins 唯一步进 SUMO，ROS2 通过 TCP 9998 接收状态、INET 收包及执行确认，并回复控制；ROS2 不调用 simulationStep。JSONL/TCP 是进程同步接口，不替代车辆间无线通信。网络安装与构建见 [OMNeT++](../omnet/README.md)。

人工模式 `network:=false` 由 sumo_bridge 唯一步进，真实前车急刹事件经测试节点产生 `simulated=true` 警告。`warning_delay:=0.1` 可调整人工延迟，不改变实际 INET 收包时间。

SUMO 内部步长 0.01 秒，状态与控制间隔 0.05 秒，雷达 10 Hz。ROS2 /clock 始终来自 SUMO。Gazebo 批量更新车辆姿态并按 SUMO 时间定步推进；`/gazebo/clock` 仅表示 Gazebo 实际时间，经 `/gazebo/synced` 确认同步。雷达模式等待本帧自车速度、近期点云和控制命令再推进，避免交通跑在渲染之前。传感器空帧进入失效制动；超过 0.35 秒失效，协调等待超过 15 秒现实时间则暂停并报错，不切回真值控制。

map 使用 SUMO 平面坐标，车头位置沿朝向减半车长得到模型中心。车道中心为 y=-8.0、-4.8、-1.6 m。Gazebo 不计算另一套车辆动力学；两个三维窗口均可自由调整视角。

## 传感器与 Topic

`sensors.launch.py` 扫描引用 `model://perception_rig` 的车辆，自动建立独立桥接和 TF，当前只安装于 car_b。安装配置见 [传感器说明](../gazebo/models/perception_rig/README.md)。

同时自动识别 `model://corner_radar_rig`，逐车启动 `corner_radar_display`。四路原始点云与各自彩色点云独立发布，安装 TF 和颜色读取独立 SDF；当前仅 car_b 装备。侧后方点云用于观察，尚未接入制动或目标跟踪。安装与复用方式见 [四角雷达说明](../gazebo/models/corner_radar_rig/README.md)。

| Topic | 功能 |
| --- | --- |
| `/v2v_warning` | 实际收包警告，或标记 simulated 的人工输入 |
| `/brake_cmd` | V2V 事件对应的停车目标 |
| `/lidar_brake/cmd` | LongitudinalCommand：序号、目标速度、加速度、测量时间和状态 |
| `/lidar_brake/markers` | 雷达测量车距、目标车距、相对速度、加速度与状态 |
| `/<id>/odom`、`/<id>/path` | 全部车辆状态和轨迹 |
| `/tf`、`/tf_static` | map、车体、雷达及相机 optical frame |
| `/clock` | SUMO 仿真时间 |
| `/gazebo/clock`、`/gazebo/synced` | 展示世界时间及逐帧同步确认 |
| `/demo/markers` | 车体、道路、传感器模组和速度状态 |
| `/v2v/markers` | 收发时间、无线延迟、V2V 命令时间、范围圆和包回放 |
| `/car_b/lidar/points` | 原始三维点云，算法输入 |
| `/car_b/lidar/points_colored` | 地面绿、其他有效回波青，仅用于 RViz 显示 |
| `/car_b/radar/{front_left,front_right,rear_left,rear_right}/points` | 四路独立三维距离回波点云，20 Hz |
| `/car_b/radar/{front_left,front_right,rear_left,rear_right}/points_colored` | 四路有效点云，障碍表面按雷达着色、地面绿色 |
| `/car_b/camera/{wide,standard,tele}/image_raw` | 独立三路图像，640×360、15 Hz |
| `/car_b/camera/{wide,standard,tele}/camera_info` | 三套独立标定 |
| `/car_b/camera/triple/image_raw` | 640×1176 同时间戳竖排拼图：上长焦、中主摄、下广角 |
| `/car_b/camera/{rear,left,right}/image_raw`、`camera_info` | 三路独立侧后图像与各自标定 |
| `/car_b/camera/surround/image_raw` | 默认 RViz 预览，1280×1176 两列三行，左列前向三目、右列后 / 左 / 右 |

`/car_b/sensors/raw/...` 是桥接内部话题。拼图没有统一内参，不发布拼图 CameraInfo。点云高度着色不是障碍物语义分类。通信 Marker 中发布时间与执行时间是 V2V 事件命令；每次连续雷达命令另见雷达日志和网络执行日志。

## 文件阅读顺序

| 文件 | 学习内容 |
| --- | --- |
| `../Algorithm/LidarBrake/` | 不依赖 ROS 的点云控制算法 |
| `src/cosim_bridge/cosim_bridge/lidar_brake_node.py` | 算法输入转换、V2V 触发、命令与 Marker |
| `src/cosim_interfaces/msg/` | 警告、停车目标和连续纵向命令 |
| `src/cosim_bridge/cosim_bridge/network_backend.py` | 实际网络事件和锁步应答 |
| `src/cosim_bridge/cosim_bridge/sumo_bridge.py` | 命令校验、逐步协调、状态发布、可选直接 TraCI |
| `src/cosim_bridge/cosim_bridge/brake_controller.py` | V2V 事件命令校验与去重 |
| `src/cosim_bridge/cosim_bridge/gazebo_sync.py` | 批量姿态、定步和实际世界时钟确认 |
| `src/cosim_bridge/cosim_bridge/perception_display.py` | 原始传感器输出、TF、三摄拼图和高度着色 |
| `src/cosim_bridge/launch/sensor_preview.launch.py` | 独立 Gazebo 时钟、静态场景、传感器与 RViz 预览入口 |
| `src/cosim_bridge/cosim_bridge/preview_scene.py` | 直接解析 Gazebo 世界中的车辆与初始姿态，无 SUMO 导入 |
| `src/cosim_bridge/cosim_bridge/sensor_preview.py` | 静态车辆 TF 和共享车体 / 传感器 / 世界道路 Marker |
| `src/cosim_bridge/cosim_bridge/surround_camera_display.py` | 独立侧后图像、标定、光轴 TF 与六路同时间戳拼图 |
| `src/cosim_bridge/cosim_bridge/corner_radar_display.py` | 四角独立点云输出、安装 TF 和回波着色 |
| `src/cosim_bridge/cosim_bridge/communication_visuals.py` | 网络通信显示 |
| `src/cosim_bridge/cosim_bridge/sensor_visuals.py` | 从独立 SDF 读取传感器外观 |
| `src/cosim_bridge/launch/demo.launch.py` | 启动顺序与参数 |

ROS2 包依赖 NumPy、PyYAML、OpenCV、cv_bridge、sensor_msgs_py、ros_gz_bridge 以及 Gazebo Python 绑定 `gz_transport_vendor`、`gz_msgs_vendor`。更改自定义消息后重新构建并 source 工作空间。

## 日志与检查

默认三车道网络雷达模式已实跑验证：301 帧真实点云、599 次外部速度命令被 Veins 接收执行，30 秒两车均零速停止，最小/最终净间距约 2.548 m，无碰撞；测距误差中位数约 0.020 m。1 倍速下约 30 秒现实时间完成。人工警告加直接 TraCI 雷达模式也已完成 30 秒演示，最终间距约 2.549 m。结果对应当前场景及默认参数；运动真值仅用于这些误差评估。

每次运行覆盖相应 `ros/log/events/` 文件，生成日志不纳入 Git：

- `lidar_brake.jsonl`：真实点云测量、相对速度、V2V 触发及连续命令。
- `brake_controller.jsonl`：V2V 事件命令。
- `sumo_bridge.jsonl`：交通事件、命令接收与执行及结束摘要。
- `trajectory.csv`：A/B 运动真值和净间距，仅供评估。
- `traffic_trajectory.csv`：全部车辆的轨迹。
- `summary.json`：本轮 control_mode、命令序号、最小/最终间距、最终速度。
- `network_process.log`：SUMO / OMNeT++ 进程输出。
- `omnet/results/network_events.jsonl`：精确无线收发与 lidar_control_accepted 执行者接收记录。

日志同时记录仿真时间与本机单调时间；无线延迟不同于 0.05 秒 ROS 桥接周期。旧日志不代表本轮运行。

从项目根目录执行：

```bash
python3 -m unittest discover -s Algorithm/LidarBrake/tests -v
python3 ros/tests/test_brake_controller.py
python3 ros/tests/test_communication_visuals.py
python3 ros/tests/test_sensor_visuals.py
python3 ros/tests/test_lidar_adapter.py
python3 ros/tests/test_corner_radar.py
# 演示播放时检查真实传感器
python3 ros/tests/check_sensor_run.py
python3 ros/tests/check_corner_radar_run.py
# control_mode:=sumo 的 30 秒三车道对照结束后
python3 ros/tests/check_traffic_run.py
# 默认真实雷达网络模式 30 秒三车道演示结束后
python3 ros/tests/check_lidar_brake_run.py
```
