# ROS2-VANET 协同紧急制动学习 Demo

Demo Videos:

https://github.com/user-attachments/assets/d4d78d67-6a68-4f9a-a504-e893fb616150




前车急刹并通过 Veins + INET 无线网络发送警告；后车实际收包后，ROS2 生成制动命令。SUMO 是车辆运动唯一真值源，Gazebo 和 RViz2 同步三维展示。目录按组件划分，优先保持学习时的可读性。

蓝色后车装有可复用传感器：三维激光雷达（水平 120°、垂直 −25°～+15°）和广角 / 标准 / 长焦三目摄像头，均安装在车顶前缘，支架与车顶相接。RViz 六路拼图的前三行保持长焦、标准主摄、广角的顺序，同时绘制传感器模组，并将亮色点云叠在车体与地面；三台前向相机分别发布独立 ROS2 图像和标定。安装、话题与一行引用方式见 [传感器模块说明](gazebo/models/perception_rig/README.md)。默认制动距离反馈来自 Gazebo 原始激光点云，算法独立放在 [Algorithm/LidarBrake](Algorithm/LidarBrake/README.md)；`control_mode:=sumo` 保留旧真值反馈对照。

蓝车另外安装独立的 [四角短距雷达模组](gazebo/models/corner_radar_rig/README.md)，朝向左前 / 右前 / 左后 / 右后，前角已调整为左右各 45°，车前有交汇区，近车侧仍有覆盖缺口。每颗水平 120°、垂直 ±15°、0.3～30 m、20 Hz，独立发布三维点云；Gazebo 与 RViz 显示四个机身，RViz 按四种颜色区分障碍回波。其他车辆一行 merge include 即可复用，桥接与 TF 自动建立。当前使用 GPU 射线近似毫米波距离回波，不模拟射频或多普勒，也未接入制动算法。

| 目录 | 版本 | 功能 |
| --- | --- | --- |
| sumo/ | SUMO 1.22.0 | 道路、车辆和运动计算 |
| veins/ | Veins 5.3.1 + veins_inet | 车辆映射、唯一 TraCI 步进者、V2V 应用 |
| omnet/ | OMNeT++ 6.1 | 网络拓扑、调度、构建与运行 |
| inet/ | INET 4.5 | UDP/IP、802.11p 和无线介质 |
| ros/ | ROS2 Jazzy | 收包事件桥接、制动决策、状态发布 |
| gazebo/ | Harmonic / Sim 8.15.0 | 三维车辆、道路与相机 / 雷达场景采样 |
| rviz/ | ROS2 Jazzy RViz2 | 三维 Marker、TF、轨迹、通信状态、图像与点云 |

后车还安装了独立 [侧后摄像头模组](gazebo/models/surround_camera_rig/README.md)：左、右各一颗 135° 相机，后向一颗 100° 相机。六颗相机的图像和标定各自独立传输，RViz 单个 Image 窗口显示六行一列，从上到下为长焦 / 主摄 / 广角 / 后向 / 左向 / 右向。六路拼图为 `/car_b/camera/surround/image_raw`，原三目拼图话题继续保留。

这些目录保存项目自己的文件，第三方安装不复制到项目中。Veins、INET 和车辆应用运行在同一个 OMNeT++ 进程中。

蓝车前轮后方新增独立的 [双侧短距雷达模组](gazebo/models/side_radar_rig/README.md)，以正侧方向为基准向前覆盖 40°、向后覆盖 80°。现在共有六颗毫米波雷达，两路侧向点云独立传输并在 RViz 显示。

## 构建与启动

只查看传感器时，可独立启动 Gazebo + RViz，无需 SUMO、Veins、INET 或 OMNeT++ 进程。ROS2 构建完成后执行：

```bash
source /opt/ros/jazzy/setup.bash
source ros/install/setup.bash
ros2 launch cosim_bridge sensor_preview.launch.py
```

车辆停在 Gazebo 世界的初始位置，Gazebo 自行推进时间，六路相机、前向激光雷达和四角雷达持续采样；RViz 显示静态车体、道路、传感器、TF、图像和点云。此入口不运行交通或制动控制，不等待 SUMO Play。参数与边界见 [ROS2 独立预览说明](ros/README.md)。完整交通演示仍使用下面的 `demo.launch.py`；切换入口前先 Ctrl+C 退出上一组，两种模式不能在同一 ROS 域 / Gazebo 分区同时运行。目前不支持在预览运行中直接启动 SUMO 接管车辆。

在项目根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/runtime"
ros2 launch cosim_bridge demo.launch.py
```

默认启用无线闭环。Gazebo 和 RViz2 先打开，8 秒后打开 SUMO；点击 SUMO 的绿色 Play 才开始运动。默认约 1 秒现实时间对应 1 秒仿真，5 秒急刹，30 秒结束并保留画面，Ctrl+C 退出。启动等待不计入演示，高负载时播放可能变慢。三维窗口可自由旋转、平移、缩放；不要单独点击 Gazebo Play 推进时间。

首次运行自动构建网络应用，需要 clang++、nlohmann/json.hpp，以及已编译的 OMNeT++、Veins、INET、veins_inet release 库。安装路径与覆盖方法见 [OMNeT++ 说明](omnet/README.md)。SUMO、netconvert 需在 PATH 中。

```bash
# 无 SUMO 窗口，自动开始，保留三维窗口
ros2 launch cosim_bridge demo.launch.py sumo_gui:=false

# 可选 Qtenv 窗口；必要时先在 Qtenv 点击 Run，再在 SUMO 点击 Play
ros2 launch cosim_bridge demo.launch.py network_gui:=true

# 半速，30 秒仿真约需 60 秒
ros2 launch cosim_bridge demo.launch.py playback_rate:=0.5

# 原有人工警告闭环
ros2 launch cosim_bridge demo.launch.py network:=false

# 普通安全跟车基线
ros2 launch cosim_bridge demo.launch.py network:=false test_warning:=false
```

运行前关闭上一组演示，避免端口冲突和多个 /clock。从项目根目录或 ros/ 启动可自动定位资源，其他目录需指定 project_root。

## 当前数据流

```text
SUMO ↔ TraCI ↔ Veins（唯一 SUMO 步进者）
                 ↓ 前车应用
          INET UDP/IP + 802.11p
                 ↓ 后车实际收包
          JSONL/TCP → ROS2 sumo_bridge
                 ↓ /v2v_warning
          brake_controller
                 ↓ /brake_cmd
          JSONL/TCP → Veins → TraCI → SUMO 后车

SUMO 状态 → ROS2 /clock、Odom、TF、Path、Marker → Gazebo / RViz2
```

收包桥接目前放在 network_backend.py，没有单独的 v2v_receiver 节点。网络模式下 ROS2 不调用 simulationStep。TCP 9998 传状态与控制，TraCI 9999 连接 SUMO；TCP 不是车间无线链路。Gazebo 从暂停状态按 SUMO 时间定步展示，不计算另一套动力学或发布另一个 ROS2 时钟。

## 验证结果与边界

RViz 的 `V2V communication` 显示实际收发时间、无线延迟、制动命令发布时间和执行时间，并用绿、橙、红区分就绪、收包与异常。另有两车范围参照圆、TX/RX 位置和 1.5 秒数据包回放，范围和动画均明确标注为示意。配置与说明见 [RViz2 README](rviz/README.md)。

此前八车配置实测：5.000 秒急刹，5.001 秒发包，5.001152 秒收包，5.05 秒执行 ROS2 制动。无线仿真延迟约 0.152 ms，ROS2 桥接周期为 0.05 秒。当前场景扩展为三车道十一车，A/B 保留原有制动策略；九辆背景车以 6～8.5 m/s 行驶。Gazebo 按全部车辆的 ROS2 状态同步展示。

极低发射功率对照没有收包，也没有执行 ROS2 制动；SUMO 自带安全跟车仍会减速，不能只凭停车判断通信生效。停车策略让后车收拢到约 2.5 m，服务于学习演示，不作为道路安全算法验证。

默认场景 `scenario:=traffic` 为 400 m 同向三车道直路、11 辆车、30 秒。A/B 在中间车道演示协同制动；左侧六辆、右侧三辆灰色背景车以 6～8.5 m/s 行驶，暂不主动换道。停车点约在 187 m，后方保留约 210 m 道路；背景车在 30 秒内不会驶出道路。`scenario:=two_cars duration:=15.0` 可回到旧两车场景。Qtenv 演示请用 F6 Fast Run，避免包动画暂停整个联合时间。日志见 omnet/results/network_events.jsonl、OMNeT++ .sca/.vec 和 ros/log/events/summary.json，生成结果不纳入 Git。

```bash
# 默认三车道十一车场景，30 秒
ros2 launch cosim_bridge demo.launch.py

# 旧两车场景，15 秒
ros2 launch cosim_bridge demo.launch.py scenario:=two_cars duration:=15.0
```

全部车辆按 ID 发布 Odom、Path、TF 和三维 Marker；轨迹另存于 `ros/log/events/traffic_trajectory.csv`。背景车也映射为 INET 网络车辆，但不会执行 A/B 制动目标。

## 模块入口

- [ROS2 节点、接口与日志](ros/README.md)
- [Veins 应用与同步](veins/README.md)
- [INET 无线配置](inet/README.md)
- [OMNeT++ 构建与运行](omnet/README.md)
- [SUMO 独立基线](sumo/README2.md)
- [Gazebo 模型与视角](gazebo/README.md)
- [RViz2 显示功能与视角](rviz/README.md)

## License

除明确标注的文件和第三方材料外，本项目原创部分采用 [Apache License 2.0](LICENSE)。Veins 网络适配文件采用 GPL-3.0-or-later。许可范围、第三方组件及其原始许可文本见 [lisence/THIRD_PARTY_NOTICES.md](lisence/THIRD_PARTY_NOTICES.md)。

## Gazebo 中删除车辆

两种入口均支持在 Gazebo 删除已有车辆：独立预览会同步撤销 RViz 模型；完整演示会同时删除 SUMO / Veins 中的对应车辆、清空轨迹并继续运行。删除前车 `car_a` 或后车 `car_b` 会结束本次制动演示。删除只影响当前运行，重启恢复原场景。

蓝车新增可复用 [前后与侧面超声波模组](gazebo/models/ultrasonic_rig/README.md)：前后保险杠各四颗、左右侧面各四颗，共 16 颗，0.15～5 m 近距探测，RViz 使用亮黄色障碍回波和白色地面回波；十六路测距与点云独立发布，未接入制动控制。

RViz 新增 [雷达地面回波面板](rviz/cosim_rviz/README.md)，一键显示 / 隐藏所有雷达的地面回波，保留障碍物显示；只影响显示点云。首次需在 ROS 工作空间构建并重新 source，两个运行模式均可使用。
