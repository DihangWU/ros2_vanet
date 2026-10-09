# RViz2：ROS2 状态可视化

RViz2 订阅 ROS2 消息，显示车辆位置、速度、坐标系和运动轨迹。它负责观察数据，不计算车辆动力学。

## 当前显示功能

增加四个独立 `Corner radar` 点云显示，分别订阅 `/car_b/radar/{front_left,front_right,rear_left,rear_right}/points_colored`。左前橙色、右前粉色、左后紫色、右后蓝青色；地面回波统一亮绿色。四颗水平视场均为 120°（左右各 60°），垂直上下各 15°；前角朝左右前方各 45°，在车前中心线距前保险杠约 3.67 m 后形成水平交汇区；后角朝左右后方各 135°，近车侧 / 车尾仍有盲区。四路都是三维场景的真实射线回波，每帧替换，不累积；可分别开关。角雷达机身通过 `/demo/markers` 的 `car_b/radars` 命名空间显示，安装 TF 和机身几何均读取独立 SDF。此处展示的是毫米波距离回波的射线近似，不是射频或多普勒仿真。复用与参数见 [四角雷达模块](../gazebo/models/corner_radar_rig/README.md)。

`Six cameras - front and surround` 在单个 `Image` 窗口显示六颗摄像头，订阅 `/car_b/camera/surround/image_raw`（1280×1176）。两列三行：左列从上到下为长焦、标准主摄、广角；右列从上到下为后向、左向、右向。六路原图 `/car_b/camera/{tele,standard,wide,rear,left,right}/image_raw` 和各自 CameraInfo 均独立传输；拼图仅拼接相同时间戳的六帧，缺帧时跳过该组。原三目 `camera/triple/image_raw` 仍保留，可以手动切换 Image 话题查看。两套 RViz 配置都只设置一个摄像头 Image 显示，可拖动、停靠并拉大查看。侧后相机安装与复用见 [独立侧后模组](../gazebo/models/surround_camera_rig/README.md)。

左右侧相机均向下俯视 10°，六路拼图右列第二、三行能看到更多路面；安装 TF 同步读取 SDF 的完整姿态。

后向相机水平视场为 140°，比原先 120° 更广角，车尾近处可见范围更大；右列第一行标题和独立 CameraInfo 随 SDF 自动更新。

长焦水平视场为 20°，预览标题从 SDF 自动读取；相比之前 15°，同距离目标显示尺寸约缩小四分之一，CameraInfo 同步反映新内参。

车顶传感器模组也通过 `/demo/markers` 绘制，命名空间为 `car_b/sensors`：底板、雷达支柱与机身、摄像头支柱与横臂、机身和三个镜头。几何直接读取 Gazebo 的独立 SDF，随车移动，不需要另外维护 RViz 安装位置。通信提示箭头位于 z=3.6 m，避开车顶模组。

`Roof lidar - bright surface returns` 订阅 `/car_b/lidar/points_colored`，用 4 像素亮点叠在车辆、地面及可见障碍物表面。亮青色表示高于地面 0.15 m 的回波，亮绿色表示地面回波；这是高度着色，不是检测算法的分类结果。点云依靠 `map → car_b/base_link → car_b/lidar_link` 的 TF 对齐；未命中点不显示。每帧替换上一帧，不累积旧点。传感器细节见 [独立传感器配置](../gazebo/models/perception_rig/README.md)。

默认十一车配置：[traffic.rviz](traffic.rviz)。旧两车配置：[two_cars.rviz](two_cars.rviz)，通过 `scenario:=two_cars duration:=15.0` 使用。多车场景默认 30 秒。

默认显示三车道：中间红色 A、蓝色 B，左侧六辆、右侧三辆灰色背景车，所有车辆均有 Odom、Path 和 TF。背景车头顶标签默认以透明度 0 隐藏，标签内容与生成代码保留；启动加 `show_background_labels:=true` 恢复显示，内容为 `BACKGROUND`。运行时也可执行 `ros2 param set /sumo_bridge show_background_labels true` 显示，或设为 `false` 隐藏；A/B 标签保持显示。道路显示两排间断车道线。通信显示仍表示 A → B 的警告闭环。道路长度读取 SUMO 节点定义，当前为 400 m；多车网格中心为 x=200、y=-4.8，覆盖整条道路；相机保持自由操作。

| 显示项 | 数据来源 | 显示内容 |
| --- | --- | --- |
| 车体与状态 | `/demo/markers` | 红色 A、蓝色 B、九辆灰色背景车及速度状态 |
| 道路参照 | `/demo/markers` | 道路、白色虚线及仿真时间 |
| 前车轨迹 | `/car_a/path` | 红色行驶轨迹 |
| 后车轨迹 | `/car_b/path` | 蓝色行驶轨迹 |
| 背景轨迹 | `/bg_01/path`～`/bg_09/path` | 左右两侧灰色背景车行驶轨迹 |
| 坐标系 | `/tf` | `map` 到全部 `<id>/base_link` 的变换 |
| 仿真时钟 | `/clock` | 为 RViz2 提供统一的仿真时间 |
| 通信信息 | `/v2v/markers` | 收发时间、无线延迟、车辆 ID、命令时间、范围圆、收发位置与包回放 |
| 雷达控制 | `/lidar_brake/markers` | 测量车距、目标车距、相对速度、加速度和控制状态 |

固定坐标系为 `map`，启用 `use_sim_time=true`。车辆由三维车身、座舱、车窗、四个车轮和前后灯的 Marker 组成，每辆车有 13 个基础部件；蓝车另外显示 9 个车顶模组部件、4 个角雷达机身，以及 6 个侧后相机机身 / 镜头部件（`car_b/surround_cameras` 命名空间），轨迹随运动增长。仿真结束后，数据发布节点继续发布最终状态，方便观察停车位置。

`Lidar distance and brake control` 显示后车旁的青色三维文字：`MEASURED GAP` 是点云测得的净间距，`TARGET GAP` 随自车速度变化，`REL SPEED` 是连续测距估计的相对速度，负值表示接近。状态包括 `LIDAR_FOLLOW`、`V2V_LIDAR_BRAKE`、`EMERGENCY_BRAKE`、`SENSOR_LOST_BRAKE` 和 `STOPPED_HOLD`。该显示可单独关闭；旧 `control_mode:=sumo` 不运行雷达控制节点，因此没有这组文字。通信 Marker 的命令时间对应单次 V2V 事件，连续雷达速度命令另存日志。

网格每格 10 m，每边 100 格，覆盖以 x=200 m 为中心约 1000 m × 1000 m 的区域，包含车辆全程运动、最终停车位置及通信范围参照圆。

支持通信连线和 ROS2 制动状态。默认来自 INET 后车实际收包，显示为 `WARNING RECEIVED`；人工测试模式显示 `TEST WARNING`。

## 启动

按 [ROS2 说明](../ros/README.md) 完成构建后，从项目根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
source ros/install/setup.bash
export ROS_LOG_DIR="$PWD/ros/log/runtime"
ros2 launch cosim_bridge demo.launch.py
```

该命令同时启动 Gazebo、RViz2 和 SUMO。只打开 RViz2、让 SUMO 在后台运行：

```bash
ros2 launch cosim_bridge demo.launch.py sumo_gui:=false gazebo:=false control_mode:=sumo
```

默认以实时速度播放 30 秒，第 5 秒前车急刹，结束后保留最终画面。按 Ctrl+C 退出；滚轮可缩放视角。

## 单独打开显示配置

桥接节点已运行时，可从项目根目录执行：

```bash
rviz2 -d rviz/traffic.rviz --ros-args -p use_sim_time:=true
```

仅打开 RViz2 不会启动车辆或生成数据，需要 `sumo_bridge` 发布对应 Topic。

## 三维自由视角

默认使用倾斜的 `Orbit` 三维视角，固定参考系为 `map`。选中工具栏的 `Move Camera`，在显示区域操作：

| 操作 | 功能 |
| --- | --- |
| 左键拖动 | 围绕焦点旋转 |
| 中键拖动，或 Shift + 左键拖动 | 平移焦点 |
| 滚轮，或右键拖动 | 缩放 |

运行时不会重置相机，行驶中和停车后都能调整。参考 [RViz 官方视角说明](https://github.com/ros2/rviz/blob/rolling/rviz2/doc/index.rst)。

## 手动开始与时间同步

三个窗口准备完成后，SUMO 默认保持等待，请点击 SUMO 工具栏的绿色“开始 / Play”按钮。点击开始后才推进演示：约 5 秒时前车急刹，30 秒时结束。启动等待时间不计入车辆演示。

RViz 使用 SUMO 发布的 `/clock`。Gazebo 从暂停状态开始，由 `gazebo_sync` 通过 `/world/cosim_demo/control` 按 SUMO 时间定步推进；结束后停在 30 秒，继续保留画面。姿态同步和视角操作不改变车辆运动真值。请用 SUMO 的开始按钮启动，不要单独点击 Gazebo 的播放按钮，以免它自行推进展示时间。

`startup_delay` 只控制 SUMO 窗口何时打开，不再自动开始行驶。`sumo_gui:=false` 时没有手动按钮，仍自动实时运行。

## 警告与后车制动

正常时显示绿色参照连线。收到警告后连线变成橙色，显示 `WARNING RECEIVED`；后车执行 ROS2 命令后，速度标签显示 `ROS BRAKING`，近似停车后显示 `STOPPED`。收包超时或进程断连显示红色。网络传播在 INET 中计算，慢放小球用于观察收发过程。

车辆与道路通过 `/demo/markers` 显示，通信信息通过 `/v2v/markers` 显示，无需新增 RViz 插件。Displays 中的 `V2V communication` 可以独立开关。`network:=false` 切换人工警告；再加 `test_warning:=false` 查看普通跟车基线。命令和时间记录见 [ROS2 README](../ros/README.md)。

## 通信信息与颜色

多行 Marker 文本显示 `V2V STATUS`、`PACKET`、`SENT`、`RECEIVED`、`NETWORK DELAY`、`SOURCE`、`TARGET`、`ROS CMD PUBLISHED` 和 `SUMO CMD APPLIED`。时间使用仿真秒，延迟使用毫秒；没有发生的事件显示 `--`。文本位于道路旁，是可随视角观察的三维文字。

绿色 `CONNECTED (reception unverified)` 表示仿真桥接正在交换状态，尚未验证车辆间收包；发送后为 `WAITING FOR PACKET`。橙色 `WARNING RECEIVED` 表示后车已收到警告。发包后超过 0.5 仿真秒仍未收包，显示红色 `NOT RECEIVED`；网络进程断开时显示红色 `DISCONNECTED` 并保留最后状态。正常 30 秒结束不标记为断连。当前只发送一次警告，不能据此监测持续通信质量。

两车各有一个范围参照圆，默认半径 100 m，可用 `communication_range_m:=50.0` 修改。圆标为 `illustrative`，仅表示几何距离，不是 INET 保证接收的半径，也不修改无线参数。

`TX position` 和 `RX position` 标记发包与收包时的车辆中心位置，保留用于复盘。位置来自最近一次 SUMO 状态，精度受 0.05 秒同步周期限制；网络时间保留 OMNeT++ 精度。收包后，小球从发送位置到接收位置播放 1.5 秒动画，标为 `PACKET REPLAY`，只用于人眼观察，不代表实际传播时长；结束后自动移除，不改变相机或仿真时间。

人工测试模式明确显示 `TEST INPUT`，无线延迟为 `--`，不把人工延迟当成网络测量。

后车默认采用雷达测距的时间车距 PD 与停车保持；`control_mode:=sumo` 可切换旧真值反馈对照。`ROS BRAKING` 表示已执行 ROS2 停车目标，减速程度随实时距离调整，不意味着收到警告时立即停车。
