# Gazebo Harmonic：三维车辆展示

当前使用本机 Gazebo Sim 8.15.0（Harmonic）。默认展示三车道直路、两排白色间断车道线、红蓝 A/B 和九辆灰色背景小车。A/B 位于中间车道，左侧六辆背景车，右侧三辆背景车。小车包含车身、车窗、四个车轮和前后灯，全部使用本地几何，无需在线下载模型。

## 文件

蓝色后车另外引用独立的 [四角短距雷达模组](models/corner_radar_rig/README.md)：四个保险杠角部机身、四路三维回波，水平 180°、垂直 ±15°、0.3～30 m、20 Hz。前角朝侧方，后角朝斜后方，覆盖侧面与后方；采用 GPU 射线作为毫米波雷达距离回波的近似，不包含射频、多普勒或多径模型。其他车辆可以一行 merge include 复用。

车顶模组含与车顶相接的底板、雷达支柱、摄像头支柱及横臂；传感器机身不再悬空。RViz 从相同的独立 SDF 读取这些几何和安装位置，保持两边外观一致。三目预览从上到下为长焦、标准主摄、广角。

蓝色后车在车顶前缘装有三维 GPU 激光雷达和三目摄像头，光心均高于车顶。独立配置与一行安装方法见 [perception_rig/README.md](models/perception_rig/README.md)。雷达水平左右各 60°（总 120°）、垂直 −25°～+15°；广角、标准、长焦水平视场为 120°、60°、15°。长焦远裁剪为 300 m，其余相机为 150 m。世界增加 Sensors / Ogre2 插件，从实际渲染场景生成图像和点云。SUMO 仍负责运动，Gazebo 生成制动算法的原始点云输入；制动判断由独立算法和 ROS2 节点完成。

默认 `scenario:=traffic` 使用 `worlds/traffic.sdf`：400 m 三车道、11 辆车，道路宽度 9.6 m；默认运行 30 秒。道路长度读取 SUMO 的 `network/traffic.nod.xml`，停车点后方保留约 210 m 道路。`config/traffic_gui.config` 提供较宽的初始视角。旧资源 `worlds/two_cars.sdf` 通过 `scenario:=two_cars duration:=15.0` 使用。

世界生成器优先使用 `models/<车辆ID>/model.sdf`，不存在时复用灰车 `models/background_car/`；传感器启动扫描采用同一规则。`gazebo_sync` 按 SUMO 路线文件中的全部车辆 ID 订阅 Odom。新增或修改初始车辆、增加独立车辆模型后，在项目根目录执行 `python3 gazebo/scripts/build_traffic_world.py` 重新生成世界。目前模型在启动前创建，不实现运行时新增或移除。

| 文件 | 功能 |
| --- | --- |
| `worlds/two_cars.sdf` | 道路、地面、灯光和两辆车的初始位置 |
| `worlds/traffic.sdf` | 默认三车道十一车世界，由脚本生成 |
| `scripts/build_traffic_world.py` | 根据 SUMO 路线定义生成默认世界 |
| `models/background_car/model.sdf` | 背景交通共用的灰色车模型 |
| `models/perception_rig/model.sdf` | 独立雷达与三目配置，车辆通过 merge include 复用 |
| `models/corner_radar_rig/model.sdf` | 独立四角短距雷达近似，四路三维点云与彩色机身 |
| `models/car_a/model.sdf` | 红色前车的三维几何 |
| `models/car_b/model.sdf` | 蓝色后车的三维几何 |
| `config/gui.config` | 初始相机、鼠标视角控制和 GUI 插件 |
| `plugins/SensorRenderSync.cc` | 订阅渲染事件，确保外部定步与暂停帧的传感器场景及时刷新 |
| `scripts/build_sensor_sync.py` | 使用本机 Harmonic SDK 构建适配器，输出到忽略的 `build/` |

## 启动

在项目根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/runtime"
ros2 launch cosim_bridge demo.launch.py
```

默认打开 Gazebo、RViz2 和 SUMO；等待 8 秒后打开 SUMO 窗口，点击 SUMO 的开始按钮后才推进交通仿真。运动默认实时播放 30 秒，第 5 秒前车急刹，结束后保留车辆和窗口，Ctrl+C 退出。若窗口启动较慢，可以加 `startup_delay:=15.0`。

不打开 SUMO 窗口：`ros2 launch cosim_bridge demo.launch.py sumo_gui:=false`。
不打开 Gazebo：加 `gazebo:=false control_mode:=sumo`，使用旧真值反馈对照。雷达模式必须开启 Gazebo。

## 自由视角

在三维场景区域操作：

| 操作 | 功能 |
| --- | --- |
| 左键拖动 | 平移 |
| 按住滚轮拖动，或 Shift + 左键拖动 | 旋转 |
| 滚轮，或右键拖动 | 缩放 |

相机只在启动时设置，没有自动跟车或周期性重置。行驶中和停车后均可调整视角。操作参考 [Gazebo Harmonic GUI 文档](https://gazebosim.org/docs/harmonic/gui/)。

## 同步原理与边界

`ros/` 中的 `gazebo_sync` 从当前 SUMO 路线读取车辆 ID，订阅全部 `/<id>/odom`，使用 Gazebo Transport 的 `/world/cosim_demo/set_pose_vector` 一次提交完整帧，再请求 `/world/cosim_demo/control` 推进。通过 ROS 桥接的 `/gazebo/clock` 确认世界实际到达目标时间，经 `/gazebo/synced` 通知交通步进者。Gazebo Python 绑定由 `gz_transport_vendor`、`gz_msgs_vendor` 提供；时钟只桥接到独立话题，不覆盖 ROS `/clock`。

SUMO 是车辆运动唯一真值源。模型设为静态、无碰撞体，用姿态更新实现展示，不模拟轮胎、转向或另一套跟车动力学。车轮目前不滚动。Gazebo 的时间按 SUMO 定步同步，但不桥接到 ROS2 `/clock`；ROS2 时间来自 SUMO。

世界的 `real_time_factor=0` 允许尽快计算请求的展示步，不另加现实时间等待；`run_to_sim_time` 到目标时间后自动暂停。现实倍速由 ROS2 交通协调器统一控制。`SensorRenderSync` 只保持渲染事件连接，让暂停与外部步进期间的场景也及时刷新，雷达与相机仍遵循自己的 update_rate。启动时自动构建，需 CMake、C++ 编译器和本机 Harmonic SDK；不修改第三方安装。可靠的原始点云 ROS 通道用于制动，彩色点云仍用于显示。

已验证模型加载、姿态服务成功、30 秒最终位置与 ROS2 一致（检查误差小于 0.02 m）。自由视角插件已加载，鼠标操作尚未自动化验证。

## 手动开始与时间同步

三个窗口准备完成后，SUMO 默认保持等待，请点击 SUMO 工具栏的绿色“开始 / Play”按钮。点击开始后才推进演示：约 5 秒时前车急刹，30 秒时结束。启动等待时间不计入车辆演示。

RViz 使用 SUMO 发布的 `/clock`。Gazebo 从暂停状态开始，由 `gazebo_sync` 通过 `/world/cosim_demo/control` 按 SUMO 时间定步推进；结束后停在 30 秒，继续保留画面。姿态同步和视角操作不改变车辆运动真值。请用 SUMO 的开始按钮启动，不要单独点击 Gazebo 的播放按钮，以免它自行推进展示时间。

`startup_delay` 只控制 SUMO 窗口何时打开，不再自动开始行驶。`sumo_gui:=false` 时没有手动按钮，仍自动实时运行。

## ROS2 制动闭环

默认运行已接入 Veins + INET 实际收包与 ROS2 制动控制：前车急刹后发送无线警告，后车收包后由控制器生成停车目标，经 Veins 执行到 SUMO。Gazebo 继续订阅同一份车辆状态，不直接执行制动。事件和日志见 [ROS2 README](../ros/README.md)。`network:=false` 切换人工警告，再加 `test_warning:=false` 运行普通跟车对照场景。

默认 `control_mode:=lidar` 从 Gazebo 雷达测距控制靠近停车，参数见 [LidarBrake](../Algorithm/LidarBrake/README.md)。`control_mode:=sumo` 保留旧真值反馈策略。模型继续同步 SUMO 状态。
