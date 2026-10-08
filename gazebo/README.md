# Gazebo Harmonic：三维车辆展示

当前使用本机 Gazebo Sim 8.15.0（Harmonic）。默认展示三车道直路、两排白色间断车道线、红蓝 A/B 和九辆灰色背景小车。A/B 位于中间车道，左侧六辆背景车，右侧三辆背景车。小车包含车身、车窗、四个车轮和前后灯，全部使用本地几何，无需在线下载模型。

## 文件

默认 `scenario:=traffic` 使用 `worlds/traffic.sdf`：400 m 三车道、11 辆车，道路宽度 9.6 m；默认运行 30 秒。道路长度读取 SUMO 的 `network/traffic.nod.xml`，停车点后方保留约 210 m 道路。`config/traffic_gui.config` 提供较宽的初始视角。旧资源 `worlds/two_cars.sdf` 通过 `scenario:=two_cars duration:=15.0` 使用。

灰车复用 `models/background_car/`。`gazebo_sync` 按 SUMO 路线文件中的全部车辆 ID 订阅 Odom。新增或修改初始车辆后，在项目根目录执行 `python3 gazebo/scripts/build_traffic_world.py` 重新生成世界。目前模型在启动前创建，不实现运行时新增或移除。

| 文件 | 功能 |
| --- | --- |
| `worlds/two_cars.sdf` | 道路、地面、灯光和两辆车的初始位置 |
| `worlds/traffic.sdf` | 默认三车道十一车世界，由脚本生成 |
| `scripts/build_traffic_world.py` | 根据 SUMO 路线定义生成默认世界 |
| `models/background_car/model.sdf` | 背景交通共用的灰色车模型 |
| `models/car_a/model.sdf` | 红色前车的三维几何 |
| `models/car_b/model.sdf` | 蓝色后车的三维几何 |
| `config/gui.config` | 初始相机、鼠标视角控制和 GUI 插件 |

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
不打开 Gazebo：加 `gazebo:=false`。

## 自由视角

在三维场景区域操作：

| 操作 | 功能 |
| --- | --- |
| 左键拖动 | 平移 |
| 按住滚轮拖动，或 Shift + 左键拖动 | 旋转 |
| 滚轮，或右键拖动 | 缩放 |

相机只在启动时设置，没有自动跟车或周期性重置。行驶中和停车后均可调整视角。操作参考 [Gazebo Harmonic GUI 文档](https://gazebosim.org/docs/harmonic/gui/)。

## 同步原理与边界

`ros/` 中的 `gazebo_sync` 从当前 SUMO 路线读取车辆 ID，订阅全部 `/<id>/odom`，通过 `ros_gz_bridge` 的 `/world/cosim_demo/set_pose` 服务更新模型中心姿态。每辆车最多一个在途请求，优先同步最新状态。

SUMO 是车辆运动唯一真值源。模型设为静态、无碰撞体，用姿态更新实现展示，不模拟轮胎、转向或另一套跟车动力学。车轮目前不滚动。Gazebo 的时间按 SUMO 定步同步，但不桥接到 ROS2 `/clock`；ROS2 时间来自 SUMO。

已验证模型加载、姿态服务成功、30 秒最终位置与 ROS2 一致（检查误差小于 0.02 m）。自由视角插件已加载，鼠标操作尚未自动化验证。

## 手动开始与时间同步

三个窗口准备完成后，SUMO 默认保持等待，请点击 SUMO 工具栏的绿色“开始 / Play”按钮。点击开始后才推进演示：约 5 秒时前车急刹，30 秒时结束。启动等待时间不计入车辆演示。

RViz 使用 SUMO 发布的 `/clock`。Gazebo 从暂停状态开始，由 `gazebo_sync` 通过 `/world/cosim_demo/control` 按 SUMO 时间定步推进；结束后停在 30 秒，继续保留画面。姿态同步和视角操作不改变车辆运动真值。请用 SUMO 的开始按钮启动，不要单独点击 Gazebo 的播放按钮，以免它自行推进展示时间。

`startup_delay` 只控制 SUMO 窗口何时打开，不再自动开始行驶。`sumo_gui:=false` 时没有手动按钮，仍自动实时运行。

## ROS2 制动闭环

默认运行已接入 Veins + INET 实际收包与 ROS2 制动控制：前车急刹后发送无线警告，后车收包后由控制器生成停车目标，经 Veins 执行到 SUMO。Gazebo 继续订阅同一份车辆状态，不直接执行制动。事件和日志见 [ROS2 README](../ros/README.md)。`network:=false` 切换人工警告，再加 `test_warning:=false` 运行普通跟车对照场景。

当前后车按目标净间距 2.5 m 靠近停车，实测最终两车停止、净间距约 2.55 m；模型继续同步 SUMO 状态。
