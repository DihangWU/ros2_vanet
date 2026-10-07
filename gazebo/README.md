# Gazebo Harmonic：三维车辆展示

当前使用本机 Gazebo Sim 8.15.0（Harmonic）。展示直路、固定白色虚线和红蓝两辆三维小车。小车包含车身、车窗、四个车轮和前后灯，全部使用本地几何，无需在线下载模型。

## 文件

| 文件 | 功能 |
| --- | --- |
| `worlds/two_cars.sdf` | 道路、地面、灯光和两辆车的初始位置 |
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

默认打开 Gazebo、RViz2 和 SUMO；等待 8 秒后打开 SUMO 窗口，点击 SUMO 的开始按钮后才推进交通仿真。运动默认实时播放 15 秒，第 5 秒前车急刹，结束后保留车辆和窗口，Ctrl+C 退出。若窗口启动较慢，可以加 `startup_delay:=15.0`。

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

`ros/` 中的 `gazebo_sync` 订阅 `/car_a/odom` 和 `/car_b/odom`，通过 `ros_gz_bridge` 的 `/world/cosim_demo/set_pose` 服务更新模型中心姿态。每辆车最多一个在途请求，优先同步最新状态。

SUMO 是车辆运动唯一真值源。模型设为静态、无碰撞体，用姿态更新实现展示，不模拟轮胎、转向或另一套跟车动力学。车轮目前不滚动。Gazebo 的时间按 SUMO 定步同步，但不桥接到 ROS2 `/clock`；ROS2 时间来自 SUMO。

已验证模型加载、姿态服务成功、15 秒最终位置与 ROS2 一致（检查误差小于 0.02 m）。自由视角插件已加载，鼠标操作尚未自动化验证。

## 手动开始与时间同步

三个窗口准备完成后，SUMO 默认保持等待，请点击 SUMO 工具栏的绿色“开始 / Play”按钮。点击开始后才推进演示：约 5 秒时前车急刹，15 秒时结束。启动等待时间不计入车辆演示。

RViz 使用 SUMO 发布的 `/clock`。Gazebo 从暂停状态开始，由 `gazebo_sync` 通过 `/world/cosim_demo/control` 按 SUMO 时间定步推进；结束后停在 15 秒，继续保留画面。姿态同步和视角操作不改变车辆运动真值。请用 SUMO 的开始按钮启动，不要单独点击 Gazebo 的播放按钮，以免它自行推进展示时间。

`startup_delay` 只控制 SUMO 窗口何时打开，不再自动开始行驶。`sumo_gui:=false` 时没有手动按钮，仍自动实时运行。
