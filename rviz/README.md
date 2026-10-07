# RViz2：ROS2 状态可视化

RViz2 订阅 ROS2 消息，显示车辆位置、速度、坐标系和运动轨迹。它负责观察数据，不计算车辆动力学。

## 当前显示功能

配置文件：[two_cars.rviz](two_cars.rviz)。

| 显示项 | 数据来源 | 显示内容 |
| --- | --- | --- |
| 车体与状态 | `/demo/markers` | 红色前车 A、蓝色后车 B，速度及行驶、跟车、制动或近似停车文字 |
| 道路参照 | `/demo/markers` | 道路、白色虚线及仿真时间 |
| 前车轨迹 | `/car_a/path` | 红色行驶轨迹 |
| 后车轨迹 | `/car_b/path` | 蓝色行驶轨迹 |
| 坐标系 | `/tf` | `map` 到 `car_a/base_link` 和 `car_b/base_link` 的变换 |
| 仿真时钟 | `/clock` | 为 RViz2 提供统一的仿真时间 |

固定坐标系为 `map`，启用 `use_sim_time=true`。车辆由三维车身、座舱、车窗、四个车轮和前后灯的 Marker 组成，每辆车 13 个部件，轨迹随运动增长。仿真结束后，数据发布节点继续发布最终状态，方便观察停车位置。

当前状态文字反映 SUMO 跟车基线；V2V 警告和通信线尚未接入。

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
ros2 launch cosim_bridge demo.launch.py sumo_gui:=false gazebo:=false
```

默认以实时速度播放 15 秒，第 5 秒前车急刹，结束后保留最终画面。按 Ctrl+C 退出；滚轮可缩放视角。

## 单独打开显示配置

桥接节点已运行时，可从项目根目录执行：

```bash
rviz2 -d rviz/two_cars.rviz --ros-args -p use_sim_time:=true
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

三个窗口准备完成后，SUMO 默认保持等待，请点击 SUMO 工具栏的绿色“开始 / Play”按钮。点击开始后才推进演示：约 5 秒时前车急刹，15 秒时结束。启动等待时间不计入车辆演示。

RViz 使用 SUMO 发布的 `/clock`。Gazebo 从暂停状态开始，由 `gazebo_sync` 通过 `/world/cosim_demo/control` 按 SUMO 时间定步推进；结束后停在 15 秒，继续保留画面。姿态同步和视角操作不改变车辆运动真值。请用 SUMO 的开始按钮启动，不要单独点击 Gazebo 的播放按钮，以免它自行推进展示时间。

`startup_delay` 只控制 SUMO 窗口何时打开，不再自动开始行驶。`sumo_gui:=false` 时没有手动按钮，仍自动实时运行。
