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

固定坐标系为 `map`，启用 `use_sim_time=true`。车辆使用 Marker 方块表示，轨迹随运动增长。仿真结束后，数据发布节点继续发布最终状态，方便观察停车位置。

当前状态文字反映 SUMO 跟车基线；V2V 警告和通信线尚未接入。

## 启动

按 [ROS2 说明](../ros/README.md) 完成构建后，从项目根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
source ros/install/setup.bash
export ROS_LOG_DIR="$PWD/ros/log/runtime"
ros2 launch cosim_bridge demo.launch.py
```

该命令同时启动 RViz2 和 SUMO。只打开 RViz2、让 SUMO 在后台运行：

```bash
ros2 launch cosim_bridge demo.launch.py sumo_gui:=false
```

默认以实时速度播放 15 秒，第 5 秒前车急刹，结束后保留最终画面。按 Ctrl+C 退出；滚轮可缩放视角。

## 单独打开显示配置

桥接节点已运行时，可从项目根目录执行：

```bash
rviz2 -d rviz/two_cars.rviz --ros-args -p use_sim_time:=true
```

仅打开 RViz2 不会启动车辆或生成数据，需要 `sumo_bridge` 发布对应 Topic。
