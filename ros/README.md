# ROS2 Jazzy：接口和状态发布

当前实现 `cosim_bridge` Python 包。`sumo_bridge` 启动并推进 SUMO，读取两车真值，发布 ROS2 消息供 RViz2 使用。尚未实现 V2V 接收和 ROS2 制动控制。

## 构建和运行

从项目根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/runtime"
ros2 launch cosim_bridge demo.launch.py
```

默认打开 RViz2，等待 3 秒后启动 SUMO GUI。车辆运动开始后，仿真 1 秒约等于现实 1 秒，第 5 秒前车急刹，第 15 秒停止推进。最终状态持续发布，窗口保留，按 Ctrl+C 退出。

只开 RViz2，不开 SUMO 窗口：

```bash
ros2 launch cosim_bridge demo.launch.py sumo_gui:=false
```

无界面验证或半速观看：

```bash
ros2 launch cosim_bridge demo.launch.py sumo_gui:=false rviz:=false
ros2 launch cosim_bridge demo.launch.py playback_rate:=0.5
```

从项目根目录或 `ros/` 启动可以自动找到项目资源。从其他目录启动时，传入 `project_root:=/home/maple/Desktop/CoSimDemo`。
SUMO 工具需在 PATH 中；TraCI 加载规则与 `../sumo/scripts/run_demo.py` 相同。

## 阅读顺序

1. `src/cosim_bridge/launch/demo.launch.py`：参数、RViz 配置路径和启动顺序。
2. `src/cosim_bridge/cosim_bridge/sumo_bridge.py` 的 `tick()`：实时定时器、SUMO 步进、前车制动和状态读取。
3. `publish_state()`：里程计、TF、Path、仿真时钟和 Marker 的构造。
4. `../rviz/two_cars.rviz`：Topic 与 RViz 显示项的对应关系。

## Topic

| Topic | 消息类型 | 内容 |
| --- | --- | --- |
| `/car_a/odom`、`/car_b/odom` | `nav_msgs/msg/Odometry` | 车辆中心位置、朝向和车体前进速度 |
| `/car_a/path`、`/car_b/path` | `nav_msgs/msg/Path` | 从开始到当前时刻的轨迹 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `map` 到各车 `base_link` 的变换 |
| `/clock` | `rosgraph_msgs/msg/Clock` | SUMO 仿真时间 |
| `/demo/markers` | `visualization_msgs/msg/MarkerArray` | 红蓝车体、速度文字、道路、虚线和时间状态 |

可在另一个终端 source 相同环境后，用 `ros2 topic list` 和 `ros2 topic echo /car_b/odom --once` 观察数据。

## 时间与坐标

SUMO 内部步长仍为 0.01 秒，节点每 0.05 仿真秒读取一次，默认以 20 Hz 发布。稳态定时器不依赖 `/clock`；RViz 使用 `use_sim_time=true`。机器负载过高时，播放可能比现实时间慢。

`map` 保留 SUMO 平面坐标，单位为米。SUMO 角度从正北顺时针计量，转换为 ROS 从正东逆时针的 yaw。SUMO 位置是车头中心，减去半个车长后作为 ROS 车体中心。`base_link` 的 x 轴沿车头方向，Odometry 的 `twist.linear.x` 为前进速度。

当前后车由 SUMO 自身跟车模型控制。前车制动剧情仍在桥接节点内触发，不属于 ROS2 制动命令闭环。

## 验证与边界

已通过 colcon 构建和实际 Topic 订阅检查；实测急刹约发生在现实 5.05 秒，仿真结束约在现实 15.00 秒。默认 SUMO GUI + RViz2 双窗口启动也已验证：前车急刹约在现实 5.35 秒，结束约在现实 15.80 秒，渲染开销会使播放略慢。最终每车保留 300 个轨迹点。

此节点是当前唯一 SUMO 启动者和步进者。运行它时不要再启动独立 SUMO 演示脚本控制同一仿真。Veins 接入后必须重新明确步进所有权。

参考：[ROS2 Marker 消息](https://docs.ros.org/en/jazzy/p/visualization_msgs/msg/Marker.html)。
