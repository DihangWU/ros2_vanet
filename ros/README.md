# ROS2 Jazzy：接口和控制层

本目录作为 ROS2 工作空间，后续包放在 `src/`。`build/`、`install/` 和 `log/` 是构建产物。

计划节点：

| 节点 | 职责 |
| --- | --- |
| `v2v_receiver` | 接收网络仿真导出的后车收包事件，发布 `/v2v_warning` |
| `brake_controller` | 根据警告发布 `/brake_cmd` |
| `sumo_bridge` | 读取车辆状态并将制动请求交给 SUMO，遵守唯一仿真步进者约定 |
| `gazebo_sync` | 将车辆状态转换为 Gazebo 模型姿态 |
| `state_visualizer` | 发布 TF、Odometry、Path 和状态 Marker |

初期优先采用容易阅读的 Python 节点。消息类型、跨进程传输方式以及 launch 文件在实现时确定。
Gazebo 资源在 `../gazebo/`，RViz 配置在 `../rviz/`，SUMO 场景在 `../sumo/`。
