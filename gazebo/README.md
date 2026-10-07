# Gazebo：3D 展示层

本目录用于道路世界、两辆车模型以及 Gazebo 展示配置。
建议随实现添加 `worlds/`、`models/` 和 `config/`。

第一版直接同步 SUMO 给出的车辆位置和朝向。负责同步的 ROS2 节点源码放在 `../ros/`，这里保留 Gazebo 资源。
Gazebo 的具体发行版、模型姿态接口和 ROS2 连接方式尚待确认。
