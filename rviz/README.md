# RViz2：系统观察层

本目录用于 `.rviz` 配置和显示约定。

计划展示 `/tf`、`/car_a/odom`、`/car_b/odom`、`/car_a/path`、`/car_b/path`，并通过 Marker 显示通信线、警告和制动状态。
`/v2v_warning` 是业务消息，由 ROS2 可视化节点转换成 Marker 供 RViz2 显示；RViz2 不会自动显示任意业务消息。
统一坐标系拟定为 `map`，具体坐标转换在 SUMO 桥接实现时明确。`map` 坐标系不意味着必须发布 `/map` 栅格地图。
