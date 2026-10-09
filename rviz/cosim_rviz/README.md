# 雷达地面回波面板

RViz 原生面板 `cosim_rviz/GroundReturns`，显示名称为“雷达地面回波”，包含“显示地面回波”复选框。勾选显示完整回波，取消后保留高于地面的点；统一控制车顶激光雷达、毫米波与超声波显示。默认勾选，保存 RViz 配置时保存开关状态。

面板发布 `/display/show_ground_returns`（`std_msgs/Bool`，Reliable / Transient Local，深度 1），后启动的显示节点也能获得当前状态。显示节点在共享 `cloud_display.py` 中过滤 `map` 点云，阈值为 z > 0.15 m；不改原始 `points`、Range、图像或制动输入，不改变点云测量时间戳与动态 TF。显示节点保留最近一帧完整点云，暂停仿真时也能隐藏或恢复地面；车辆删除后只恢复空点云，不会把旧车辆回波重新显示出来。

这是当前平坦道路的高度过滤，不是地面分割；高度不超过 0.15 m 的低矮障碍物也会被隐藏。原始传感器数据仍包含这些回波。

插件源码与 CMake 包位于本目录，`ros/src/cosim_rviz` 是指向本目录的符号链接，使源码归属 RViz 模块、构建归属 ROS 工作空间。

首次构建：

```bash
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install
source install/setup.bash
```

传感器预览、完整交通和两车 RViz 配置均已添加面板。手动添加时使用 RViz 的 Panels → Add New Panel → `cosim_rviz/GroundReturns`。开关只作用于已接入共享显示逻辑的 `points_colored`。

验证：在 ROS 工作空间构建 `cosim_rviz` 时指定 `--cmake-args -DBUILD_TESTING=ON`，然后执行 `QT_QPA_PLATFORM=offscreen build/cosim_rviz/panel_check`，验证插件发现、复选框和保存 / 恢复状态。显示过滤测试为 `python3 ros/tests/test_cloud_display.py`；`check_ground_toggle_run.py` 可在独立测试实例旁验证 23 路点云隐藏 / 恢复及原始数据保留，会临时改变开关并在结束时恢复开启。
