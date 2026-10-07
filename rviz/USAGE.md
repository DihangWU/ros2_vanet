# 两车显示配置

`two_cars.rviz` 展示 `/demo/markers`、两车 Path 和 TF，固定坐标系为 `map`。红色为前车，蓝色为后车；道路虚线提供运动参照。当前车体是 Marker 方块，不是 Gazebo 三维车辆模型。

推荐按 `../ros/README.md` 构建后，用 `ros2 launch cosim_bridge demo.launch.py` 自动打开。
单独运行 RViz 时需先启动桥接节点，然后执行：

```bash
rviz2 -d rviz/two_cars.rviz --ros-args -p use_sim_time:=true
```

该命令从项目根目录执行。仿真结束后仍显示最终状态。路径起点保持不动，车体沿道路移动；滚轮可缩放视角。
