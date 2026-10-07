# SUMO 1.22.0：两车直路学习场景

本文先说明独立 SUMO 跟车基线；联合运行已接入 ROS2、Veins 与 INET，入口和步进者见文末。

## 运行

在项目根目录执行：

```bash
# 无界面运行，自动生成路网并记录结果
python3 sumo/scripts/run_demo.py

# 打开 SUMO GUI，观察红色前车 A 和蓝色后车 B（默认与现实时间同步）
python3 sumo/scripts/run_demo.py --gui

# 自定义：60 秒仿真，实时播放
python3 sumo/scripts/run_demo.py --gui --duration 60

# 半速观看：仿真 5 秒对应现实约 10 秒
python3 sumo/scripts/run_demo.py --gui --playback-rate 0.5
```

需要 `sumo`、`sumo-gui` 和 `netconvert` 在 PATH 中。脚本会优先使用可导入的 TraCI；否则使用 `SUMO_HOME/tools`，未设置时尝试可执行文件所在安装目录的 `tools`。只直接打开 `.sumocfg` 不会触发 5 秒急刹，急刹逻辑在 Python 脚本中。

GUI 默认实时播放：车辆开始运行后，仿真 1 秒对应现实约 1 秒，前车约 5 秒后开始急刹，默认整段运动约 15 秒。窗口启动和路网生成时间不计入播放时间。Python 使用单调时钟和每步绝对目标时间控制节拍，SUMO GUI 的 delay 设为 0。旧的 `--delay-ms` 参数已替换为 `--playback-rate`。

`--playback-rate 0.5` 表示半速，`2` 表示两倍速。无界面模式默认快速计算，加 `--realtime` 可以验证真实播放时间。终端和摘要同时记录仿真时间与实际经过的时间。若计算或渲染耗时超过目标节拍，实际播放会变慢。

## 文件阅读顺序

| 文件 | 学习内容 |
| --- | --- |
| `network/straight.nod.xml` | 道路起点和终点，长度 1000 m |
| `network/straight.edg.xml` | 单车道直路，限速 15 m/s |
| `routes/two_cars.rou.xml` | 车型、路线、初始车头位置和速度 |
| `config/demo.sumocfg` | 交通输入、默认 15 秒时长、0.01 秒步长 |
| `scripts/run_demo.py` | 路网生成、TraCI 连接、逐步推进、制动和日志 |
| `network/straight.net.xml` | netconvert 自动生成的路网，无需手动编辑 |

## 演示过程

- 两车以 15 m/s 出发，前车车头位置 100 m，后车 55 m；车长 5 m，因此初始净间距为 40 m。
- 仿真时间 5.00 秒，脚本对前车调用 `slowDown`，目标是在 2 秒内降至零速。
- 7.00 秒起用 `setSpeed(0)` 保持前车停止。
- 后车未收到脚本的制动命令，完全由 SUMO 的 Krauss 跟车模型减速。
- 默认仿真推进至 15.00 秒，输出轨迹和检查摘要。

TraCI 速度控制仍受默认安全及减速度规则约束，具体行为见 [SUMO 官方文档](https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html)。本场景没有关闭这些规则。

## 输出和检查

`output/trajectory.csv` 每步记录两车的仿真时间、位置、速度和加速度，以及后车到前车的净间距。
`output/summary.json` 记录前车制动事件、最小间距、发生碰撞的步数、近似停车时间和最终速度。
每次运行会覆盖这两个结果文件，输出不纳入 Git。

脚本检查：两车始终存在、没有碰撞、净间距不低于 2.5 m、最终速度均不超过 0.05 m/s。近似停车阈值不等于数学上的零速，原始速度仍保留在日志中。

本机无界面运行验证：前车在 5.00 秒开始制动，最小净间距约 2.521 m，无碰撞；15 秒时前车为 0 m/s，后车约 0.020 m/s。GUI 配置已进行短时加载和截图检查。

## 与后续联合仿真的边界

独立运行时当前脚本是唯一启动 SUMO 并调用 `simulationStep()` 的客户端。默认联合运行时由 Veins 唯一步进，不能同时运行本脚本推进同一仿真。
当前后车减速证明的是 SUMO 跟车模型的行为，不证明 V2V 协同制动效果。后续应记录后车收包时间与 ROS2 控制命令，并与这份基线比较。

## 车辆显示

GUI 自动加载 `config/viewsettings.xml`，聚焦车辆活动区域，并显示俯视车辆外形、红蓝颜色和车辆 ID。可以用鼠标滚轮继续放大。SUMO 显示二维车辆，联合运行的 Gazebo 和 RViz2 展示三维车辆。

道路两侧增加固定白色虚线：每段 3 m，间隔 5 m，文件为 `network/road_markings.add.xml`。镜头跟随车辆时，虚线向后移动，提供速度参照；这些图形只影响显示。形状配置参见 [SUMO Shapes](https://sumo.dlr.de/docs/Simulation/Shapes.html)。

## ROS2 制动入口

独立脚本仍是普通跟车基线。`ros2 launch cosim_bridge demo.launch.py` 默认启用 Veins + INET：Veins 通过 TraCI 9999 唯一步进 SUMO，后车实际收包后 ROS2 生成命令，再交给 Veins 执行。SUMO GUI 等待点击 Play，默认现实时间播放 15 秒。两种入口不要同时控制同一仿真。

`network:=false` 恢复 ROS2 直接步进及人工警告；再加 `test_warning:=false` 关闭测试输入。闭环日志在 `ros/log/events/`，精确网络事件在 `omnet/results/`。接口与结果见 [ROS2 README](../ros/README.md)。
