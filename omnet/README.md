# OMNeT++ 6.1：网络组装与运行

本目录组装 Veins 应用与 INET 无线模型，构建项目动态库并启动 SUMO 和网络仿真。

| 文件 | 功能 |
| --- | --- |
| ned/CoSim.ned | 管理器、无线介质、动态车辆及应用定义 |
| omnetpp.ini | 时间、随机种子、TraCI 步进和结果记录；引用 INET 配置 |
| scripts/build_network.py | 生成消息源码并构建 build/libcosim_network.so |
| scripts/run_network.py | 启动 SUMO，加载库与 NED，运行 Cmdenv 或 Qtenv |

## 运行

从项目根目录、已构建 ROS2 工作空间后运行：

```bash
source /opt/ros/jazzy/setup.bash
source ros/install/setup.bash
ros2 launch cosim_bridge demo.launch.py

# 可选网络界面
ros2 launch cosim_bridge demo.launch.py network_gui:=true
```

默认 Cmdenv 已验证。Qtenv 使用同一配置，演示时先点击 Fast Run（F6），再点击 SUMO Play。不开 Qtenv，INET 也实际参与计算。run_network.py 依赖 ROS2 TCP 接收端，不是独立演示入口。

仅构建可执行 python3 omnet/scripts/build_network.py。需要 clang++、nlohmann/json.hpp 和已构建的第三方 release 库。

| 环境变量 | 默认路径 |
| --- | --- |
| OMNETPP_ROOT | /home/maple/omnetpp-6.1 |
| VEINS_ROOT | /home/maple/veins-5.3.1 |
| INET_ROOT | /home/maple/inet4.5 |

veins_inet 位于 Veins 的 subprojects/veins_inet，也需已编译。安装库更新后，删除 omnet/build/libcosim_network.so 再构建。脚本不修改第三方源码。

## 时间与结果

网络和 SUMO 使用同一仿真时间。Veins 每 0.05 秒推进 SUMO，通过 JSONL 等待 ROS2 应答；ROS2 用稳态时钟控制现实播放速度。无线事件可发生在两次 SUMO 步进之间，保留 OMNeT++ 的接收时间精度。

results/network_events.jsonl 记录急刹、发包、收包和命令执行，每次覆盖；.sca/.vec/.vci 按运行 ID 保存统计量。build/、results/ 不纳入 Git。

TraCI 9999、ROS2 交换 9998 均为固定本机端口。不要同时启动多组演示。网络模式默认 30 秒，管理器 duration 与 sim-time-limit 同步接收 ROS2 duration，ROS2 不独立推进 SUMO。默认十一个 VeinsInetCar；`scenario:=two_cars duration:=15.0` 使用旧两车场景。

Qtenv 联合演示请用 F6 Fast Run；普通 Run 在 5 秒发包附近可能为动画暂停联合时间。Fast Run 不改变网络事件和收发计算。

SUMO GUI 模式自动启用管理器 followSumoVehicles，并设 ignoreGuiCommands=false，使视野随两车移动；后台 SUMO 模式禁用这两项 GUI 操作。

默认雷达模式由 ROS2 包装传入 `--lidar-control`，对应 `CoSim.manager.lidarControl`。逐步应答携带连续目标速度，网络管理器只执行，算法在 `Algorithm/LidarBrake/`。INET 的发包与收包模型保持相同。`results/network_events.jsonl` 新增 `lidar_control_accepted`（命令序号、生成时间、执行者接收时间与目标速度）；不是无线数据包。
