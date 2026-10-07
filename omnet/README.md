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

默认 Cmdenv 已验证。Qtenv 使用同一配置，必要时先点击 Run，再点击 SUMO Play；界面操作尚未验证。不开 Qtenv，INET 也实际参与计算。run_network.py 依赖 ROS2 TCP 接收端，不是独立演示入口。

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

TraCI 9999、ROS2 交换 9998 均为固定本机端口。不要同时启动多组演示。网络模式固定 15 秒，ROS2 不独立推进 SUMO。
