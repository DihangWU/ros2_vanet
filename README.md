# ROS2-VANET 协同紧急制动学习 Demo

前车急刹并通过 Veins + INET 无线网络发送警告；后车实际收包后，ROS2 生成制动命令。SUMO 是车辆运动唯一真值源，Gazebo 和 RViz2 同步三维展示。目录按组件划分，优先保持学习时的可读性。

| 目录 | 版本 | 功能 |
| --- | --- | --- |
| sumo/ | SUMO 1.22.0 | 道路、车辆和运动计算 |
| veins/ | Veins 5.3.1 + veins_inet | 车辆映射、唯一 TraCI 步进者、V2V 应用 |
| omnet/ | OMNeT++ 6.1 | 网络拓扑、调度、构建与运行 |
| inet/ | INET 4.5 | UDP/IP、802.11p 和无线介质 |
| ros/ | ROS2 Jazzy | 收包事件桥接、制动决策、状态发布 |
| gazebo/ | Harmonic / Sim 8.15.0 | 三维模型和道路展示 |
| rviz/ | ROS2 Jazzy RViz2 | 三维 Marker、TF、轨迹与通信状态 |

这些目录保存项目自己的文件，第三方安装不复制到项目中。Veins、INET 和车辆应用运行在同一个 OMNeT++ 进程中。

## 构建与启动

在项目根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/runtime"
ros2 launch cosim_bridge demo.launch.py
```

默认启用无线闭环。Gazebo 和 RViz2 先打开，8 秒后打开 SUMO；点击 SUMO 的绿色 Play 才开始运动。默认约 1 秒现实时间对应 1 秒仿真，5 秒急刹，15 秒结束并保留画面，Ctrl+C 退出。启动等待不计入演示，高负载时播放可能变慢。三维窗口可自由旋转、平移、缩放；不要单独点击 Gazebo Play 推进时间。

首次运行自动构建网络应用，需要 clang++、nlohmann/json.hpp，以及已编译的 OMNeT++、Veins、INET、veins_inet release 库。安装路径与覆盖方法见 [OMNeT++ 说明](omnet/README.md)。SUMO、netconvert 需在 PATH 中。

```bash
# 无 SUMO 窗口，自动开始，保留三维窗口
ros2 launch cosim_bridge demo.launch.py sumo_gui:=false

# 可选 Qtenv 窗口；必要时先在 Qtenv 点击 Run，再在 SUMO 点击 Play
ros2 launch cosim_bridge demo.launch.py network_gui:=true

# 半速，15 秒仿真约需 30 秒
ros2 launch cosim_bridge demo.launch.py playback_rate:=0.5

# 原有人工警告闭环
ros2 launch cosim_bridge demo.launch.py network:=false

# 普通安全跟车基线
ros2 launch cosim_bridge demo.launch.py network:=false test_warning:=false
```

运行前关闭上一组演示，避免端口冲突和多个 /clock。从项目根目录或 ros/ 启动可自动定位资源，其他目录需指定 project_root。

## 当前数据流

```text
SUMO ↔ TraCI ↔ Veins（唯一 SUMO 步进者）
                 ↓ 前车应用
          INET UDP/IP + 802.11p
                 ↓ 后车实际收包
          JSONL/TCP → ROS2 sumo_bridge
                 ↓ /v2v_warning
          brake_controller
                 ↓ /brake_cmd
          JSONL/TCP → Veins → TraCI → SUMO 后车

SUMO 状态 → ROS2 /clock、Odom、TF、Path、Marker → Gazebo / RViz2
```

收包桥接目前放在 network_backend.py，没有单独的 v2v_receiver 节点。网络模式下 ROS2 不调用 simulationStep。TCP 9998 传状态与控制，TraCI 9999 连接 SUMO；TCP 不是车间无线链路。Gazebo 从暂停状态按 SUMO 时间定步展示，不计算另一套动力学或发布另一个 ROS2 时钟。

## 验证结果与边界

RViz 的 `V2V communication` 显示实际收发时间、无线延迟、制动命令发布时间和执行时间，并用绿、橙、红区分就绪、收包与异常。另有两车范围参照圆、TX/RX 位置和 1.5 秒数据包回放，范围和动画均明确标注为示意。配置与说明见 [RViz2 README](rviz/README.md)。

当前配置实测：5.000 秒急刹，5.001 秒发包，5.001152 秒收包，5.05 秒执行 ROS2 制动。无线仿真延迟约 0.152 ms，ROS2 桥接周期为 0.05 秒。15 秒两车均为零速，最终净间距约 2.548 m。

极低发射功率对照没有收包，也没有执行 ROS2 制动；SUMO 自带安全跟车仍会减速，不能只凭停车判断通信生效。停车策略让后车收拢到约 2.5 m，服务于学习演示，不作为道路安全算法验证。

网络模式当前固定两车、单条直路、15 秒和单次警告。Cmdenv 无线闭环与两个三维窗口已联调；Qtenv 界面操作尚未验证。日志见 omnet/results/network_events.jsonl、OMNeT++ .sca/.vec 和 ros/log/events/summary.json，生成结果不纳入 Git。

## 模块入口

- [ROS2 节点、接口与日志](ros/README.md)
- [Veins 应用与同步](veins/README.md)
- [INET 无线配置](inet/README.md)
- [OMNeT++ 构建与运行](omnet/README.md)
- [SUMO 独立基线](sumo/README2.md)
- [Gazebo 模型与视角](gazebo/README.md)
- [RViz2 显示功能与视角](rviz/README.md)
