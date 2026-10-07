# ROS2-VANET 协同紧急制动学习 Demo

本项目以学习和可读性为优先：按组件组织文件，逐层跑通，最后连接闭环。

## 环境基线

以下版本由当前使用环境提供。已验证本机 ROS2 Jazzy、SUMO 1.22.0 和 RViz2 的当前桥接场景；网络组件尚未在本项目中验证集成。

| 组件 | 版本 | 职责 |
| --- | --- | --- |
| ROS2 | Jazzy | 警告接收、制动决策、SUMO 桥接与状态发布 |
| Gazebo | 待确认 | 同步展示 SUMO 车辆的 3D 运动 |
| RViz2 | 随 ROS2 Jazzy 环境，具体版本待确认 | 展示 TF、轨迹及通信和制动状态 |
| SUMO | 1.22.0 | 交通仿真、车辆运动与制动执行 |
| Veins | 5.3.1 | TraCI 车辆映射、V2V 应用逻辑 |
| OMNeT++ | 6.1 | 离散事件调度、网络拓扑和仿真运行 |
| INET Framework | 4.5 | 网络协议和无线通信模型 |

## 目录

```text
CoSimDemo/
├── gazebo/   # 世界、车辆模型和可视化配置
├── rviz/     # RViz 配置与显示约定
├── ros/      # ROS2 工作空间、节点和接口
├── sumo/     # 道路、车辆路线和交通配置
├── veins/    # V2V 应用、车辆映射和 TraCI 相关配置
├── omnet/    # 联合网络拓扑、omnetpp.ini 和运行配置
└── inet/     # 本项目使用的 INET 协议和无线模型配置
```

这些目录存放 Demo 自己的文件，不要求复制第三方软件源码或安装文件。
已实现并验证 SUMO 两车直路独立场景，运行方法见 `sumo/README2.md`。已实现 ROS2 两车状态桥接和 RViz2 配置，构建运行方法见 `ros/README.md`。Gazebo 和网络组件目前只有目录说明。
跨组件的文件通过明确的路径引用连接，避免复制相同配置。

## 第一版约定

1. SUMO 是车辆位置和速度的唯一真值源，负责推进车辆运动和执行制动。
2. Gazebo 只同步 SUMO 状态，不计算另一套车辆运动结果。
3. ROS2 接收 V2V 警告，生成后车制动命令，再交给 SUMO 桥接节点。
4. RViz2 展示 ROS2 发布的状态，不直接控制车辆。
5. 联合运行时必须明确唯一的 SUMO 启动者和仿真步进者。由 Veins 管理步进时，ROS2 Bridge 不得自行调用 simulationStep；桥接方式在接入阶段确定。
6. 仿真时间、坐标系和跨进程消息格式在接入前统一，所有事件记录使用同一仿真时间基准。

## 目标数据流

```text
SUMO ──TraCI──> Veins / OMNeT++ / INET
                         │
                    后车收到 V2V 警告
                         │ 跨进程接口（待实现）
                         ▼
                ROS2 v2v_receiver
                         │ /v2v_warning
                         ▼
                ROS2 brake_controller
                         │ /brake_cmd
                         ▼
                ROS2 sumo_bridge ──控制请求──> SUMO
                         │
                   车辆状态 / TF / Path
                         ├──> Gazebo 同步展示
                         └──> RViz2 状态展示
```

OMNeT++、Veins 和 INET 在同一个网络仿真体系中协作；目录分开是为了分清学习职责，不表示三个独立运行进程。INET 与 Veins 的具体集成方式需结合已有工程确认。

## 当前进度

| 阶段 | 状态 | 已完成内容 |
| --- | --- | --- |
| SUMO 独立场景 | 已完成 | 单车道直路、两车、道路虚线、实时播放、前车急刹、轨迹日志 |
| SUMO → ROS2 | 已完成 | `cosim_bridge` 包、真实车辆状态、Odometry、TF、Path 和 `/clock` |
| RViz2 展示 | 已完成 | 红蓝车体 Marker、速度和状态文字、道路虚线、轨迹及 TF |
| Gazebo 展示 | 待实现 | 道路与三维车辆模型、同步 SUMO 状态 |
| ROS2 制动控制 | 待实现 | 接收警告、生成后车制动命令、通过桥接节点执行 |
| VANET 接入 | 待实现 | Veins / OMNeT++ / INET 后车收包事件传入 ROS2 |

当前使用真实 SUMO 数据，不使用假车状态发布器。后车仍由 SUMO 跟车模型控制，尚未接入 V2V 警告和 ROS2 制动命令。

## 快速开始：SUMO + ROS2 + RViz2

在项目根目录执行，首次运行需构建工作空间：

```bash
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/runtime"
ros2 launch cosim_bridge demo.launch.py
```

RViz2 先启动，3 秒后启动 SUMO GUI。车辆开始运动后按现实时间播放，仿真第 5 秒前车急刹，第 15 秒结束；最终画面持续保留，按 Ctrl+C 退出。默认双窗口运行实测约 15.8 秒，机器渲染负载可能使播放略慢。

常用启动参数：

```bash
# 只显示 RViz2，SUMO 在后台运行
ros2 launch cosim_bridge demo.launch.py sumo_gui:=false

# 半速播放，15 秒仿真约需 30 秒
ros2 launch cosim_bridge demo.launch.py playback_rate:=0.5

# 不开窗口，验证 ROS2 消息（仍按实时节拍运行）
ros2 launch cosim_bridge demo.launch.py sumo_gui:=false rviz:=false
```

SUMO 可执行文件需在 PATH 中；TraCI 加载方式见 [SUMO 说明](sumo/README2.md)。从项目根目录或 `ros/` 启动可自动定位资源，其他目录需指定 `project_root`。

## 当前数据流

```text
SUMO（车辆运动真值）
  ↕ TraCI：ROS2 sumo_bridge 统一启动和推进仿真
ROS2
  ├── /car_a/odom、/car_b/odom
  ├── /car_a/path、/car_b/path
  ├── /tf：map → car_a/base_link、car_b/base_link
  ├── /clock：SUMO 仿真时间
  └── /demo/markers：车体、道路、速度与状态
        ↓
      RViz2
```

ROS2 桥接和独立 SUMO 脚本是两种运行入口。运行桥接节点时，不再运行独立脚本推进同一仿真。

## 学习入口与下一步

- [SUMO 场景和独立运行](sumo/README2.md)
- [ROS2 构建、节点、Topic、时间与坐标转换](ros/README.md)
- [RViz2 显示配置使用方法](rviz/USAGE.md)
- [RViz2 配置文件](rviz/two_cars.rviz)

下一步添加 Gazebo 三维展示，继续以 SUMO 为运动真值源；之后实现 ROS2 制动命令，再接入 VANET 通信和完整闭环。

最终演示目标为 15 秒，前车在约 5 秒急刹。具体消息延迟和制动响应时间以实际仿真日志为准。
