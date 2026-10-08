# Veins 5.3.1：车辆映射与 V2V 应用

本目录的网络适配源码采用 GPL-3.0-or-later，许可范围及上游说明见 [第三方许可说明](../lisence/THIRD_PARTY_NOTICES.md)。

本目录保存车辆应用和 SUMO 同步管理器，使用已有 Veins 安装中的 veins_inet 连接 INET 4.5。

| 文件 / 类 | 功能 |
| --- | --- |
| src/CoSimNetwork.cc：CoSimManager | TraCI 映射、唯一 SUMO 步进者、ROS2 状态与控制交换 |
| src/CoSimNetwork.cc：EmergencyWarningApp | 前车发包、后车实际收包与去重 |
| src/EmergencyWarning.msg | INET FieldsChunk 载荷：事件 ID、源时间、发送时间、位置与速度 |

管理器每 0.05 仿真秒推进 SUMO，5 秒急刹，7 秒起保持前车停止。前车在 5.001 秒发送 100 字节 UDP 组播警告；只有后车 processPacket() 实际收到包，才将警告送给 ROS2。

每步通过本机 TCP 9998 发送一行 JSON，包含 time、cars、warnings、network_events、collisions、command_active、applied_time；等待 ROS2 回复 command 后继续。network_events 导出实际发包、收包与位置，用于 RViz 通信显示；位置取自当前 SUMO 状态。collisions 导出碰撞车辆，ROS2 检测到碰撞后停止继续推进并显示异常。默认雷达模式接收应答中的 `longitudinal`（序号、生成时间、目标速度），只执行外部速度，不计算前车真值车距；`speedMode=30` 关闭 SUMO 安全跟车速度覆盖，保留加减速度及交通规则限制。命令超过 0.35 秒未更新时按自车速度制动。`control_mode:=sumo` 才使用旧真值跟车策略。

管理器遍历实际创建的车辆模块，按 ID 导出全部状态。位置直接查询 SUMO 当前二维车头坐标，角度转 ROS yaw，再沿朝向减半车长得到车体中心，不再写死 y 和 yaw。默认有十一个 VeinsInetCar，仅 A 发包、B 收包后触发制动，背景车也有 INET 网卡。时长由启动参数传入，默认 30 秒。A/B 控制策略仍只适用于同一直路同一车道。

SUMO GUI 启用时，管理器每步更新视野边界，镜头以两车中点为中心，横向至少 100 m。启动脚本启用 followSumoVehicles 并关闭 ignoreGuiCommands；无界面运行保持 GUI 命令禁用。

[OMNeT++ 模块](../omnet/README.md)提供构建与启动，[INET 模块](../inet/README.md)提供无线参数。EmergencyWarning_m.cc/.h 自动生成，不纳入 Git；第三方源码不复制到本目录。
