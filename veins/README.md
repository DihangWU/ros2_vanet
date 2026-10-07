# Veins 5.3.1：车辆映射与 V2V 应用

本目录保存车辆应用和 SUMO 同步管理器，使用已有 Veins 安装中的 veins_inet 连接 INET 4.5。

| 文件 / 类 | 功能 |
| --- | --- |
| src/CoSimNetwork.cc：CoSimManager | TraCI 映射、唯一 SUMO 步进者、ROS2 状态与控制交换 |
| src/CoSimNetwork.cc：EmergencyWarningApp | 前车发包、后车实际收包与去重 |
| src/EmergencyWarning.msg | INET FieldsChunk 载荷：事件 ID、源时间、发送时间、位置与速度 |

管理器每 0.05 仿真秒推进 SUMO，5 秒急刹，7 秒起保持前车停止。前车在 5.001 秒发送 100 字节 UDP 组播警告；只有后车 processPacket() 实际收到包，才将警告送给 ROS2。

每步通过本机 TCP 9998 发送一行 JSON，包含 time、cars、warnings、command_active、applied_time；等待 ROS2 回复 command 后继续。收到停车目标后，在后续步进前更新后车目标速度，并保留 SUMO 安全跟车约束。

坐标转换当前专用于单条直路：中心 x 为车头车道位置减半车长，y=-1.6、yaw=0。固定 car_a、car_b 和 15 秒；扩展路网需修改坐标和事件逻辑。

[OMNeT++ 模块](../omnet/README.md)提供构建与启动，[INET 模块](../inet/README.md)提供无线参数。EmergencyWarning_m.cc/.h 自动生成，不纳入 Git；第三方源码不复制到本目录。
