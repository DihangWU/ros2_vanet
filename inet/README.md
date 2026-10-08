# INET 4.5：协议与无线模型

config/wireless.ini 保存实际参与运行的 INET 参数，由 omnet/omnetpp.ini 引用。车辆使用 Veins 提供的 VeinsInetCar。

| 配置 | 当前选用 |
| --- | --- |
| 传输 | UDP / IPv4 组播 224.0.0.1 |
| 网卡 | IEEE 802.11，p 模式 |
| 无线模型 | Ieee80211DimensionalRadio 和 dimensional 无线介质 |
| 频率 / 带宽 | 5.9 GHz / 10 MHz |
| 发射功率 | 20 mW |
| 地址配置 | HostAutoConfigurator |
| 移动模型 | VeinsInetMobility，状态来自 SUMO |

警告经过 INET 的协议栈、网卡和无线介质，后车实际收包后触发 ROS2。无线延迟来自网络模型，精确发送与接收时间见 omnet/results/network_events.jsonl。

默认配置实测延迟约 0.152 ms。启动前设置 COSIM_RADIO_POWER=0.000000001mW，已验证无收包、无 ROS2 制动；SUMO 安全跟车仍会减速。修改参数需重启演示。

INET 实现来自已有安装，本目录只保存项目配置。路径和启动见 [OMNeT++ README](../omnet/README.md)。

默认三车道交通场景有十一个 VeinsInetCar，全部复用本目录的 802.11p/UDP 配置。仅前车 A 发送一次警告；背景车辆拥有网卡，但其应用不触发 A/B 控制。增加车数不改变网络桥接接口。
