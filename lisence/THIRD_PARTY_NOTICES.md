# 项目许可范围与第三方许可说明

## 项目自身的许可

除下列明确列出的例外及第三方材料外，本仓库原创代码、配置和文档采用 [Apache License 2.0](../LICENSE)。原创部分版权归相应贡献者所有。根目录 LICENSE 保留 Apache 2.0 原文，不修改其条款。

以下网络适配文件采用 **GPL-3.0-or-later**，其文件头也标注了 SPDX 标识：

- `veins/src/CoSimNetwork.cc`
- `veins/src/EmergencyWarning.msg`
- `omnet/ned/CoSim.ned`

这些文件实现或组装 Veins / veins_inet / INET 网络模块。分发包含 Veins 的组合网络程序时，应遵循 GPLv3 及各依赖适用条款，不能仅用根目录 Apache LICENSE 声称组合程序是 Apache-only。ROS2 进程通过本机 JSONL/TCP 与网络进程交换数据，其独立原创源码仍采用 Apache 2.0。

[GPLv3 全文](GPL-3.0.txt)。Apache 2.0 与 GPLv3 的兼容关系见 [Apache 官方说明](https://www.apache.org/licenses/GPL-compatibility.html)。根目录 Apache LICENSE 不重新授权第三方组件，也不取消 OMNeT++ 的使用限制。

## 第三方组件

本表记录学习 Demo 使用的主要直接依赖，不是所有操作系统、工具链和传递依赖的完整清单。版本以模块 README 和实际安装为准，具体文件可能带有不同许可，以上游原文及文件声明为准。

| 组件 | 使用版本 / 系列 | 许可与本地文本 | 上游来源 |
| --- | --- | --- | --- |
| OMNeT++ | 6.1 | Academic Public License；[安装版原文](OMNeTpp-ACADEMIC-LICENSE.txt) | [官网许可说明](https://omnetpp.org/intro/license.html) |
| Veins / veins_inet | 5.3.1 | GPL-2.0-or-later；[Veins COPYING](Veins-COPYING.txt)，本项目组合网络程序使用 GPLv3 路径 | [Veins 源码](https://github.com/sommer/veins) |
| INET Framework | 4.5 | 主要为 LGPL-3.0-or-later；[安装版 LICENSE](INET-LICENSE.md)、[LGPLv3](LGPL-3.0.txt) 和 [GPLv3](GPL-3.0.txt) | [INET 源码](https://github.com/inet-framework/inet) |
| Eclipse SUMO / TraCI | 1.22.0 | EPL-2.0，带有条件适用的 GPL-2.0-or-later 次级许可选项；[LICENSE](SUMO-LICENSE.txt)、[NOTICE](SUMO-NOTICE.md) | [SUMO 源码](https://github.com/eclipse-sumo/sumo) |
| ROS2 rclpy、ros_gz_bridge、ros_gz_interfaces | Jazzy 环境 | Apache 2.0；[全文](../LICENSE)，各包的版权归各自作者所有 | [rclpy](https://github.com/ros2/rclpy)、[ros_gz](https://github.com/gazebosim/ros_gz) |
| RViz2 | Jazzy 环境 | Clear BSD（BSD-3-Clause-Clear）；[Jazzy 分支原文](RViz-LICENSE.txt) | [RViz 源码](https://github.com/ros2/rviz/tree/jazzy) |
| Gazebo Sim | Harmonic / Sim 8.15.0 | Apache 2.0；[全文](../LICENSE)，其他 Gazebo 组件按其自身声明 | [gz-sim8 许可](https://github.com/gazebosim/gz-sim/blob/gz-sim8/LICENSE) |
| nlohmann/json | 构建环境提供 | MIT / Expat；[安装包版权与许可记录](nlohmann-json-copyright.txt) | [上游源码](https://github.com/nlohmann/json) |

Veins 网络基类的文件头声明包含 Christoph Sommer 的版权；INET 文件声明包含 OpenSim Ltd. 等贡献者的版权。这里的摘要不取代上游文件中的完整版权和归属声明。

本机 `nlohmann/json.hpp` 声明 `SPDX-FileCopyrightText: 2013-2023 Niels Lohmann` 和 `SPDX-License-Identifier: MIT`；安装包版权文件另外列出了 Florian Loitsch、Bjoern Hoehrmann 等作者及其适用文件。

## 保存文本的来源

- `Veins-COPYING.txt`：本机 Veins 5.3.1 的 `COPYING`。
- `INET-LICENSE.md`：本机 INET 4.5 的 `LICENSE.md`。
- `SUMO-LICENSE.txt`、`SUMO-NOTICE.md`：本机 SUMO 1.22.0 的 `LICENSE`、`NOTICE.md`。
- `OMNeTpp-ACADEMIC-LICENSE.txt`：本机 OMNeT++ 6.1 的 `doc/License`。
- `RViz-LICENSE.txt`：ROS2 RViz 官方仓库 Jazzy 分支的 `LICENSE`。
- `nlohmann-json-copyright.txt`：本机构建依赖安装包的版权文件，保留其列出的各文件许可及版权。
- 根目录 `LICENSE`、`GPL-3.0.txt`、`LGPL-3.0.txt`：系统提供的标准许可证文本。

本目录仅保存许可文本和说明，不包含上述第三方软件的完整源码或二进制。自动生成的 Veins 消息代码、项目构建结果和 ROS2 安装结果不纳入 Git；生成代码及组合程序仍受相应许可约束。

## 使用与分发

本仓库主要通过脚本调用或链接用户自行安装的第三方组件。仅将软件列为外部依赖，并不意味着把该软件重新许可为 Apache 2.0。

复制、修改第三方源码时，必须保留适用的版权、许可、NOTICE 和修改说明。分发网络动态库、第三方二进制、Docker 镜像或安装包时，需按实际包含的组件另行满足对应源码、许可、归属声明和其他分发要求；本清单不自动完成这些义务。

OMNeT++ Academic Public License 的非商业使用条件继续适用；商业用途应按其官方条款取得适当许可。没有修改任何第三方许可证。
