# 可复用双侧短距雷达模组

在左右前轮后方各安装一颗雷达，补充车身侧面和斜后方的观察。当前安装在蓝色后车 `car_b`，与四角 `corner_radar_rig` 和车顶 `perception_rig` 独立。

## 一行安装

在车辆 SDF 的 `<model>` 内引用：

```xml
<include merge="true"><uri>model://side_radar_rig</uri></include>
```

桥接自动识别装备车辆，建立两路点云、安装 TF 和机身 Marker。位置与参数统一读取本目录 `model.sdf`，修改后重启即可生效。

## 安装与参数

车辆坐标为 x 前、y 左、z 上，yaw 从车头逆时针计算。

| 雷达 | 射线原点（m） | 中心 yaw | 水平扇区 | 障碍回波颜色 |
| --- | --- | --- | --- | --- |
| `side_left` | (1.05, 0.97, 0.65) | +110° | +50°～+170° | 绿色 |
| `side_right` | (1.05, -0.97, 0.65) | −110° | −170°～−50° | 蓝色 |

以正侧方向（±90°）为基准，向车前覆盖 **40°**、向车后覆盖 **80°**，总水平视场 **120°**。垂直 **上下各 15°**，安装俯仰角为 0°；量程 **0.3～30 m**、频率 **20 Hz**、采样 **240×16**、距离分辨率 **0.05 m**，与四角雷达保持一致。

机身中心位于 (1.05, ±0.88, 0.65)，尺寸 0.18×0.12×0.12 m，连接车身侧面。机身前缘 x=1.14 m，位于前轮后缘 x=1.27 m 后方，避免与轮胎重叠。射线原点位于机身外侧，避免自车遮挡。安装按当前 5×1.8 m 车身设计，其他车辆尺寸需相应调整。

## 独立数据与显示

```text
/<车辆ID>/radar/side_left/points
/<车辆ID>/radar/side_right/points
/<车辆ID>/radar/side_left/points_colored
/<车辆ID>/radar/side_right/points_colored
```

原始点云保留 Gazebo 时间戳和无效回波，算法应筛除非有限值。彩色点云仅保留有效回波，障碍表面按雷达着色，地面统一亮绿色。安装 TF 为 `<车辆ID>/radar/side_left_link` 和 `side_right_link`；机身 Marker 命名空间为 `<车辆ID>/side_radars`。

ROS2 复用 `corner_radar_display` 节点，通过 `rig_name=side_radar_rig` 选择模组；桥接与节点名称独立，和四角雷达共存。RViz 的 `Side radar - left/right` 显示可分别开关。完整演示与无需 SUMO 的独立传感器预览都支持。

当前采用 Gazebo GPU 射线近似毫米波距离回波，不模拟射频、多普勒、RCS、多径或噪声，尚未接入制动算法。覆盖受近距盲区、车身遮挡和垂直视场限制。

## 检查

```bash
source /opt/ros/jazzy/setup.bash
source ros/install/setup.bash
python3 ros/tests/test_side_radar.py
# 运行中检查六路毫米波雷达点云和 TF；各雷达范围内需有障碍目标
python3 ros/tests/check_corner_radar_run.py
```

默认静态预览中右侧雷达附近可能没有车辆，因此只有地面回波；点云持续更新即可说明传感器工作正常。障碍回波检查要求目标进入对应视场，测试时可在独立测试场景放置目标车。

独立传感器预览和完整交通演示均保留动态车辆 TF。七路 `points_colored` 显示点云由共享 `cloud_display.py` 等待测量时刻的 TF 后，实际转换到 `map` 再发布，避免 RViz 点云早于车辆 TF 到达而短暂报 Transform 错误。测量时间戳保留原值，不使用最新 TF 替代。每路最多缓存 20 帧，删除车辆时清空缓存和显示；原始 `points` 保持传感器坐标系并直接发布，制动算法输入不变。
