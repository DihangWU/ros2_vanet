# 可复用四角短距雷达模组

当前安装在蓝色后车 `car_b`，用于侧面与后方点云观察。四个传感器使用 Gazebo GPU 射线采样真实场景，作为毫米波雷达的距离回波近似；尚未模拟射频传播、多普勒速度、RCS、噪声、多径或稀疏目标检测。点云的 intensity 不作为真实毫米波反射强度使用。

## 一行安装

在车辆 SDF 的 `<model>` 内加入：

```xml
<include merge="true"><uri>model://corner_radar_rig</uri></include>
```

模组独立于车顶 `perception_rig`，可以单独安装或同时引用。`model.sdf` 是安装位置、朝向、视场、频率、量程及外观的统一定义；不需要为每辆车复制四个传感器、TF 或 ROS2 桥接配置。

启动扫描自动识别当前 SUMO 场景中的装备车辆，逐车创建桥接、点云输出、TF 和 RViz 机身 Marker。优先使用 `gazebo/models/<车辆ID>/model.sdf`，不存在时复用 `background_car`。在背景车共用模型中引用会给所有复用者安装；只装备 `bg_01` 时建立独立的 `gazebo/models/bg_01/model.sdf`，再生成交通世界。

添加场景车辆或独立车辆模型后，在项目根目录执行 `python3 gazebo/scripts/build_traffic_world.py`，并重启演示。已有车辆只增加此 include 或修改雷达参数时，重启即可。模组按当前 5 m × 1.8 m 车身设计，其他尺寸车辆需调整安装位置。

## 安装与覆盖

车辆中心为原点：x 前、y 左、z 上，yaw 从 +x 逆时针。

| 雷达名 | 射线原点（m） | yaw | 主要方向 / 回波颜色 |
| --- | --- | --- | --- |
| `front_left` | (2.55, 0.97, 0.65) | +45° | 左前方 / 橙色 |
| `front_right` | (2.55, -0.97, 0.65) | -45° | 右前方 / 粉色 |
| `rear_left` | (-2.55, 0.97, 0.65) | +135° | 左后方 / 紫色 |
| `rear_right` | (-2.55, -0.97, 0.65) | -135° | 右后方 / 蓝青色 |

每颗水平视场 **120°**（左右各 60°）、垂直 **−15°～+15°**（合计 30°），量程 **0.3～30 m**，**20 Hz**，**240×16** 射线。四颗安装俯仰角均为 0°，以水平面为中心扫描上下各 15°。前角由正侧方转为左右前方各 45°，后角仍为左右后方各 135°。前角水平扇区相对车头分别是 −15°～+105°、−105°～+15°，前方存在交汇区。视场从 180° 收窄为 120° 后，覆盖重叠减少，车侧近处及紧贴车尾的正后方会出现几何盲区，不保证侧后方无缝覆盖；前角转向车头也会缩小对车侧中部的覆盖，不保证四角雷达在近车侧相互补齐；覆盖测试检查远侧 / 后方目标、前角交汇区和保留的盲区。0.3 m 内仍属于配置的近距盲区，遮挡和垂直视场也会限制实际可见回波。前向远距离感知继续由现有车顶激光雷达承担。

按目前前角射线原点 (2.55, ±0.97, 0.65)，车前中心线在 x≈6.17 m 后进入两颗前角雷达的共同水平视场，即距前保险杠约 3.67 m。此结果是安装与视场的几何计算，不保证任意高度 / 遮挡情况下都能产生回波；测试目标高度为 0.65 m。更近的车前中央区域仍需要前向激光雷达等传感器补充。

四个小机身分别连接保险杠角部，采用不同颜色方便观察；RViz 直接读取同一份可视几何，命名空间为 `car_b/radars`。射线原点略放在机身和车身外侧，避免光学射线被自车遮挡；这不模拟毫米波穿透保险杠。真实车辆通常将角雷达藏在保险杠内部，本模组保留可见机身用于学习。

当前支持水平安装和 yaw 旋转；修改 roll/pitch 时需要扩展点云地面高度着色处理，节点会明确拒绝未支持的安装旋转。

## 四路独立 ROS2 数据

```text
/<车辆ID>/radar/front_left/points
/<车辆ID>/radar/front_right/points
/<车辆ID>/radar/rear_left/points
/<车辆ID>/radar/rear_right/points
```

均为独立 `sensor_msgs/PointCloud2`，坐标系为 `<车辆ID>/radar/<雷达名>_link`，保留 Gazebo 测量时间戳。原始通道使用可靠 QoS，保留未命中回波的无效值；使用算法前应筛除非有限值。

每路另发布 `points_colored`，只保留有效回波：地面亮绿色，障碍表面采用对应雷达颜色。这是高度着色，不是语义分类。RViz 使用 `map → <车辆ID>/base_link → <车辆ID>/radar/<雷达名>_link` 对齐，四个 PointCloud2 显示可以分别关闭，按帧替换点云。

桥接内部话题为 `/<车辆ID>/radar/raw/<雷达名>/points`。桥接入口为 `ros/src/cosim_bridge/launch/sensors.launch.py`；点云 / TF 和机身显示分别位于 `ros/src/cosim_bridge/cosim_bridge/corner_radar_display.py` 与 `sensor_visuals.py`。点云从 Gazebo 渲染采样产生，不从 SUMO 车辆坐标构造。

## 运行与检查

按 ROS2 模块说明构建，使用原启动命令：

```bash
source /opt/ros/jazzy/setup.bash
source ros/install/setup.bash
ros2 launch cosim_bridge demo.launch.py
```

Gazebo 与 RViz 打开后，在 SUMO 点击 Play。当前只增加感知与显示，不接入角雷达制动、盲区告警或与前向激光雷达融合。

```bash
# 不启动仿真：检查覆盖与独立车辆的一行引用
python3 ros/tests/test_corner_radar.py
# 三车道演示正在播放时：验证四路真实点云、障碍物/地面回波及 TF
python3 ros/tests/check_corner_radar_run.py
```
