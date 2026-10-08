# 可复用侧后摄像头模组

三颗独立摄像头补充左侧、右侧和后方观察，当前安装在蓝色后车。模型、安装参数和光学参数都在 `model.sdf`，在其他车辆 SDF 的 `<model>` 中一行引用：

```xml
<include merge="true"><uri>model://surround_camera_rig</uri></include>
```

与车顶 `perception_rig` 和四角 `corner_radar_rig` 分别引用。启动扫描按车辆 ID 自动建立独立桥接、TF、图像输出和 RViz 模组 Marker，不必复制节点配置。优先加载 `gazebo/models/<车辆ID>/model.sdf`，不存在时加载 `background_car`。共享背景车模型中的引用会装备所有复用它的车辆；新增独立车辆模型后运行 `python3 gazebo/scripts/build_traffic_world.py`，已有模型增加引用后重启即可。

## 位置与光学参数

以车体中心为原点：x 向前、y 向左、z 向上。安装参数按当前 5 m × 1.8 m 车身设置，其他车型需调整。

| 相机 | 光心位置（m） | yaw | 观察方向 |
| --- | --- | --- | --- |
| `left` | (0.65, 1.16, 1.30) | +110° | 左侧，向后偏 20° |
| `right` | (0.65, -1.16, 1.30) | -110° | 右侧，向后偏 20° |
| `rear` | (-2.62, 0, 0.95) | 180° | 正后方 |

三颗均为水平安装，水平视场 120°、640×360 RGB、15 Hz、近裁剪 0.05 m、远裁剪 150 m。侧相机采用与车窗相接的后视镜式机身，后相机连接车尾上部；光心在机身外，避免被自身机身遮挡。当前是普通透视相机，不是鱼眼或环视鸟瞰模型，不模拟畸变、曝光和镜头物理。安装支持 yaw 旋转，roll/pitch 当前要求为 0。

Gazebo 从三维场景渲染真实图像；RViz 读取同一 SDF 的六个机身 / 镜头部件，通过 `<车辆ID>/surround_cameras` Marker 命名空间随车显示。

## 独立 ROS2 通信与单窗预览

每颗相机分别发布：

```text
/<车辆ID>/camera/left/image_raw       /<车辆ID>/camera/left/camera_info
/<车辆ID>/camera/right/image_raw      /<车辆ID>/camera/right/camera_info
/<车辆ID>/camera/rear/image_raw       /<车辆ID>/camera/rear/camera_info
```

图像是 `sensor_msgs/Image`，标定是 `sensor_msgs/CameraInfo`，使用传感器 Best Effort QoS。分别保留测量时间戳，frame 为 `<车辆ID>/<相机名>_optical_frame`；TF 链为 `map → <车辆ID>/base_link → <相机名>_link → <相机名>_optical_frame`。算法可独立订阅任意相机，不依赖预览是否凑齐。桥接内部话题为 `/<车辆ID>/surround/raw/<相机名>/{image,camera_info}`。

同时安装前向三目时，`/<车辆ID>/camera/surround/image_raw` 按相同时间戳拼接六帧，输出 1280×1176 RGB，在 RViz 一个 Image 窗口显示：

| 第一列 | 第二列 |
| --- | --- |
| 长焦 | 后向 |
| 标准主摄 | 左向 |
| 广角 | 右向 |

每格保留原始 640×360 像素，顶部增加 32 像素标题。不同时间戳不混拼；缺帧时不发布该组预览，不影响六路独立传输，缓存最多保留十组。拼图只用于观察，不对应单一光心，不发布虚构 CameraInfo。原前向 `camera/triple/image_raw` 继续保留。若车辆只安装侧后模组，则同一预览话题输出后 / 左 / 右三行一列。

六路拼图功能位于 `ros/src/cosim_bridge/cosim_bridge/surround_camera_display.py`，桥接入口为 `ros/src/cosim_bridge/launch/sensors.launch.py`，模型显示在 `sensor_visuals.py`。

## 启动

从项目根目录构建并启动：

```bash
source /opt/ros/jazzy/setup.bash
cd ros
colcon build --symlink-install
source install/setup.bash
ros2 launch cosim_bridge demo.launch.py
```

RViz 使用 `Six cameras - front and surround` 显示，拖大 Image 窗口可以看清两列三行。已有演示需退出后重新启动，才能加载新模组和新显示配置。相机只用于感知数据和显示，未接入现有激光雷达制动算法。
