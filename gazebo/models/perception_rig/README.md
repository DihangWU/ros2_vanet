# 可复用车辆传感器

`model.sdf` 是唯一安装与光学参数定义。以车辆中心为原点：x 前、y 左、z 上。当前仅蓝车 `car_b` 引用，在其他车辆模型的 `<model>` 内加入一行即可安装同一套传感器：

```xml
<include merge="true"><uri>model://perception_rig</uri></include>
```

Gazebo 为各车辆自动生成独立作用域的话题。ROS2 的 `sensors.launch.py` 扫描当前 SUMO 场景中的车辆模型，自动建立桥接、TF 和预览节点，不需要复制桥接配置。引用 `background_car` 会为所有共用该模型的背景车安装；只给某一背景车安装时，需先使用独立车辆模型并在启动扫描中指定该模型。

| 传感器 | 安装位置（m） | 视场与输出 |
| --- | --- | --- |
| 广角 | (1.05, 0.10, 1.85) | 水平 120°，640×360，15 Hz |
| 标准 | (1.05, 0, 1.85) | 水平 60°，640×360，15 Hz |
| 长焦 | (1.05, -0.10, 1.85) | 水平 30°，640×360，15 Hz |
| 三维雷达 | (0.90, 0, 1.85) | 水平左右各 60°；垂直 −25°～+15°；720×64 射线，10 Hz，0.3～100 m |

三摄和雷达统一安装在车顶前缘（当前车顶前缘 x=0.95 m、顶面 z=1.55 m）。镜头略向前伸，光心高于车顶 0.30 m，避免拍到车顶；120° 广角画面底部仍可能看到真实的前机盖。不同焦段通过不同水平视场模拟，不包含真实镜头畸变、自动曝光和镜头物理模型。雷达使用 Gazebo GPU 渲染场景采样，可扫描车辆、地面及其他可见障碍物；不是读取 SUMO 坐标生成点云。

ROS2 对每台相机分别发布 `/<vehicle>/camera/{wide,standard,tele}/image_raw` 和 `camera_info`，保留各自标定。`camera/triple/image_raw` 只拼接同一时间戳的三帧，从上到下为长焦、标准主摄、广角，带标题栏，640×1176；拼图没有单一相机内参，因此不发布拼图 CameraInfo。

安装底板直接接触 z=1.55 m 的车顶，雷达和摄像头分别通过支柱及横臂连接到底板。Gazebo 与 RViz 使用同一份 SDF 可视几何：底板、支柱、雷达机身、摄像头机身及三个镜头。RViz 的 `/<vehicle>/sensors` Marker 命名空间随车辆姿态一起更新，通信提示箭头抬高到 z=3.6 m，避开模组。

雷达原始输出为 `/<vehicle>/lidar/points`；显示输出为 `lidar/points_colored`。高于地面 0.15 m 的有效回波着亮青色，地面回波着亮绿色。这是平坦路面的高度着色，不是车辆识别或障碍物分类。RViz 用真实传感器 TF 将点云叠在车体和路面；无有效回波的位置不补点。当前只显示感知数据，未接入制动距离反馈。

修改传感器参数后重启演示。TF 与可视几何位置从此 SDF 读取，当前支持与车体平行安装；改变安装旋转时，需要扩展 `perception_display.py` 与 `sensor_visuals.py` 中对应旋转处理。摄像头分辨率应保持一致以便纵向拼图。

实现依据：[Gazebo Harmonic Sensors](https://gazebosim.org/docs/harmonic/sensors/)、[SDFormat 模型合并](https://sdformat.org/tutorials/?tut=composition_merge_proposal)、[ROS Gazebo 桥接](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge)。
