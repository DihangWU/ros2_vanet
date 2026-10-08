# LidarBrake：点云车距控制与制动

独立 Python/NumPy 算法库，不导入 ROS2、Gazebo 或 SUMO。当前用于直路同车道的前方障碍物跟随和停车，不识别车辆类别、不处理弯道、换道或多目标身份。

## 输入与输出

- 输入：后车 `base_link` 坐标系下的 Nx3 原始雷达点、自车速度、采样时间、控制时间、有效 V2V 警告触发。
- 输出：目标速度、加速度、测得净间距、相对速度、目标间距、测量时间和控制状态。
- 不输入前车坐标、前车速度、SUMO 真实间距或 V2V 报文携带的前车位置/速度。ROS2 包装只使用后车 Odom 的速度，当前相当于 SUMO 提供的模拟车速传感器；不使用 Odom 位置计算间距。

## 文件与逻辑

| 文件 | 功能 |
| --- | --- |
| `front_obstacle.py` | 排除无效点、地面、自车和侧方点，按相邻体素聚类；选择最近有效前方障碍物的后表面 |
| `distance_tracker.py` | 距离低通滤波，按连续测距变化估计相对速度，拒绝倒序帧 |
| `pd_controller.py` | 时间车距 PD：`a = Kp × (d - d_desired) + Kd × d_dot` |
| `brake_guard.py` | TTC 和相对制动距离保护 |
| `controller.py` | 组合算法、V2V 提前制动、加减速度与 jerk 限制、失效制动、停车保持 |
| `config.py` / `config.yaml` | 参数定义、校验和默认配置 |
| `tests/` | 不启动仿真的纯算法测试 |

车距是本车前保险杠到障碍物可见后表面的距离。雷达点先由 ROS2 包装从安装坐标系转换到 `base_link`；保险杠位置来自本车模型几何。点云按高度和本车道走廊筛选，避免选择邻道灰车。最低 X 的有效聚类使用 10% 分位数，不信任单个最近点。侧车切入、稀疏或遮挡回波可能改变目标，第一版尚未做身份关联。

期望间距为 `d_desired = 2.5 + 1.2 × v_ego`：行驶时保留时间车距，低速时收拢到约 2.5 m。`d_dot` 正值表示距离增大，负值表示正在接近。V2V 警告让控制器提前进入短时制动，后续强度仍由雷达反馈计算，不直接读取报文中的前车运动状态。

数据超时、缺少有效障碍物或空点云进入 `SENSOR_LOST_BRAKE`，不会当作道路空旷而加速；因此当前适合有前车的固定演示，不是通用自由巡航控制器。紧急制动与传感器失效制动优先于舒适 jerk 限制。接近停车目标且低速时进入 `STOPPED_HOLD`，本次演示不自动恢复行驶。

算法超时按仿真时间计算。联合仿真的锁步协调器会先等待缺失的传感器帧，此时 `/clock` 冻结，不继续积分车辆运动；现实等待超时后报错并保持暂停。空帧或已经过期的输入由算法生成失效制动。这两种行为分别是仿真同步保护和算法的数据失效处理。

## 运行和对比

在项目根目录：

```bash
python3 -m unittest discover -s Algorithm/LidarBrake/tests -v
source /opt/ros/jazzy/setup.bash
source ros/install/setup.bash
ros2 launch cosim_bridge demo.launch.py
```

默认 `control_mode:=lidar`：必须开启 Gazebo 生成真实点云；`sumo_gui:=false` 仍可自动播放，`playback_rate` 与 `duration` 仍可指定。传感器处理能力可能限制实际倍速。

旧真值反馈对照模式：`control_mode:=sumo`。无 Gazebo 运行时需同时指定 `control_mode:=sumo gazebo:=false`；不允许雷达模式悄悄退回真值反馈。

ROS2 包装位于 `ros/src/cosim_bridge/cosim_bridge/lidar_brake_node.py`。它订阅 `/car_b/lidar/points`、`/car_b/odom` 和 `/v2v_warning`，发布 `/lidar_brake/cmd`、`/lidar_brake/markers`。`/brake_cmd` 保留作为 V2V 制动事件/停车目标；真正连续速度命令是 `LongitudinalCommand`。

Gazebo 同步器通过非阻塞 `set_pose_vector` 批量提交完整车辆姿态，再运行到目标时间并自动暂停，根据实际世界时钟确认完成。同步完成经 `/gazebo/synced` 返回；交通步进者等这一帧同步及控制完成，并确认最近 0.2 秒内的雷达帧已到达，才继续推进，避免渲染启动或负载造成交通超前。原始雷达 10 Hz、控制周期 0.05 s，点云控制通道使用可靠 QoS；控制可以使用最近一帧，超过 0.35 s 则制动。15 秒现实时间未完成同步/控制会暂停仿真并报错，不继续盲行。Gazebo 不额外等待现实节拍，由交通协调器统一控制倍速。

SUMO 后车使用 `speedMode=30`，关闭内置安全跟车速度覆盖，保留加减速度和交通规则限制；执行者只应用外部目标速度。真值车距仍可记入轨迹作为评估依据，但不进入雷达控制分支。[SUMO speedMode 文档](https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#speed_mode_0xb3)。时间车距策略参考 [ACC 传感器融合说明](https://www.mathworks.com/help/driving/ug/adaptive-cruise-control-with-sensor-fusion.html)。

日志：`ros/log/events/lidar_brake.jsonl` 记录测量、相对速度、V2V 触发、期望间距与每次命令；`omnet/results/network_events.jsonl` 记录网络执行者收到的连续命令。轨迹中的真值只用于评估测距误差及最终停车间距。

真实网络模式 30 秒演示完成后，可执行 `python3 ros/tests/check_lidar_brake_run.py` 检查停车、实收 V2V、点云测距误差与命令对应关系；不要用人工警告或旧真值模式的日志运行该检查。
