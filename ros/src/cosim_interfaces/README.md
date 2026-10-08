# 协同制动消息接口

`V2VWarning.msg` 描述交通警告及其源事件时间，`BrakeCommand.msg` 描述控制器命令及其生成时间。两条消息通过 `event_id` 关联，`simulated` 标记人工测试输入。

接口包使用 ament_cmake 生成 Python 消息绑定，节点实现位于相邻的 `cosim_bridge` 包。修改字段后需重新构建工作空间并重新 source `install/setup.bash`。

`LongitudinalCommand.msg` 描述每个控制周期的外部纵向速度命令，通过 `/lidar_brake/cmd` 发布：目标车辆、递增序号、命令生成时间、目标速度、加速度、雷达测量时间、测量有效性、测得净间距和状态。执行者只应用目标速度，测距字段用于观察，不用于重新计算前车真值距离。它不依赖单次 V2V 事件 ID，允许仅靠点云控制。
