# 协同制动消息接口

`V2VWarning.msg` 描述交通警告及其源事件时间，`BrakeCommand.msg` 描述控制器命令及其生成时间。两条消息通过 `event_id` 关联，`simulated` 标记人工测试输入。

接口包使用 ament_cmake 生成 Python 消息绑定，节点实现位于相邻的 `cosim_bridge` 包。修改字段后需重新构建工作空间并重新 source `install/setup.bash`。
