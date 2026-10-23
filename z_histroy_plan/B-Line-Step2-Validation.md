> 历史归档：2026-10-21。本文保留原记录的时间语境，不作为当前进度或启动指令。最新工作入口见 [A 线当前状态与下一步](../A线-当前状态与下一步.md)；代码块中的项目相对路径仍以项目根目录为基准。

# DexArena B 线路第 2 步验收

验收日期：2026-10-20。

## 本步目标

建立独立的 Quest 3 ROS 2 工作区和 `franka_vr_adapter` Python package。它与
`gello_software/ros2` 分开构建和启动。本步骤只验证 ROS 边界、订阅关系、状态
输出和默认不发命令的行为，不启动 controller、fake hardware 或真实机器人。

## 目录隔离

- Quest 3 ROS 2：`quest3_ros2/`
- GELLO ROS 2：`gello_software/ros2/`
- `git status --short -- gello_software` 无输出，说明本步骤没有修改 GELLO 文件。
- 原计划中 `ros2/src/franka_vr_adapter` 已更正为
  `quest3_ros2/src/franka_vr_adapter`。

## 实现内容

`franka_vr_adapter`：

- 订阅 `/right/joint_states`，接收机器人实测关节状态。
- 订阅 `/right/vr/accepted_joint_states`，记录 controller 接受目标的数量。
- 预备发布 `/right/vr/raw_joint_states`，只有 `JointCommandGate` 返回允许时才发布。
- 发布 `/right/vr/status`，用 JSON 报告状态、原因、是否收到机器人状态、raw
  发布数量和 accepted 数量。
- 默认参数 `enable_synthetic_input=false`。默认启动不会建立 command timer，因而
  不会生成或发布合成关节目标。
- 内置的小幅合成轨迹源只为后续 fake hardware 验收准备；必须显式打开参数才会
  启用，且仍需通过第 1 步的 `JointCommandGate`。

## client 临时验收环境

为避免修改 `/home/landau/real-exp-vr`，测试文件只复制到了：

```text
/tmp/dexarena-b2-validation-20261020
```

构建命令等价于：

```bash
source /opt/ros/humble/setup.bash
cd /tmp/dexarena-b2-validation-20261020/quest3_ros2
/usr/bin/colcon build \
  --base-paths src \
  --build-base build \
  --install-base install \
  --symlink-install \
  --packages-select franka_vr_adapter
```

结果：`1 package finished`。使用 Python 3.10.12，可以导入 `rclpy` 和
`franka_vr_adapter.adapter_node`。`colcon list` 将该包识别为
`ros.ament_python`。

## 测试结果

1. 合成轨迹纯 Python 测试：`2 passed in 1.37s`。
2. 默认启动状态 topic 类型：`std_msgs/msg/String`。
3. raw target topic 类型：`sensor_msgs/msg/JointState`。
4. 默认参数：`enable_synthetic_input=False`。
5. 没有机器人状态时：`WAIT_ROBOT / robot_state_missing`。
6. 注入一条安全的模拟机器人状态后：
   `HOLD / synthetic_input_disabled`。
7. 两个阶段的 `raw_publish_count` 均为 0。
8. 对 raw topic 做三秒监听，进程以 timeout 状态 124 结束，即没有收到任何目标。
9. `compileall`、`git diff --check` 均通过。
10. GELLO 目录没有改动。

运行时状态样例：

```json
{"accepted_count":0,"have_robot_state":false,"raw_publish_count":0,"reason":"robot_state_missing","state":"WAIT_ROBOT","synthetic_input_enabled":false}
{"accepted_count":0,"have_robot_state":true,"raw_publish_count":0,"reason":"synthetic_input_disabled","state":"HOLD","synthetic_input_enabled":false}
```

client 临时日志：

```text
/tmp/dexarena-b2-validation-20261020/quest3_ros2/validation/adapter_node.log
/tmp/dexarena-b2-validation-20261020/quest3_ros2/validation/status_wait_robot.txt
/tmp/dexarena-b2-validation-20261020/quest3_ros2/validation/status_after_robot_state.txt
/tmp/dexarena-b2-validation-20261020/quest3_ros2/validation/node_info.txt
/tmp/dexarena-b2-validation-20261020/quest3_ros2/validation/topic_list.txt
/tmp/dexarena-b2-validation-20261020/quest3_ros2/validation/raw_once.txt
```

## 安全结论

本步骤没有启动 FR3 controller、fake hardware、GELLO 或真实机器人。Quest 3
adapter 默认只发布状态。即使收到机器人状态，只要合成输入没有显式启用，它也
保持 HOLD，且不发布关节目标。

## 下一步

第 3 步将在 `quest3_ros2` 内新增 Quest 3 专用 controller override、fake hardware
launch、独立构建脚本和独立启动脚本。GELLO 的包和启动入口继续保持不变。
