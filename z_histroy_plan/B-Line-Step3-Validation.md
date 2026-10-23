> 历史归档：2026-10-21。本文保留原记录的时间语境，不作为当前进度或启动指令。最新工作入口见 [A 线当前状态与下一步](../A线-当前状态与下一步.md)；代码块中的项目相对路径仍以项目根目录为基准。

# DexArena B 线路第 3 步验收

验收日期：2026-10-20。

## 本步目标

为 Quest 3 建立独立的 ROS 2 构建和启动入口，并把现有 FR3 impedance
controller 的输入/回执 topic 覆盖为 VR topic。启动入口只接受 fake hardware，
不连接真实机器人，不启动 GELLO publisher。

## 实现边界

- Quest 3 package：`quest3_ros2/src/franka_vr_adapter`
- Quest 3 构建输出：`quest3_ros2/build`、`quest3_ros2/install`、`quest3_ros2/log`
- Quest 3 构建脚本：`scripts/build_quest3_ros2.sh`
- Quest 3 启动脚本：`scripts/start_quest3_teleoperation_client.sh`
- GELLO package 和 `scripts/start_teleoperation.sh` 没有修改。
- 复用通用的 `franka_fr3_arm_controllers` 控制器二进制、限位、1 kHz reference
  generator 和 0.5 秒 command watchdog；不启动其中的 GELLO publisher。

## Controller override

`config/vr_controller.yaml` 只覆盖以下参数：

```yaml
command_topic: vr/raw_joint_states
accepted_target_topic: vr/accepted_joint_states
activation_mode: always
command_timeout_sec: 0.5
```

运行时参数查询确认：

```text
command_topic          = vr/raw_joint_states
accepted_target_topic  = vr/accepted_joint_states
command_timeout_sec    = 0.5
```

这些是 `/right` namespace 内的相对 topic，因此完整名称分别是
`/right/vr/raw_joint_states` 和 `/right/vr/accepted_joint_states`。

## Fake hardware 保护

`right_fake.launch.py` 固定：

```text
use_fake_hardware = true
robot_ip = fake-hardware.invalid
load_gripper = false
```

`fake-hardware.invalid` 不是机器人地址，只是满足上游 xacro 必填字符串接口。
启动脚本只接受 `--fake`；传入 `--real` 会显示用法并退出。

## client 临时验收

为避免修改 `/home/landau/real-exp-vr`，测试副本位于：

```text
/tmp/dexarena-b3-validation-20261020
```

启动使用独立 ROS domain 94。ROS launch 日志位于：

```text
/home/landau/.ros/log/2026-10-20-15-57-12-214493-landau-Robotics-2443096
```

启动结果：

```text
joint_state_broadcaster      active
joint_impedance_controller   active
```

日志表明硬件由 `mock_generic_system` 初始化，随后 controller manager 在 1000 Hz
运行。fake measured state 的七关节初值为：

```text
[0.0, -0.7853981633974483, 0.0, -2.356194490192345,
 0.0, 1.5707963267948966, 0.7853981633974483]
```

## Topic 验收

存在且类型正确：

```text
/right/joint_states                 sensor_msgs/msg/JointState
/right/vr/raw_joint_states          sensor_msgs/msg/JointState
/right/vr/accepted_joint_states     sensor_msgs/msg/JointState
/right/vr/status                    std_msgs/msg/String
```

ROS graph 连接关系：

- raw：`franka_vr_adapter` 是唯一 publisher，`/right/joint_impedance_controller`
  是唯一 subscriber。
- accepted：controller 是唯一 publisher，adapter 是唯一 subscriber。
- `/right/gello/raw_joint_states` 不存在。
- adapter 默认 `enable_synthetic_input=False`。
- adapter 已收到 fake measured state，但保持
  `HOLD / synthetic_input_disabled`，`raw_publish_count=0`。
- 三秒 raw topic 检查没有收到消息。

## 命令源互斥验收

1. Quest fake 栈运行时再次启动 Quest，入口被共享锁拒绝：

```text
Error: another GELLO or Quest 3 arm teleoperation instance is already running
```

2. 在独立 ROS domain 95 模拟一个 GELLO raw publisher，再启动 Quest，入口在启动
   controller 之前拒绝：

```text
Error: GELLO is already publishing /right/gello/raw_joint_states; stop it before Quest 3
```

测试结束后，fake stack 和模拟 GELLO publisher 均已停止，没有残留进程。

## 其他检查

- package 和 launch 契约测试：`4 passed`（最终复跑 `1.02s`）。
- `bash -n`：构建脚本、启动脚本均通过。
- `colcon build`：`1 package finished`。
- Python `compileall` 和 `git diff --check` 通过。
- `git status --short -- gello_software` 无输出。

## 下一步

第 4 步增加持续 watchdog、逐项故障注入和 requested/accepted/measured 三层记录。
仍然只在 fake hardware 上进行。
