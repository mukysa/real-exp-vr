# VR 遥操与数采优化 TODO

更新日期：2026-10-29。

现有 VR 遥操与数采流程已由用户跑通。原 TODO 已整体替换；当前仅保留以下两项优化，不再沿用旧版的链路搭建、功能扩展和验收任务。下述方案来自代码阅读，均为待实施内容。

工作约定：优先读代码分析；除非确有特殊需要，无需运行代码、测试或重新启动已跑通的流程。本文中的完成标准用于后续实施时确认结果，不代表现在需要执行现场验证。

## 1. 不佩戴 Quest 头显时保持 streaming

**目标：** 头显保持开启并固定放置，操作者不佩戴头显，只使用 Touch 手柄完成遥操与数采。

### 当前代码现状

- [power.py](data_collection/meta_quest/power.py) 已提供 `QuestKeepAwake`，通过 ADB 广播 `com.oculus.vrpowermanager.prox_close` 请求模拟“已佩戴”，退出时通过 `com.oculus.vrpowermanager.automation_disable` 恢复正常接近传感器行为。
- [openvr_bridge.py](data_collection/meta_quest/openvr_bridge.py) 的常规转发模式已使用异步保活，但 `--check-left-touch`／`--check-both-touch` 检查分支会在创建保活对象前返回。
- [start_quest3_hardware.sh](scripts/start_quest3_hardware.sh) 启动前使用上述 OpenVR 检查，正式运行则调用 [steamvr_bridge.py](data_collection/meta_quest/steamvr_bridge.py) 的 SAPIEN 输入路径；后者没有接入保活。因此当前快速启动流程不会主动保持离头唤醒。
- 现有保活实现只判断 ADB 命令退出状态，不能据此确认 Quest 系统实际接受了广播，或手柄位姿仍有效。

### 待办

- [ ] 复用现有保活机制，接入当前快速启动会话，覆盖等待 Streaming、手柄检查及正式遥操阶段；保留已跑通的 SAPIEN 输入路径。
- [ ] 明确保活的启用／关闭方式，检查 ADB 可达性和授权状态，提供成功请求、失败及恢复状态的日志。
- [ ] 将 ADB 调用与位姿／机器人控制循环隔离，避免命令超时阻塞实时输入；复用现有异步处理思路。
- [ ] 明确保活生命周期及单一管理方，正常退出、启动失败和可处理的中断均恢复正常休眠行为；说明强制终止后的手动恢复方式。
- [ ] 更新 [README-QUICKSTART.md](README-QUICKSTART.md)，说明 ADB 前提、头显摆放、关闭保活及恢复方法。

### 完成标准与边界

- 摘下头显后 ALVR 持续 Streaming，操作区域内的手柄位姿持续有效更新，按键仍可读取。
- 退出会话后恢复正常接近传感器休眠行为；ADB 不可用时给出明确提示，不将“命令成功退出”表述为“已验证保活生效”。
- 头显仍须开启、ALVR 保持前台、摄像头能够追踪手柄；不以 Dashboard 的 Streaming 标签代替有效追踪判断。
- 该任务解决“不佩戴头显”，不涉及关闭头显后独立使用手柄。保活能否生效受 Quest 系统版本及 ADB 授权状态影响。

## 2. 使用 Touch 手柄控制数采 episode，同时保留键盘 s/e/d/q

**目标：** Quest 数采可通过手柄完成开始、保存和丢弃 episode，同时完整保留键盘 s/e/d/q；两种输入同时可用、行为一致，无需切换控制模式。保留现有 Y 接管／回位行为。

**实施范围：** 当前只做左臂。后续右臂和双臂数采也采用完全相同的实体按键映射：左 Y 接管／回位、左 X 开始、右 A 保存、右 B 丢弃，键盘 s/e/d/q 同时保留。键位不随机械臂模式变化；仅控制右臂时也使用左 Y 和左 X。episode 操作作用于当前会话所选机械臂，双臂模式共同开始、保存或丢弃一个 episode。此约定不代表本轮需要实现右臂或双臂数采。

| 输入 | 目标行为 | 当前协议字段 |
|---|---|---|
| 左 Y | 接管所选机械臂；再次按下回位 | 左侧 `clutch` |
| 左 X 或键盘 s + 回车 | 开始 episode，两者等效且同时可用 | 左侧 `record`／录制器键盘命令 |
| 右 A 或键盘 e + 回车 | 结束并保存当前 episode，两者等效且同时可用 | 右侧 `record`／录制器键盘命令 |
| 右 B 或键盘 d + 回车 | 丢弃当前未保存 episode，两者等效且同时可用 | 右侧 `clutch`／录制器键盘命令 |
| 键盘 q + 回车 | 保留现有退出行为 | 录制器键盘命令 |

### 当前代码现状

- [steamvr_bridge.py](data_collection/meta_quest/steamvr_bridge.py) 已将 X/Y/A/B 编入位姿协议；`record` 只是布尔字段名，尚未连接 episode 录制功能。无需为这套映射修改 Quest APK 或扩展现有位姿协议。
- [live_input.py](data_collection/fr3_ik/live_input.py) 处理左 Y 接管并保留右 B 状态，但未将 X/A/B 转换成录制命令。左臂模式运行中也读取双手柄。
- [lerobot_collection.py](data_collection/lerobot_collection.py) 当前通过终端 `input()` 接收 `s/e/d/q`，已有开始、保存、清空缓冲区及退出逻辑。
- 缺少的链路是：client 手柄按下事件 → ROS 2 DDS → server → 录制器命令处理。

### 待办

- [ ] 在 client 原始手柄记录消费处识别 X/A/B 按下沿，生成录制控制事件；可在 [multi_live_ik_node.py](quest3_ros2/src/franka_vr_adapter/franka_vr_adapter/multi_live_ik_node.py) 接入。保留左 Y 的接管／回位逻辑，右 B 不触发接管。
- [ ] 通过现有 DDS 链路把事件传到 server，由 [quest_observation_bridge.py](data_collection/quest_observation_bridge.py) 转发给录制器。事件传输应独立于相机样本就绪条件，确保相机暂时无新帧时仍能保存或丢弃已有缓冲区；具体话题、端口和事件格式在实施时确定。
- [ ] 为录制器接入 Quest 手柄事件，与键盘 s/e/d/q 同时工作，统一进入现有命令处理；不得因启用手柄而禁用键盘 s/e/d。GELLO 默认键盘模式保持可用。
- [ ] X 或 s 开始录制采用相同门控：当前左臂已处于 `FOLLOWING`，且录制器收到新鲜有效的 Quest 样本；拒绝原因应可见。设计上按所选机械臂检查，后续右臂检查右臂，双臂要求两侧均满足条件，不将左臂写成通用逻辑中的固定限制。
- [ ] 长按只触发一次；启动、重连及追踪恢复时已经按住的键，必须先释放再按。按下沿在原始输入记录处识别，避免合并位姿帧漏掉短按。
- [ ] 对事件去重并限制有效期，防止 `save_episode()` 同步保存期间积压的旧事件误触发下一轮；手柄和键盘命令统一处理，重复输入不得重复保存或重开 episode。明确同时按键的优先级，建议丢弃优先于保存和开始。
- [ ] 明确显示开始、保存完成、丢弃及命令被拒绝的结果，避免把“按键已收到”当成“episode 已保存”。
- [ ] 更新数采入口和 [README-QUICKSTART.md](README-QUICKSTART.md)，说明新模式参数、双机改动位置、按键映射及操作顺序；覆盖当前左臂 arm-only 与 gripper 数采用法。

### 完成标准与操作语义

- 正常一轮：**Y 接管 → X 或 s 开始 → 操作 → A 或 e 保存 → Y 回位**。
- 弃用一轮：**Y 接管 → X 或 s 开始 → 操作 → B 或 d 丢弃 → Y 回位**。
- 同一轮允许混合使用两种输入，例如 X 开始、e 保存，或 s 开始、B 丢弃。
- A/e 只保存当前未保存 episode；B/d 只清空当前未保存缓冲区，不删除已保存的历史 episode。
- 没有活动 episode 时按 A/e 或 B/d 不创建空 episode；录制中再次按 X/s 不清空或重开当前 episode。
- q 保持现有行为：退出录制器，若有尚未保存的有效缓冲区则保存。q 不等同于弃用。
- Y 与 episode 控制保持独立。现有代码不会在回位／故障时自动结束录制，本任务不默认引入该行为；操作上先 A/e 保存或 B/d 丢弃，再按 Y 回位。
- 完成后既可只用手柄管理 episode，也可继续使用键盘 s/e/d/q，且不影响已有 Y 接管／回位、扳机夹爪控制和 GELLO 键盘模式。后续右臂、双臂复用同一套键位及命令语义。
