# DexArena / 左臂 ROS 与真机交接

更新日期：2026-10-23。项目：`/home/landau/real-exp-vr`。

## 当前结论

**ROS/fake 的通用右臂链已经在隔离临时工程完成 B4/B5 验收；正式 `quest3_ros2` 已在 2026-10-22 构建成功，但同步后的 package 测试和左臂停止/收敛仍没有有效通过证据。2026-10-23 A 线离线复核通过 109 项测试，但严格残差、真实反馈和 A/B 契约尚未闭合，因此还不能进入真机运动。**

本交接针对的当前目标是左臂 `left / 172.16.0.3`。本次没有连接或运动真实机器人；右臂 `right / 172.16.0.2` 不属于本轮操作范围。用户已表示可将左臂 FCI 关闭并锁定；后续解除锁定和真机运动须按现场步骤单独确认。

## 按规划阶段核对

| 阶段 | 状态 | 依据与边界 |
|---|---|---|
| 0：GELLO 与双机基线 | 已完成 | 按 PDF 和项目状态记录，双臂 GELLO 采集、校验、回放已有用户确认。 |
| 1：Quest 手柄输入 | 已完成 | ALVR/SteamVR → OpenVR → TCP/JSONL 有采集证据。 |
| 2：坐标、锚定与离线安全 | 已完成 | 坐标转换、状态机和离线安全回放已有验收。 |
| 3：FR3 离线 IK | 部分完成 | A1 运动学/工具核对已完成，既有 IK 核心和回放已有大量测试；但当前新版追赶结果仍标记 `DIAGNOSTIC_ONLY`、`control_eligible=false`，残差/跟随验收尚未通过。不能把已有可视化或历史旧模式等同于当前 IK 已可执行。 |
| 4：ROS 隔离 topic 与 fake | 通用右臂 B4/B5/B6 已通过历史验收；正式构建已通过，针对性测试待做 | 第 7 步源码同步已做；`franka_vr_adapter` 最近构建返回码为 0。左臂是新增适配，左臂停止与收敛未验收。 |
| 5：Quest 控制 Franka 真机 | 未开始 | 仍需先完成 A/B 联合门槛，再按现场流程分步验收。 |
| 6–9：MANUS/Sharpa、联合遥操作、数采、任务验收 | 未开始/后续 | 不属于当前左臂 ROS/fake 收尾范围。 |

PDF 原版阶段 3/4 以右臂为例；项目后来明确先做左臂单臂。左右臂通道不交换，右臂既有通过结果不自动证明左臂专用配置通过。

## 已由同事完成的通用 ROS/fake 验收

以下是**真实存在的历史验收**，在独立临时副本和 fake hardware 上完成，不是本轮重新跑出的结果：

- **B4：watchdog/故障注入**。8 类故障全部通过，包括输入停止、机器人反馈过期、旧时间戳、错误关节顺序、越限、NaN、IK failure 和 GELLO 命令源冲突；故障时 raw/accepted 停止，恢复时重新锚定。报告记录 `all_passed=true`。
- **B5：fake 闭环**。12 秒轨迹加 2 秒退出观察；requested/raw/accepted 各 531 对一致，adapter 退出后 raw/accepted 没有新增目标，两个 controller 保持 active；报告记录 `all_passed=true`。
- **B6：GELLO 无硬件回归**。GELLO 默认 topic/入口、脚本、互斥锁和模拟硬件测试已有通过记录。
- 历史 B4/B5 使用右臂 fake；effort mock 的 measured 状态静止，所以通过的是传输、安全 gate、退出静默和控制器参考，不是机械臂动力学跟踪。

证据入口：

- [B4 故障验收记录](z_histroy_plan/B-Line-Step4-Validation.md)
- [B5 闭环验收记录](z_histroy_plan/B-Line-Step5-Validation.md)
- [B6 GELLO 回归记录](z_histroy_plan/B-Line-Step6-Validation.md)
- [第 7 步正式目录同步记录](B-Line-Step7-Sync.md)
- [B 线总计划及当前待办](B-Line-Implementation-Plan.md)
- [项目 A 线当前状态](A线-当前状态与下一步.md)

**因此不需要重写或从头重跑 B4/B5 的通用右臂工作。**需要的是核对同步后的正式目录，并只对同步后改动和左臂扩展做针对性验证。

## 左臂新增工作与本轮证据

当前正式目录新增/调整的左臂入口和适配包括：

| 文件 | 内容 |
|---|---|
| `quest3_ros2/src/franka_vr_adapter/launch/left_fake.launch.py` | 左臂 namespace、prefix、fake hardware 和 `/left/...` topic 配置 |
| `scripts/start_quest3_left_fake.sh` | 显式 `--fake` 启动入口；synthetic 默认关闭 |
| `quest3_ros2/src/franka_vr_adapter/franka_vr_adapter/adapter_node.py` | GELLO 冲突 topic 参数化 |
| `quest3_ros2/src/franka_vr_adapter/franka_vr_adapter/closed_loop_runner.py` | 增加 namespace 选择，支持订阅左臂采样流 |
| `data_collection/meta_quest/joint_command_gate.py` | joint name canonicalization 支持 `left_` 前缀，并保留右臂行为 |

左臂运行时 topic：measured `/left/joint_states`；raw `/left/vr/raw_joint_states`；accepted `/left/vr/accepted_joint_states`；controller reference `/left/franka/commanded_joint_states`；status `/left/vr/status`；GELLO conflict `/left/gello/raw_joint_states`。

本轮实际启动过左臂 fake hardware，日志确认 fake hardware、`joint_state_broadcaster` 和 `joint_impedance_controller` 激活；一次 5 秒 synthetic 采样得到 raw/accepted/measured/commanded 分别 **99/99/146/489** 条，窗口内 raw 与 accepted 完全一致，controller reference 变化约 **0.01998 rad**。这证明数据传输和控制器参考更新。

**本轮左臂 runner 的 `all_passed=false`，不能作为 B4/B5 重复验收失败，也不能当作左臂停止通过。** 停止脚本没有正确命中 adapter；报告里的空 post-exit 计数不是有效静默窗口。控制器收敛也没有有效通过。本轮正式目录构建卡住后被中断，pytest 命令因写了不存在的测试路径而没有运行测试。DDS 在沙箱中有网卡/socket 权限错误。相关失败诊断材料只供排查：

- [左臂未通过诊断报告](docs/handoff/left-ros-fake-2026-10-22/left-closed-incomplete.json)
- [左臂 runner 日志](docs/handoff/left-ros-fake-2026-10-22/left-runner.log)

它们不能覆盖历史 B4/B5 通过结论，也不应被引用为有效左臂停止验收。

## 下一步：按这个顺序推进

### 1. 先做正式目录的窄范围复核，不重复通用 B4/B5

检查当前正式工作区和源码差异，核对 Step7 同步内容、左臂 launch/topic/prefix、共享 gate 左右臂行为；复用 2026-10-22 的成功构建，只补 package/相关离线测试。不要再跑一套泛化的右臂 B4/B5 全量验收，除非正式目录差异或回归失败提供了理由。fake 运行应使用独立 ROS domain，并避开现场已有进程。

### 2. 先把 A 线 IK 输出变成可接入的严格结果

依照 [A 线当前状态](A线-当前状态与下一步.md)完成当前左臂策略的控制验收：使用真实 measured state 锚定；验证首帧无跳变、连续轨迹 FK 残差、关节位置/速度/加速度、不可达和突变时 HOLD；明确失败原因和求解耗时。`DIAGNOSTIC_ONLY` 或 `control_eligible=false` 的输出不得送到 ROS raw topic。对左臂录制仅处理 `side=left`，保留原始 JSONL 并验证哈希。

### 3. 冻结并打通 A/B 接口，再做左臂 fake 联合验收

对齐 A 线 `IKResult` 与 B 线 `IKResult` 字段、关节名/顺序/单位、来源时间戳与连接 ID、失败/HOLD 语义。让 adapter 消费已检查的真实 IK 结果；synthetic 保留为隔离测试输入，默认关闭。fake 运行覆盖 left raw → accepted → measured/reference、首帧锚定、输入停止、反馈过期、IK failure/NaN/越限、GELLO 冲突、恢复重锚定及 adapter 退出后的 watchdog。runner 必须确认 adapter 确实退出并采集非空后置窗口，才可验收静默/收敛。

### 4. 完成真机前 GELLO 回归，然后进入阶段 5 现场流程

复用 B6 已通过证据；只在真实 IK/左臂整合造成相关共用代码变化时补针对性 GELLO 回归。之后按 PDF 阶段 5：由现场操作者确认急停和低速/低增益条件，先只读确认 FCI、状态、fresh measured state 和工具配置，再从固定姿态接管与小范围分轴动作开始，逐项验收保持、停止、断流、不可达、工作空间限制和重新接管。右臂保持不操作。真机归位属于运动行为，须独立定义和验收，不能从虚拟预览动画推定安全。

## 真机门槛与当前禁止误读

进入阶段 5 前至少需要：A 线严格 IK 对左臂数据通过；A/B 接口契约通过；左臂 fake 联合链和故障/停止行为有可信报告；GELLO 不会与 VR 同臂并发；现场只读状态与急停准备完成。fake 静态 measured 只能证明消息链，不能证明真实机械臂动态跟踪。

当前不需要用户戴头显、启动 FCI 或操作机器人。当前最合理的技术下一步是**正式目录针对性构建/测试 + A 线严格 IK 与 adapter 接口闭合**；通用右臂 B4/B5 已通过，不要再重复修它。
