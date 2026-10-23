# DexArena B 线路实施计划

更新日期：2026-10-21。

本文保留 B 线实施步骤。最新跨线状态见 [A 线交接文档](A线-当前状态与下一步.md)：A 线已实现双臂离线 IK，B 线当前主要验收合成输入与右臂框架，真实 IK 和双臂联合接入仍待完成。

## 目标和边界

B 线路负责把“已经检查过的七关节目标”安全地送进 ROS 2，并先在 fake hardware 上完成闭环验证。

目标链路：

```text
合成七关节目标（前期替代真实 IK）
  → VR adapter 的检查和 watchdog
  → /right/vr/raw_joint_states
  → 现有 FR3 joint impedance controller
  → /right/vr/accepted_joint_states
  → /right/joint_states（fake measured state）
```

本线路不会启动真实 Franka，不会修改 GELLO 的默认 topic，也不会把未验收的 IK 输出连接到真机。

本机的 `DexArena` 仓库是已跑通的仿真参考，主要用于核对关节顺序、控制数据语义和后续 Sharpa/MANUS 接口。B 线路本身运行在 client 的 `real-exp-vr` 工程中，但使用独立的 `quest3_ros2` 工作区；GELLO 继续使用 `gello_software/ros2`。两个输入链分别构建和启动，不能直接照搬 ManiSkill 仿真控制器。

## 初学者需要知道的三个 topic

| topic | 含义 | 谁发布 |
|---|---|---|
| `/right/vr/raw_joint_states` | VR/IK 希望机器人到达的七关节目标；只有通过上层检查的目标才能进入这里 | VR adapter |
| `/right/vr/accepted_joint_states` | FR3 controller 真正接收的目标；它是后续数采应记录的 action | FR3 controller |
| `/right/joint_states` | fake hardware 或真实机器人反馈的实测关节状态 | ROS controller stack |

`requested`、`accepted` 和 `measured` 三层都保留，出现问题时才能判断是上层没有发、控制器拒绝了，还是机器人没有跟上。

## 分步实施

### 第 0 步：冻结范围并保存基线

工作：

- 记录 client 当前分支、提交和未提交文件。
- 记录 GELLO 当前 topic、启动入口和控制器参数。
- 记录 B 线路只允许 fake hardware，禁止真实机器人输出。
- 所有开发先在本机隔离工作树 `.b-line-work` 完成，验证后再同步指定文件到 client。

产出：

- 本计划文件 `B-Line-Implementation-Plan.md`。
- 后续验收报告中的 baseline 小节。

验收：现有 client 用户文件没有被覆盖，GELLO 子模块没有被修改。

### 第 1 步：Python 3.10 和 A/B 接口契约

工作：

- 将 Python 3.11 才有的 `enum.StrEnum` 改为 Python 3.10 可用的等价写法。
- 定义 `IKResult`：七关节位置、成功/失败、失败原因、残差、耗时、限位结果、来源时间、连接 ID 和序号。
- 实现不依赖 ROS 的 `JointCommandGate`，先检查输入是否新鲜、关节顺序、NaN、限位、步长、速度和命令源冲突。
- 明确 `HOLD` 和 `STOP` 都停止继续发布旧命令，让下层 controller watchdog 减速停止；二者的恢复条件不同。

修改文件：

- `data_collection/meta_quest/state_machine.py`
- `data_collection/meta_quest/models.py`
- `data_collection/meta_quest/__init__.py`

新增文件：

- `data_collection/meta_quest/joint_command_gate.py`
- `test/test_vr_joint_command_gate.py`

补充测试：

- `test/test_meta_quest_models.py`

验收：

- client 的 ROS Python 3.10 可以导入状态机和接口类型。
- 原有 Meta Quest 测试继续通过。
- 故障注入测试对每一种拒绝给出明确原因。

### 第 2 步：建立独立的 ROS VR adapter package

工作：

- 在独立的 `quest3_ros2` 工作区新增 `franka_vr_adapter` ROS 2 Python package。
- adapter 订阅 `/right/joint_states` 和 `/right/vr/accepted_joint_states`。
- adapter 只把通过 `JointCommandGate` 的结果发布到 `/right/vr/raw_joint_states`。
- 前期使用小幅合成关节轨迹，真实 IK 后续在汇合点替换进去。
- 增加 `/right/vr/status`，报告当前状态和拒绝原因。

计划新增文件：

- `quest3_ros2/src/franka_vr_adapter/package.xml`
- `quest3_ros2/src/franka_vr_adapter/setup.py`
- `quest3_ros2/src/franka_vr_adapter/setup.cfg`
- `quest3_ros2/src/franka_vr_adapter/resource/franka_vr_adapter`
- `quest3_ros2/src/franka_vr_adapter/franka_vr_adapter/adapter_node.py`
- `quest3_ros2/src/franka_vr_adapter/franka_vr_adapter/__init__.py`
- `quest3_ros2/src/franka_vr_adapter/test/test_synthetic_target_source.py`
- `quest3_ros2/validate_step2.sh`

验收：package 可以由系统 Python 3.10 构建和导入；没有 ROS 环境时，纯 Python gate 测试仍可独立运行。

### 第 3 步：VR 专用 controller 配置和 fake hardware 启动入口

工作：

- 新建 VR controller override，只覆盖 command 和 accepted topic。
- 复用现有 FR3 controller、限位、1 kHz reference generator 和 command timeout。
- 新建 Quest 3 独立启动脚本 `scripts/start_quest3_teleoperation_client.sh`，本阶段只接受 `--fake`。
- 新建 Quest 3 独立构建脚本 `scripts/build_quest3_ros2.sh`，不修改或复用 GELLO 的构建脚本。
- 与 GELLO 使用同一个进程锁；同时在 ROS graph 检查 `/right/gello/raw_joint_states` 是否有 publisher。
- 不改现有 `scripts/start_teleoperation.sh` 的默认行为。

计划新增或修改文件：

- `quest3_ros2/src/franka_vr_adapter/config/vr_controller.yaml`
- `quest3_ros2/src/franka_vr_adapter/launch/right_fake.launch.py`
- `scripts/start_quest3_teleoperation_client.sh`
- `scripts/build_quest3_ros2.sh`

验收：

- `/right/vr/raw_joint_states` 和 `/right/vr/accepted_joint_states` 类型都是 `sensor_msgs/msg/JointState`。
- GELLO topic 不出现 VR publisher。
- fake launch 中没有真实 `robot_ip` 连接。

### 第 4 步：watchdog、故障注入和三层记录

依次测试：

1. 停止合成输入。
2. 停止机器人状态。
3. 发送旧时间戳。
4. 发送错误关节顺序。
5. 发送越限关节。
6. 发送 NaN。
7. 模拟 IK failure。
8. 模拟 GELLO 已启动。

每个测试都必须满足：不再发布新 raw target、状态 topic 给出原因、controller 在 timeout 后停止推进、恢复时不追赶旧目标。

计划产出：

- `logs/vr_fake_<timestamp>.jsonl`：requested、accepted、measured 和状态事件。
- `outputs/vr_fake_fault_injection_<timestamp>.json`：每项故障的通过/失败结果。

### 第 5 步：fake hardware 闭环验收

工作：

- 从 fake measured state 原地接管，首帧目标等于当前关节状态。
- 发送厘米级动作对应的小幅合成关节轨迹。
- 对比 requested、accepted 和 measured。
- 检查退出 adapter 后 controller timeout 行为。

验收：

- 首帧无关节跳变。
- accepted 与 requested 顺序和数值一致。
- measured 数据连续、无 NaN。
- 节点退出后不再有新目标。
- 全程没有真实机器人输出。

产出：

- `outputs/vr_fake_validation_<timestamp>.json`。
- 一份可复查的 JSONL trace。

### 第 6 步：GELLO 回归

工作：

- 关闭 VR 后运行原 GELLO 启动入口的 dry-run/现有测试。
- 核对原 topic、配置文件和默认参数未变。
- 验证同时启动 VR 与 GELLO 时第二个入口会被拒绝。

验收：原 GELLO 默认链路不受影响；VR 与 GELLO 不能同时成为右臂命令源。

### 第 7 步：同步 client 和更新 Obsidian

只有前六步通过后：

- 将明确列出的代码文件同步到 `/home/landau/real-exp-vr`。
- 在 client 重新构建、重新跑测试和 fake 验收。
- 更新 Obsidian 的 DexArena 规划，记录 commit、命令、输出文件和仍未完成的真实 IK 汇合点。

本步骤仍不会进入真实 Franka 控制。

## 每一步的汇报格式

每完成一步，向用户报告：

1. 这一步解决了什么问题。
2. 实际执行了哪些命令或测试。
3. 新增、修改了哪些文件。
4. 生成了哪些日志或报告，给出完整路径。
5. 哪些检查通过，哪些还没通过。
6. 下一步准备做什么。

## 当前状态

第 0–6 步已在隔离/临时运行目录完成既定验收，报告归档至 [历史索引](z_histroy_plan/README.md)。这些验收以合成关节输入为主，没有验收当前 A 线真实 IK 的联合链。

- Python 3.10 兼容、B 线 IKResult/gate、独立 ROS package、右臂 fake launch、watchdog、8 类故障注入和无硬件 GELLO 回归已有证据。
- 合成输入 trace 的 requested/raw/accepted 531 组一致。effort mock 的 measured state 静止，验证了传输、数据连续性和超时，未验证真实动力学。
- 第 7 步的源码、测试和报告同步已经完成，代码位于当前项目。记录见 [Step7](B-Line-Step7-Sync.md)；不再重复安排文件同步。

仍待完成：

- [ ] 在正式 `quest3_ros2` 目录重新构建、测试和 fake 验收，先核对现场进程与 ROS 隔离条件。
- [ ] 对齐 A/B 两种 IKResult、时间域、反馈 freshness 与 HOLD/STOP 责任，并以真实 IK 替换合成输入接入点。
- [ ] 扩展并验证双臂 launch、topic、各臂命令源互斥和共享控制事件。
- [ ] 完成 Quest JSONL → 真实 IK → adapter → fake controller 的 requested/accepted/measured 联合验收。
- [ ] 联合接入后完成真机前 GELLO 最终回归。
- [ ] 按后续任务安排更新 Obsidian 与提交记录；本次文档整理不执行这些工作。

A 线当前优先处理接管起步与执行约束，详见 [交接文档第 5 节](A线-当前状态与下一步.md#5-当前未完成下一轮按此顺序推进)。以上未完成项不表示已获真机运动授权。
