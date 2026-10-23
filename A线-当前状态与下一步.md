# A 线：当前状态与下一步

更新日期：2026-10-23。项目：`/home/landau/real-exp-vr`。

本文是 A 线唯一的当前状态入口。详细问题和本次测试证据见 [A 线离线可行性复核](docs/A-Line-Offline-Feasibility-Review-20261023.md)，历史过程见 [归档索引](z_histroy_plan/README.md)。

## 当前结论

A 线已经具备完整的离线原型：

```text
Quest Touch
  → ALVR / SteamVR / OpenVR
  → TCP / JSONL
  → 坐标映射与接管状态机
  → 双臂 FR3 IK
  → 受限关节参考
  → FK / HTML 诊断与只读实时预览
```

离线方案可行，但当前输出不能进入真机控制：

- 最新 V4/Stage12 仍为 `DIAGNOSTIC_ONLY`、`control_eligible=false`；
- A/B 两套 `IKResult` 尚未统一；
- 离线回放没有真实 measured/accepted feedback 闭环；
- 严格残差、20 ms 长尾、碰撞和真实停止行为尚未通过；
- 本轮不继续向 fake 或真机推进。

## 已确认的约定

- left / `172.16.0.3` 对应左手；right / `172.16.0.2` 对应右手，不交换通道。
- V4 桌面轴为 `+X 前、+Y 左、+Z 上`。
- 用户已确认实时虚拟预览的左右、主要方向和接管观感基本正确。
- Stage12 三轴实录支持上述轴符号。约 10°偏航只是候选解释，数据不一致，未作为标定值应用。
- 默认双臂共享右手 B 接管/停止；显式左臂模式使用左手 X。
- `record`、`clutch`、trigger 是不同字段。当前 trigger 没有形成夹爪控制闭环。

## 本次离线复核

2026-10-23 在 client 的项目实际 ROS/Franka 环境中运行 A 线选定测试：

```text
109 passed, 1 xacro compatibility warning, 119.09 s
```

通过内容包括 Quest 输入/坐标/安全状态机、IK 接口与约束、严格离线求解、A3 回放、V4 映射、DexArena 数学回归、FR3 模型审计和实时只读预览引擎。

环境结论：client 没有创建名为 `ms3` 的 Conda 环境。`ms3-enviroment.yaml` 是来自 `/home/pair/miniconda/envs/ms3` 的快照。A 线验证应使用 `scripts/validate_fr3_ik_offline.sh` 所采用的 ROS Humble、`franka_ros2_ws` 和系统 Python，不要混用 `lerobot-vr`。

## 唯一保留的离线示例

历史 `outputs/fr3_ik` 曾约 2.2 GB。本轮只保留一组最新、真实录制驱动的示例：

- [Stage12 示例入口](outputs/fr3_ik/stage12-direction-audit-20261022/index.html)
- [结果与边界](outputs/fr3_ik/stage12-direction-audit-20261022/README.md)
- [方向曲线](outputs/fr3_ik/stage12-direction-audit-20261022/direction-traces.svg)
- [双臂 V4 回放](outputs/fr3_ik/stage12-direction-audit-20261022/replay/playback.html#t=15&view=rear-right)

这个示例证明输入完整性、轴方向和离线消息处理，不证明真机可控。Stage12 最大模型位置滞后约为左 38.3 mm、右 77.0 mm，严格残差失败左 1139 帧、右 1134 帧。

原始 Quest 录制和机器人状态快照保留在 `recordings/`；删除的是可重新生成的重复输出。

## 可行性分层

| 层级 | 状态 | 含义 |
|---|---|---|
| Quest 输入与原始记录 | 可用 | 真实 OpenVR 输入、连接和 lossless JSONL 已验证 |
| 坐标、接管与故障保持 | 离线可用 | 通道、轴符号、首帧锚定和主要故障有测试 |
| FR3 IK 与受限参考 | 诊断可用 | 关节约束通过，但残差和实时性没有达到控制门槛 |
| 实时网页预览 | 可用 | mock 和只读反馈，无命令发布路径 |
| A → B ROS adapter | 未完成 | 类型、时间、feedback、HOLD/FAULT 语义未闭合 |
| fake 联合验收 | 未完成 | 历史 B 线 synthetic 通过，真实 A 线 IK 尚未接入 |
| 真机运动 | 未开始 | 当前没有授权，也不具备完整门槛 |

## 主要问题

1. A 线 `data_collection/fr3_ik/interfaces.py` 与 B 线 `data_collection/meta_quest/models.py` 各有一套 `IKResult`。
2. A3 离线输出的 `measured_joint_state` 为 `null`，没有 controller accepted target 回灌。
3. Stage11/12 的受限追赶存在严格残差失败，大动作滞后明显。
4. 双臂 step 最大约 44.5 ms，出现超过 20 ms 的长尾。
5. 当前约束不包括自碰撞、双臂互碰、桌面、夹爪和物体碰撞。
6. OpenVR 的 device/frame 信息由主机轮询生成，不能单独证明设备新帧；A/B 时间域仍需显式转换。
7. 腕部旋转、物理偏航、夹爪、episode 边界和真实停止都没有完成同等级验收。

## 代码入口

- `data_collection/meta_quest/`：输入、协议、坐标和安全状态机。
- `data_collection/fr3_ik/interfaces.py`：A 线严格类型和状态语义。
- `data_collection/fr3_ik/kinematics.py`：FR3/TCP 模型。
- `data_collection/fr3_ik/solver.py`：严格 IK 与离线诊断跟随。
- `data_collection/fr3_ik/a3_replay.py`：双臂离线回放。
- `data_collection/fr3_ik/live/`：无命令路径的实时只读预览。
- `scripts/replay_fr3_ik_a3.sh`：离线回放入口。
- `scripts/validate_fr3_ik_offline.sh`：正确环境下的离线验证入口。

## 恢复开发时的顺序

本轮停在 Git 快照，不执行以下任务。以后恢复时按顺序处理：

1. 冻结 A/B joint、状态、时间戳和失败/HOLD 契约。
2. 接入新鲜 measured state 与 controller accepted target。
3. 明确控制周期、残差、停止、恢复和碰撞验收标准。
4. 让真实 A 线 IK 进入隔离的左臂 fake，并覆盖断流/故障/退出。
5. 完成 GELLO 回归后，再制定真机小范围试验。

任何旧 HTML、synthetic/fake 报告或只读预览都不能替代上述门槛。
