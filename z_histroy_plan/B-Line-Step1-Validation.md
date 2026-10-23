> 历史归档：2026-10-21。本文保留原记录的时间语境，不作为当前进度或启动指令。最新工作入口见 [A 线当前状态与下一步](../A线-当前状态与下一步.md)；代码块中的项目相对路径仍以项目根目录为基准。

# B 线路第 1 步验收报告

日期：2026-10-20。

## 本步目标

验证 ROS Humble 的 Python 3.10 能复用 Meta Quest 状态机，并冻结 A 线向 B 线提供的七关节 `IKResult` 接口。在 ROS 发布之前，用纯 Python gate 拒绝陈旧、越限、不连续或来源冲突的关节目标。

## 实际改动

- `data_collection/meta_quest/state_machine.py`
  - 将 Python 3.11 的 `StrEnum` 改成 Python 3.10 可用的 `str, Enum`。
  - 保留 `str(state) == state.value` 的原有行为。
- `data_collection/meta_quest/models.py`
  - 新增不可变的 `IKResult`。
  - 强制七个关节、有限数值、递增时间语义、明确成功/失败原因和安全检查结果。
- `data_collection/meta_quest/__init__.py`
  - 导出 `IKResult`。
- `data_collection/meta_quest/joint_command_gate.py`
  - 新增关节顺序、位置限位、步长、速度、输入超时、IK 超时、机器人状态超时、连接切换和命令源冲突检查。
  - `HOLD` 与 `STOP` 都不重复发布旧命令，交给 controller watchdog 做有界停止。
- `test/test_meta_quest_models.py`
  - 新增 `IKResult` 合约测试。
- `test/test_vr_joint_command_gate.py`
  - 新增正常目标与故障注入测试。

## 测试结果

测试在 client 的临时目录 `/tmp/dexarena-b1-validation-20261020` 运行，没有修改 `/home/landau/real-exp-vr`。

- 新接口与 gate 专项测试：`31 passed in 0.22s`。
- client 原有 Meta Quest 测试加新 gate 测试：`83 passed in 6.67s`。
- `git diff --check`：通过。
- 本机 Python 3.12 compileall：通过。
- client ROS Python：`3.10.12`。
- client ROS Python 导入：`rclpy`、`TeleoperationStateMachine`、`IKResult`、`JointCommandGate` 全部通过。
- Python 3.10 实际 gate 检查：
  - 合法七关节目标：`accepted`。
  - 模拟 GELLO 命令源冲突：`command_source_conflict`，拒绝发布。

固定的命令关节顺序为：

```text
fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4,
fr3_joint5, fr3_joint6, fr3_joint7
```

## 当前边界

- 改动只存在于本机隔离工作树 `.b-line-work`。
- client 项目目录尚未同步这些文件。
- 尚未创建 ROS package 或 `/right/vr/*` topic。
- 尚未启动 fake hardware。
- 没有真实机器人输出。

## 下一步

第 2 步只建立 `franka_vr_adapter` ROS 2 package，让系统 Python 3.10 构建并启动一个等待机器人状态的节点。该步先不启动 FR3 controller，也不发送关节目标。
