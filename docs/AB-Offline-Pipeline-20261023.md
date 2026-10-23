# A/B 离线汇合与 Go/No-Go（2026-10-23）

## 当前结论

已经完成不依赖真机数据的工程部分：A/B 唯一接口、纯 Python `AdapterCore`、真实
A 线 IK 结果到 ROS adapter 的输入路径、无 ROS 的完整离线入口，以及 18 类故障注入。

当前结论仍是 **NO-GO**。原因不是软件接口仍未接通，而是正向放行需要用户后续提供
专门的小动作 Quest 录制和新的左臂只读状态；腕部 lever 标定、实时 measured/accepted
反馈、碰撞和控制器实际行为也只能在后续分级真机阶段确认。

本轮没有启动 ROS、fake hardware、GELLO、Quest、Franka 或夹爪进程，没有发布机器人命令。

## 基线文件清单

应进入 Git：

- `data_collection/fr3_ik/`：严格接口、求解器、DexArena 数学移植与既有离线工具。
- `data_collection/meta_quest/adapter_core.py` 和兼容导出 `joint_command_gate.py`。
- `data_collection/offline/ab_pipeline.py` 与两个 `scripts/validate_ab_*.py` 入口。
- `quest3_ros2/src/franka_vr_adapter/`、独立 build/start 脚本和 package README。
- 对应测试、当前状态文档和本报告。

不进入 Git：`recordings/quest_raw/`、`outputs/`、`logs/`、ROS `build/install/log`、
`__pycache__`、`.pytest_cache`、本机环境和 APK/归档文件。GELLO 继续保留为独立 submodule/
workspace，不复制进 `quest3_ros2`。

## 分步状态

| 步骤 | 状态 | 证据/边界 |
|---|---|---|
| 0 保护基线 | 完成（本地 + client） | client 已建立备份分支和 `/tmp` 归档后完成 A/B 合并；缓存、日志、录制和输出保持未跟踪/忽略；Quest ROS 与 GELLO ROS 仍是两个目录和入口 |
| 1 唯一接口 | 完成 | `fr3_ik/interfaces.py` 是唯一 `IKResult`；含 side、序号、连接、source/target/computed 时钟、状态、约束和 JSON round-trip |
| 2 纯 Python 门控 | 完成 | `meta_quest/adapter_core.py`；ROS wrapper 只反序列化、调用 core、发布/记录 |
| 3 DexArena 数学核对 | 完成离线代码核对 | DLS、精确零空间投影、统一速度缩放、限位 margin、绝对目标与可选自适应阻尼均保留；诊断型 `DiagnosticIKResult` 不能进入 adapter |
| 4 腕部偏移标定 | 待用户采集 | 需要左右手各自原地旋转实录；先做左侧 |
| 5 A/B 离线入口 | 完成 | `scripts/validate_ab_offline_pipeline.py`；只输出 `frames.jsonl`、`summary.json`、`manifest.json`、`README.md` |
| 6 专用小动作录制 | 待用户采集 | 见下方采集清单 |
| 7 正向验收 | 待第 6 步数据 | 脚本会保持 NO-GO，不以 Stage12 大动作数据冒充放行样例 |
| 8 故障注入 | 通过 | 18/18 异常均 `would_publish=false` |
| 9 Go/No-Go | 已形成 NO-GO | client 无硬件回归和独立 ROS package 构建已通过；待新数据后生成带输入哈希和统计的最终报告 |

## 接口与门控约束

- `data_collection/meta_quest/models.py` 只保留 Quest `TargetPose`，不再定义第二套 `IKResult`。
- `IKResult` 的 source、target、computed 时间戳必须显式携带 domain 与 clock id。
- 只有严格 `FOLLOWING`、全部约束为真、命令存在时才可能通过。
- 首条命令及每次显式 reanchor 后的首条命令必须等于最新 measured q。
- HOLD 不重复发布旧命令；FAULT/STOP 锁存，正常输入不能自动恢复。
- 断连、连接变化、序号重复/倒退/丢帧、时钟域不一致和左右臂错配均拒绝。
- GELLO 同臂发布者存在时拒绝 Quest 命令。
- ROS adapter 不再包含 synthetic target source，也不启动控制器或 fake hardware。

## 故障注入结果

命令：

```bash
python scripts/validate_ab_fault_injection.py
```

覆盖：Quest 数据过期、机器人状态过期、tracking 丢失、Quest 断连、目标跳变、IK
残差失败、关节范围、GELLO 冲突、左右臂错配、无效 measured state、重复序号、倒序、
丢帧、关节步长、关节速度、FAULT 后正常输入、NaN 和 Inf。18 项全部拒绝，且没有硬件发布。

## 用户后续采集清单

先只做左臂，机器人保持不动，只录 Quest：

1. 静止 3 秒。
2. 按既定锚定流程进入 FOLLOWING。
3. X 正方向约 5 mm，再回中心。
4. Y 正方向约 5 mm，再回中心。
5. Z 正方向约 5 mm，再回中心。
6. 触发 HOLD；暂不做大角度旋转。
7. 全程保持单一 TCP connection id；断连则整条重录。

同时补一份新的左臂只读 snapshot，记录采集主机时间、关节名/顺序、q、TCP、topic、
Git commit 和实际启动命令。不要启动 GELLO，不发送运动命令。

拿到两份输入后运行：

```bash
python scripts/validate_ab_offline_pipeline.py RECORDING.jsonl \
  --initial-snapshot SNAPSHOT.json --side left --output outputs/ab_offline/RUN_NAME
```

只有正向小动作全部满足 1 mm / 0.002 rad、首帧为 measured q、非法接受数为 0，且上述
18 项故障注入仍全部拒绝，才能把报告从 NO-GO 改为“允许进入真机第 1 级只读检查”。
