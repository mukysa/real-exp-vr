> 历史归档：2026-10-21。本文保留原记录的时间语境，不作为当前进度或启动指令。最新工作入口见 [A 线当前状态与下一步](../A线-当前状态与下一步.md)；代码块中的项目相对路径仍以项目根目录为基准。

# DexArena B 线路第 4 步验收

验收日期：2026-10-20。

## 本步目标

为 Quest 3 adapter 增加持续 watchdog、fake-only 故障注入、安全恢复和三层轨迹
记录。所有动态故障都必须停止新 raw target，使 controller 的 0.5 秒 watchdog
有界停止；解除故障后从最新 measured state 重新锚定。

## 实现内容

### 持续安全检查

- adapter 在运行期间持续检查 `/right/gello/raw_joint_states` 的 publisher 数量。
- 出现 GELLO publisher 时进入 `STOP / command_source_conflict`，停止 raw 发布。
- publisher 消失后，以最新 measured state 重新锚定再恢复。
- `JointCommandGate.reanchor_command_history()` 清除旧目标历史，使恢复首帧与最新
  robot state 比较。

### 启动状态稳定窗口

首次验收时，安全门发现 ROS 启动早期的 URDF 默认关节值与随后 fake measured
state 相差过大，并以 `joint_step_limit` 拒绝。未放宽限位，而是增加：

```text
robot_state_settle_s = 0.5
robot_state_settle_tolerance_rad = 0.01
```

只有 measured state 连续稳定 0.5 秒才建立初始锚点，避免把启动瞬态当成命令
基准。

### 故障注入边界

- 默认 `enable_fault_injection=false`。
- 普通启动尝试设置 `fault_mode=nan` 时被拒绝：
  `fault injection is disabled for this launch`。
- 只有 fake 验收 launch 显式设置 `enable_fault_injection=true` 才能使用故障模式。

### 三层记录

adapter 可通过 `trace_path` 写 JSONL，记录：

- `requested`：安全门之前的请求。
- `raw`：安全门允许并发布给 controller 的目标。
- `accepted`：controller 实际接受的目标。
- `measured`：fake hardware 实测关节状态。
- `decision`、`status`、`fault_mode`、`reanchor` 和非法输入事件。

## client 临时验收环境

正式工程 `/home/landau/real-exp-vr` 没有修改。测试副本位于：

```text
/tmp/dexarena-b4-validation-20261020
```

使用独立 ROS domain 96，启动 fake hardware、FR3 impedance controller 和 adapter。
测试完成后所有 ROS 进程均已正常停止。

## 故障结果

| 故障 | 状态原因 | raw 停止 | controller 稳定 | 恢复重锚定 | 结果 |
|---|---|---:|---:|---:|---:|
| 停止合成输入 | `synthetic_input_stopped` | 是 | 是 | 是 | 通过 |
| 停止接收 robot state | `robot_state_stale` | 是 | 是 | 是 | 通过 |
| 旧输入时间戳 | `input_stale` | 是 | 是 | 是 | 通过 |
| 错误关节顺序 | `joint_order_mismatch` | 是 | 是 | 是 | 通过 |
| 关节越限 | `joint_position_limit` | 是 | 是 | 是 | 通过 |
| NaN | `ik_result_invalid:positions_non_finite` | 是 | 是 | 是 | 通过 |
| IK failure | `ik_failure:injected_failure` | 是 | 是 | 是 | 通过 |
| 运行中出现 GELLO publisher | `command_source_conflict` | 是 | 是 | 是 | 通过 |

机器可读结果为 `all_passed=true`。8 项故障的 quiet window 中 raw 和 accepted
数量都不再增加，controller 最终命令跨度均为 `0.0 rad`。恢复首条 raw 相对最新
measured state 的最大误差为 `7.926227796999364e-06 rad`。

## Trace 验证

正式 trace 共 3950 行、1,079,138 字节。严格 JSON 解析和有限数值检查通过：

```text
requested           729
requested_invalid    25
decision            757
raw                 573
accepted            573
measured           1191
status               79
fault_mode           14
reanchor              8
initial_anchor         1
```

所有必需事件均存在。raw 与 accepted 的总数量一致。

## 本地输出文件

- `logs/vr_fake_20261020_step4.jsonl`
- `outputs/vr_fake_fault_injection_20261020_step4.json`
- `outputs/vr_fake_trace_summary_20261020_step4.json`

client 临时副本：

```text
/tmp/dexarena-b4-validation-20261020/logs/vr_fake_20261020_step4.jsonl
/tmp/dexarena-b4-validation-20261020/outputs/vr_fake_fault_injection_20261020_step4.json
```

## 测试汇总

- adapter package 测试：`6 passed`。
- `JointCommandGate` 专项测试：`15 passed`。
- 8 项运行时故障注入：全部通过。
- trace 严格 JSON、有限数值、事件覆盖：通过。
- Python `compileall` 和 `git diff --check`：通过。
- GELLO 源码目录：无改动。
- 真实机器人：未连接、未启动。

## 下一步

第 5 步进行较长时间的 fake hardware 闭环验收，正式比较 requested、accepted 和
measured 的连续性、首帧接管、跟踪误差及 adapter 退出后的 controller timeout。
