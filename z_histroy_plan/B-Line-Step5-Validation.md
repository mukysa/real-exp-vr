> 历史归档：2026-10-21。本文保留原记录的时间语境，不作为当前进度或启动指令。最新工作入口见 [A 线当前状态与下一步](../A线-当前状态与下一步.md)；代码块中的项目相对路径仍以项目根目录为基准。

# DexArena B 线路第 5 步验收

验收日期：2026-10-20。

## 本步解决的问题

本步验证 Quest 目标从安全门到现有 FR3 controller 的完整 fake hardware 命令链，
并检查 Quest adapter 退出后是否停止产生新目标。验证只使用 fake hardware，没有
连接或启动真实 Franka。

数据链分为四层：

```text
requested（合成 IK 请求）
  → raw（安全门允许发布）
  → accepted（controller 实际接收）
  → measured（fake hardware 反馈）
```

## 新增实现

新增 `closed_loop_runner.py`，它订阅并统计以下 topic：

- `/right/vr/raw_joint_states`
- `/right/vr/accepted_joint_states`
- `/right/joint_states`
- `/right/franka/commanded_joint_states`

runner 先记录 12 秒正常轨迹，再等待 adapter 的命令流停止，继续观察 2 秒。它会
检查关节顺序、有限数值、消息连续性、raw/accepted 一致性、首帧接管、adapter
退出后的静默，以及 controller 参考值是否稳定。

## client 临时验收环境

正式工程 `/home/landau/real-exp-vr` 没有修改。构建和运行副本位于：

```text
/tmp/dexarena-b5-validation-20261020
```

使用独立 ROS domain 106，启动参数固定为 fake hardware。测试中只结束了该临时
目录下的 adapter 进程；随后确认：

- `joint_impedance_controller` 仍为 `active`。
- `joint_state_broadcaster` 仍为 `active`。
- adapter 进程已经退出。
- 验收完成后整个临时 ROS 栈已经停止。

## 验收结果

机器可读结果为 `all_passed=true`。

| 检查 | 结果 | 证据 |
|---|---:|---|
| 首帧原地接管 | 通过 | 第一条 raw 与 initial anchor 最大误差 `0.0 rad` |
| requested → raw | 通过 | trace 531 对，最大误差 `0.0 rad` |
| raw → accepted | 通过 | trace 531 对，最大误差 `0.0 rad` |
| 关节顺序 | 通过 | raw 和 accepted 都是 `fr3_joint1..7` |
| 数值有效 | 通过 | 四条数据流均为有限数值，无 NaN/Inf |
| measured 连续 | 通过 | 361 个测量样本，最大间隔约 `0.0489 s` |
| controller 参考轨迹 | 通过 | 1200 个样本，跨度约 `0.019997 rad` |
| adapter 退出后 raw | 通过 | 2 秒观察窗新增 `0` 条 |
| adapter 退出后 accepted | 通过 | 2 秒观察窗新增 `0` 条 |
| controller 保持运行 | 通过 | 两个 controller 均保持 `active` |

正常测量窗的 topic 样本数为：raw 240、accepted 240、measured 361、controller
commanded 1200。在线订阅开始边界允许丢失一个边界样本；本次完整 JSONL trace
中 requested、raw、accepted 各 531 条，三层逐项完全一致。

## fake measured 的限制

本次使用的 ROS `mock_generic_system` 在当前 effort controller 配置下发布静态
measured joint state。测量数据连续、有限且最大步长为 0，但它不会随 effort 输出
模拟真实机械臂动力学。因此：

- 本步已经验证命令传输、安全门、controller 接收、退出静默和 watchdog 后稳定。
- `dynamic_tracking_validated=false`，不能据此声称真实机械臂能够跟踪目标。
- 真实动态跟踪必须在后续受控真机验收或支持动力学的仿真器中单独完成。

## 新增和修改的文件

新增：

- `quest3_ros2/src/franka_vr_adapter/franka_vr_adapter/closed_loop_runner.py`
- `logs/vr_fake_20261020_step5.jsonl`
- `outputs/vr_fake_validation_20261020_step5.json`
- `B-Line-Step5-Validation.md`

修改：

- `quest3_ros2/src/franka_vr_adapter/setup.py`：注册 `closed_loop_runner` 命令。
- `B-Line-Implementation-Plan.md`：将第 5 步标记为完成，并指向第 6 步。

GELLO 的源码和 ROS package 没有修改，Quest 启动文件仍独立位于 `quest3_ros2`。

## 复查结果

- client ROS Humble / Python 3.10.12 构建成功。
- adapter package：`8 tests, 0 errors, 0 failures`。
- 12 秒正常轨迹加 2 秒退出观察：全部检查通过。
- JSON 结果可严格解析；JSONL trace 共 2997 行、825772 字节。
- 本地 Python 编译检查和 `git diff --check` 通过。
- client 正式工程未同步，真实机器人未连接、未启动。

测试结束后用 Ctrl-C 停止整个 fake launch 时，ROS Humble 自带的
`joint_state_publisher` 在退出阶段打印过一次 Python shutdown traceback；controller
manager 和其余进程均已正常停止，client 上没有遗留本次 ROS 进程。该信息发生在
验收数据写完之后，不影响本步结果，后续若需要整洁的一键关停可单独处理。

## 下一步

第 6 步只做 GELLO 回归：验证原 GELLO 启动入口、topic、配置和默认参数未被 B
线路影响，并验证 GELLO 与 Quest 不能同时取得右臂命令源。
