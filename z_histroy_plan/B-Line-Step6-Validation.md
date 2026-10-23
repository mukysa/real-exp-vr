> 历史归档：2026-10-21。本文保留原记录的时间语境，不作为当前进度或启动指令。最新工作入口见 [A 线当前状态与下一步](../A线-当前状态与下一步.md)；代码块中的项目相对路径仍以项目根目录为基准。

# DexArena B 线路第 6 步验收

验收日期：2026-10-20。

## 本步解决的问题

本步确认新增 Quest B 线路没有破坏原有 GELLO 控制链，并验证 GELLO 与 Quest
不能同时取得右臂命令源。由于 client 正有人使用，本次只做无硬件测试，不启动或
停止任何真实 GELLO、Quest、Franka 或正在运行的夹爪进程。

## 隔离方式

- 临时目录：`/tmp/dexarena-b6-validation-20261020`。
- C++ ROS 单元测试：`ROS_DOMAIN_ID=120`。
- Python 模拟 publisher 测试：`ROS_DOMAIN_ID=121`。
- 两组 ROS 测试均设置 `ROS_LOCALHOST_ONLY=1`。
- 互斥锁测试使用临时目录中的锁文件，没有占用正式 `/tmp` teleoperation 锁。
- 没有访问 `/dev/ttyUSB_left`、`/dev/ttyUSB_right` 或真实机械臂 IP。

测试前发现有人正在使用 `/home/landau/hfang/real-exp` 的左右夹爪 ROS 进程。测试
完成后原来的 5 个相关 PID 仍在运行，未向它们发送任何信号。

## 静态契约检查

新增 `scripts/validate_b_line_step6_no_hardware.py`，验证以下契约：

- GELLO 默认 command topic 仍是 `gello/raw_joint_states`。
- GELLO 默认 accepted topic 仍是 `gello/accepted_joint_states`。
- GELLO controller 默认模式仍为 `always`，timeout 仍为 `0.5 s`。
- GELLO publisher 仍只发布自己的 GELLO topic。
- Quest 使用独立的 `vr/raw_joint_states` 和 `vr/accepted_joint_states`。
- Quest 和 GELLO 启动脚本使用同一个互斥锁名字。
- Quest 启动前会检查右臂 GELLO publisher。
- Quest launch 仍固定为 fake hardware 和无效机器人地址。
- Quest 和 GELLO 保持各自独立的 ROS build/install 路径。
- `gello_software/ros2` 和原 `scripts/start_teleoperation.sh` 没有 B 线路改动。

以上 10 项全部通过，静态结果为 `all_passed=true`。

## 无硬件运行测试

### GELLO controller C++ 测试

- `JointReferenceGeneratorTest`：5 项通过。
- `JointImpedanceControllerTest` 与非法配置测试：14 项通过。
- 1 项跳过：`TestUpdateInvalidGelloPositionValues`。

跳过项不是本次新增问题。原测试源码已经写有
`GTEST_SKIP() << "Skipping this test because yet not implemented"`。

### GELLO Python 测试

使用 `FakeGelloHardware`，没有打开串口：

- 多圈编码器归一化与连续性测试：通过。
- 模拟 raw waypoint 发布频率和 topic 测试：通过。
- 汇总：`4 passed in 1.52s`。

### 启动脚本和互斥锁

- GELLO 和 Quest 脚本 `bash -n`：通过。
- 两个脚本的 `--help` 路径：通过。
- 在隔离目录预先持有共享锁后，GELLO 启动入口正确拒绝：通过。
- 在同一把隔离锁下，Quest 启动入口正确拒绝：通过。

拒绝发生在任何 ROS launch、串口检查或硬件访问之前，因此没有启动控制进程。

## 新增和修改的文件

新增：

- `scripts/validate_b_line_step6_no_hardware.py`
- `outputs/gello_no_hardware_regression_20261020_step6.json`
- `outputs/gello_no_hardware_runtime_20261020_step6.json`
- `B-Line-Step6-Validation.md`

修改：

- `B-Line-Implementation-Plan.md`：把第 6 步标记为完成，下一步改为第 7 步。

GELLO 源码、配置和原启动脚本均未修改。client 正式工程也未同步。

## 验收结论

第 6 步在“无硬件、不干扰现有 client 进程”的限制下通过：

- GELLO 默认 topic、参数和启动入口保持原样。
- GELLO 轨迹生成和 controller 单元测试通过。
- GELLO Python 模拟硬件测试通过。
- Quest/GELLO 的共享互斥锁确实会拒绝第二个命令源。
- 正在运行的用户进程未中断。

## 下一步

第 7 步才会把明确列出的 B 线路文件同步到 client 正式工程，并更新 Obsidian。
同步前应再次确认 client 空闲，因为第 7 步包含正式目录构建和 fake 验收；本步没有
执行这些操作。
