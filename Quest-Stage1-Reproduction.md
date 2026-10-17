# Quest 阶段 1 成功复现与下一步交接

日期：2026-10-17

本文只记录本次已经跑通的 Quest 3 双手柄输入流程和下一步工作入口，不替代或覆盖现有的 `VR-import.md`、`Status-TODOs.md` 与 `VR-Collection-Workflow-Draft.md`。

## 1. 本次完成结果

在 client `/home/landau/real-exp-vr` 上完成了以下链路：

```text
Quest 3 + 左右 Touch 手柄
  -> Hand Tracking Streamer（Controllers Only）
  -> USB / ADB reverse / TCP 8000
  -> data_collection.meta_quest.receiver
  -> data_collection.meta_quest.monitor
```

实测结果：

- Quest 3 通过 ADB 授权并显示为 `device`。
- `adb reverse tcp:8000 tcp:8000` 建立成功。
- 左右手柄均收到独立的 position 和 `quaternion_xyzw`。
- 左手柄约 45--51 Hz，右手柄约 44--52 Hz。
- 两侧 `grasp` 均出现按下/松开变化；同时观测到 `clutch=1` 和 `record=1`。
- 最终监视快照：`connections=12 poses=3659 rejected=0`。
- 本次只验证 Quest 输入，没有启动 GELLO、Franka、MANUS、Sharpa、相机或 server recorder。
- 原 GELLO 启动脚本、topic、控制器配置和数据语义均未修改。

## 2. 固定环境与文件

| 项目 | 已验证配置 |
|---|---|
| client 工程 | `/home/landau/real-exp-vr` |
| Python | `/home/landau/anaconda3/envs/lerobot-vr/bin/python`，Python 3.12.13 |
| ADB | 1.0.41 / Debian 28.0.2 |
| 头显 | Quest 3，Android 14 |
| Quest 应用包名 | `com.wengmister.handtrackingstreamer` |
| Quest 应用版本 | versionName 1.1.0，versionCode 7 |
| 源码 | `Soo0803/VR-hand-tracking-and-controller`，`maniskill-shadowhand` 分支，提交 `a8f16e376d1090b93a5ba314e8d9afd0f3a7b7a3` |
| APK | `/home/landau/real-exp-vr/downloads/hand_tracking_streamer-maniskill-shadowhand.apk` |
| APK SHA-256 | `e0b41ab36e0bcbbc1a1d616bd659bad6d93c02efcb332c1d87c2bee1c6e04879` |
| 验收日志 | `/home/landau/real-exp-vr/logs/quest-input-stage1-20261017.log` |

## 3. 完整成功复现流程

### 3.1 准备 Quest

1. 打开 Quest 3，确认左右手柄已经配对并有电。
2. 在 Meta Horizon App 中确认该头显已开启 Developer Mode。
3. 在 Quest 系统设置中开启 USB 调试、USB 连接提示和 Hand Tracking。
4. 将 Quest 连接到 Wi-Fi；本次使用 `Pair`。
5. 使用支持数据传输的 USB 线将 Quest 连接 client。
6. 首次连接时，在头显中勾选 `Always allow from this computer`，再选择允许 USB 调试。

### 3.2 在 client 检查设备

```bash
ssh client
cd /home/landau/real-exp-vr
adb devices -l
```

应看到类似：

```text
2G97C5ZJ0Z01R1  device  product:eureka model:Quest_3 device:eureka
```

### 3.3 启动只读监视器

在 client 新终端中执行：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
unset PYTHONPATH PYTHONHOME
./scripts/start_meta_quest_pose_monitor.sh --no-visualization
```

该脚本会建立：

```text
Quest tcp:8000 -> client tcp:8000
```

可以用下面的命令确认：

```bash
adb reverse --list
```

应包含：

```text
UsbFfs tcp:8000 tcp:8000
```

若需要完全复现本次无窗口命令，也可以直接运行：

```bash
/home/landau/anaconda3/envs/lerobot-vr/bin/python -u \
  -m data_collection.meta_quest.monitor \
  --port 8000 --print-hz 1 --keep-awake-grace-s 2 --no-visualization
```

### 3.4 打开 Quest 应用

可以在 Quest 的“应用库 -> 未知来源”中打开 Hand Tracking Streamer，也可以从 client 启动：

```bash
adb shell monkey \
  -p com.wengmister.handtrackingstreamer \
  -c android.intent.category.LAUNCHER 1
```

本次成功配置为：

```text
Protocol: TCP (Wired / ADB)
IP:       127.0.0.1
Port:     8000
Mode:     Controllers Only
```

使用 Quest 裸手追踪和捏合动作操作设置页面，然后点击 `Start Streaming`。开始后设置面板隐藏、进入透视串流界面属于正常状态。

### 3.5 验证双手柄输入

拿起并移动左右手柄，分别进行平移和旋转，再按下扳机及需要验证的按钮。client 终端应持续出现：

```text
controller left  live ... p=[...] q_xyzw=[...] grasp=... clutch=... record=...
controller right live ... p=[...] q_xyzw=[...] grasp=... clutch=... record=...
```

通过标准：

- 左右两侧都出现 `live`。
- position 和 quaternion 随对应手柄动作变化。
- 接收频率稳定在约 45--52 Hz。
- 按下和松开后，`grasp/clutch/record` 中对应字段出现变化。
- `rejected=0`。

### 3.6 查看本次验收日志

```bash
less /home/landau/real-exp-vr/logs/quest-input-stage1-20261017.log
```

实时查看：

```bash
tail -f /home/landau/real-exp-vr/logs/quest-input-stage1-20261017.log
```

查看双手柄有效输入：

```bash
grep -E 'controller (left|right) +live' \
  /home/landau/real-exp-vr/logs/quest-input-stage1-20261017.log | tail -50
```

查看按钮字段：

```bash
grep -E 'grasp=1|clutch=1|record=1' \
  /home/landau/real-exp-vr/logs/quest-input-stage1-20261017.log
```

该日志是阶段 1 的终端验收记录，不是 LeRobot episode。本次没有在 client 或 server 生成训练数据。

### 3.7 正常退出

1. 在 Quest 中按右手柄 Meta 键打开系统菜单，关闭 Hand Tracking Streamer；或使用左手柄 Menu 键停止串流并返回应用面板。
2. 在 client 监视器终端按 `Ctrl+C`。
3. 下次重新连接或 ADB transport 改变后，重新运行启动脚本以恢复 reverse 映射。

## 4. 原 GELLO 链路保持约定

阶段 1 没有修改或运行以下原 GELLO 入口：

```text
scripts/start_data_collection_client.sh
scripts/start_teleoperation.sh
/right/gello/raw_joint_states
/right/gello/accepted_joint_states
```

后续新增 VR 功能继续遵守：

- GELLO 与 VR 使用独立启动入口和独立 topic。
- 同一机械臂同一时刻只允许一个命令源。
- 未传入 VR 配置时，原 GELLO 参数和默认行为保持不变。
- VR 原始位姿不能直接替代 controller accepted target，也不能改变已有数据 action 语义。
- 在 VR 离线和 fake hardware 验证完成前，不修改 server recorder 或现有 GELLO 数据流程。

## 5. 下一步交接

阶段 2 从 client 侧离线链路开始，不直接启动真实机器人：

1. 固定物理按钮到 `grasp/clutch/record` 的映射，并验证单击、长按、连续点击与松开。
2. 保存一段完整频率的 Quest 原始输入，作为坐标映射和状态机的可重复离线测试输入。
3. 明确 Quest Unity 坐标系到 Franka base 坐标系的轴向、手性、单位和四元数顺序。
4. 实现相对锚定：接管瞬间将当前手柄姿态映射到连续的机器人目标，避免目标跳变。
5. 增加 stale/超时保持、工作空间、速度、加速度和旋转限制。
6. 先做目标位姿可视化与离线回放，再接 FR3 IK。
7. 使用 fake hardware 或隔离的目标 topic 验证 IK 与状态机。
8. 回归运行原 GELLO 启动与数据链路，确认默认行为没有变化。
9. client 的 Quest -> IK -> accepted target 链路通过后，再接 MANUS/Sharpa 和 pair1 相机/recorder。

阶段 2 的首个可检验交付应为：同一段已保存的 Quest 输入可以重复回放，产生连续、限幅、可视化的末端目标；断流时目标保持，重新接管时没有跳变，且未连接真实机器人。
