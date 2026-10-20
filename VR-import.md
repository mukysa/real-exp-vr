# Quest 硬件接入交接与当前进度

更新日期：2026-10-20。

## 当前结论

client 的 Quest 3 + 左右 Touch 手柄输入链路已跑通。当前唯一支持的现场启动方式是：

```text
Quest Touch
  → Quest ALVR client
  → Wi-Fi/TCP
  → client ALVR Dashboard + SteamVR + ALVR driver
  → direct OpenVR bridge
  → local TCP 127.0.0.1:8000
  → PoseReceiver/raw recorder
  → lossless JSONL
```

完整逐命令操作只维护在 [Quest-ALVR-OpenVR-Reproduction.md](Quest-ALVR-OpenVR-Reproduction.md)。不要再按历史 Hand Tracking Streamer/ADB 或 SAPIEN bridge 教程启动。

当前已完成输入、坐标映射、相对锚定、离线目标预览和安全状态机。下一步是 P2 FR3 离线 IK、隔离 topic 和 fake hardware；尚未启动或控制真实机器人。

## 当前入口

正式录制只有一个入口：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
unset PYTHONPATH PYTHONHOME

./scripts/record_meta_quest_openvr.sh \
  --output recordings/quest_raw/stage8-openvr-guided-20261020.jsonl
```

前提是 Quest 保持佩戴、唤醒，Quest 内 ALVR 位于前台，电脑端 Dashboard 显示 `Streaming`。结束时只按一次 `Ctrl+C`，等待出现 `Saved ... poses`。`stage8` 已存在后必须换用新的 stage 文件名；录制器拒绝覆盖已有文件。

相关代码：

| 文件 | 当前职责 |
|---|---|
| [Quest-ALVR-OpenVR-Reproduction.md](Quest-ALVR-OpenVR-Reproduction.md) | 唯一现场操作手册 |
| [Meta Quest README](data_collection/meta_quest/README.md) | 当前协议、坐标和安全边界 |
| [openvr_bridge.py](data_collection/meta_quest/openvr_bridge.py) | 读取 OpenVR 双手柄并输出既有 TCP 协议 |
| [record_meta_quest_openvr.sh](scripts/record_meta_quest_openvr.sh) | 一键启动 bridge 与 raw recorder，安全处理 `Ctrl+C` |
| [raw_recorder.py](data_collection/meta_quest/raw_recorder.py) | `.partial` 写入、completion summary 和原子改名 |
| [target_preview.py](data_collection/meta_quest/target_preview.py) | 离线坐标转换和目标轨迹预览 |
| [state_machine.py](data_collection/meta_quest/state_machine.py) | 离线接管/保持状态机 |
| [safety.py](data_collection/meta_quest/safety.py) | stale、断流、跳变、速度和工作空间检查 |

## 最新实测证据

### 完整人工引导复现

```text
文件：recordings/quest_raw/stage6-openvr-guided-20261020.jsonl
大小：404,675 bytes
pose：928（left=464，right=464）
连接：accepted_connections=1
完整性：schema=1, lossless，rejected=0，dropped=0，completed=true
SHA-256：0c6ba8544968137151d45e580722eeec4fd138a40db2213ecf742a1046fd3ed7
```

### 右手动作与按键复现

`stage7-openvr-guided-20261020.jsonl` 保存 904 pose，左右各 452 条、单一连接、lossless、`dropped=0`。其中右手有 379 个不同完整位姿和 378 次相邻变化，clutch/record 均有有效记录；左手大部分时间静止或分段更新，因此该文件只能作为右手输入证据，不能当双手连续运动 episode。

JSONL 每个 OpenVR frame 依次写一条 `left` 和一条 `right`。如果不按 `side` 分组而把所有 position 画成一条曲线，会表现为在左右手两个空间位置之间来回跳；这不是同一手柄跳变。单右臂分析必须显式选择 `--side right`。

### 早期 ALVR/OpenVR 实测

```text
文件：recordings/quest_raw/stage2-openvr-controller-20261020.jsonl
pose：4151
完整性：schema=1, lossless，rejected=0，dropped=0
SHA-256：411fd0428a830a47185d70e379b81c34f1e6db601a6db779fb23dc2e581e0e41
```

左右手柄位置和姿态均产生连续变化，右手 trigger、clutch、record 均记录到状态切换。

## 阶段二离线安全结果

保留的坐标与安全回归夹具：

```text
recordings/quest_raw/stage2-sixdof-safety3.jsonl
3945 poses，dropped=0
SHA-256：2d84d7eb95cebb133998711684fe98038b862a47b72939a10eadf9beadbf7a66
```

固定坐标映射：

```text
robot_x =  quest_z
robot_y = -quest_x
robot_z =  quest_y
```

姿态通过旋转矩阵 `B R B^T` 变换，不直接交换四元数分量。离线链路已实现相对锚定、按钮去抖、`WAIT_INPUT / READY / FOLLOWING / MANUAL_HOLD / FAULT_HOLD` 状态，以及 stale、连接变化、tracking、源位姿跳变、目标速度、工作空间和四元数连续性检查。

`stage2-sixdof-safety3.jsonl` 包含一次真实断流，只能用于回归测试，不是训练数据或连续 episode。正式采集一旦断流或 connection id 变化，当前 episode 整条重采，不跨连接拼接。

## 已停用的启动方式

以下方式不再出现在现场操作步骤中：

- Hand Tracking Streamer + USB/ADB + `Controllers Only`：当前设备上的 APK 没有该模式，并且旧应用存在网络门禁、UI 和追踪有效性限制。
- SAPIEN `RenderVRDisplay`/SteamVR bridge：client 上不提交 VR 图像帧时会得到缓存姿态，不能作为正式输入。
- 单独启动 raw recorder 再在另一终端启动 bridge：仅用于开发诊断，现场统一使用一键脚本。
- monitor 终端摘要：只显示最新值，不保存全速轨迹，不能替代 JSONL recorder。

相关旧脚本和历史文件可保留用于诊断，但不能写进新的采集 SOP。历史细节通过 Git 查看，不在当前交接文档中保留可复制命令。

## GELLO 与系统边界

- client：`/home/landau/real-exp-vr`，ROS Humble；server：`/home/pair1/real-exp-vr`，ROS Jazzy。
- 原 GELLO 启动入口、默认参数和 topic 不变：`scripts/start_data_collection_client.sh`、`scripts/start_teleoperation.sh`、`/right/gello/raw_joint_states`、`/right/gello/accepted_joint_states`。
- GELLO 与 VR 不得同时控制同一机械臂；首期使用不同入口和 topic，保持唯一命令源。
- Quest 原始位姿、未经安全层接受的目标和未经控制器接受的 IK 结果不能替代 dataset action。
- 当前 Quest 输入流程不修改 server recorder，不启动 ROS、Franka、MANUS、Sharpa 或相机。

## 测试与提交

- 输入链实现：`bd5153a feat(quest): add direct OpenVR recording pipeline`。
- `Ctrl+C` 安全落盘修复：`9988dac fix(quest): finalize OpenVR capture on interrupt`。
- 完整逐命令文档：`00e12d0 docs(quest): add complete OpenVR capture reproduction`。
- 最新简化操作文档：`0ca747d docs(quest): simplify guided capture command`。
- Meta Quest 专项测试曾完成 `67 passed`；信号安全回归相关测试完成 `16 passed`。
- 原 GELLO pipeline 未因上述提交改变。

## 下一步

1. 将右手安全末端目标接入 FR3 离线 IK。
2. 验证可达性、关节限位、连续性、奇异点、延迟和 IK 失败保持。
3. 接入独立 `/right/vr/raw_joint_states` 与 `/right/vr/accepted_joint_states`。
4. 使用 fake hardware 或隔离 topic 验证完整命令链。
5. 回归 GELLO，确认原启动、采集、历史数据读取与回放不变。
6. 上述项目通过后，再由现场操作者安排低速、限幅的 Franka 真机验证。
7. Quest → FR3 单右臂完成后，再接 MANUS/Sharpa 和 pair1 相机/recorder。
