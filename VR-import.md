# Quest 硬件接入交接与待办

更新日期：2026-10-20。

## 1. 接手后先做什么

Quest 头显和双手柄的阶段 1 输入验收已于 2026-10-17 在 client 完成。
USB/ADB、TCP 8000、双侧 6-DoF 位姿和控制字段均已实测通过；本阶段仍然只验证输入，未连接 GELLO、Franka、MANUS、Sharpa 或 server recorder。

阶段 2 的坐标映射、相对锚定、安全状态机和离线目标可视化已完成。2026-10-20 又完成了 ALVR/SteamVR → OpenVR → 既有 raw recorder 的双手柄实采链路，解决了当前 APK 没有 `Controllers Only` 模式的问题。下一步进入 P2 FR3 控制适配：先做离线 IK，再做隔离目标发布与 fake hardware；暂不直接上真实机器人，也不把阶段 2 夹具当成训练数据。

阶段 2 保留的只读实测夹具为 `recordings/quest_raw/stage2-sixdof-safety3.jsonl`：3945 个有效 pose、`dropped=0`，SHA-256 为 `2d84d7eb95cebb133998711684fe98038b862a47b72939a10eadf9beadbf7a66`。实测上、右、前三轴映射和两次 pitch 与当前转换一致，三个锚点处目标跳变为零。该文件包含一次实际断流，仅用于安全回放，不是连续 episode 或训练数据。

正式采集策略已经冻结：TCP 连接变化即判当前 episode 无效并整条重采，不把重连前后的片段拼接为一条数据，也不要求在同一 episode 内恢复跟随。

当前推荐的手柄原始采集入口为 `scripts/record_meta_quest_openvr.sh`。成功实测文件 `recordings/quest_raw/stage2-openvr-controller-20261020.jsonl` 包含 4151 个 lossless pose，`rejected=0`、`dropped=0`，左右手柄位姿均连续变化，右手 trigger、clutch、record 均记录到状态切换。详细复现与结果见本文第 11 节。

## 2. 三份文档的分工与进度

| 文档 | 用途 | 当前进度 |
|---|---|---|
| [Status-TODOs.md](Status-TODOs.md) | 工程迁移状态、环境边界、复现入口及整体待办 | client/server 隔离和 GELLO 基线已验收，可进入 VR；VR 控制与数采闭环未完成 |
| [VR-Collection-Workflow-Draft.md](VR-Collection-Workflow-Draft.md) | 后续 VR 数采行为设计 | 主要流程已通过讨论明确，尚未实现；暂不合并到 Status-TODOs |
| 本文 | 当前 Quest 接入交接清单 | APK/ADB 与 ALVR/OpenVR 两条输入路径均已验证；机器人侧功能未完成 |

### 已有工程基线

- 用户已确认新 client/server 的双臂 FR3 + Franka 夹爪 GELLO 采集、校验和回放通过。
- server 的 `data/test` 已完成初始段裁剪和完整数据校验。
- client 为 `/home/landau/real-exp-vr`，系统 ROS Humble / Python 3.10；server 为 `/home/pair1/real-exp-vr`，系统 ROS Jazzy / Python 3.12。
- client 和 server 的 ROS 构建产物分别使用 `install/client` 与 `install/server`，由各端本地构建。
- 本阶段重点在 client，server 的录制环境无需为了 Quest 输入重新迁移。

### 后续数采设计摘要

这里可以看一下[VR-Collection-Workflow-Draft.md](VR-Collection-Workflow-Draft.md)，目前VR仿真那边类似B，但是那边可以自动判定停止，我们不行，所以初步设计比较傻瓜。

```text
电脑 s：准备，不录制
  → Home 处按手柄接管键：开始录制和跟随
  → 完成任务，人工到 ReturnReady，夹爪打开
  → 再按接管键：解除接管，自动开爪并归位
      A：记录到 Home 到位并稳定
      B：记录到 ReturnReady，自动收尾与归位不录制
  → 回电脑 e 保留 / d 丢弃
```

- A/B 通过启动录制脚本参数选择，尚未实现该参数。
- Home 是固定起始关节姿态；ReturnReady 是固定的人工操作终点和自动归位入口。
- 手柄采用切换接管，夹爪采用离散开闭。解除接管是任务收尾，不再是途中临时离合。
- 断流和失跟踪属于异常保持，不能当作主动解除接管触发自动开爪归位。
- Status-TODOs 中较早的离合/录制描述尚未合并更新；后续新功能行为以流程草案为准。现有 GELLO 按键行为不因此改变。

## 3. 环境决定与修改边界

**已确定：Quest 输入使用现有 client `lerobot-vr`，检查后缺什么补什么，多问问AI。**

- 本机环境路径：`/home/landau/anaconda3/envs/lerobot-vr/`，此前检查 Python 为 3.12.13。
- 不整体安装 [ms3-enviroment.yaml](ms3-enviroment.yaml)。它是包含 ManiSkill、SAPIEN、训练和视频工具的 Python 3.10 环境快照，不是 Quest 最小依赖清单。
- 当前 Quest 接收、协议与锚点模块使用 Python 标准库；可视化使用 Tk；设备操作调用外部 `adb`。
- 此前在清除继承的 Python 搜索路径后，receiver、calibration、visualizer 导入通过，Tk 8.6 可用。这不代表 GUI 和设备连接已经验收。
- 系统 ROS 控制继续使用系统 Python。不要将 Humble 的 Python 3.10 包路径注入 `lerobot-vr` 的 Python 3.12。
- 检查时发现继承的 `PYTHONPATH` 可使 `lerobot-vr` 搜索到 `/opt/ros/humble` 下的包，需要避免这种路径污染。
- 缺依赖时只补实际需要的部分，记录安装前后版本；若需要替换 Python、Torch、NumPy 等核心库，先评估对已有流程的影响。
- 不修改旧 `lerobot`、Conda base、系统 Python/ROS 或公共 Franka 工作空间；工程修改限制在当前项目。若缺 ADB 或设备权限，记录缺口并按本机管理权限处理，不顺带修改系统 Python。
- ALVR/OpenVR bridge 使用 client 已有的独立 Python 3.11、`openvr==2.12.1401` 和 SteamVR runtime；路径由 `scripts/start_meta_quest_openvr_bridge.sh` 固定并检查，不把 OpenVR/SteamVR 依赖强装进 `lerobot-vr`。

当前启动脚本使用裸 `python`，尚未固定环境。因此接手时先显式激活 `lerobot-vr`；后续可复用项目的环境选择机制改进入口，而不是依赖操作者终端的偶然状态。

### GELLO 保留与 VR 旁路约定

当前先在 client 完成 Quest 单右臂摇操，server 暂不进行代码同步、bridge/recorder 适配或数据格式修改。开发直接使用 `/home/landau/real-exp-vr`，但不得改动或清理当前可能由其他人使用的运行进程。

- 原 GELLO 启动入口、参数和 topic 保持不变：`scripts/start_data_collection_client.sh`、`scripts/start_teleoperation.sh`、`/right/gello/raw_joint_states` 和 `/right/gello/accepted_joint_states`。
- VR 使用独立启动入口和独立 topic。计划入口为 `scripts/start_vr_teleoperation_client.sh`，计划 topic 为 `/right/vr/raw_joint_states` 和 `/right/vr/accepted_joint_states`；当前仅为接口约定，尚未实现。
- GELLO 与 VR 不同时控制同一机械臂。首期通过启动入口互斥保证唯一命令源，不实现运行中的输入源切换。
- VR 允许复用现有 FR3 controller，但新增 launch/config 参数必须是可选的；不传 VR 参数时，控制器仍使用当前 GELLO 默认配置。
- VR 输出仍需经过控制器检查，后续采集只记录 controller accepted target。Quest 原始位姿和未经接受的 IK 结果不进入既有 action 语义。
- client 验证顺序为输入回放、目标可视化、离线安全/IK、fake hardware、用户安排的真实摇操。前一步未通过时不推进 server 数采改造。

```text
GELLO：GELLO -> /right/gello/raw_joint_states -> controller
VR：   Quest -> mapping/state/safety -> IK -> /right/vr/raw_joint_states -> controller
```

这项边界确认本身不要求重新运行 GELLO 真机测试，也不包含 Quest 控制、MANUS/Sharpa 或 server 数采功能的实现。

## 4. 需要拿到的硬件与信息

***我放在实验室桌子上了，一个头显两个手柄，下面是AI写的，反正确认能接入就行。***

- [x] Quest 3，Android 14，构建号 `UP1A.231005.007.A1`。
- [x] 左右 Touch 手柄均已连接并产生独立位姿。
- [x] 支持数据传输的 USB 线连接当前 client。
- [x] 开发者模式和 USB 调试已开启；已选择 `Always allow from this computer`。
- [x] Hand Tracking Streamer controller build 已安装并实测。
- [x] 包名、版本、APK、源码分支/提交和 SHA-256 已记录在本文阶段 1 验收记录。

注意：Android 的 `versionName=1.1.0` 不能单独证明是否包含 controller 扩展。本轮固定的是 `maniskill-shadowhand` 分支提交与 APK 摘要，不能随便安装同版本号的上游 APK 并认为等价。

## 5. 现有代码入口

| 文件 | 当前职责 |
|---|---|
| [Quest README](data_collection/meta_quest/README.md) | 应用设置、协议、坐标约定和当前边界 |
| [start_meta_quest_pose_monitor.sh](scripts/start_meta_quest_pose_monitor.sh) | ADB 检查、端口映射、监视器启动与退出清理 |
| [receiver.py](data_collection/meta_quest/receiver.py) | TCP 接收与最新输入管理 |
| [protocol.py](data_collection/meta_quest/protocol.py) | 位姿与按钮解析 |
| [monitor.py](data_collection/meta_quest/monitor.py) | 终端监视入口 |
| [visualizer.py](data_collection/meta_quest/visualizer.py) | 原始 Unity 坐标系中的位姿显示 |
| [power.py](data_collection/meta_quest/power.py) | 通过 ADB 管理头显唤醒 |
| [calibration.py](data_collection/meta_quest/calibration.py) | 锚点数学基础，不是完整机器人坐标转换或控制状态机 |
| [openvr_bridge.py](data_collection/meta_quest/openvr_bridge.py) | 从 SteamVR/OpenVR 读取双手柄位姿与按键，转换成既有 TCP 输入格式；无机器人输出 |
| [start_meta_quest_openvr_bridge.sh](scripts/start_meta_quest_openvr_bridge.sh) | 检查并启动独立 OpenVR 运行环境 |
| [record_meta_quest_openvr.sh](scripts/record_meta_quest_openvr.sh) | 一键启动 raw recorder 与 OpenVR bridge，`Ctrl+C` 安全完成 JSONL |

原 APK/ADB 链路：

```text
头显应用 → USB/ADB 上的 TCP 8000 → receiver → 协议校验 → 终端与位姿窗口
```

当前推荐的 ALVR/OpenVR 采集链路：

```text
Quest Touch 手柄 → ALVR/SteamVR → OpenVR bridge
  → 本机 TCP 127.0.0.1:8000 → raw recorder → lossless JSONL
```

原始位姿属于 Quest/Unity 追踪坐标系，四元数按 `xyzw` 解析；尚不能直接作为机器人基座目标。

## 6. 首次接入步骤

### 6.1 检查环境与设备

在 client 的新终端执行，建议使用未 source ROS 的终端：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
unset PYTHONPATH PYTHONHOME
command -v python
python --version
python -c 'import tkinter; from data_collection.meta_quest.receiver import PoseReceiver; from data_collection.meta_quest.visualizer import ControllerPoseVisualizer; print("Quest imports OK; Tk", tkinter.TkVersion)'
command -v adb
adb version
adb devices -l
```

若当前 shell 没有 conda 命令，先按本机已有 Conda 安装方式初始化 shell，不重新安装 Conda。

设备需显示为 `device`。显示 `unauthorized` 时戴上头显确认 USB 调试授权；无设备时检查数据线、USB 接口和开发者设置。初期仅连接一个 ADB 设备；现有脚本未显式选择设备，多设备时可能无法工作。

### 6.2 启动监视器与头显应用

```bash
./scripts/start_meta_quest_pose_monitor.sh
```

在头显应用选择：

```text
Protocol: TCP (Wired / ADB)
IP:       127.0.0.1
Port:     8000
Mode:     Controllers Only（支持手柄的版本）
```

点击 `Start Streaming`。电脑应出现位姿窗口，并每秒打印最新输入。头显面板开始串流后隐藏、只显示透视属于文档描述的正常行为；左手柄平面 Menu 键停止串流并恢复面板。

没有图形桌面时可使用：

```bash
./scripts/start_meta_quest_pose_monitor.sh --no-visualization
```

这只能验证终端接收，不能代替 GUI 验收。电脑端用 `Ctrl+C` 退出。

### 6.3 常见恢复项

- 端口 8000 被占用时先确认占用者，停止旧监视器，不盲目结束其他进程。
- USB 或 ADB transport 重启会清除 reverse 映射；重新运行启动脚本建立映射。
- 当前脚本在启动时恢复正常接近传感器行为，收到控制器数据后监视器才请求保持唤醒；超过设定无数据宽限或正常退出后恢复休眠行为。
- 需要实测正常退出及异常退出后的行为。不要将“保持唤醒”当作“追踪仍然有效”。

## 7. 按顺序执行的验收待办

### P0-1：设备与环境可复现

- [x] 明确 Python 路径属于当前 `lerobot-vr`，最小导入通过。
- [ ] GUI 可打开；记录显示环境与 Tk 情况。
- [x] ADB 识别、授权、reverse 建立成功。
- [x] 核对 APK/controller 模式与实际输出布局。
- [x] 退出后重新启动，能够再次接收输入。

本轮通过 SSH 使用 `--no-visualization` 验收，未把本地图形窗口作为完成项；Tk 导入可用不等于 GUI 已实测。

### P0-2：位姿与按钮实测

- [x] 左右手来源不混淆，左右手柄位置和朝向均随动作连续变化。
- [ ] 分别定向验证三个平移方向和三个旋转方向；本轮只确认数值随自由运动变化，不标记机器人方向映射已通过。
- [x] 连续操作时左侧约 45--51 Hz、右侧约 44--52 Hz，`rejected=0`。
- [x] 控制字段出现按下/松开变化：两侧 `grasp` 非零，左侧观测到 `record=1`，右侧观测到 `clutch=1`。
- [ ] 分别测试单击、长按、连续点击，区分持续按钮状态与单次事件。

文档描述的紧凑输入为 `position, quaternion_xyzw, trigger, clutch, record`，其中 `grasp` 显示扳机值、B 对应 clutch、A 对应 record。这些是旧协议字段名，不代表新设计已经实现接管/录制功能。本阶段只核对并记录，为后续选定接管与夹爪按键提供依据。

### P0-3：异常、脱戴与重连

- [ ] 遮挡手柄或移出追踪范围，观察停止发包、无效标志或旧值继续发送中的哪种行为实际发生。
- [ ] 手柄暂时不可用、头显休眠、应用停止串流分别测试。
- [ ] 拔插 USB、重启应用后，记录恢复步骤及是否需要重建映射。
- [ ] 重连时检查按钮是否仍保持按下，是否有旧数据残留。
- [ ] Quest recenter 前后记录坐标变化，不能把变化误认为真实手柄运动。
- [ ] 先验证戴头显使用，再验证脱戴放置时的追踪范围、遮挡和休眠表现；记录头显摆放和操作者站位。

重要限制：当前紧凑协议解析把收到的控制器记录视为 tracked；仅有数据到达不证明追踪有效。若应用失跟踪时继续发送旧值，需要记录并为后续协议/异常检测提出补充需求。手柄静止本身也不能直接判定冻结，需结合时间戳、帧号、追踪状态等可用证据。

## 8. 本阶段交付与完成判据

完成后，应在本文件记录实测结果，或链接一份项目内的验收记录，至少包含：

| 项目 | 应记录内容 |
|---|---|
| 工程 | client Git commit、相关未提交修改、实际启动命令 |
| 环境 | Python 路径/版本、ADB 版本、新增依赖及版本 |
| 设备与应用 | 头显/手柄型号、应用包名/版本、APK 来源与源码复现缺口 |
| 正常输入 | 测试时长、接收频率、位姿表现、物理按键到字段的对应 |
| 异常输入 | 遮挡、休眠、断线、recenter 的表现和恢复步骤 |
| 操作方式 | 戴头显和脱戴结果、可用范围、未通过项 |

完成判据：另一位同事能按记录在同一 client 复现输入；位姿和按钮来源明确；异常与恢复行为有证据；未解决问题显式列出。不要将模块导入成功、监视器成功、fake hardware 成功和真实机器人验证混为一谈。

## 9. 2026-10-17 阶段 1 验收记录

### 9.1 固定版本与设备

| 项目 | 实测值 |
|---|---|
| client 工程 | `/home/landau/real-exp-vr`，HEAD `6006fb46235fbf10e6b83bb04f603fdaba8e9ae0`；验收时存在未提交文档、`logs/`、`tools/` |
| Python | `/home/landau/anaconda3/envs/lerobot-vr/bin/python`，3.12.13 |
| ADB | 1.0.41 / Debian 28.0.2 |
| 头显 | Quest 3，Android 14，`UP1A.231005.007.A1` |
| ADB serial | `2G97C5ZJ0Z01R1`；最终状态 `device` |
| Android 包 | `com.wengmister.handtrackingstreamer`，versionName 1.1.0，versionCode 7 |
| 源码 | `Soo0803/VR-hand-tracking-and-controller`，分支 `maniskill-shadowhand`，提交 `a8f16e376d1090b93a5ba314e8d9afd0f3a7b7a3` |
| APK | `downloads/hand_tracking_streamer-maniskill-shadowhand.apk` |
| APK SHA-256 | `e0b41ab36e0bcbbc1a1d616bd659bad6d93c02efcb332c1d87c2bee1c6e04879` |
| 新增 Python 依赖 | 无；使用现有 `lerobot-vr` |

本轮实际使用的无窗口监视命令为：

```bash
cd /home/landau/real-exp-vr
adb reverse tcp:8000 tcp:8000
/home/landau/anaconda3/envs/lerobot-vr/bin/python -u \
  -m data_collection.meta_quest.monitor \
  --port 8000 --print-hz 1 --keep-awake-grace-s 2 --no-visualization
```

正常复现仍优先使用 `./scripts/start_meta_quest_pose_monitor.sh --no-visualization`，但要先激活 `lerobot-vr` 并清除不兼容的 ROS `PYTHONPATH`。

### 9.2 实测结果

- `adb devices -l` 最终显示 Quest 3 为 `device`；`adb reverse --list` 显示 `UsbFfs tcp:8000 tcp:8000`。
- APK 在 `TCP (Wired / ADB)`、`127.0.0.1:8000`、`Controllers Only` 下建立连接。
- 最终快照为 `connections=12 poses=3659 rejected=0`。
- 左右两侧均出现独立的 position 和 `quaternion_xyzw`，自由移动时数值连续变化。
- 左侧接收频率约 45--51 Hz，右侧约 44--52 Hz。
- 两侧均观测到 `grasp` 从 0 到非零/1 再回到 0；左侧观测到 `record=1`，右侧观测到 `clutch=1`。
- 当前字段验证只证明输入可达。物理按钮到未来“接管、夹爪、录制”语义的最终映射尚未冻结，也未完成单击/长按/连击的边沿与去抖测试。
- 摘下头显或停止操作后监视器显示 `STALE`；重新戴上并保持应用处于 Streaming 时可继续接收。本轮未完成遮挡、recenter、长时稳定性和断线残留的完整异常矩阵。
- 未启动 GELLO、ROS 控制器、Franka、MANUS、Sharpa、相机、server bridge 或 recorder。

阶段 1 的验收证据位于：

```text
/home/landau/real-exp-vr/logs/quest-input-stage1-20261017.log
```

该文件是 monitor 每秒一次的终端摘要，包含累计连接/位姿计数、每侧最新位姿、估计频率、`grasp/clutch/record` 和 stale 状态。它不是 APK 发出的完整约 50 Hz 原始包，也不是 LeRobot episode，不能用于精确回放或训练。

常用查看命令：

```bash
tail -f /home/landau/real-exp-vr/logs/quest-input-stage1-20261017.log
grep -E 'controller (left|right) +live' \
  /home/landau/real-exp-vr/logs/quest-input-stage1-20261017.log | tail -50
grep -E 'grasp=1|clutch=1|record=1' \
  /home/landau/real-exp-vr/logs/quest-input-stage1-20261017.log
```

### 9.3 本轮确认的原设计问题

1. **Controller-only 页面却不能用 controller 操作。** Unity 代码只自动绑定 `LeftHandRayInteractor` / `RightHandRayInteractor`，没有 controller UI ray。必须先在 Quest 系统启用 Hand Tracking，再用裸手捏合配置。应为 Canvas 增加左右 controller ray 与 XR UI input module，并保留裸手作为备选。
2. **USB 模式错误依赖 Wi-Fi。** `ValidateNetwork()` 无条件使用 `Application.internetReachability` 禁用 Start；即使目标是 ADB loopback `127.0.0.1:8000`，Wi-Fi 未连接也会显示 `No Active Network Connection`。Wired/ADB 模式不应受该门禁影响，应以实际 TCP 连接检查为准。
3. **退出入口依赖左手柄。** Start 后 `menuPanel.SetActive(false)` 隐藏面板，源码只用左手柄 Menu 键恢复；右手柄单臂场景没有可靠的应用内返回入口。应增加通用手势/右手柄/超时返回或始终可见的退出控件。
4. **版本号无法标识 controller 能力。** controller build 和上游版本都显示 1.1.0。必须固定源码提交和 APK digest，不能只记录 versionName。
5. **协议没有可靠追踪有效性。** 紧凑 10-float layout 收到包就被 host 视为 tracked，缺少显式 tracking-valid、递增 frame id 和稳定 device timestamp。冻结、失跟踪和重复旧值只能间接用 host 到达时间判断，后续控制前必须补协议或强 watchdog。
6. **监视器不保存原始轨迹。** `PoseReceiver` 只保存每侧最新值，monitor 默认只以 1 Hz 打印摘要。本轮日志可审计但不能重建约 50 Hz 运动；正式数采前应新增独立 raw input recorder，并明确它与 server LeRobot recorder 的职责。
7. **按钮字段名与最终行为尚未对齐。** `grasp/clutch/record` 是旧协议语义，而流程草案把接管、夹爪和 episode 边界重新设计为状态机事件。不能直接把当前持续电平接到机器人或 recorder；必须冻结物理映射、做 rising-edge/debounce、重连清零和模式互斥。
8. **保持唤醒不等于追踪有效。** Quest 脱戴、应用失焦或 tracking lifecycle 变化可使数据 stale。后续安全层必须按新鲜度进入保持，且不得把断流当成主动解除接管或自动归位信号。
9. **ALVR 的图像渲染限制不等于手柄输入不可用。** 阶段 1 时 client 的 Intel iGPU 在 SteamVR direct-mode 图像路径上出现过 RenderThread 问题，因此先采用 Unity/TCP fallback。2026-10-20 已确认 H.264 software encoding 下 ALVR 可以进入 `Streaming`，并且后台 OpenVR 应用能够连续读取左右手柄；当前只复用追踪输入，不依赖 SAPIEN/native mirror 提交 VR 图像。SAPIEN `RenderVRDisplay` 在不提交帧时会返回缓存姿态，不能再作为正式采集入口。

## 10. 通过接入验收后再做什么

1. 实现坐标映射、目标可视化和新接管状态机，离线演练 A/B 事件。
2. 适配 FR3 在线 IK、离散夹爪与归位模块，先完成离线/fake hardware 验证。
3. 接入 server recorder 的准备、录制边界和候选审核，验证 A/B 数据内容。
4. 另行安排实机联调，配置 Home、ReturnReady 和返回路径，完成单右臂 A/B 采集、校验与回放。

## 11. 2026-10-20 ALVR/OpenVR 双手柄实采验收

### 11.1 一键复现

先在 client 打开 ALVR Dashboard 和 SteamVR。戴上 Quest，在头显中打开 ALVR，等 Dashboard 显示 `Streaming`；采集期间保持佩戴、唤醒并让 ALVR 位于前台。然后运行：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
unset PYTHONPATH PYTHONHOME

./scripts/record_meta_quest_openvr.sh \
  --output recordings/quest_raw/<new-name>.jsonl
```

看到 pose 计数持续增长后执行动作，结束时按一次 `Ctrl+C`。一键脚本会先让 raw recorder 写 completion summary、将 `.partial` 原子改名，再停止 OpenVR bridge；已有输出不会覆盖。ALVR 离开 `Streaming` 时 bridge 会停止，避免继续写缓存姿态。该路线使用 Wi-Fi/SteamVR，不需要 `adb reverse`。

环境自检但不连接录制器：

```bash
./scripts/start_meta_quest_openvr_bridge.sh --dry-run
```

若 SteamVR 重启后看不到 ALVR external driver，再执行一次：

```bash
/home/landau/.local/share/Steam/steamapps/common/SteamVR/bin/vrpathreg.sh \
  adddriver /home/landau/real-exp-vr/tools/alvr-v20.8.1/lib64/alvr
```

### 11.2 实测结果

```text
文件：recordings/quest_raw/stage2-openvr-controller-20261020.jsonl
大小：1,819,154 bytes
记录：4151 poses，left=2076，right=2075
完整性：schema=1, lossless，rejected=0，dropped=0
SHA-256：411fd0428a830a47185d70e379b81c34f1e6db601a6db779fb23dc2e581e0e41
```

- 左手得到 1276 个不同 pose，右手得到 1489 个不同 pose，排除了持续缓存同一姿态。
- 右手最大平移范围约为 x `0.3186 m`、y `0.4090 m`、z `0.2778 m`，相对起始最大旋转约 `179.3°`。
- 模拟 trigger 写为 `grasp`；B/Y 共用的按键掩码写为 `clutch`；A/X 共用的按键掩码写为 `record`。实测 trigger 最大值 `1.0`，clutch 有 4 次状态切换，record 有 6 次状态切换。
- `stage2-alvr-controller-20261020.jsonl` 和 `stage2-alvr-controller-worn-20261020.jsonl` 是 SAPIEN 缓存姿态失败样本，只保留诊断，不能用于动作分析或训练。
- 采集只验证 Quest 手柄输入，没有启动 ROS、GELLO、Franka、MANUS、Sharpa 或 pair1 recorder。

验证文件：

```bash
python -m data_collection.meta_quest.replay \
  recordings/quest_raw/<new-name>.jsonl --summary-only
sha256sum recordings/quest_raw/<new-name>.jsonl
```

### 11.3 测试与提交

- Meta Quest 专项测试：`67 passed`。
- `bash -n`、OpenVR dry-run 和 `git diff --check` 通过。
- 分支：`feature/quest-stage2-target-preview`。
- commit：`bd5153a feat(quest): add direct OpenVR recording pipeline`。
- 原 GELLO pipeline、启动入口和 topic 未修改。下一步仍是 P2 FR3 离线 IK 与 fake hardware，不直接上真实机器人。
