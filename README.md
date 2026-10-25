# Real Experiments：Quest 3 FR3 遥操

更新：2026-10-25。日常操作只需看 [README-QUICKSTART.md](README-QUICKSTART.md)；本文记录已跑通的配置、完整操作、验证结果和仍需注意的边界。

## 当前状态

Quest 3 的**实体左 Touch 手柄**已通过 ALVR、SteamVR、OpenVR 和 ROS 2 控制左 FR3（`172.16.0.3`）。2026-10-25 现场验证了手柄平移、三轴旋转、左 Y 接管、第二次 Y 回到本轮接管点，以及回位后再次接管。操作者确认机械臂移动流畅。现提供 `--left`、`--right`、`--duo` 三种选择；右臂与双臂已通过离线检查，**尚未做真机验证**。

```text
Quest 3 左／右实体 Touch
  → ALVR 20.14.1 → SteamVR 2.17.10 → 双手柄 OpenVR bridge
  → 左 Y 统一接管／回位 → 所选侧相对位姿映射 → 严格 IKResult
  → 每侧独立安全适配器 → 每侧阻抗控制器 → 所选 FR3
```

手柄位置和角度变化均按 **1:1** 映射。第一次按左 Y 时，以当时的实测关节／TCP 位姿和手柄位姿为锚点；接管瞬间不会要求机械臂跳到手柄的绝对朝向。第二次按 Y 返回本轮锚点，松开 Y 后可重新接管，无需重启终端。当前不设额外 TCP 球形活动范围；IK、关节包络、控制器和指令 watchdog 仍生效。

**重要边界：** `scripts/start_quest3_hardware.sh --left|--right|--duo --enable-hardware` 是真机遥操入口，只启动所选机械臂的阻抗控制器。它当前**不控制夹爪，也不生成 episode 数据集**。原始手柄录制是独立的 `scripts/record_meta_quest_openvr.sh` 流程，不能把遥操日志当成训练 episode。现有检查不包括双臂互撞、自碰撞、桌面、夹爪、物体和人员碰撞规划；尤其双臂测试须留出充分间距，急停触手可及。

## 已验证环境

| 项目 | 当前 client 上的配置 |
|---|---|
| Quest / client ALVR | 均为 20.14.1；Quest 中 ALVR 保持前台和 Streaming |
| SteamVR | `previous` 分支 2.17.10，build `25330290` |
| Linux 图形环境 | X11；启动图形程序时 `env -u WAYLAND_DISPLAY DISPLAY=:0` |
| 输入设备 | 实体左／右 Touch，OpenVR 物理手柄位姿；ALVR hand skeleton 关闭 |
| 头显显示 | ALVR `Blend`、`threshold=1.0`、`premultiplied_alpha=false`，佩戴时透视真实环境 |
| 左臂 | Franka Desk 绿色、FCI 开启、IP `172.16.0.3` |
| 右臂 | IP `172.16.0.2`；本次仅检查地址和代码路径，真机仍待验证 |
| ROS 2 / Python | ROS 2 Humble；启动脚本加载系统 ROS/Franka 环境，交互终端可用 `lerobot-vr` |
| ALVR 配置备份 | `/home/landau/.local/state/real-exp-vr-backups/working-vr-20261025-133855` |

真机稳定运行时，`gello_software/ros2/src/franka_fr3_arm_controllers/config/controllers.yaml` 的 1 kHz 控制线程绑定 CPU 4；左臂专用网卡 `enp114s0` 的中断绑定 CPU 5；网卡 RX ring 为 4096，CPU 性能档为 `performance`。`/home/landau/franka_ros2_ws` 中的本机 Franka hardware 构建也包含恢复 libfranka 构造函数后 ROS 线程调度策略的修改。它们是**当前 client 的本机配置**；新机器只克隆本仓库并不能自动得到完全相同的系统环境。

CPU 性能档、网卡接收环和 IRQ 绑定可能在重启后恢复默认值。以下为本机已验证设置；IRQ 编号从当前 `/proc/interrupts` 获取：

```bash
cd /home/landau/real-exp-vr
sed -i 's/cpu_affinity: \[4,5\]/cpu_affinity: [4]/' \
  gello_software/ros2/src/franka_fr3_arm_controllers/config/controllers.yaml
sudo powerprofilesctl set performance
printf 'performance\n' | sudo tee /sys/devices/system/cpu/cpu{4,5}/cpufreq/scaling_governor
sudo ethtool -G enp114s0 rx 4096
sudo systemctl stop irqbalance
awk -F: '/enp114s0($|-)/ {gsub(/[[:space:]]/, "", $1); print $1}' /proc/interrupts |
  while read -r irq; do printf '5\n' | sudo tee "/proc/irq/${irq}/smp_affinity_list" >/dev/null; done
```

`gello_software` 是单独的 Git 子仓库。上面的 `sed` 在当前安装方式中更新其符号链接的 ROS 参数文件；主仓库提交不包含该子仓库的本机修改。

## 完整操作顺序

### 1. 准备现场

清空所选 FR3 工作空间，确认急停可触及。在 Franka Desk 中确认所选机械臂绿色、无待处理错误并开启 FCI。结束其他控制所选机械臂的程序。戴好 Quest，握住两只实体 Touch 手柄，让 ALVR 保持前台、头显唤醒。双臂模式还须确认两臂活动范围不会互相碰撞。

### 2. 启动 VR

在 client 图形桌面分别启动 Steam、ALVR Dashboard 和 SteamVR；已经运行的程序无需重复打开。

```bash
env -u WAYLAND_DISPLAY DISPLAY=:0 /home/landau/.local/share/Steam/steam.sh -silent
```

```bash
cd /home/landau/real-exp-vr
env -u WAYLAND_DISPLAY DISPLAY=:0 ./tools/alvr-v20.14.1/bin/alvr_dashboard
```

```bash
env -u WAYLAND_DISPLAY DISPLAY=:0 /home/landau/.local/share/Steam/steam.sh -applaunch 250820
```

在 Quest 打开 ALVR 20.14.1，连接 Dashboard 中已信任且 IP 正确的电脑，等待 Dashboard 显示 `Streaming`。保持头显佩戴，两只实体手柄都在追踪范围内。当前透视配置允许佩戴时看见电脑和现场。

### 3. 确认双手柄输入

如需在启动真机前单独确认物理双手柄，可以运行无机器人连接的检查：

```bash
cd /home/landau/real-exp-vr
bash scripts/start_meta_quest_openvr_bridge.sh --check-both-touch
```

成功输出包含左右各自的 `30/30 valid OpenVR poses`。三种真机启动模式都会检查并持续读取双手柄；左 Y 始终是唯一接管／回位键，右 B 不触发接管。Quest 系统菜单中的射线不能单独证明 OpenVR 正在收到实体手柄位姿；保持 ALVR 在前台。

### 4. 选择机械臂并启动

在新的 client 终端先执行：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
```

然后只选择一条：

```bash
bash scripts/start_quest3_hardware.sh --left --enable-hardware
```

```bash
bash scripts/start_quest3_hardware.sh --right --enable-hardware
```

```bash
bash scripts/start_quest3_hardware.sh --duo --enable-hardware
```

`--left` 将左手柄映射到左臂，`--right` 将右手柄映射到右臂，`--duo` 同时映射两侧。旧命令 `bash scripts/start_quest3_left_hardware.sh --enable-hardware` 仍作为左臂兼容入口。无机器人连接的脚本检查可把 `--enable-hardware` 换成 `--check-only`。

等待 `Hardware chain ready` 和所选侧 `SOURCE_STATUS ... "state":"READY"`。双臂模式要**两侧都为 READY**。此时短按一次实体左 Y 接管，再缓慢移动和转动对应手柄。终端显示所选侧 `FOLLOWING` 后，第二次短按左 Y 返回本轮各自的接管点；看到 `return complete` 与 `READY`，松开 Y 即可开始下一轮。双臂中任一侧未就绪时按下的 Y 会被拒绝，须松开后重新按。双臂运行中若任一侧安全适配器进入 STOP，两侧都停止产生目标；需要排除故障并重启该控制会话。实际动作以实体机械臂为准。

### 5. 正常停止

在启动真机控制的终端按 `Ctrl+C`，等待输出 `Stopped. FR3 command watchdogs no longer receive targets.`；随后关闭所选机械臂 FCI、SteamVR 和 ALVR Dashboard。

## 验证记录与实现入口

- 2026-10-20：原始 OpenVR 手柄位姿可无损录成 JSONL，左右手样本须按 `side` 分开分析。
- 2026-10-23：完成离线 IK、ROS 适配器和左臂真机前检查；早期网页预览、A/B 阶段报告仅代表当时的验证状态。
- 2026-10-25：ALVR/SteamVR 连接、实体左 Touch 选择、Y 按键、真实左臂平移与旋转、第二次 Y 回位均由现场验证。FR3 通信稳定性改善包括本机调度修复、控制线程与网卡 IRQ 分核、RX ring 4096。
- 2026-10-25：加入左／右／双臂选择，双手柄持续输入与左 Y 统一控制；完成 ROS 构建、无机器人连接的三模式脚本检查和离线双臂协调测试。右臂与双臂真机运动、回位、FCI 实时性尚待现场验证。
- 清理旧文件后运行当前 OpenVR、坐标映射、IK、Y 回位及 ROS 命令门控的相关测试，共 **52 个通过**；`--check-only` 无机器人连接检查通过。三轴相对旋转经过实际 IK 路径的离线检查，现场操作者随后确认动作流畅。

### 从离线预览到真机的关键变化

1. 早期左臂预览使用 mock／网页和较小的平移比例，只能说明输入与 IK 方向可观察；当前真机入口直接使用严格 IK 结果。平移比例最终设为 `1.0`，额外 TCP 径向限制关闭。
2. ALVR 升至 Quest 与 client 同为 20.14.1，SteamVR 使用上述版本；OpenVR 桥接明确选择实体左 Touch。Quest 系统手部追踪或虚拟手柄的角色信息不能替代实体控制器位姿。
3. OpenVR 输入积压时先合并并丢弃旧位姿；正常连续移动使用经现场调节的位姿连续性阈值。第一次 Y 锚定实测位姿，第二次 Y 使用测得的初始关节位置回位，可继续下一轮。
4. 左臂安装标定先用于平移，随后加入同一桌面坐标系中的三轴相对旋转；手柄原地转动现在会改变严格 IK 的 TCP 朝向目标。
5. 首次真机联调曾遇到 FCI 通信中断，即使不按 Y 也可能出现，因此定位到控制机调度／网卡时延。当前 client 使用本机重编的 Franka hardware、CPU 4 控制线程、CPU 5 网卡 IRQ、RX ring 4096 和性能档。此处记录的是已成功运行的组合，不代表任意机器具备同样的实时性。

主要代码：`data_collection/meta_quest/openvr_bridge.py` 选择双实体 Touch；`data_collection/meta_quest/coordinates.py` 处理 Unity 到桌面坐标；`data_collection/fr3_ik/live_input.py` 配对输入和保留左 Y 按键边沿；`data_collection/fr3_ik/live_control.py` 处理每侧 Y 锚点、相对六自由度位姿和回位；`quest3_ros2/src/franka_vr_adapter/franka_vr_adapter/multi_live_ik_node.py` 协调所选侧，双臂中任一侧故障会停止两侧；`quest3_ros2/src/franka_vr_adapter/` 的每侧适配器处理严格 IKResult 到 ROS 的受控发布；`scripts/start_quest3_hardware.sh` 是三模式真机启动入口。历史试验过程可以从 Git 历史查看，当前文档以本页为准。

## 仓库中的其他工作

- `data_collection/DATA_COLLECTION_README.md`：GELLO/Franka 数据采集和回放。
- `train/TRAIN_README.md`：训练。
- `deploy/DEPLOY_README.md`：策略部署。
- `calibration/`：机械臂安装及坐标标定；左臂遥操使用其中的桌面标定。
- `quest3_ros2/`：Quest ROS 2 适配器；与 GELLO ROS 工作区分开。
- `lerobot/`、`gello_software/` 等为 Git 子仓库；首次检出时运行 `git submodule update --init --recursive`。

`recordings/` 中的原始采样和机器人快照是实验数据。三模式运行时日志会在 `outputs/quest3_hardware/` 自动新建；历史日志清理不影响新会话，也不代表已有原始实验数据可删除。双臂 1 kHz FCI 线程当前沿用 client 上 CPU 4 的已跑通配置；同时运行两臂的实时调度与网络时延仍须在真机现场观察。
