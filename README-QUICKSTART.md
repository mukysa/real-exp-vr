# Quest 3 控制 FR3：快速启动

适用于已配置好的 client（`/home/landau/real-exp-vr`）。`--left` 控制左 FR3（`172.16.0.3`），`--right` 控制右 FR3（`172.16.0.2`），`--duo` 控制双臂。左臂已完成现场真机验证；右臂与双臂已完成离线检查，仍待现场真机测试。详细记录和注意事项见 [README.md](README.md)。

## 1. client 开机后设置性能

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

## 2. 启动 Steam、ALVR Dashboard、SteamVR

分别在 client 图形桌面终端执行；已运行的程序无需重复启动。

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

戴上 Quest，打开 ALVR 20.14.1，通过 WiFi 连接电脑，保持 `Streaming`，握住并抬起两只实体 Touch 手柄，使两只手柄都处于头显可追踪范围内；即使只控制左臂也需要如此。仅显示 `Streaming` 不代表手柄追踪有效，未抬起两只手柄时可能出现 `0/30 valid OpenVR poses`。关闭 Quest 系统菜单，使 ALVR 保持前台。默认三种模式都会先检查并持续读取双手柄；实体左手柄 Y 是接管／回位键。显式启用 `--hand-arm` 时改为右 B，见第 5.5 节。

### 2.1 按键前置验证

按键检查默认使用 OpenVR Background 直接读取双手柄，不再创建 SAPIEN
`Scene`/`RenderVRDisplay`，也不调用渲染、合成或提交 VR 视频帧。正式
遥操入口同样通过 `start_meta_quest_openvr_bridge.sh` 读取位姿和按键，避免
在仅有 Intel 集显的 client 上执行应用侧 Vulkan 画面渲染。SteamVR 和
ALVR 仍须运行，Quest 仍须连接并保持 ALVR `Streaming`；OpenVR 位姿和
手柄输入继续使用。这里关闭的是应用侧 VR 画面渲染，不是关闭 SteamVR、
ALVR 或控制器追踪。

在启动任何真机控制或数采入口前，建议先运行只读按键检查：

```bash
cd /home/landau/real-exp-vr
bash scripts/check_quest3_buttons.sh
```

脚本会等待 ALVR `Streaming`，然后通过和正式 bridge 相同的 OpenVR 输入路径读取两只 Touch；它不会打开 TCP 连接，不启动 ROS，也不会发送任何机器人控制。看到类似下面的输出后，依次按下并释放两只手柄的按键：

```text
controllers=(1, 2) side_ids={'left': 1, 'right': 2}
left: signal=OK mask=0x2 pressed=Y trigger=0.00
right: signal=OK mask=0x2 pressed=B trigger=0.00
```

左手柄应将 Y 显示为 `pressed=Y`，右手柄对应显示为 `pressed=B`；`signal=OK` 仅表示设备 ID 存在、按键和扳机轴调用返回了可读值，不保证物理输入正在刷新。确认完成后按 `Ctrl+C`，再继续第 3 节或第 5 节。

检查脚本默认每 2 秒输出状态和 `TIMING`：包括实际循环频率 `loops_hz`、循环最大间隔，以及各阶段平均／最大耗时（毫秒）。`--poll-hz` 是轮询频率上限。需要比较旧的 SAPIEN 渲染链路时，可传 `--backend sapien`，但这会重新启用 VR 画面渲染。

排查卡顿、扳机恒为零或遥操的 `Quest input gap` 时，先退出遥操入口，运行以下独立检查并保存日志；`--check-pose` 会额外读取位姿，检查 NaN、无穷或零范数四元数：

```bash
bash scripts/check_quest3_buttons.sh --check-pose 2>&1 | tee /tmp/quest3-buttons-diagnostic.log
```

- `STALL stage=... call_not_returned=true`：当前调用尚未返回且已经超过默认 250 ms。后台观察线程不调用 VR API；如果原生调用一直占用 Python GIL，报警可能延迟。
- `SLOW stage=... duration_ms=...`：调用返回后记录的实际耗时。OpenVR 后端主要包含 `controller_ids`、左右手的 `buttons`／`axis` 和可选的 `pose` 阶段。
- `LOOP_GAP`／`READ_GAP`：循环或同一只手的成功轴读取间隔超过默认 2 秒。它测量本地调用完成间隔，不能证明远端跟踪时间戳新鲜，也不包含遥操的 TCP、ROS、IK 链路。
- `signal=DEGRADED ... trigger_source=button_fallback axis_error=...`：扳机轴读取失败或返回非法值，显示的是按扳机按钮位回退的 0／1，并非正常模拟轴数值。`pose=ERROR(...)` 表示额外位姿检查失败；`pose=FINITE` 仅表示数值有限且四元数非零。

可用 `--report-interval-s 2 --stall-ms 250 --input-gap-s 2` 调整诊断阈值；这些参数仅影响检查脚本的报告，不修改正式遥操的超时或接管条件。

按键检查新增的诊断记录保存在本仓库的 `vr_debug/` 下，
该目录已加入 `.gitignore`。`VR_DEBUG_CAPTURE=0` 仅关闭独立观察进程，
`VR_DEBUG_DISABLE=1` 则关闭新增的文件诊断记录和独立观察进程；例如
`VR_DEBUG_DISABLE=1 bash scripts/check_quest3_buttons.sh --duration-s 30 --check-pose`。
这两个设置均不改变 OpenVR 输入或渲染后端。

## 3. 选择机械臂并启动

确认所选机械臂 Desk 绿色、FCI 开启、工作空间空旷且急停可触及；双臂模式要分别确认左右臂。在新的 client 终端先执行：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
```

然后按目标**只执行一条**：

左臂：

```bash
env -u CYCLONEDDS_URI \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ROS_DOMAIN_ID=0 \
  bash scripts/start_quest3_hardware.sh --left --enable-hardware
```

右臂：

```bash
env -u CYCLONEDDS_URI \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ROS_DOMAIN_ID=0 \
  bash scripts/start_quest3_hardware.sh --right --enable-hardware
```

双臂：

```bash
env -u CYCLONEDDS_URI \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ROS_DOMAIN_ID=0 \
  bash scripts/start_quest3_hardware.sh --duo --enable-hardware
```

脚本最多等待 5 分钟：左臂模式确认左实体 Touch `30/30 valid poses and button reads`；右臂和双臂模式确认左右两只都为 `30/30`，随后才启动所选机械臂。运行中仍监听双手柄按键。等待 `Hardware chain ready` 和所选侧的 `LEFT_SOURCE_STATUS`／`RIGHT_SOURCE_STATUS` 显示 `"state":"READY"`；双臂模式必须**两侧都 READY**，此时才按左 Y。

默认保留原有 `standing` 位姿参考系，禁用渲染本身不会切换坐标原点。
如需试用 debug 版本新增的启动时头显位置原点，可给启动命令追加
`--pose-reference=headset-start-position`；它只冻结启动时头显的位置，
仍保留 standing／标定坐标轴，并将参考信息写入本次日志目录的
`headset-start-reference.json`。此选项与禁用 VR 渲染相互独立。

第一次短按左 Y 接管所选机械臂：`--left` 跟随左手柄，`--right` 跟随右手柄，`--duo` 同时跟随两只手柄。第二次短按左 Y 返回本轮各自的接管点；回位完成并松开 Y 后可再次接管。右手柄 B 不触发接管。若双臂中任一侧显示适配器 `STOP`，在启动终端按 `Ctrl+C`，排除故障后重启会话。

## 4. 结束

在真机启动终端按 `Ctrl+C`，等待 `Stopped`；再关闭 FCI、SteamVR 和 ALVR Dashboard。

### 4.1 回放或测试后的左臂归位

`scripts/replay.sh` 会在回放开始前移动到 episode 初始状态，但回放结束后不会自动回到固定归位姿态。需要单独归位时，先预览硬件 fallback 目标：

```bash
cd /home/landau/real-exp-vr
python3 data_collection/reset_pylibfranka.py \
  --left \
  --skip-grippers \
  --dry-run
```

确认目标姿态和现场安全条件后，执行左臂归位：

```bash
python3 data_collection/reset_pylibfranka.py \
  --left \
  --skip-grippers
```

执行前应确认没有其他 FR3 控制器、回放进程或真机入口正在运行。当前 `lerobot_data/quest_left` 的 `observation.state` 是 7 维左臂数据，而该归位脚本要求 16 维双臂加夹爪状态，因此不要把 `quest_left` 作为 `--dataset-root` 传给归位脚本；这里使用的是代码中记录的本机硬件 fallback 姿态。

## 5. 左臂 VR 双机采集验证

完成第 1、2 节的准备后，用下面的 **client 数采入口替代第 3 节的独立遥操命令**；不要同时启动两个真机入口。Quest 数采目前只支持左臂，可选 arm-only 或 gripper。client 负责左 FR3 控制，机器人状态和 accepted target 经有线 ROS 2 DDS 到 server；server 只采集左手相机 `cam_left`，并在本机 `127.0.0.1:5555` 发布 ZMQ 样本，录制文件保存在 server。

### 5.1 client：检查并启动左臂

先确认左臂 Desk 绿色、FCI 开启、工作空间安全，两只实体 Touch 都在 ALVR 中有效追踪。在 client 终端执行：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
bash scripts/start_data_collection_client.sh --teleop-source quest --left --arm --check-only
bash scripts/start_data_collection_client.sh --teleop-source quest --left --arm --enable-hardware
```

第一个命令不连接机械臂；第二个命令最多等待 5 分钟，直到左实体 Touch 达到 `30/30 valid OpenVR poses`，然后才连接左 FR3。右手柄可以静置，运行中仍会监听双手柄。等待 `Hardware chain ready` 和左臂 `READY`。这个入口自动加载 client 的有线 DDS 配置，不要加第 3 节的 `env -u CYCLONEDDS_URI`。

### 5.2 server：启动相机和数据桥

在 server（`/home/pair1/real-exp-vr`）的另一个终端执行：

```bash
cd /home/pair1/real-exp-vr
bash scripts/start_data_collection_server.sh \
  --teleop-source quest --left --arm --ros-domain-id 0
```

server 当前的 ROS 采集包已按源码构建。若之后改动了 GELLO bridge 或相机源码，先在 server 执行 `bash scripts/build_data_collection_server.sh`。server 收到左臂实测状态、VR 状态和 `cam_left` 图像后，会在**按 Y 前**报告 `Quest left observation path ready before Y` 和 `Data-server stack is running`，同时输出零动作 ZMQ 观测样本。此时再由你短按一次左 Y 接管并移动左手柄；接管和回位期间样本使用真实的 `/left/vr/accepted_joint_states`。server 不初始化也不控制机械臂。

### 5.3 server：检查并录制数据

先在 server 的第二个终端检查 60 秒 ZMQ 样本；按 Y 前即可开始检查：（可选）

```bash
cd /home/pair1/real-exp-vr
conda activate lerobot-vr
python -m data_collection.diagnostics.samples \
  --host 127.0.0.1 --port 5555 \
  --duration 60 --silence-timeout 5 \
  --output outputs/m2/live-samples.json
```

收到有效样本后，在 server 的另一个终端启动**交互式录制器**：

```bash
cd /home/pair1/real-exp-vr
env -u PYTHONHOME -u PYTHONPATH -u LD_LIBRARY_PATH \
  /home/pair1/miniconda3/envs/lerobot-vr/bin/python \
  -m data_collection.lerobot_collection \
  --host 127.0.0.1 --port 5555 \
  --repo-id local/franka_quest_left_teleop \
  --local-dir ./lerobot_data/quest_left_test1
```

左臂处于 `FOLLOWING` 且录制器收到新样本后，在**录制器终端**输入 `s` 回车开始一个 episode；完成动作后输入 `e` 回车保存，再按第二次左 Y 回位。需要丢弃当前 episode 时输入 `d` 回车；结束录制输入 `q` 回车。先停录制器，再分别在 server 和 client 启动终端按 `Ctrl+C`。

双机真实相机、真实状态和录制结果仍需这次现场验证。左 Y 不会自动开始或结束 episode；目前也没有回位／故障自动切断录制的门控，保存的数据先作为链路验证材料，不直接视为可用训练数据。省略 `--teleop-source` 继续使用原 GELLO 模式。

### 5.4 夹爪数采

Quest 左臂夹爪数采需要 client 和 server 两端都使用 `--gripper`；两端参数必须保持一致。client 端执行：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
bash scripts/start_data_collection_client.sh \
  --teleop-source quest --left --gripper --enable-hardware
```

server 端执行：

```bash
cd /home/pair1/real-exp-vr
conda activate lerobot-vr
bash scripts/start_data_collection_server.sh \
  --teleop-source quest --left --gripper --ros-domain-id 0
```

`--gripper` 会启动并 homing 左 Franka Hand，把左扳机映射为夹爪开合，并将夹爪状态和动作写入 8 维 LeRobot 数据。录制完成后，可在 client 上复制 server 的数据集：

```bash
mkdir -p ~/real-exp-vr/lerobot_data
scp -rp pair1@192.168.52.160:/home/pair1/real-exp-vr/lerobot_data/quest_left_test1 \
  ~/real-exp-vr/lerobot_data/
```

复制后的数据集位于 `~/real-exp-vr/lerobot_data/quest_left_gripper`。确认其 `meta/real_exp_trajectory_config.json` 中为 `"end_effector":"gripper"`、`"include_gripper":true` 后再回放。

### 5.5 左手套位姿预演：`--hand-arm`

该模式用于接入 Wuji 手前的左臂测试。左 Touch 可绑在手套上，只提供位姿；右 Touch 的位姿不参与机械臂目标计算。client 和 server 两端都增加 `--hand-arm`，限定 `--teleop-source quest --left`，自动选择 arm-only；与 `--gripper`、`--hand`、右臂、双臂组合会直接报错。当前不启动 Wuji 手或 Franka gripper。省略此参数继续走原有流程。

| 输入 | 行为 |
|---|---|
| 左 Touch 位姿 | 驱动左臂，保留现有坐标映射与实测接管锚定 |
| 左 Touch 所有按键/扳机 | 忽略 |
| 右 B 短按 | 接管；再次短按返回本轮接管点 |
| 右 A 短按 | 录制器空闲时开始；录制中结束并保存 |
| 右食指扳机 | 放弃当前未保存 episode，不删除历史 |
| server 录制器键盘 s/e/d/q + Enter | 保留；q 有未保存帧时按原流程保存后退出 |

先在 client 构建并做不连接机器人的入口检查：

```bash
cd /home/landau/real-exp-vr
bash scripts/build_quest3_ros2.sh
bash scripts/start_data_collection_client.sh \
  --teleop-source quest --left --hand-arm --check-only
```

现场测试时，用下面命令替换原 client 遥操入口：

```bash
bash scripts/start_data_collection_client.sh \
  --teleop-source quest --left --hand-arm --enable-hardware
```

server 同步本次代码，在仓库根目录启动采集和录制控制 relay：

```bash
bash scripts/start_data_collection_server.sh \
  --teleop-source quest --left --hand-arm --ros-domain-id 0
```

在 server 的另一个终端启动交互式录制器（推荐先用这一方式验收键盘兜底）：

```bash
conda activate lerobot-vr
python data_collection/lerobot_collection.py \
  --host 127.0.0.1 --port 5555 --hand-arm \
  --repo-id local/franka_quest_left_hand_arm_teleop \
  --local-dir ./lerobot_data/quest_left_hand_arm_test1
```

也可给 server 启动命令增加 `--record --local-dir ./lerobot_data/quest_left_hand_arm_test1` 自动启动录制器，此时 server 启动终端保留键盘输入，q 退出录制器后 supervisor 会结束 server 栈。手动和自动方式只选一个录制器。控制端口默认为 server 本机 `5559`，若修改，server 与手动录制器均传 `--record-control-port PORT`。数采样本仍走原有 `5555`；跨机录制指令和回执走既有有线 DDS。

先松开右侧 B/A/扳机，待录制器可用后按 **B 接管 → 确认 FOLLOWING → A 开录 → 移动左 Touch → A 保存 → 等待保存完成 → B 回位**。弃用一条时，把保存步骤换成右食指扳机。每次按键只处理按下沿，长按不重复；触发器默认达到 0.75 触发，低于 0.25 后重新使能。ROS launch 提供 `trigger_press`/`trigger_release` 参数供后续调试。

client 的 `LEFT_SOURCE_STATUS` 包含 `button: physical right B`、`right_control_fresh` 和 `recorder` 状态；详细回执同时写入启动时输出的日志目录中的 `quest_control.log`。两端 ROS 终端也可只读查看回执：

```bash
ros2 topic echo /left/vr/record_status std_msgs/msg/String
```

开始录制要求新鲜采集样本、左臂 FOLLOWING 和有效右控制输入；开始被拒后，下一次 A 仍尝试开始。结束/丢弃不要求相机新帧或左臂继续跟随。保存中忽略操作；保存失败会报告 error，停止继续加帧，先在录制器终端检查错误，再用 e 重试或 d 放弃。无有效帧时不保存空 episode。

启动时已按住的键、重连后已按住的键均须先释放；过期、重复和旧录制状态的事件会被拒绝，不在恢复时重放。两台机器需通过 NTP/chrony 同步系统时钟，远程事件有效期为 1 秒。右输入超时或控制栈故障后需重启 client 栈，恢复数据不会自动接管。B 不自动结束录制；先保存/丢弃，再回位。退出录制器后，在 client 启动终端按 Ctrl+C 停止臂控制。

本次完成离线与本机传输验证；真实双机 DDS、相机、手柄和左臂的组合验收仍待现场进行。

## 6. LeRobot 数据回放

回放在连接 FR3 的 client 上执行。先激活环境并进入仓库：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
```

先做不启动 ROS、不连接机器人的检查：

```bash
bash scripts/replay.sh \
  --dataset-root ~/real-exp-vr/lerobot_data/quest_left_test1 \
  --episode 0 \
  --left \
  --gripper \
  --dry-run
```

确认输出为 `trajectory setting: gripper / left` 后，执行真实回放：

```bash
bash scripts/replay.sh \
  --dataset-root ~/real-exp-vr/lerobot_data/quest_left_gripper \
  --episode 0 \
  --left \
  --gripper
```

脚本会先启动左 FR3 和 Franka Hand，等待你输入 `s`，再将机械臂和夹爪移动到 episode 初始状态并播放动作。看到 `Starting LeRobot episode replay` 后不要输入 `q`，等待 episode 自然结束。回放结束后脚本不会自动移动到固定归位姿态；需要归位时使用第 4.1 节的左臂归位命令。
