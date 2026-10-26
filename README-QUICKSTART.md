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

戴上 Quest，打开 ALVR 20.14.1，通过 WiFi 连接电脑，保持 `Streaming`，握住并抬起两只实体 Touch 手柄，使两只手柄都处于头显可追踪范围内；即使只控制左臂也需要如此。仅显示 `Streaming` 不代表手柄追踪有效，未抬起两只手柄时可能出现 `0/30 valid OpenVR poses`。关闭 Quest 系统菜单，使 ALVR 保持前台。三种模式都会先检查并持续读取双手柄；实体左手柄 Y 是唯一接管／回位键。

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

脚本先报告左右实体 Touch 均为 `30/30 valid OpenVR poses`，再启动所选机械臂。等待 `Hardware chain ready` 和所选侧的 `LEFT_SOURCE_STATUS`／`RIGHT_SOURCE_STATUS` 显示 `"state":"READY"`；双臂模式必须**两侧都 READY**，此时才按左 Y。

第一次短按左 Y 接管所选机械臂：`--left` 跟随左手柄，`--right` 跟随右手柄，`--duo` 同时跟随两只手柄。第二次短按左 Y 返回本轮各自的接管点；回位完成并松开 Y 后可再次接管。右手柄 B 不触发接管。若双臂中任一侧显示适配器 `STOP`，在启动终端按 `Ctrl+C`，排除故障后重启会话。

## 4. 结束

在真机启动终端按 `Ctrl+C`，等待 `Stopped`；再关闭 FCI、SteamVR 和 ALVR Dashboard。

## 5. 左臂 VR 数采入口（入口已实现，跨机采集待验收）

完成上述性能和 VR 准备后，可用统一数采 client 入口替代第 3 节的启动命令；两者只启动一个。首版仅支持 Quest 左臂 arm-only，仍需两只 Touch 有效追踪。

先做离线配置检查（不连接机器人）：

```bash
bash scripts/start_data_collection_client.sh --teleop-source quest --left --arm --check-only
```

client 真机启动，沿用第 3 节已验证的 DDS 环境：

```bash
env -u CYCLONEDDS_URI RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 \
  bash scripts/start_data_collection_client.sh \
  --teleop-source quest --left --arm --enable-hardware
```

server 更新主仓库与 gello_software 子仓库后，需要重新构建 server ROS 包（新增了 bridge 的 teleop_source 参数）：

```bash
bash scripts/build_data_collection_server.sh
bash scripts/start_data_collection_server.sh \
  --teleop-source quest --left --arm
```

server 使用 `config/data_collection/quest_left.yaml`，只要求左臂和 cam_left / cam_front；action 来源为 `/left/vr/accepted_joint_states`。等 client READY 后按左 Y 进入跟随，server 才能收到 accepted target；Quest 额外话题各等待最多 120 秒。该检查验证收到消息，FOLLOWING 录制门控和跨机同步验收属于后续步骤。

后续跨机及录制边界验收时可添加 `--record --local-dir ./lerobot_data/quest_left`；默认 repo id 为 `local/franka_quest_left_teleop`，动作元数据记录 `teleop_source: quest`。当前尚未实现回位／故障自动切断 episode，请勿把入口检查通过视为训练数据验收通过。

省略 `--teleop-source` 继续使用原 GELLO 模式。
