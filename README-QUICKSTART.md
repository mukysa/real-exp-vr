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

戴上 Quest，打开 ALVR 20.14.1，连接电脑，保持 `Streaming`，握住两只实体 Touch 手柄。关闭 Quest 系统菜单，使 ALVR 保持前台。三种模式都会先检查并持续读取双手柄；实体左手柄 Y 是唯一接管／回位键。

## 3. 选择机械臂并启动

确认所选机械臂 Desk 绿色、FCI 开启、工作空间空旷且急停可触及；双臂模式要分别确认左右臂。在新的 client 终端先执行：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
```

然后按目标**只执行一条**：

左臂：

```bash
bash scripts/start_quest3_hardware.sh --left --enable-hardware
```

右臂：

```bash
bash scripts/start_quest3_hardware.sh --right --enable-hardware
```

双臂：

```bash
bash scripts/start_quest3_hardware.sh --duo --enable-hardware
```

脚本先报告左右实体 Touch 均为 `30/30 valid OpenVR poses`，再启动所选机械臂。等待 `Hardware chain ready` 和所选侧的 `LEFT_SOURCE_STATUS`／`RIGHT_SOURCE_STATUS` 显示 `"state":"READY"`；双臂模式必须**两侧都 READY**，此时才按左 Y。

第一次短按左 Y 接管所选机械臂：`--left` 跟随左手柄，`--right` 跟随右手柄，`--duo` 同时跟随两只手柄。第二次短按左 Y 返回本轮各自的接管点；回位完成并松开 Y 后可再次接管。右手柄 B 不触发接管。若双臂中任一侧显示适配器 `STOP`，在启动终端按 `Ctrl+C`，排除故障后重启会话。

## 4. 结束

在真机启动终端按 `Ctrl+C`，等待 `Stopped`；再关闭 FCI、SteamVR 和 ALVR Dashboard。
