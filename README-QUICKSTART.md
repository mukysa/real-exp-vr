# Quest 3 左手柄控制左 FR3：快速启动

适用于已配置好的 client（`/home/landau/real-exp-vr`）与左 FR3（`172.16.0.3`）。详细记录和注意事项见 [README.md](README.md)。

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

戴上 Quest，打开 ALVR 20.14.1，连接电脑，保持 `Streaming`，握住实体左 Touch 手柄。

## 3. 启动左 FR3

确认左臂 Desk 绿色、FCI 开启、工作空间空旷且急停可触及。在新的 client 终端执行：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
bash scripts/start_quest3_left_hardware.sh --enable-hardware
```

等待 `Hardware chain ready` 和 `SOURCE_STATUS ... READY`。左 Y 第一次接管；移动或转动左手柄；第二次左 Y 回到接管点。回位完成、松开 Y 后可再次接管。

## 4. 结束

在真机启动终端按 `Ctrl+C`，等待 `Stopped`；再关闭 FCI、SteamVR 和 ALVR Dashboard。
