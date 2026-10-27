# Quest 3 左臂遥操：快速启动

本页只列出在 client（`/home/landau/real-exp-vr`）现场跑通的左手柄控制左 FR3 流程。完整记录和注意事项见 [README.md](README.md)；双机有线通信的无硬件验证见 [M2-NETWORK-SETUP.md](M2-NETWORK-SETUP.md)。

## 1. client 开机后设置性能

```bash
cd /home/landau/real-exp-vr
sudo powerprofilesctl set performance
printf 'performance\n' | sudo tee /sys/devices/system/cpu/cpu{4,5}/cpufreq/scaling_governor
sudo ethtool -G enp114s0 rx 4096
sudo systemctl stop irqbalance
awk -F: '/enp114s0($|-)/ {gsub(/[[:space:]]/, "", $1); print $1}' /proc/interrupts |
  while read -r irq; do printf '5\n' | sudo tee "/proc/irq/${irq}/smp_affinity_list" >/dev/null; done
```

左 FR3 控制器线程使用 CPU 4，FR3 网卡中断使用 CPU 5；本机的控制器配置已设置好。

## 2. 启动 VR

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

戴上 Quest，打开 ALVR 20.14.1 并连接 client，保持 `Streaming`。关闭 Quest 系统菜单，让 ALVR 保持前台。握住并抬起**两只实体 Touch 手柄**，确保都在头显追踪范围内。

## 3. 启动左臂

确认左臂 Franka Desk 显示绿色、FCI 已开启，工作空间安全且急停可触及。在新的 client 终端运行：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
env -u CYCLONEDDS_URI \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ROS_DOMAIN_ID=0 \
  bash scripts/start_quest3_hardware.sh --left --enable-hardware
```

等待左右实体 Touch 均显示 `30/30 valid OpenVR poses`，以及 `Hardware chain ready: left`、`LEFT_SOURCE_STATUS` 中的 `"state":"READY"`。短按一次**左 Y** 接管左臂；移动和转动左手柄控制左臂。再短按一次左 Y 返回本轮接管点；回位完成并松开 Y 后，可以再次接管。

## 4. 结束

在真机启动终端按 `Ctrl+C`，等待 `Stopped`，再关闭 FCI、SteamVR 和 ALVR Dashboard。

## 已验证的不连接机械臂检查

若只需检查双机 Quest 数采入口的 client 配置，可在完成 VR 准备后运行：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
bash scripts/start_data_collection_client.sh --teleop-source quest --left --arm --check-only
```

此命令已通过无硬件检查，不连接 FR3。双机有线 DDS、ROS 话题和 server 本机 ZMQ 已通过合成消息验证，验证范围及命令见 [M2-VALIDATION.md](M2-VALIDATION.md)。
