# Quest 3 + ALVR/OpenVR + 左 FR3 真机遥操复现

更新日期：2026-10-25。

本文只记录当前已经跑通的左臂真机遥操路径：

```text
Quest 3 左 Touch 手柄
  → Quest ALVR client
  → client ALVR Dashboard + SteamVR + ALVR driver
  → OpenVR bridge
  → 左臂实时 IK
  → 安全适配器与阻抗控制器
  → 左 FR3（172.16.0.3）
```

已验证的软件组合：Quest 与 client 上的 ALVR 均为 `20.14.1`；SteamVR 为
`previous` 分支 `2.17.10`（build `25330290`）。当前配置快照保存在
`/home/landau/.local/state/real-exp-vr-backups/working-vr-20261025-133855`。

当前控制方式：

- 只控制左 FR3，不控制右臂。
- 左手柄位置映射比例为 `1.0`，即 1:1 相对位移。
- 左手柄三轴旋转按当前桌面标定映射到机械臂末端，角度比例为 1:1。
- 不启用额外的 TCP 球形活动范围限制，机械臂自身关节、IK 和控制器约束仍然生效。
- 第一次按左手柄 Y：记录当时机械臂实测位姿和手柄位姿，并开始相对位姿接管。
- 第二次按左手柄 Y：机械臂返回本轮接管时记录的初始关节位置。
- 回位完成并松开 Y 后，可以再次按 Y 开始下一轮接管，无需重启终端。

## 1. 真机和现场准备

1. 清空左 FR3 周围工作空间，确保人员不在机械臂运动范围内。
2. 将 Franka Desk 急停放在操作者随手可触及的位置。
3. 在 Franka Desk 中解锁左臂并开启 FCI。
4. 保持 FCI 开启，确认没有其他程序控制左臂。
5. 戴好 Quest 3，确认左 Touch 手柄在 Quest 主界面中能够正常平移、转动并显示射线。

## 2. 准备 FR3 控制机的实时通信

在 client 本机终端执行以下命令。当前成功运行时，FR3 控制线程固定在 CPU 4，
连接左臂的 `enp114s0` 网卡中断固定在 CPU 5，接收环为 4096。
重启电脑后重新执行本节命令：

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

`controllers.yaml` 中的 `cpu_affinity: [4]` 用于当前左 FR3 的 1 kHz 控制器。
该文件位于 `gello_software` 子仓库，以上命令会在首次使用时设置它。

## 3. 启动 ALVR Dashboard

在 client 图形桌面确认 Steam 已运行。尚未运行时，从 X11 环境启动：

```bash
env -u WAYLAND_DISPLAY DISPLAY=:0 /home/landau/.local/share/Steam/steam.sh -silent
```

随后在 client 本地图形桌面打开另一个终端，执行：

```bash
cd /home/landau/real-exp-vr
env -u WAYLAND_DISPLAY DISPLAY=:0 ./tools/alvr-v20.14.1/bin/alvr_dashboard
```

Quest 端使用 `ALVR 20.14.1`。确认 Dashboard 中已信任的 Quest IP 与头显当前 IP 一致。
保持 Dashboard 窗口和终端打开。

## 4. 启动 SteamVR

SteamVR 尚未运行时，在 client 本地图形桌面再打开一个终端，执行：

```bash
env -u WAYLAND_DISPLAY DISPLAY=:0 /home/landau/.local/share/Steam/steam.sh -applaunch 250820
```

当前配置已启用 `disableLinuxWaitForPresent`，并指向 ALVR 20.14.1 的 OpenVR 驱动。等待 SteamVR 完成启动。

## 5. 建立 Quest Streaming

在 Quest 3 中执行：

1. 打开 Quest 上的 ALVR 20.14.1。
2. 连接已经信任的 client 电脑。
3. 保持 ALVR 在前台并持续佩戴头显。
4. 等待电脑端 ALVR Dashboard 显示 `Streaming`，并让头显保持唤醒。
5. 拿起实体左 Touch 手柄，保持它在头显摄像头视野内。

当前 ALVR 配置的 `Headset → Controllers → Hand skeleton` 已关闭
（`hand_skeleton.enabled=false`），OpenVR 使用实体 Touch 手柄位姿。
在 client 上运行以下无动作检查：

```bash
cd /home/landau/real-exp-vr
bash scripts/start_meta_quest_openvr_bridge.sh --check-left-touch
```

确认输出 `Physical left Touch: 30/30 valid OpenVR poses` 后再继续。
透视混合已设为
`Blend`、`threshold=1.0`、`premultiplied_alpha=false`：佩戴时看到真实环境，
VR 串流画面完全透明，Streaming 和实体左手柄位姿仍保持有效。

## 6. 可选的无动作检查

需要在连接真机前检查软件环境时，在新终端执行：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
bash scripts/start_quest3_left_hardware.sh --check-only
```

正确结果为：

```text
OpenVR bridge dry-run OK; robot_output_enabled=false
CHECK OK: left-only configuration, Y button, strict IK source, adapter, and VR controller files are available.
No robot connection was opened.
```

## 7. 启动左臂真机遥操

确认 FCI 和 ALVR Streaming 都已保持开启后，在新终端执行：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
bash scripts/start_quest3_left_hardware.sh --enable-hardware
```

保持该终端运行。链路准备完成时会显示：

```text
OpenVR bridge dry-run OK; robot_output_enabled=false
Physical left Touch: 30/30 valid OpenVR poses; id=...
Starting LEFT FR3 hardware controller only (172.16.0.3).
Hardware chain ready. Keep the arm clear. First left Y takes over; second left Y returns.
Press Ctrl+C at any time to stop the chain.
```

终端进入以下状态后即可操作：

```text
SOURCE_STATUS {..."reason":"ready; press left Y to take over"..."state":"READY"...}
```

## 8. 遥操操作顺序

1. 将左手柄放在舒适、稳定的位置。
2. 短按并松开左手柄 Y，记录当前机械臂和手柄位姿，开始接管。
3. 看到 `state":"FOLLOWING"` 后，缓慢移动和转动左手柄，分别进行 1:1 平移和三轴旋转遥操。
4. 再次短按并松开 Y，机械臂平滑返回本轮接管时记录的关节位置。
5. 看到 `return complete` 和 `state":"READY"` 后，本轮回位完成。
6. 需要继续时，重新摆好手柄并再次短按 Y，即可开始下一轮接管。

接管状态示例：

```text
SOURCE_STATUS {..."reason":"following left controller; second Y returns home"..."state":"FOLLOWING"...}
ADAPTER_STATUS {..."reason":"accepted"..."state":"RUNNING"...}
```

回位完成状态示例：

```text
SOURCE_STATUS {..."reason":"return complete; release Y before next takeover"..."state":"READY"...}
```

## 9. 正常停止

实验结束时，在启动遥操的终端按一次：

```text
Ctrl+C
```

等待终端显示：

```text
Stopped. The FR3 command watchdog is no longer receiving targets. Logs: ...
```

确认遥操进程已经退出后，依次结束 FCI、SteamVR，再关闭 ALVR Dashboard。
Quest 端会随 SteamVR 退出而离开 `Streaming`。

## 一行启动命令

当 FCI、SteamVR 和 ALVR Streaming 都已经准备好时，直接执行：

```bash
cd /home/landau/real-exp-vr && conda activate lerobot-vr && bash scripts/start_quest3_left_hardware.sh --enable-hardware
```
