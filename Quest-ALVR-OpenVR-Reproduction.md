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

当前控制方式：

- 只控制左 FR3，不控制右臂。
- 左手柄位置映射比例为 `1.0`，即 1:1 相对位移。
- 不启用额外的 TCP 球形活动范围限制，机械臂自身关节、IK 和控制器约束仍然生效。
- 第一次按左手柄 Y：记录当时机械臂实测位置和手柄位置，并开始接管。
- 第二次按左手柄 Y：机械臂返回本轮接管时记录的初始位置。
- 回位完成并松开 Y 后，可以再次按 Y 开始下一轮接管，无需重启终端。

## 1. 真机和现场准备

1. 清空左 FR3 周围工作空间，确保人员不在机械臂运动范围内。
2. 将 Franka Desk 急停放在操作者随手可触及的位置。
3. 在 Franka Desk 中解锁左臂并开启 FCI。
4. 保持 FCI 开启，确认没有其他程序控制左臂。
5. 戴好 Quest 3，确认左 Touch 手柄在 Quest 主界面中能够正常平移、转动并显示射线。

## 2. 启动 ALVR Dashboard

在 client 本地图形桌面打开一个终端，执行：

```bash
cd /home/landau/real-exp-vr
./tools/alvr-v20.8.1/bin/alvr_dashboard
```

保持 Dashboard 窗口和终端打开。

## 3. 启动 SteamVR

在 client 本地图形桌面再打开一个终端，执行：

```bash
/home/landau/.local/share/Steam/steam.sh -applaunch 250820
```

等待 SteamVR 完成启动。

## 4. 建立 Quest Streaming

在 Quest 3 中执行：

1. 打开 ALVR。
2. 连接已经信任的 client 电脑。
3. 保持 ALVR 在前台并持续佩戴头显。
4. 等待电脑端 ALVR Dashboard 显示 `Streaming`。
5. 再次移动左手柄，确认其位置和射线持续正常更新。

当前 ALVR 配置关闭手势模拟输入，使用实体 Touch 控制器作为遥操输入。

## 5. 可选的无动作检查

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

## 6. 启动左臂真机遥操

确认 FCI 和 ALVR Streaming 都已保持开启后，在新终端执行：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
bash scripts/start_quest3_left_hardware.sh --enable-hardware
```

保持该终端运行。链路准备完成时会显示：

```text
OpenVR bridge dry-run OK; robot_output_enabled=false
Starting LEFT FR3 hardware controller only (172.16.0.3).
Hardware chain ready. Keep the arm clear. First left Y takes over; second left Y returns.
Press Ctrl+C at any time to stop the chain.
```

终端进入以下状态后即可操作：

```text
SOURCE_STATUS {..."reason":"ready; press left Y to take over"..."state":"READY"...}
```

## 7. 遥操操作顺序

1. 将左手柄放在舒适、稳定的位置。
2. 短按并松开左手柄 Y，机械臂从当前位置开始接受相对位移控制。
3. 看到 `state":"FOLLOWING"` 后，缓慢移动左手柄进行 1:1 位置遥操。
4. 再次短按并松开 Y，机械臂平滑返回本轮接管时记录的位置。
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

## 8. 正常停止

实验结束时，在启动遥操的终端按一次：

```text
Ctrl+C
```

等待终端显示：

```text
Stopped. The FR3 command watchdog is no longer receiving targets. Logs: ...
```

确认遥操进程已经退出后，再结束 FCI、SteamVR 和 ALVR Dashboard。

## 一行启动命令

当 FCI、SteamVR 和 ALVR Streaming 都已经准备好时，直接执行：

```bash
cd /home/landau/real-exp-vr && conda activate lerobot-vr && bash scripts/start_quest3_left_hardware.sh --enable-hardware
```
