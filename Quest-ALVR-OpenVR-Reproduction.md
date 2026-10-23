# Quest 3 + ALVR/OpenVR 原始手柄数据采集复现

更新日期：2026-10-20。

本文用于从零复现已经跑通的安全输入链路：

```text
Quest 3 左右 Touch 手柄
  → Quest ALVR client
  → Wi-Fi/TCP
  → client ALVR Dashboard + SteamVR + ALVR driver
  → 后台 OpenVR bridge
  → 本机 TCP 127.0.0.1:8000
  → raw recorder
  → lossless JSONL
```

该流程只保存 Quest 手柄原始输入，不启动 ROS、GELLO、Franka、MANUS、Sharpa 或 pair1 recorder，也不会向机器人发送目标。

## 0. 操作约定

- 下列电脑命令都在 client `/home/landau/real-exp-vr` 中执行。
- ALVR Dashboard 与 SteamVR 是图形程序；从 client 本地图形桌面的终端启动最稳定。通过 VS Code SSH 操作时，如果窗口没有显示，就到 client 桌面启动。
- 每次录制在启动命令里使用新的 `.jsonl` 文件名；录制器不会覆盖已有文件。
- 采集期间持续佩戴并唤醒 Quest，保持 Quest 内 ALVR 位于前台，电脑端 Dashboard 必须显示 `Streaming`。
- 正常结束只在采集终端按一次 `Ctrl+C`，然后等待出现 `Saved ... poses` 和命令提示符。

## 1. 进入项目并检查 Python/OpenVR 环境

打开终端，逐条执行：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
unset PYTHONPATH PYTHONHOME
pwd
python --version
./scripts/start_meta_quest_openvr_bridge.sh --dry-run
```

最后一条只检查依赖，不连接手柄、不录数据。正确结果包含：

```text
OpenVR bridge dry-run OK; robot_output_enabled=false
```

## 2. 检查旧进程和 TCP 8000

执行：

```bash
ss -ltnp | grep ':8000' || echo "TCP 8000 is free"

pgrep -af 'raw_recorder|openvr_bridge|steamvr_bridge|record_meta_quest_openvr' \
  || echo "No old Quest recording process"
```

正常结果是端口空闲且没有旧采集进程。如果输出进程号，先回到旧采集终端用 `Ctrl+C` 正常结束；不要直接删除 `.partial` 文件，也不要在不知道 PID 含义时使用 `pkill`。

## 3. 启动 ALVR Dashboard

在 client 图形桌面另开一个终端，执行：

```bash
cd /home/landau/real-exp-vr
./tools/alvr-v20.8.1/bin/alvr_dashboard
```

保持这个终端和 Dashboard 窗口打开。Quest 尚未连接时显示 `Disconnected` 是正常的。

如果 Dashboard 已经开着，只检查一次，不要重复启动：

```bash
pgrep -af alvr_dashboard
```

## 4. 启动 SteamVR

在 client 图形桌面再开一个终端，执行：

```bash
/home/landau/.local/share/Steam/steam.sh -applaunch 250820
```

等待 SteamVR 状态小窗口出现，然后检查运行进程：

```bash
pgrep -af 'vrserver|vrmonitor|vrcompositor'
```

至少应看到 `vrserver` 和 `vrmonitor`。只有 Steam 主程序而没有这两个进程，说明 SteamVR runtime 还没有真正启动。

如果 SteamVR 重启后完全看不到 ALVR external driver，执行一次：

```bash
/home/landau/.local/share/Steam/steamapps/common/SteamVR/bin/vrpathreg.sh \
  adddriver /home/landau/real-exp-vr/tools/alvr-v20.8.1/lib64/alvr
```

正常情况下不需要每次重复注册。编码保持已经跑通的 H.264 software encoding；`HEVC profile does not contain encoding entry` 是 HEVC 能力提示，不是 JSONL 采集端口错误。

## 5. 在 Quest 中建立 ALVR Streaming

这一部分在头显中操作：

1. 戴好并唤醒 Quest，确认左右 Touch 手柄在 Quest 系统界面中都能移动。
2. 打开 Quest 内的 ALVR。
3. 选择已信任的电脑 `5056.client`；如电脑端出现确认提示，选择 Trust/Accept。
4. 保持 ALVR 在头显前台并继续佩戴，直到电脑端 Dashboard 显示 `Streaming`。

网络异常时可在 client 检查本次已知 Quest 地址：

```bash
ping -c 2 192.168.52.70
```

IP 可能随网络变化；应以 ALVR Dashboard 当前显示为准。

## 6. 启动正式录制

再次确认 Quest 仍佩戴、ALVR 在前台且 Dashboard 为 `Streaming`，然后执行：

本轮正式执行命令为：

```bash
./scripts/record_meta_quest_openvr.sh \
  --output recordings/quest_raw/<new-recording-name>.jsonl
```

每次录制必须使用新的文件名；录制器不会覆盖已有 JSONL。

正常启动过程应包含：

```text
Starting validated raw recorder on tcp:8000...
Recording every accepted Quest pose on 127.0.0.1:8000
Starting direct ALVR/OpenVR controller bridge...
OpenVR controller ids: {} -> {'left': 1, 'right': 2}
poses=... written=... rejected=0 dropped=0
```

只有 `poses` 和 `written` 持续增加，才表示数据真的在写入。然后完成计划动作，例如左右手柄平移、转动、trigger、A/X、B/Y。不要通过 ALVR Dashboard 的图像流是否流畅来代替 pose 计数检查。

## 7. 正常结束并等待安全落盘

在正在输出 `poses=...` 的采集终端按一次：

```text
Ctrl+C
```

不要连续按键，也不要立刻关闭终端。当前修复后的脚本会把中断转发给后台 raw recorder，等待它写完 summary 并把 `.jsonl.partial` 原子改名。正确结束包含：

```text
Saved <数量> poses to recordings/quest_raw/<文件名>.jsonl; dropped=0
```

修复提交为：

```text
9988dac fix(quest): finalize OpenVR capture on interrupt
```

如果 ALVR 在采集中离开 `Streaming`，OpenVR bridge 会停止而不是继续写缓存姿态，录制器随后安全收尾。但正式 episode 应判为中断并整条重采，不拼接断线前后的数据。

## 8. 校验新 JSONL

仍在同一个终端执行：

```bash
python -m data_collection.meta_quest.replay \
  recordings/quest_raw/stage8-openvr-guided-20261020.jsonl \
  --summary-only

grep -c '"side":"right"' \
  recordings/quest_raw/stage8-openvr-guided-20261020.jsonl
grep -c '"side":"left"' \
  recordings/quest_raw/stage8-openvr-guided-20261020.jsonl

sha256sum recordings/quest_raw/stage8-openvr-guided-20261020.jsonl
tail -1 recordings/quest_raw/stage8-openvr-guided-20261020.jsonl

test ! -e recordings/quest_raw/stage8-openvr-guided-20261020.jsonl.partial \
  && echo "No partial file remains"
```

合格文件应满足：

- replay 显示 `schema=1, lossless`；
- 左右手数量都大于 0；
- 最后一行含 `"record_type":"summary"` 和 `"completed":true`；
- `rejected_pose_records` 与 `subscriber_dropped_records` 都为 0；
- `accepted_connections` 为 1；
- 没有同名 `.partial` 文件。

最后确认采集进程已经退出：

```bash
pgrep -af 'raw_recorder|openvr_bridge|steamvr_bridge|record_meta_quest_openvr' \
  || echo "All Quest recording processes stopped"
```

## 9. 结束设备软件

确认 JSONL 合格后，可从图形界面依次退出 SteamVR 和 ALVR Dashboard，再退出 Quest 内的 ALVR。不要在 JSONL 仍为 `.partial` 时先关闭整个 client。

## 10. 2026-10-20 完整人工复现实测

本流程已由操作者从环境检查开始完整走通，最终文件为：

```text
recordings/quest_raw/stage6-openvr-guided-20261020.jsonl
```

验收结果：

```text
文件大小：404,675 bytes
pose 总数：928
左手：464
右手：464
accepted_connections：1
rejected：0
dropped：0
completed：true
duration：16.065424114000052 s
SHA-256：0c6ba8544968137151d45e580722eeec4fd138a40db2213ecf742a1046fd3ed7
```

回放结果为：

```text
Loaded 928 poses, schema=1, lossless
```

因此当前“启动环境 → ALVR/SteamVR Streaming → 一键录制 → 单次 `Ctrl+C` → 完整 JSONL → 回放校验”的复现链路已经通过。
