# Meta Quest 输入模块

当前真机操作与软件版本见仓库根目录的 [完整说明](../../README.md) 和 [快速启动](../../README-QUICKSTART.md)。本目录负责 Touch 位姿读取、协议、录制与坐标转换；它本身不连接 Franka。

## 当前输入路径

```text
Quest 3 实体 Touch → ALVR 20.14.1 → SteamVR/OpenVR
  → openvr_bridge.py → PoseReceiver
  → 原始 JSONL 录制，或双手柄配对后所选 FR3 的 live_control.py
```

`openvr_bridge.py` 按 ALVR 实体 Touch 的序列号选择控制器，避免虚拟手部追踪设备占用左右手角色时误选。三种真机模式均以 `--global-left-y` 持续发送两只实体手柄的位姿和按键；实体左手柄 Y 提供所选臂的接管／回位，右 B 不触发接管。手柄位姿仍须是有效 OpenVR tracking pose。ALVR 必须处于 `Streaming`，应用保持前台，头显保持唤醒。

原始位姿使用 Quest/Unity 类坐标；`coordinates.py` 把位置转为桌面轴 `+X 前、+Y 左、+Z 上`，把旋转矩阵按 `B R Bᵀ` 换基。真机控制在第一次 Y 时以当前实测 TCP 和手柄位姿作相对锚点。

## 独立原始录制

`scripts/record_meta_quest_openvr.sh` 在机器人断开时录制原始 JSONL；它与真机遥操命令链是不同进程。输出使用临时 `.partial`、完成摘要及原子重命名。按 `side` 分析左右手样本，不把交替写入的两手位置连成一条轨迹。

这里的 `grasp`、`clutch`、`record` 是输入协议字段；当前真机入口只使用左 Y 接管／回位，尚不执行夹爪或 episode 录制。`--right` 与 `--duo` 的现场真机测试尚未完成。历史录制和机器人快照位于 `recordings/`，清理运行日志时应保留这些原始数据。
