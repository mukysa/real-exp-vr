# Meta Quest 输入模块

当前真机操作与软件版本见仓库根目录的 [完整说明](../../README.md) 和 [快速启动](../../README-QUICKSTART.md)。本目录负责 Touch 位姿读取、协议、录制与坐标转换；它本身不连接 Franka。

## 当前输入路径

```text
Quest 3 实体 Touch → ALVR 20.14.1 → SteamVR/OpenVR
  → openvr_bridge.py → PoseReceiver
  → 原始 JSONL 录制，或双手柄配对后所选 FR3 的 live_control.py
```

`openvr_bridge.py` 按 ALVR 实体 Touch 的序列号选择控制器，避免虚拟手部追踪设备占用左右手角色时误选。默认三种真机模式均以 `--global-left-y` 持续发送两只实体手柄的位姿和按键；实体左手柄 Y 提供所选臂的接管／回位。手柄位姿仍须是有效 OpenVR tracking pose。ALVR 必须处于 `Streaming`，应用保持前台，头显保持唤醒。

显式启用 `--hand-arm` 时，bridge 只发送左 Touch 位姿（左按键置空）及独立的 `right buttons|f=...|t=...:trigger,B,A` 记录。右记录 `source=buttons, tracked=False`，不会伪装成有效位姿；右按钮可读时不要求右位姿有效。`HandArmInput` 在位姿合并前提取边沿：右 B 接管/回位，A 请求录制切换，食指扳机优先丢弃。长按只触发一次，启动/重连按住须先松开。只支持左臂 arm-only，启动方法见快速启动第 5.5 节。

录制请求通过 `/left/vr/record_event` 到 server relay，再经本机 ZMQ 到录制器；结果由 `/left/vr/record_status` 返回。录制器决定 A 是开始还是保存，并用 recorder ID、revision、session 和 sequence 拒绝重复/旧请求。两端须校时；有效期为 1 秒，不跨机比较 monotonic 时钟。此链路不依赖相机新帧或左位姿。

输入桥使用 OpenVR Background 直接读取位姿和按键，不创建 SAPIEN 场景或 VR 渲染目标，也不访问 VRCompositor、不提交 VR 视频帧。SteamVR、ALVR 和头显追踪仍须工作；这绕过的是电脑侧应用渲染路径，不会关闭控制器追踪。

原始位姿使用 Quest/Unity 类坐标；`coordinates.py` 把位置转为桌面轴 `+X 前、+Y 左、+Z 上`，把旋转矩阵按 `B R Bᵀ` 换基。真机控制在第一次 Y 时以当前实测 TCP 和手柄位姿作相对锚点。

## 独立原始录制

`scripts/record_meta_quest_openvr.sh` 在机器人断开时录制原始 JSONL；它与真机遥操命令链是不同进程。输出使用临时 `.partial`、完成摘要及原子重命名。按 `side` 分析左右手样本，不把交替写入的两手位置连成一条轨迹。

这里的 `grasp`、`clutch`、`record` 是输入协议字段，独立原始录制不会执行夹爪或 episode 控制。真机 `--gripper` 可用左扳机控制夹爪，新增 `--hand-arm` 使用上述右侧录制按键；这些行为均需通过相应启动参数显式启用。`--right`、`--duo` 和新 `--hand-arm` 的组合现场验收尚未完成。历史录制和机器人快照位于 `recordings/`，清理运行日志时应保留这些原始数据。
