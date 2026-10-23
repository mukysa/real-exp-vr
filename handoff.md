# DexArena / real-exp-vr handoff

更新时间：2026-10-23。

## 已验证

- A/B 使用 `data_collection/fr3_ik/interfaces.py` 中唯一的严格 `IKResult`。
- `AdapterCore` 与 ROS wrapper 共用同一门控路径；ROS runtime 已移除 synthetic source。
- 18 类离线故障注入全部拒绝，`hardware_command_published=false`。
- 接口序列化 round-trip 的标准库单测通过；新增/修改 Python 文件通过 compileall。
- client 核心契约/门控 27 项测试（另 10 个 subtests）通过；运动学/轨迹回放 10 项测试
  （另 3 个 subtests）通过。
- client 上 `franka_vr_adapter` 独立构建通过，包级 2 项测试通过；没有启动任何 ROS 节点。
- 既有 A 线 109 项完整回归是在 client ROS/Franka 环境中的上一个已记录基线，本地 Windows
  快照缺少 numpy/scipy、ROS 与完整 GELLO submodule，不能在本机冒充重跑通过。

## 尚未验证

- 专用左手 5 mm 三轴小动作录制的正向通过率和严格残差。
- 左/右 controller-to-wrist lever 实物标定。
- 新鲜逐帧 measured state、controller accepted target 和真实 watchdog/停止行为。
- ROS adapter 与真实话题的在线连接。
- 自碰撞、双臂互碰、桌面、夹爪和环境碰撞。
- 任何真机运动。

## 下一步

1. 用户按 `docs/AB-Offline-Pipeline-20261023.md` 采集左手小动作和新左臂只读 snapshot。
2. 运行 `scripts/validate_ab_offline_pipeline.py`，审阅四个输出文件。
3. 正向全部通过且故障注入保持 18/18 拒绝后，再进入真机第 1 级只读检查。

## 阻碍

当前工程侧没有新的阻碍。最终 Go/No-Go 仍由用户后续提供的专用左手 5 mm 三轴小动作录制和
新左臂只读 snapshot 阻塞；在收到这两份输入前保持 NO-GO。
