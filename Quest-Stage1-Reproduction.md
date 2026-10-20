# Quest 阶段一历史验收记录

> 历史文档，不是当前启动教程。当前唯一操作手册为
> [Quest-ALVR-OpenVR-Reproduction.md](Quest-ALVR-OpenVR-Reproduction.md)。

## 历史结论

2026-10-17 曾使用 controller-enabled Hand Tracking Streamer APK、USB/ADB 和 TCP 8000 完成 Quest 3 左右 Touch 手柄的输入验收：双侧位置、四元数及控制字段均可到达，未启动机器人输出。

随后实际设备上的 APK 版本不再提供 `Controllers Only`，且该路径存在 Wi-Fi 门禁、controller UI、追踪有效性和全速持久化等问题，因此不再作为当前复现入口。原始命令、ADB 设置和 APK 页面操作已从本文移除，避免与当前 ALVR/OpenVR 流程混用；需要审计历史时请查看 Git 历史。

## 当前替代方案

当前已跑通并维护的输入路径为：

```text
Quest Touch
  → ALVR/SteamVR
  → direct OpenVR bridge
  → local TCP PoseReceiver
  → lossless JSONL raw recorder
```

请只按 [Quest-ALVR-OpenVR-Reproduction.md](Quest-ALVR-OpenVR-Reproduction.md) 操作。旧的 Hand Tracking Streamer/ADB 入口和 SAPIEN SteamVR bridge 仅保留在代码与 Git 历史中用于诊断，不作为采集启动方式。

## GELLO 与安全边界

- 原 GELLO pipeline、入口和 topic 保持不变。
- Quest 采集仍是机器人断开的输入验证，不启动 ROS、Franka、MANUS、Sharpa 或 pair1 recorder。
- 当前正式 episode 遇到 ALVR/TCP 断流必须整条重采，不跨连接拼接。
- Quest → FR3 IK、fake hardware 和真实机器人验证仍属于后续阶段。
