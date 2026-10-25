# Quest 3 ROS 2 适配器

本工作区承载 Quest 3 到 FR3 的 ROS 2 指令链，与 `gello_software/ros2` 分离。当前已跑通的**左 FR3 真机入口**是仓库根目录的 `scripts/start_quest3_left_hardware.sh --enable-hardware`；按 [README-QUICKSTART.md](../README-QUICKSTART.md) 操作，完整配置和重要边界见 [README.md](../README.md)。

`left_live_control.launch.py` 启动左臂 IK 源和安全适配器。IK 源只发布严格 `fr3_ik_result_v1`，适配器收到新鲜机器人实测状态并通过独立检查后，才向左臂 raw 关节目标 topic 发布。控制器自身还有 1 kHz 参考生成器和通信 watchdog。现有真机入口不启动右臂、不控制夹爪。

构建工作区：

```bash
cd /home/landau/real-exp-vr
bash scripts/build_quest3_ros2.sh
```

`scripts/start_quest3_teleoperation_client.sh` 是组件级入口，不负责完整的左臂 FCI/控制器/VR 启动顺序；现场请使用上面的根目录真机脚本。ROS 适配器拒绝与同一机械臂上的 GELLO raw-command publisher 并行发送。
