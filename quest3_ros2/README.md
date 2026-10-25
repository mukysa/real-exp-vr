# Quest 3 ROS 2 适配器

本工作区承载 Quest 3 到 FR3 的 ROS 2 指令链，与 `gello_software/ros2` 分离。三模式入口为仓库根目录的 `scripts/start_quest3_hardware.sh --left|--right|--duo --enable-hardware`；按 [README-QUICKSTART.md](../README-QUICKSTART.md) 操作，完整配置和重要边界见 [README.md](../README.md)。左臂已真机验证；右臂、双臂仍待真机验证。

`quest_live_control.launch.py` 为每个所选侧启动独立安全适配器，并启动一个共享双手柄输入与左 Y 开关的 `multi_live_ik_node`。`--left`、`--right` 只启动所选侧；`--duo` 同时启动两侧，双臂接管须共同就绪，任一侧 IK 故障或适配器 STOP 会停止两侧发目标；适配器 STOP 后需排障并重启会话。IK 源只发布严格 `fr3_ik_result_v1`，每侧适配器收到新鲜机器人实测状态并通过独立检查后才向对应 raw 关节目标 topic 发布。控制器自身还有 1 kHz 参考生成器和通信 watchdog。`left_live_control.launch.py` 保留为旧左臂组件入口。当前不控制夹爪。

构建工作区：

```bash
cd /home/landau/real-exp-vr
bash scripts/build_quest3_ros2.sh
```

`scripts/start_quest3_teleoperation_client.sh` 是组件级入口，不负责完整的 FCI/控制器/VR 启动顺序；现场请使用上面的根目录真机脚本。ROS 适配器拒绝与同一机械臂上的 GELLO raw-command publisher 并行发送。
