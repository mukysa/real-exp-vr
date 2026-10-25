# FR3 IK 与实时左臂控制

仓库根目录的 [README.md](../../README.md) 记录当前真机链路，[README-QUICKSTART.md](../../README-QUICKSTART.md) 是唯一快速启动入口。本目录包含离线 IK、轨迹回放、只读预览和左臂实时目标生成；ROS 发布由 `quest3_ros2/src/franka_vr_adapter/` 单独负责。

## 当前真机数据流

```text
左 Touch OpenVR PoseSample
  → Unity/桌面换基 → LeftLiveControl（Y 锚点、相对六自由度位姿、回位）
  → ContinuousIK 严格 IKResult
  → ROS 安全适配器 → 左臂阻抗控制器
```

`live_control.py` 在第一次 Y 时读取新鲜的实测关节状态，保存 `home_q`、`home_pose` 和手柄 `source_anchor`。后续平移先转到桌面坐标，再按左臂安装标定转回基座；旋转按相同标定做空间相对旋转，三个轴均为 1:1。第二次 Y 使用受限关节参考返回 `home_q`，回位完成并松开 Y 后可再次接管。

`interfaces.py` 定义 `IKRequest`、`IKResult`、时间域和关节目标；`solver.py` 使用 `kinematics.py` 的模型和 `constraints.py` 的关节／残差检查。诊断用的 `DiagnosticIKResult` 不可作为硬件命令。`preview_mapping.py` 提供坐标变换；`live/` 是旧的网页只读预览工具，不是真机启动入口。

当前 live 控制默认由 `scripts/start_quest3_left_hardware.sh` 以 `position_scale=1.0`、禁用额外 TCP 径向上限启动。严格 IK、适配器、控制器和 watchdog 仍各自检查输入；这里没有碰撞规划，也没有夹爪或 episode 记录。

## 开发验证

`test/test_fr3_live_control.py` 验证 Y 接管／回位、桌面标定、三轴旋转进入 IK；`test/test_fr3_ik_constraints.py` 与 `test/test_fr3_ik_offline.py` 验证数值约束和离线求解。测试需加载 ROS 2 Humble 与本机 Franka 安装，以系统 `/usr/bin/python3` 运行。测试文件不由真机启动脚本导入。
