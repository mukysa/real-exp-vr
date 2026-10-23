> 历史归档：2026-10-21。本文保留原记录的时间语境，不作为当前进度或启动指令。最新工作入口见 [A 线当前状态与下一步](../A线-当前状态与下一步.md)；代码块中的项目相对路径仍以项目根目录为基准。

# FR3 + Franka 夹爪离线运动学核对

日期：2026-10-20。范围：单右臂名义模型、关节约定与 FK/IK 自洽性；未连接真机。

## 配置结论

- `scripts/start_teleoperation.sh` 的右臂夹爪分支使用 `example_fr3_right_config.yaml`：`arm_id=fr3`、`arm_prefix=right`、`namespace=right`、`load_gripper=true`。
- description 来自现有只读依赖 `/home/landau/franka_ros2_ws/install/franka_description/share/franka_description`。
- 硬件模型关节为 `right_fr3_joint1..7`；控制器发布消息名称为 `fr3_joint1..7`，目标回调按 position 数组前七项读取。因此顺序必须始终为 joint1 到 joint7，单位为 rad。
- 名义基坐标系为 `right_fr3_link0`，法兰为 `right_fr3_link8`，夹爪 TCP 为 `right_fr3_hand_tcp`。
- 当前 launch 沿用 xacro 的工具默认参数：法兰到名义 TCP 的旋转为 `Rz(-π/4)`，平移为 `[0, 0, 0.1034]` m。验证程序从完整模型计算此变换，不在 IK 中另写一套工具常量。
- 原有 IK 关节上下限与控制器 `JointReferenceGenerator` 的有效范围一致；验证入口会比较两者，避免后续配置漂移。速度、加速度限制也写入报告，但本次单点求解没有验证动态约束。

## 复现入口

在项目根目录执行：

```bash
bash scripts/validate_fr3_kinematics.sh
# 可选：保存新报告，已有文件不会覆盖
bash scripts/validate_fr3_kinematics.sh --output /tmp/fr3-audit-new.json
# 可选：指定另一组合法关节姿态
bash scripts/validate_fr3_kinematics.sh --q 0.3 -0.6 0.2 -2.0 0.1 1.8 -0.2
```

脚本仅在子进程内加载已有 ROS 环境，使用 `/usr/bin/python3`，不修改父 shell、环境配置文件或依赖，不安装软件、不启动 ROS 节点、不建立机器人连接。默认仅输出 JSON；指定 `--output` 时写入新文件。

## 验证内容与结果

1. 读取并核对右臂配置、关节顺序和控制器限位。
2. 展开含夹爪的右臂模型；夹爪关节固定为零位，仅供运动学验证。
3. 比较完整模型的 TCP FK 与原有七关节模型 FK 乘以工具变换，矩阵差异要求不超过 `1e-10`。
4. 用 `q₀` 计算目标，对 IK 种子施加扰动，再求解原目标，避免仅以原解为种子的平凡检查。
5. 对解再次执行 FK，并使用完整模型交叉检查；位置和姿态残差均要求不超过 `1e-5`（m / rad）。

默认 `q₀ = [0, -0.4, 0, -1.8, 0, 1.5, 0.7]` rad，名义 TCP 位置约为 `[0.425887, 0, 0.602186]` m。首次运行位置残差约 `1.55e-12` m，姿态残差按现有计算函数为 `0` rad，优化器报告 18 次函数评估。七自由度冗余允许解与原始 q 不完全相同。

JSON 包含展开后模型 SHA-256、模型位置、工具变换、输入与求解关节、误差及耗时。单次耗时包含懒加载导入，不代表稳定运行时延，也不构成 50 Hz 等在线频率的验收。

专项测试文件 `test/test_fr3_kinematics_audit.py` 覆盖两组姿态、名义工具变换、FK/IK 回环，以及越界、NaN 和错误维度的输入拒绝。

## 仍待确认

现有 `move_to_target_ee.py` 从 `/{side}/franka_robot_state_broadcaster/robot_state` 读取 `f_t_ee.pose` 与 `o_t_ee.pose`。实际 EE 由机器人配置决定，不能仅根据 URDF 断言实际 `F_T_EE` 与名义 TCP 相同。

后续读取同一份状态对应的 q、`F_T_EE`、`O_T_EE`，验证 `FK_flange(q) @ F_T_EE` 与 `O_T_EE`。本次没有读取这些数据，报告明确标记 `actual_F_T_EE_verified=false`。此项不阻塞名义模型上的离线连续 IK 开发，但真机接入前必须核对。

本次未验证：连续轨迹、奇异点、碰撞、实际跟踪误差、fake hardware 或真机控制。下一项为合成末端轨迹上的连续 IK、失败保持与稳态时延统计。
