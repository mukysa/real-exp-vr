# FR3 IK 离线链

本目录实现 Quest 目标到 FR3 关节参考的离线求解、约束检查、回放和只读预览。它没有机器人命令发布器；诊断输出不能直接送入控制器。

项目当前结论见 [A 线状态](../../A线-当前状态与下一步.md)，2026-10-23 的测试与问题见 [离线可行性复核](../../docs/A-Line-Offline-Feasibility-Review-20261023.md)。

## 数据流

```text
PoseSample
  → meta_quest 坐标/安全状态机
  → TargetPose
  → IKRequest
  → ContinuousIK
  → IKResult 或 DiagnosticIKResult
  → 离线 JSON/HTML 或只读网页
```

严格结果和诊断结果必须分开：

- `IKResult` 只有在输入有效、FK 残差和全部关节检查通过时才允许 `FOLLOWING + command`。
- `DiagnosticIKResult` 用于 DexArena 微分跟随与可视化，始终 `control_eligible=false`，没有 `command_for_adapter()`。
- HOLD/FAULT/REANCHOR_REQUIRED 不携带新 command；故障需要显式 reset 和重新锚定。

## 主要模块

| 文件 | 作用 |
|---|---|
| `interfaces.py` | 时间域、关节向量、measured state、请求和严格结果契约 |
| `kinematics.py` | FR3 xacro、Pinocchio FK/Jacobian 和 TCP 工具变换 |
| `constraints.py` | 残差、关节、步长、速度和加速度检查 |
| `solver.py` | 严格优化 IK 与离线微分跟随入口 |
| `dexarena_tracking.py` | DexArena DLS/零空间数学移植 |
| `joint_reference.py` | 受限参考、减速和停止 |
| `a3_replay.py` | 单/双臂 Quest JSONL 离线回放 |
| `preview_mapping.py` | 桌面轴和显示变换 |
| `live/` | mock 或 libfranka 只读反馈的本地网页预览 |

## 环境

client 当前没有安装名为 `ms3` 的 Conda 环境。仓库的 `ms3-enviroment.yaml` 是另一台机器的环境快照，不能直接代表 client 运行环境。

FR3 IK 使用项目验证脚本加载的环境：

```bash
unset PYTHONPATH PYTHONHOME AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
source /home/landau/franka_ros2_ws/install/local_setup.bash
```

原因是 `/usr/bin/python3` 通过 ROS 路径获得 xacro/Franka 包；`lerobot-vr` 虽有 Pinocchio但缺 xacro。不要为解决此问题升级或混装环境。

## 离线验证

严格合成轨迹：

```bash
bash scripts/validate_fr3_ik_offline.sh
```

真实 Quest 回放：

```bash
bash scripts/replay_fr3_ik_a3.sh \
  recordings/quest_raw/stage12-openvr-guided-20261020.jsonl \
  --initial-snapshot recordings/robot_state/initial-20261021-103516/snapshot.json \
  --side both --engage-mode shared --engage-side right \
  --tracking-mode dexarena_velocity --motion-mapping tabletop-relative \
  --output outputs/fr3_ik/<new-run>
```

输出目录必须是新目录。脚本不连接机器人，也不发布 ROS command。

2026-10-23 选定 A 线测试结果：

```text
109 passed, 1 xacro compatibility warning, 119.09 s
```

## 唯一示例

为避免历史 HTML 和重复仿真混淆，只保留：

```text
outputs/fr3_ik/stage12-direction-audit-20261022/
```

入口为 `index.html`。它包含真实 Stage12 输入的方向分析和一组 V4 双臂回放。结果仍是诊断用途：最大模型位置滞后约左 38.3 mm、右 77.0 mm，严格残差失败左 1139、右 1134 帧。

## 当前安全边界

- 左/右关节顺序是 `fr3_joint1..7`；硬件前缀由 B 线边界处理。
- 角度单位是 rad；位置是 m；四元数是 xyzw。
- V4 平移使用共同桌面轴 `+X 前、+Y 左、+Z 上`。
- 旧快照只用于复现，不能替代运行时的新鲜 q 和 `F_T_EE`。
- 离线 `measured_joint_state=None` 是明确的缺失值，不能用 q target 冒充反馈。
- `success` 只表示产生了通过 A 线检查的新目标，不等于 B 线允许发布。
- 当前没有自碰撞、双臂互碰、桌面/夹爪/物体碰撞检查。
- wall-clock timeout 是返回后的拒绝检查，不是硬实时截止保证。
- OpenVR device/frame 字段来自主机轮询，不能单独证明设备产生了新帧。

## 只读实时预览

详细操作见 [live/README.md](live/README.md)。默认 mock：

```bash
bash scripts/start_fr3_live_preview.sh \
  --demo-recording recordings/quest_raw/stage12-openvr-guided-20261020.jsonl
```

左臂只读模式只有在现场确认后才使用：

```bash
bash scripts/start_fr3_live_preview.sh \
  --single-left --robot-source franka --connect-readonly --position-scale 0.5
```

`server.py` 在 `--single-left` 下只创建 left/.3 reader。页面和日志没有 ROS publisher、libfranka control 或 gripper command。

## 接入 B 线前必须完成

1. 对齐 A/B 两套 `IKResult`、关节名、单位、时间和 HOLD/FAULT 语义。
2. 接入新鲜 measured state 和 controller accepted target。
3. 为残差、周期长尾、停止/恢复和碰撞设定可验收门槛。
4. 用真实 A 线 IK 完成隔离 fake 验证，而非只用 synthetic source。

完成这些项目以前，保持 `control_eligible=false`。
