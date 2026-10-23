# A 线离线可行性复核

复核日期：2026-10-23。复核目录：`/home/landau/real-exp-vr`。

## 结论

A 线作为“Quest 输入到安全、可审计的离线 FR3 关节参考”是可行的。当前代码能够解析双手输入，完成坐标映射、接管锚定、双臂 IK、受限关节参考、故障保持和只读网页预览。

A 线目前不能作为真机控制输入。最新 V4/Stage12 输出仍明确标记 `DIAGNOSTIC_ONLY` 和 `control_eligible=false`。这不是一个待删除的保守标志，而是对现状的准确描述：离线参考没有闭合真实 measured/accepted feedback，严格残差、实时长尾、碰撞和 A/B 接口仍未通过。

## 本次实际检查

在不启动 ROS 节点、不连接机器人和不修改环境的条件下，使用项目脚本依赖的 ROS Humble、Franka 工作空间和系统 Python 运行 A 线测试：

```bash
unset PYTHONPATH PYTHONHOME AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
source /home/landau/franka_ros2_ws/install/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -p no:cacheprovider \
  -q <A-line test selection>
```

结果：`109 passed, 1 warning`，耗时 119.09 秒。警告来自 ROS xacro 对 Python 3.11 私有变量行为的兼容提示，不影响本次 Python 3.10 测试结果。

通过范围包括：

- Quest 坐标、协议模型、OpenVR 输入和安全状态机；
- A/B 草案接口的数据约束；
- FR3 关节、步长、速度、加速度和残差检查；
- 严格离线 IK、A3 双臂回放和 V4 映射；
- DexArena 微分跟随的数学回归；
- FR3 模型/TCP 审计；
- 实时只读预览引擎。

## 已确认可行的部分

1. **输入链可用。** ALVR/SteamVR → OpenVR → TCP/JSONL 已有真实录制，左右手、连接 ID、主机接收时间和 tracking 状态能够进入离线处理。
2. **坐标和通道约定基本成立。** 左手对应 left/.3，右手对应 right/.2；V4 使用桌面 `+X 前、+Y 左、+Z 上`。Stage12 的前/左/上实录支持轴符号，没有整轴反向或左右交换证据。
3. **接管不会首帧跳变。** Stage11 和合成停止/恢复场景的接管首帧关节变化为零，HOLD、故障锁定和重新锚定有测试覆盖。
4. **关节参考受限。** 离线参考遵守当前关节范围、步长、速度和加速度限制，诊断结果与严格 `IKResult` 分离。
5. **预览没有命令路径。** `data_collection/fr3_ik/live/server.py` 只读反馈和显示；默认 mock，真机读取要求显式 `--connect-readonly`。`--single-left` 只打开左臂 reader。

## 当前问题

### P0：阻止真机控制的问题

- **A/B 接口未闭合。** A 线 `fr3_ik.interfaces.IKResult` 与 B 线 `meta_quest.models.IKResult` 是两套结构，没有经过验证的转换、joint contract 和 HOLD/FAULT 映射。
- **真实反馈闭环缺失。** A3 离线回放的 `measured_joint_state` 为 `null`；没有 controller accepted target 回灌。离线虚拟参考不能代替机器人 measured state。
- **严格残差不通过。** Stage11 V4 严格残差失败左 658 帧、右 681 帧；Stage12 大动作失败左 1139 帧、右 1134 帧，最大模型位置滞后约 38.3/77.0 mm。
- **碰撞约束缺失。** 当前检查覆盖关节和末端工作空间，不覆盖自碰撞、双臂互碰、桌面、夹爪和任务物体碰撞。
- **停止责任未闭合。** A 线能给出 HOLD/FAULT；真实控制器如何保持、停止、接受目标和恢复必须由 B 线与控制器共同定义和验证。

### P1：影响可靠性的问题

- **20 ms 周期有长尾。** Stage11 双臂 step 最大约 44.5 ms，有 2 次超过 20 ms，不能声明稳定 50 Hz。
- **时间域尚未贯通。** OpenVR frame/device 字段来自主机轮询，不能证明设备生成了新帧；A/B 仍需显式转换 source、host monotonic 和 ROS time。
- **物理偏航和腕部旋转未定标。** Stage12 支持轴符号；约 10°偏航候选不一致，未应用。腕部旋转没有同等级实录验收。
- **工具和起点必须现场刷新。** 历史快照只适合复现，不能代替每次启动时的新鲜 q、`F_T_EE` 和 controller 状态。
- **夹爪未进入 A 线控制契约。** trigger 可观察，但当前 IK/预览没有真实夹爪命令与反馈闭环。

### P2：工程与文档问题

- `ms3-enviroment.yaml` 声明环境名 `ms3`，但 prefix 是另一台机器的 `/home/pair/miniconda/envs/ms3`；client 上没有安装 `ms3` Conda 环境。A 线实际依赖项目验证脚本加载的 ROS Humble + `franka_ros2_ws` + `/usr/bin/python3`。
- 默认系统 Python 没有 Pinocchio；`lerobot-vr` 有 Pinocchio但没有 xacro。直接运行 `python -m pytest` 会在收集阶段失败，必须使用文档化的环境入口。
- 历史输出曾占用约 2.2 GB，包含大量重复 HTML、samples、对照版本和浏览器 profile，容易让“演示结果”与“控制证据”混淆。
- 文档此前同时引用 V2/V3/V4、Stage7/10/11/12 多个入口，部分“待确认”表述已被后续现场结论覆盖。

## 保留的唯一示例

仅保留：

```text
outputs/fr3_ik/stage12-direction-audit-20261022/
```

它使用最新真实双手录制，包含方向分析、原始分析数据、可复现脚本、SVG 曲线和一组 V4 双臂回放。它适合说明坐标方向、接管区间和诊断边界。

该示例仍是离线诊断，不是控制验收。`replay/playback.html` 较大是因为嵌入了完整轨迹数据；项目不再保留其他重复 HTML/仿真输出。

## 本轮停止位置

本轮只完成离线复核、输出清理、文档整理和 Git 快照，不继续实现 A/B 适配，也不进入 fake 或真机阶段。后续恢复开发时，先解决 P0 的接口、反馈、残差、碰撞和停止责任，再讨论真机运动。
