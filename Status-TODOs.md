# VR 数采升级：现状与待办

更新日期：2026-10-23。本文只保留项目级长期待办；A 线现状以 [A 线入口](A线-当前状态与下一步.md) 和 [离线可行性复核](docs/A-Line-Offline-Feasibility-Review-20261023.md) 为准，历史过程见 [归档索引](z_histroy_plan/README.md)。

## 当前结论

本轮已继续完成 A/B 离线汇合。A 线严格 `IKResult` 现为唯一协议，B 线已提取为纯 Python `AdapterCore`，Quest ROS adapter 改为消费真实序列化 IK 结果，不再生成 synthetic target。18 类无硬件故障注入全部拒绝。详见 [A/B 离线报告](docs/AB-Offline-Pipeline-20261023.md)。

离线代码可行不等于真机可行。旧 V4/Stage12 输出仍为 `DIAGNOSTIC_ONLY`、`control_eligible=false`；它们不能进入新 adapter。新的正向 A/B 报告仍是 NO-GO，因为专用 5 mm 小动作数据、新鲜 measured/accepted feedback、严格残差、实时长尾、碰撞和真实停止行为尚未闭合。本轮没有启动 ROS/fake 或真机，也没有发布命令。

重复仿真和 HTML 已清理，只保留 `outputs/fr3_ik/stage12-direction-audit-20261022/` 作为坐标、方向和诊断边界示例。原始录制继续保留。

后续默认双臂处理，单臂为显式模式。client 在当前项目继续开发，VR ROS 产物独立放在 `quest3_ros2`，不混用 GELLO 构建目录；在 client 侧验证通过前，不推进 server VR bridge/recorder 改造，不重做已经通过的 GELLO 迁移。

其他 arm/gripper 组合暂不单独测试，不阻塞主线，也不标记为实机通过。本机 data/test 旧副本未确认同步 server 处理结果，不能据此否定 server 验收。

## 环境与隔离范围

| 项目 | Client | Server |
|---|---|---|
| 项目 | /home/landau/real-exp-vr | /home/pair1/real-exp-vr |
| 系统 ROS / Python | Humble / 3.10 | Jazzy / 3.12 |
| 独立 ROS 产物 | gello_software/ros2/{build,install,log}/client | gello_software/ros2/{build,install,log}/server |
| 职责 | GELLO、机器人及夹爪控制，发布状态与 accepted target | 相机、DDS 订阅、ROS → ZMQ bridge |
| Conda | GELLO arm/gripper 控制不调用 Conda；VR raw recorder 使用 lerobot-vr，OpenVR bridge 使用已有独立 Python 3.11 | recorder、预处理与校验使用 lerobot-vr |

- 本次 client 复核：干净系统环境下，franka_gello_state_publisher、franka_fr3_arm_controllers、franka_gripper_manager 的包前缀均位于当前项目 install/client。数采入口分别选择 install/client 和 install/server，并清理继承的旧路径。
- Server 已有本机记录确认 LeRobot editable 和普通导入指向 /home/pair1/real-exp-vr/lerobot，recorder 导入、H.264 写入与 TorchCodec 读回通过；迁移提交为 0eaf5dba4f377f25e4880cf2360ca9c6919281bc。本次 SSH 认证失败，未重新复核远端，端到端结果依据用户确认。
- 两端 ROS 产物分别本地构建；系统 ROS 与公共 Franka 工作空间继续作为只读依赖，不迁入 Conda。

- Client LeRobot 重绑已完成（2026-10-15）：lerobot-vr 的 editable 已绑定当前项目，清空 PYTHONPATH 后普通导入确认指向 /home/landau/real-exp-vr/lerobot/src/lerobot/__init__.py；未升级其他依赖。

- 回放与直接遥操作入口已统一（2026-10-15）：默认加载本项目 install/client，清理继承的 Conda/ROS 搜索路径，使用系统 ROS Python；保留 REAL_EXP_ROS_INSTALL_PREFIX 显式覆盖。Wuji 分支默认环境改为 lerobot-vr，仍允许 WUJI_CONDA_ENV / LEROBOT_CONDA_ENV 覆盖，不代表 Wuji 已验收。离线验证三个 ROS 包均来自当前 install/client，data/test episode 0 回放 dry-run 通过（331 帧、15 Hz）；本次未启动机器人。

### 未覆盖的边界

- Wuji editable、训练/部署依赖和历史 pip check 冲突未全面处理；历史完整 Python 回归未全通过。按后续功能需要处理，不重开已完成的 GELLO 采集迁移。

## 必要复现入口

完整参数、处理和回放步骤见 [DATA_COLLECTION_README.md](data_collection/DATA_COLLECTION_README.md)。以下命令仅作为复现入口，本次未启动设备；实机操作由用户安排。

按需在对应机器构建，不复制另一台机器的构建产物：

```bash
cd ~/real-exp-vr
# client
./scripts/build_data_collection_client.sh
# server
DATA_SERVER_ROS_DISTRO=jazzy ./scripts/build_data_collection_server.sh
```

双臂 Franka 夹爪启动，两端 ROS_DOMAIN_ID 保持一致：

```bash
# client
ROS_DOMAIN_ID=0 ./scripts/start_data_collection_client.sh --duo --gripper
# server：相机与 bridge
ROS_DOMAIN_ID=0 DATA_SERVER_ROS_DISTRO=jazzy ./scripts/start_data_collection_server.sh --duo --gripper
```

Server 另开终端前台录制，使用新目录，保留已通过的 data/test：

```bash
cd ~/real-exp-vr
conda activate lerobot-vr
python data_collection/lerobot_collection.py --host 192.168.50.13 --port 5555 --repo-id local/gello_baseline --local-dir data/new_episode
```

键盘 s/e/d/q 控制录制。已有基线为 schema-v2、15 Hz、三路视频。后续回归记录两端 git rev-parse HEAD、未提交改动与实际启动参数；预处理和校验在 server lerobot-vr 中执行，裁剪先 dry-run。

## 已确认的工作原则与修改边界

- **最小化修改**：只解决当前阶段的旧工程、旧环境依赖，不顺带重构或扩展功能。保留现有代码风格，优先使用已有入口和配置机制。
- **变更检查**：修改后运行 `git diff --check` 和与范围匹配的测试；生成输出不作为源码提交的一部分。
- **接口兼容**：保持 client/server 间 topic 名称、消息类型、关节名称及顺序、单位、坐标约定、动作语义和必要的时间行为一致。accepted target 必须仍来自控制器接受后的目标。
- **文件边界**：只修改 `/home/landau/real-exp-vr/` 内的工程文件和构建产物；原 real-exp、DexArena 和其他工程只读使用。写入前注意符号链接和生成文件，避免间接修改外部目录。
- **环境边界**：按当前用户要求不修改环境，不安装或升级依赖，不修改 Conda、系统 Python/ROS 或公共 Franka 工作空间。已有依赖可在子进程中加载，配置文件保持原样。
- **验证边界**：构建、静态配置检查、离线单元测试、fake hardware 联调和真实采集分别记录，不能互相替代。本阶段没有授权启动真实机器人控制，实机联调由用户另行安排。
- **按实际依赖处理**：不为使用 lerobot-vr 而把 ROS 强行迁入 Python 3.12，不整体安装 ms3 环境快照，不把 client 不使用的视频或训练依赖当作 GELLO 控制前置条件。
- **迁移与功能改造分开验收**：先建立原 GELLO 流程的隔离基线，再引入 VR，便于定位故障来源。
- **状态维护**：完成后就地更新结论并删除过时待办，不在开头累积历史日志或保留矛盾状态。

## GELLO 兼容契约与 VR 旁路边界

以下内容作为后续 client 改造的硬约束：

- 原 GELLO 入口 `scripts/start_data_collection_client.sh` 和 `scripts/start_teleoperation.sh` 的现有参数、默认行为及启动顺序保持不变；VR 首期不在这两个入口中增加默认分支。
- GELLO 命令与 accepted target 继续使用 `/{side}/gello/raw_joint_states` 和 `/{side}/gello/accepted_joint_states`，消息类型、七关节名称与顺序、弧度单位及控制器默认参数保持不变。
- 历史 schema-v2 数据继续可读、可校验和可回放；动作仍来自控制器实际接受的目标，并保持 `q_target[t+1] - q_measured[t]` 语义。VR 原始位姿或未经控制器接受的 IK 输出不能代替 accepted target。
- VR 使用独立命令源入口 `scripts/start_quest3_teleoperation_client.sh --side left|right`。adapter 已消费严格 IK result topic，并使用各臂独立的 raw/accepted 命名空间；该入口不启动机器人控制器或 GELLO。
- GELLO 和 VR 启动入口必须互斥，同一机械臂同一时刻只允许一个命令源；首期不实现运行中热切换，也不以 topic remap 让两个发布者同时连接同一控制器输入。
- VR 可以复用现有 FR3 关节阻抗控制器及安全检查，但任何新增配置必须保留 GELLO 默认值。若需给 launch 增加可选 command/accepted topic 参数，未传参时必须得到与当前 GELLO 完全相同的配置。
- client 首先独立完成 `Quest 输入 -> 坐标/锚点/安全状态机 -> FR3 IK -> AdapterCore -> VR joint target -> controller accepted target`。在离线正向验收和用户安排的分级真机验证前，不修改 server bridge、recorder、数据 schema 或 `dataset/test`。

计划中的 client 结构为：

```text
现有 GELLO（保留）
  GELLO publisher -> /right/gello/raw_joint_states
                  -> FR3 controller
                  -> /right/gello/accepted_joint_states

新增 VR（严格 IK 已接入独立 ROS adapter；client 构建和真机只读核对待完成）
  Quest -> ALVR/SteamVR -> OpenVR bridge -> receiver
        -> 坐标映射/相对锚定/安全状态机 -> FR3 IK
                 -> /right/vr/raw_joint_states
                 -> FR3 controller
                 -> /right/vr/accepted_joint_states
```

输入与离线 IK 已进入实现/验收阶段；MANUS/Sharpa、server VR 数采和真实机器人控制仍是后续独立任务。

## VR 接入参考

- Client 承担 Quest 输入、坐标转换、离合、约束及 FR3 IK；client 摇操验证完成后，server 再沿用相机、bridge 与 recorder，并适配输入模式预检和录制事件。
- 优先采用 VR 位姿 → FR3 IK → 现有关节控制器 → accepted target → bridge → recorder。保留 GELLO 默认行为，运行时只允许唯一命令源。
- 保持既有动作 q_target[t+1] - q_measured[t] 及夹爪字段语义；目标必须来自控制器实际接受的目标。
- Quest 参考 [meta_quest/README.md](data_collection/meta_quest/README.md)。当前推荐输入为 `scripts/record_meta_quest_openvr.sh`：Quest Touch → ALVR/SteamVR → 后台 OpenVR → 本机 TCP receiver/raw recorder。DexArena 参考 /home/landau/yys/DexArena/data_gen/teleop/ 下的 io.py、wrist_servo.py、arm_ik.py；坐标手性和四元数顺序已验证，当前原装夹爪工具变换已通过双臂只读快照数值核对，新增末端配置仍需单独确认。
- [move_to_target_ee.py](data_collection/move_to_target_ee.py) 的优化 IK 可参考模型与约束，在线使用前需测量延迟、连续性与奇异点表现。
- Server 预检目前包含 GELLO raw topic，VR 模式需显式适配；录制事件需边沿、去重与重连处理。
- 当前 bridge 为 15 Hz；输入、控制和写入频率可不同，但 dataset FPS 必须匹配采样策略。断流后禁止 pending_packet 跨长间隔配对；跨机 monotonic 时钟不能直接比较。
- 失跟踪、不可达或 IK 跳变时保持并重新锚定；保存来源、标定与质量信息，VR 数据先使用独立目录。

## VR 后续任务

### P1：VR 输入与目标生成

- [x] 固定 APK、源码与当前 controller 协议；物理按钮到最终接管/夹爪/录制语义仍待真机冻结。
- [x] 增加不依赖 `Controllers Only` APK 的 ALVR/OpenVR 输入路径，并复用既有 TCP 协议与 raw recorder。`stage2-openvr-controller-20261020.jsonl` 已验证 4151 个 lossless pose，左右位姿连续变化，trigger、clutch、record 状态切换可达；SHA-256 为 `411fd0428a830a47185d70e379b81c34f1e6db601a6db779fb23dc2e581e0e41`。
- [x] 统一输入适配器，复用 TCP receiver，并为每个有效包增加全局 sequence、TCP connection id 和主机接收时间。
- [x] 实现非阻塞全速 JSONL 录制、完整性汇总和确定性回放；只读夹具 `stage2-sixdof-safety3.jsonl` 已验证 3945 包、无拒收、无队列丢包，SHA-256 为 `2d84d7eb95cebb133998711684fe98038b862a47b72939a10eadf9beadbf7a66`。
- [x] 验证坐标手性、四元数顺序和六自由度运动方向。实测按上、右、前的顺序与 `robot_x=quest_z, robot_y=-quest_x, robot_z=quest_y` 一致，两次 pitch 也与旋转分量一致；固定工具偏置的代码接口已实现，当前原装夹爪实际工具变换已于 2026-10-21 数值核对。
- [x] 实现锚点标定、clutch-relative 跟随/保持状态机；正常保持与故障保持分离，接管、恢复和重复锚定均保持上一安全目标。
- [x] 加入可观测输入范围内的安全检查：host 到达超时、重连、显式 tracking、源位姿跳变、目标速度、工作空间和四元数连续性。旧紧凑协议的元数据有限；当前 OpenVR 的 frame id/轮询时间由主机生成，也不能据此证明设备产生了新帧或识别显式 recenter。静止、设备冻结与追踪空间重置的识别边界仍需验证。
- [x] 离线验证离合切换、重复锚定、超时及连接变化不会造成目标跳变。`safety3` 的三个锚点跳变均为零，最大连续有效目标位移约 0.010 m；实测断流进入 `FAULT_HOLD`。采集策略确定为 TCP 连接变化即判当前 episode 无效并整条重采，不跨连接恢复或拼接数据。

验收：坐标映射正确；异常输入不会触发目标跳变；陈旧数据进入保持。

`stage2-sixdof-safety3.jsonl` 仅作为阶段 2 坐标与安全回放夹具，不是训练数据。它包含一次实际断流和连接变化，正好用于验证故障保持；正式 episode 必须只有一个 TCP connection id。P1 在当前紧凑协议可观测范围内收尾，recenter/冻结不可区分是已知限制，不用恢复式重连测试阻塞 P2 离线 IK。

ALVR/OpenVR 的唯一操作手册为 [Quest-ALVR-OpenVR-Reproduction.md](Quest-ALVR-OpenVR-Reproduction.md)，其他 Markdown 不再维护第二套启动步骤。采集期间 Quest 必须保持佩戴、唤醒，ALVR Dashboard 必须为 `Streaming`；离开 Streaming 时本条数据整条重采。2026-10-20 引导复现文件 `stage6-openvr-guided-20261020.jsonl` 已验证 928 pose、左右各 464、单一连接、lossless、`dropped=0`。输入链提交为 `bd5153a`，后台 `Ctrl+C` 安全收尾修复为 `9988dac`。

### P2：FR3 控制适配

- [x] 实现 FR3 离线 FK/IK、合成轨迹、双臂 Quest 回放和只读实时预览；V4/Stage12 方向基本获得现场认可。
- [x] 统一 A/B 结果结构、状态和显式时钟；B 线只接受严格 A 线 `IKResult`。
- [x] 提取纯 Python `AdapterCore`，覆盖 freshness、序号、左右臂、首帧、限位、速度、GELLO 冲突和故障锁存。
- [x] Quest ROS 包改为真实 IK result topic 输入；移除 runtime synthetic source，保持与 GELLO 独立构建/启动。
- [x] 完成离线接管首帧、受限参考、停止/重锚定和主要故障回归；保持诊断用途。
- [x] 完成 18 类无硬件故障注入，全部 `would_publish=false`。
- [ ] 解决严格残差、20 ms 周期长尾、奇异点和碰撞验收，不以放宽控制器限制掩盖问题。
- [ ] 用户录制左手 5 mm 三轴小动作并补充新的左臂只读 snapshot，完成正向离线验收。
- [ ] 在 client 构建 ROS package，并在真机第 1 级只读检查中核对 measured/accepted topic；不以 fake 作为放行条件。
- [ ] 联合接入后回归 GELLO 启动、采集、历史数据读取与回放。

验收：唯一明确命令源；目标连续；异常保持；旧 GELLO 工作流可用。离线诊断结果不作为硬件命令。

### P3：VR 数采闭环

- [ ] 接入按钮录制事件，保留键盘控制。
- [ ] 增加中断检测与禁止跨间隔配对。
- [ ] 保存实际时间、来源与质量元数据。
- [ ] 对齐采样频率、dataset FPS、图像与机器人状态。
- [ ] 实现 episode 预览、审核、标签与丢弃。
- [ ] 完成一条单右臂 VR 数据的记录、校验与回放。

验收：动作可追溯；异常不会静默污染连续轨迹；episode 可审核、校验和回放。

### P4：VR 组合扩展（按需）

- [ ] 左臂分级真机通过后再扩展右臂和双臂；双臂离线 IK 与回放已实现。
- [ ] 验证 Franka 夹爪与 Wuji glove/hand 组合。
- [ ] 验证 GELLO/VR 数据 schema 的兼容性与混合训练前提。
- [ ] 按需增加 DexArena 仿真对照验证。

验收：各组合有明确配置；历史数据读取、处理与回放保持兼容。
