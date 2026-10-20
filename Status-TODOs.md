# VR 数采升级：现状与待办

更新日期：2026-10-20。只维护当前结论、复现入口与未完成任务；历史过程查 Git。

## 当前结论

两端独立 ROS 构建及 server recorder 迁移已完成。用户确认新 client/server 的双臂 Franka 夹爪 GELLO 采集、校验与回放通过；server data/test 已完成 trim-initial 的 dry-run、正式裁剪及完整 validate_dataset 校验。Quest 阶段 2 输入、坐标映射、相对锚定、安全状态机和离线回放已完成；ALVR/SteamVR → OpenVR → 既有 raw recorder 的双手柄链路也已实采通过，可以进入 P2 FR3 离线 IK 与 fake hardware 适配；尚未连接真实机器人。

当前阶段只固化 GELLO 兼容契约和 VR 旁路边界。直接在 client 的 `/home/landau/real-exp-vr` 中继续开发，不创建独立工作区；在 client 完成 Quest 单右臂摇操前，不同步或修改 server，也不重做已经通过的 GELLO 真机测试。

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
- **小改动处理**：对局部文档或配置修改，先只编辑用户指定的段落，再运行 `git diff --check` 和局部 diff 核对；不顺带做大范围搜索、重构或扩展文档整理。
- **接口兼容**：保持 client/server 间 topic 名称、消息类型、关节名称及顺序、单位、坐标约定、动作语义和必要的时间行为一致。accepted target 必须仍来自控制器接受后的目标。
- **文件边界**：只修改 `/home/landau/real-exp-vr/` 内的工程文件和构建产物；原 real-exp、DexArena 和其他工程只读使用。写入前注意符号链接和生成文件，避免间接修改外部目录。
- **环境边界**：如确需改动 Conda，只允许修改当前机器职责对应的 `lerobot-vr` 环境（client 使用时为 `/home/landau/anaconda3/envs/lerobot-vr/`，server 为 `/home/pair1/miniconda3/envs/lerobot-vr/`）。不得修改旧 `lerobot`、Conda base、系统 Python/ROS 或公共 Franka 工作空间。系统 ROS 和公共工作空间仅作为只读依赖使用。
- **验证边界**：构建、静态配置检查、离线单元测试、fake hardware 联调和真实采集分别记录，不能互相替代。本阶段没有授权启动真实机器人控制，实机联调由用户另行安排。
- **按实际依赖处理**：不为使用 lerobot-vr 而把 ROS 强行迁入 Python 3.12，不整体安装 ms3 环境快照，不把 client 不使用的视频或训练依赖当作 GELLO 控制前置条件。
- **迁移与功能改造分开验收**：先建立原 GELLO 流程的隔离基线，再引入 VR，便于定位故障来源。
- **状态维护**：完成后就地更新结论并删除过时待办，不在开头累积历史日志或保留矛盾状态。

## GELLO 兼容契约与 VR 旁路边界

以下内容作为后续 client 改造的硬约束：

- 原 GELLO 入口 `scripts/start_data_collection_client.sh` 和 `scripts/start_teleoperation.sh` 的现有参数、默认行为及启动顺序保持不变；VR 首期不在这两个入口中增加默认分支。
- GELLO 命令与 accepted target 继续使用 `/{side}/gello/raw_joint_states` 和 `/{side}/gello/accepted_joint_states`，消息类型、七关节名称与顺序、弧度单位及控制器默认参数保持不变。
- 历史 schema-v2 数据继续可读、可校验和可回放；动作仍来自控制器实际接受的目标，并保持 `q_target[t+1] - q_measured[t]` 语义。VR 原始位姿或未经控制器接受的 IK 输出不能代替 accepted target。
- VR 作为独立命令源旁路新增，首期只做右臂。计划使用独立启动入口 `scripts/start_vr_teleoperation_client.sh`、命令话题 `/right/vr/raw_joint_states` 和 accepted target `/right/vr/accepted_joint_states`；这些名称在代码实现前只是接口约定，不表示功能已经存在。
- GELLO 和 VR 启动入口必须互斥，同一机械臂同一时刻只允许一个命令源；首期不实现运行中热切换，也不以 topic remap 让两个发布者同时连接同一控制器输入。
- VR 可以复用现有 FR3 关节阻抗控制器及安全检查，但任何新增配置必须保留 GELLO 默认值。若需给 launch 增加可选 command/accepted topic 参数，未传参时必须得到与当前 GELLO 完全相同的配置。
- client 首先独立完成 `Quest 输入 -> 坐标/锚点/安全状态机 -> FR3 IK -> VR joint target -> controller accepted target`。在该链路完成离线、fake hardware 和用户安排的摇操验证前，不修改 server bridge、recorder、数据 schema 或 `dataset/test`。

计划中的 client 结构为：

```text
现有 GELLO（保留）
  GELLO publisher -> /right/gello/raw_joint_states
                  -> FR3 controller
                  -> /right/gello/accepted_joint_states

新增 VR（输入与离线安全已实现；IK/控制未实现）
  Quest -> ALVR/SteamVR -> OpenVR bridge -> receiver
        -> 坐标映射/相对锚定/安全状态机 -> FR3 IK
                 -> /right/vr/raw_joint_states
                 -> FR3 controller
                 -> /right/vr/accepted_joint_states
```

本阶段只确认上述契约和结构，不实现 Quest 控制、IK、MANUS、Sharpa 或 server 数采适配。

## VR 接入参考

- Client 承担 Quest 输入、坐标转换、离合、约束及 FR3 IK；client 摇操验证完成后，server 再沿用相机、bridge 与 recorder，并适配输入模式预检和录制事件。
- 优先采用 VR 位姿 → FR3 IK → 现有关节控制器 → accepted target → bridge → recorder。保留 GELLO 默认行为，运行时只允许唯一命令源。
- 保持既有动作 q_target[t+1] - q_measured[t] 及夹爪字段语义；目标必须来自控制器实际接受的目标。
- Quest 参考 [meta_quest/README.md](data_collection/meta_quest/README.md)。当前推荐输入为 `scripts/record_meta_quest_openvr.sh`：Quest Touch → ALVR/SteamVR → 后台 OpenVR → 本机 TCP receiver/raw recorder。DexArena 参考 /home/landau/yys/DexArena/data_gen/teleop/ 下的 io.py、wrist_servo.py、arm_ik.py；坐标手性和四元数顺序已验证，工具偏置留到 FR3 末端确认时标定。
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
- [x] 验证坐标手性、四元数顺序和六自由度运动方向。实测按上、右、前的顺序与 `robot_x=quest_z, robot_y=-quest_x, robot_z=quest_y` 一致，两次 pitch 也与旋转分量一致；固定工具偏置的代码接口已实现，实际数值留到 FR3 末端工具确认时标定。
- [x] 实现锚点标定、clutch-relative 跟随/保持状态机；正常保持与故障保持分离，接管、恢复和重复锚定均保持上一安全目标。
- [x] 加入可观测输入范围内的安全检查：host 到达超时、重连、显式 tracking、源位姿跳变、目标速度、工作空间和四元数连续性。紧凑协议缺少 frame id、设备时间戳和显式 recenter，因此不能可靠区分静止与设备帧冻结；这项协议限制必须在真机启用条件中保留。
- [x] 离线验证离合切换、重复锚定、超时及连接变化不会造成目标跳变。`safety3` 的三个锚点跳变均为零，最大连续有效目标位移约 0.010 m；实测断流进入 `FAULT_HOLD`。采集策略确定为 TCP 连接变化即判当前 episode 无效并整条重采，不跨连接恢复或拼接数据。

验收：坐标映射正确；异常输入不会触发目标跳变；陈旧数据进入保持。

`stage2-sixdof-safety3.jsonl` 仅作为阶段 2 坐标与安全回放夹具，不是训练数据。它包含一次实际断流和连接变化，正好用于验证故障保持；正式 episode 必须只有一个 TCP connection id。P1 在当前紧凑协议可观测范围内收尾，recenter/冻结不可区分是已知限制，不用恢复式重连测试阻塞 P2 离线 IK。

ALVR/OpenVR 的固定复现入口为：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
unset PYTHONPATH PYTHONHOME
./scripts/record_meta_quest_openvr.sh \
  --output recordings/quest_raw/<new-name>.jsonl
```

采集期间 Quest 必须保持佩戴、唤醒，ALVR Dashboard 必须为 `Streaming`。ALVR 离开 Streaming 时停止本条采集并整条重采，不使用冻结姿态续写。该输入链已提交为 `bd5153a feat(quest): add direct OpenVR recording pipeline`，Meta Quest 专项测试为 `67 passed`。

### P2：FR3 控制适配

- [ ] 实现或适配在线 FR3 IK，测量延迟、连续性和奇异点行为。
- [ ] 配置 VR 命令话题与 accepted target。
- [ ] 在 client 侧实现启动预检和 watchdog；bridge 适配推迟到 client 摇操通过之后。
- [ ] 用 fake hardware、离线数据或隔离的目标发布验证命令链路。
- [ ] 回归 GELLO 启动、采集、历史数据读取与回放。

验收：唯一明确命令源；目标连续；异常保持；旧 GELLO 工作流可用。

### P3：VR 数采闭环

- [ ] 接入按钮录制事件，保留键盘控制。
- [ ] 增加中断检测与禁止跨间隔配对。
- [ ] 保存实际时间、来源与质量元数据。
- [ ] 对齐采样频率、dataset FPS、图像与机器人状态。
- [ ] 实现 episode 预览、审核、标签与丢弃。
- [ ] 完成一条单右臂 VR 数据的记录、校验与回放。

验收：动作可追溯；异常不会静默污染连续轨迹；episode 可审核、校验和回放。

### P4：VR 组合扩展（按需）

- [ ] 从单臂扩展至双臂。
- [ ] 验证 Franka 夹爪与 Wuji glove/hand 组合。
- [ ] 验证 GELLO/VR 数据 schema 的兼容性与混合训练前提。
- [ ] 按需增加 DexArena 仿真对照验证。

验收：各组合有明确配置；历史数据读取、处理与回放保持兼容。
