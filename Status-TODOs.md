# VR 数采升级：现状与待办

更新日期：2026-10-15。只维护当前结论、复现入口与未完成任务；历史过程查 Git。

## 当前结论

两端独立 ROS 构建及 server recorder 迁移已完成。用户确认新 client/server 的双臂 Franka 夹爪 GELLO 采集、校验与回放通过；server data/test 已完成 trim-initial 的 dry-run、正式裁剪及完整 validate_dataset 校验。可以进入单右臂 VR 接入，VR 尚未实现。

其他 arm/gripper 组合暂不单独测试，不阻塞主线，也不标记为实机通过。本机 data/test 旧副本未确认同步 server 处理结果，不能据此否定 server 验收。

## 环境与隔离范围

| 项目 | Client | Server |
|---|---|---|
| 项目 | /home/landau/real-exp-vr | /home/pair1/real-exp-vr |
| 系统 ROS / Python | Humble / 3.10 | Jazzy / 3.12 |
| 独立 ROS 产物 | gello_software/ros2/{build,install,log}/client | gello_software/ros2/{build,install,log}/server |
| 职责 | GELLO、机器人及夹爪控制，发布状态与 accepted target | 相机、DDS 订阅、ROS → ZMQ bridge |
| Conda | GELLO arm/gripper 控制不调用 Conda；VR 按需使用 lerobot-vr | recorder、预处理与校验使用 lerobot-vr |

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

## VR 接入参考

- Client 承担 Quest 输入、坐标转换、离合、约束及 FR3 IK；server 沿用相机、bridge 与 recorder，适配输入模式预检和录制事件。
- 优先采用 VR 位姿 → FR3 IK → 现有关节控制器 → accepted target → bridge → recorder。保留 GELLO 默认行为，运行时只允许唯一命令源。
- 保持既有动作 q_target[t+1] - q_measured[t] 及夹爪字段语义；目标必须来自控制器实际接受的目标。
- Quest 参考 [meta_quest/README.md](data_collection/meta_quest/README.md)。DexArena 参考 /home/landau/yys/DexArena/data_gen/teleop/ 下的 io.py、wrist_servo.py、arm_ik.py；先确认 APK、协议、坐标手性、四元数顺序与工具偏置，再适配 FR3。
- [move_to_target_ee.py](data_collection/move_to_target_ee.py) 的优化 IK 可参考模型与约束，在线使用前需测量延迟、连续性与奇异点表现。
- Server 预检目前包含 GELLO raw topic，VR 模式需显式适配；录制事件需边沿、去重与重连处理。
- 当前 bridge 为 15 Hz；输入、控制和写入频率可不同，但 dataset FPS 必须匹配采样策略。断流后禁止 pending_packet 跨长间隔配对；跨机 monotonic 时钟不能直接比较。
- 失跟踪、不可达或 IK 跳变时保持并重新锚定；保存来源、标定与质量信息，VR 数据先使用独立目录。

## VR 后续任务

### P1：VR 输入与目标生成

- [ ] 固定 APK、源码、协议与按钮映射。
- [ ] 统一输入适配器，优先复用已有 TCP receiver。
- [ ] 测试坐标手性、四元数顺序、六自由度运动方向和工具偏置。
- [ ] 实现锚点标定、clutch-relative 跟随/保持状态机。
- [ ] 加入位姿整形、冻结检测、超时保持、工作空间和速度约束。
- [ ] 离线验证重连、recenter、离合切换无目标跳变。

验收：坐标映射正确；异常输入不会触发目标跳变；陈旧数据进入保持。

### P2：FR3 控制适配

- [ ] 实现或适配在线 FR3 IK，测量延迟、连续性和奇异点行为。
- [ ] 配置 VR 命令话题与 accepted target。
- [ ] 适配 bridge、启动预检和 watchdog。
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
