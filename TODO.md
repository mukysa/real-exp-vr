# VR 遥操与数据采集 TODO

更新日期：2026-10-26。

**主线目标：用 Quest 3 VR 遥操替代 GELLO 输入，完成 robot-control client 与 data server 的通信，采集可验证、可回放、可用于训练的 LeRobot episode。**

本文区分已完成的代码、现场验证与待验收工作。文中的性能指标和验收数量是建议目标，不是当前实测结果；标注“拟新增”的参数、配置和接口尚未实现。2026-10-26 的左臂接管和 server 同步结果依据用户现场操作反馈及终端输出记录，不代表跨机采集已验收。

## 当前左臂实施主线

1. **完成左臂入口与配置接线（左臂 M1 已验证）：** client/server 支持 `--teleop-source quest --left --arm`；client 复用现有 VR 启动与显式硬件启用，server 使用主仓库 Quest 配置、检查左臂 VR 话题，保存输入源标识。右臂／双臂／夹爪暂不开放。server 源码及子模块已同步，采集 ROS 包已于 2026-10-27 重建；真实输入仍待 M2 现场验收。
2. **验证 server 收到完整样本（待验收）：** 核对远端环境和 DDS，收到左臂状态、accepted action、VR 状态及两路图像，生成有效 schema-v2 包。
3. **补齐录制边界，采真实 episode（待实现／验收）：** FOLLOWING 门控、回位／故障边界、清空跨段配对，再做数据校验、loader 和回放 dry-run。

本地验证：新增入口／配置／来源元数据 13 项测试通过；左臂统一入口 `--check-only` 通过；随后真机启动到 `Hardware chain ready`，用户确认一次 Y 接管成功，并 Ctrl+C 正常退出。扩展测试另有 4 项通过、3 项因缺少本地测试数据集元数据失败；原 recorder 两项测试在修改前 HEAD 上同样失败。server 跨机与真机 episode 尚未验收。

第一步的入口就绪检查仅验证收到消息；不代表 FOLLOWING 门控、跨机同步或真实 episode 已验收。具体使用见 README-QUICKSTART 第 5 节。

## 1. 当前基线与缺口

| 模块 | 已有基础 | 仍需完成 |
|---|---|---|
| Quest 输入 | 双实体 Touch、OpenVR 位姿、左 Y 统一接管／回位 | 输入时延、丢帧和追踪丢失的连续统计 |
| 左臂遥操 | 已现场验证六自由度跟随、回位、再次接管 | 重复启动与长时间稳定性；单臂日志仍有 FCI 通信故障 |
| 右臂／双臂 | 三模式入口、独立适配器、共同就绪与故障联动；已有双臂现场试运行 | 优化后流畅度、双侧回位完成和完整循环验收 |
| client 采集入口 | Quest 左臂 arm-only 分支已通过离线检查、真机启动及一次 Y 接管 | 重复接管循环和长期稳定性 |
| server 采集入口 | Quest 左臂配置与就绪检查已实现；当前源码已重建，跨机合成样本通过 | 真实相机、机器人状态与 accepted action 联调 |
| 数据格式 | schema-v2、关节／末端表示、校验与回放工具；Quest 来源标识已实现并离线测试 | 会话状态、录制边界及端到端采集验收 |

基线证据：

- [主 README](README.md)：2026-10-25 左臂真机验证、双臂试运行、IK 卡顿和 FCI 故障记录。
- [双臂运行日志](outputs/quest3_hardware/20261025-163811-duo/quest_control.log)：两侧进入回位，左侧完成后右侧仍在回位时再次 Y 被拒绝；还不足以证明完整双臂循环验收。
- [左臂故障日志](outputs/quest3_hardware/20261025-165426-left/hardware-left.log)：单臂启动也出现 communication_constraints_violation。
- 2026-10-26 前次检查：59 项相关离线测试通过，三模式 check-only 通过；这些结果不代表真机或跨机采集已验收。

## 2. 目标架构与实施原则

~~~text
client：ROS 2 Humble，机器人控制机
Quest 3 → ALVR / SteamVR / OpenVR
        → 双手柄配对、左 Y、相对位姿、严格 IK
        → VR 安全适配器 → FR3 阻抗控制器 → 机械臂
                              │
                  实测状态 + 控制器 accepted targets
                              │ ROS 2 DDS，跨机
                              ▼
server：192.168.52.160，现有文档约定 ROS 2 Jazzy
RealSense 图像 + 机器人状态／动作 → ROS 数据 bridge
                              → ZMQ :5555 → LeRobot recorder
                              → episode / Parquet / 视频 / 元数据

后续可选：server 的录制状态／故障 → client 状态显示
~~~

- **替代的是 GELLO 操作输入与采集入口的绑定。** 复用现有 FR3 控制器、ROS bridge、LeRobot 格式和校验工具；相关代码仍位于 gello_software 子仓库并不影响 VR 独立使用。
- client 保持机器人实时控制；相机、编码、落盘留在 server。首版沿用已有 DDS 跨机数据流，ZMQ 5555 是 server bridge 到 recorder 的样本通道，不是现成的机器人跨机控制协议。
- 第一阶段完成 **左臂、arm-only、图像 + 实测状态 + accepted action** 的最小采集闭环，再扩展右臂、双臂、夹爪。不要让夹爪或双臂优化阻塞左臂采集接口开发。
- 数据 action 来源以控制器实际接受的目标为准；原始 Touch 位姿、未验收 IK 结果和控制器内部插值参考分别作为诊断数据处理。
- client/server 的 ROS 发行版不同，消息定义、RMW、QoS 和时钟兼容性必须实测。先核对现有链路，确认问题后再决定统一环境或添加明确的数据传输适配层。

## 3. P0 主线：VR 替代 GELLO，完成跨机通信与采集

### M1 — 接入 VR 采集模式与话题契约（左臂范围已完成并验证）

本轮范围限定为 `quest + left + arm`。右臂／双臂／夹爪／hand 扩展移至 M4，不作为左臂 M1 完成条件。

- [x] client/server 支持 `--teleop-source gello|quest`；默认保留 GELLO 路由。
- [x] client Quest 分支调用现有 `start_quest3_hardware.sh --left`；明确拒绝右臂、双臂及 gripper/hand 组合。
- [x] Quest 分支不依赖 GELLO USB 或 publisher，复用单实例锁、进程检查、清理和显式 `--enable-hardware`；支持 `--check-only`。
- [x] 主仓库新增 `config/data_collection/quest_left.yaml`，通过绝对路径加载；关闭右臂、夹爪、hand 和第三路相机的数据依赖。
- [x] server Quest 配置与启动检查均使用 VR accepted topic，不等待 GELLO 话题；额外检查左臂 FrankaRobotState、VR 状态和两路图像。此项为代码完成，跨机实际接收归 M2。
- [x] 配置 Quest 任务名、默认 repo id，以及 bridge 数据包和 recorder 动作元数据中的 `teleop_source`。
- [x] 13 项入口路由、非法组合拒绝、左臂配置和来源元数据测试通过；实际 `--check-only` 通过。
- [x] 修复启动探测每 3 秒重建订阅的问题：改为明确消息类型、持续等待最多 45 秒并保留诊断日志。
- [x] 新入口真机启动成功，用户确认一次 Y 接管，并正常退出。未据此认定回位再接管完整循环或长期稳定性通过。
- [x] Git bundle 同步到 server，主仓库／子模块提交、配置文件和入口 help 均已核对。

验证记录：

- 首次失败日志：`outputs/quest3_hardware/20261026-170331-left/hardware-left.log`；硬件及 broadcaster 已激活，但等待 joint_states 超时。更改订阅等待方式后重试成功；未单独证明超时的唯一根因。
- 成功启动日志目录：`outputs/quest3_hardware/20261026-171106-left`；终端显示 `Hardware chain ready: left`，用户确认 Y 接管成功。
- server `/home/pair1/real-exp-vr`：分支 `server/quest-left`，主仓库 `2acc5a1`，gello_software `135d3bf`，lerobot `4eecbad`；子模块与主仓库记录一致。以上是首次迁移检查点，不包含后续文档修改。
- server 的 SSH 地址仍可用 `192.168.52.160`；Quest 数采 DDS 使用有线 `192.168.1.100 ↔ 192.168.1.50`，server bridge 的 Quest ZMQ 默认绑定本机 `127.0.0.1:5555`。离线迁移方法见 [SERVER-OFFLINE-SYNC.md](SERVER-OFFLINE-SYNC.md)，有线配置见 [M2-NETWORK-SETUP.md](M2-NETWORK-SETUP.md)。
- 测试限制：扩展测试 3 项因缺少数据集元数据失败；原 recorder 2 项测试在修改前 HEAD 上同样失败，未作为本轮通过项。


| 数据 | client 来源 / server 订阅 | 用途 |
|---|---|---|
| 实测关节 | /left/joint_states、/right/joint_states | observation.joint_state |
| 控制器接受的 VR 目标 | /left/vr/accepted_joint_states、/right/vr/accepted_joint_states | action.target_joint 的来源 |
| 实测末端与工具变换 | /<side>/franka_robot_state_broadcaster/robot_state | observation.ee_pose、目标 FK 所需变换 |
| 遥操源状态 | /<side>/vr/source_status | 判断 READY、FOLLOWING、RETURNING、FAULT |
| 适配器状态 | /<side>/vr/status | 记录 STOP 与拒绝原因 |
| server 本地图像 | /cameras/<camera_name>/image_raw | observation.images.* |

**验收：** 选 Quest 时，client 不触发 GELLO 设备依赖，server 配置及就绪检查均指向所选侧 VR accepted topic；非所选臂不成为必需输入。已实现参数有 help、失败提示和测试。左臂入口验收已通过；server 运行时契约由 M2 进一步验证。

涉及入口：[client supervisor](scripts/start_data_collection_client.sh)、[server supervisor](scripts/start_data_collection_server.sh)、[Quest 真机入口](scripts/start_quest3_hardware.sh)、[VR topic 配置](quest3_ros2/src/franka_vr_adapter/config/vr_controller.yaml)、[bridge launch](gello_software/ros2/src/franka_lerobot_data_bridge/launch/bridge.launch.py)。

### M2 — client/server 通信与同步验收（需两机；可先用隔离测试话题）

**本机及跨机合成链路已通过（2026-10-27），真实机器人和相机输入未验收。** 工具与操作步骤见 [M2-VALIDATION.md](M2-VALIDATION.md)。

- [x] 新增环境 JSON 报告、并行只读话题探测、ZMQ schema-v2／频率／时间戳质量报告工具。
- [x] bridge 增加 action 时间戳、状态动作时间差、拒绝回调计数；collection 模式拒绝非法／未来输入时间戳。
- [x] 新增 domain 173、/m2 命名空间隔离合成测试；正常链路、动作／状态／相机断流、超前 5 秒时间戳五种情形通过。
- [x] 完整 schema 审计 121 包全部有效，合成链路约 15.002 Hz；22 项诊断及入口单元测试通过。这些数字不代表真实跨机性能。
- [x] split collection 默认 server 地址更新为 192.168.52.160；server 等待话题期间增加子进程退出检查及探测日志。
- [x] Quest 数采 DDS 固定到现有千兆有线网段；双向字符串话题通过，server bridge 从 client 合成关节、accepted action、EE 与图像形成 901/901 有效 ZMQ 样本，60 秒约 15.000 Hz。原 GELLO 入口保留。


- [x] 根据 server 实际终端输出核对 checkout 和子仓库版本（见 M1 迁移检查点）。
- [x] server 重建 bridge 和 RealSense publisher，并核对实际加载路径；跨机合成 JointState、FrankaRobotState、状态字符串和图像已通过。Humble/Jazzy type-hash 警告仍存在，真实 QoS 与消息源待现场验证。
- [ ] 用真实 ROS topic 逐项验证 JointState、FrankaRobotState、状态字符串和两路图像的发现、QoS、时钟及新鲜度。
- [ ] server 启动预检查覆盖实测关节、accepted action、FrankaRobotState、所选相机和遥操状态；区分“等待 Y 接管产生首个目标”与“通信失败”。
- [x] 在独立 ROS_DOMAIN_ID=173／`/m2` 命名空间用合成数据验证通信，未向真机 raw-command topic 注入测试目标。
- [ ] 验证跨机时钟偏差、ROS header 时间、server 接收时间和数据新鲜度判定；跨主机不直接相减各自的 monotonic 时间。
- [ ] 保留并核查 bridge 的相机时间偏差和数据年龄检查，补充机器人状态与 action 的时间对齐统计；明确重采样、复用目标和丢弃过期帧的规则。
- [ ] 核验 30 Hz 左右输入／状态、实际 IK 输出和 15 Hz 采集之间的关系；避免通过重复旧目标掩盖低频 IK。
- [ ] 测试 client 退出、网络中断、相机停流和 server 重启：采集端明确报错或中止当前 episode，不静默保存过期样本；控制侧 watchdog 保持有效。

**验收：** server 实际收到所选臂所有必需数据并产生有效 schema-v2 样本；保存频率、延迟、最大间隔、数据年龄、相机偏差和丢弃计数报告。建议先完成一次 10 分钟两机连续接收测试，阈值依据现有配置与测量确定。

### M3 — 左臂 LeRobot episode 最小闭环（主线首个可交付里程碑）

- [ ] 使用左臂 arm-only + cam_left / cam_front 采集，复用 server 的 --record 和现有 recorder。
- [ ] 首版保留 recorder 的 s/e/d/q 键盘操作：开始、保存、丢弃、退出；左 Y 继续只负责机械臂接管／回位。
- [ ] 增加录制就绪门控：所选臂处于有效跟随状态、accepted target 与实测状态新鲜、相机完整、schema-v2 校验通过后才能开始 episode。
- [ ] 设计回位、追踪丢失、FAULT/STOP 和断流的 episode 策略：停止追加有效训练帧，记录结束原因，并由操作者保存有效片段或丢弃；不把自动回位悄悄混入示教。
- [ ] 沿用 packet_pair_to_frame 的时间语义：当前 observation 对应下一包 accepted target；核验 action.target_joint、action.delta_joint、action.target_ee_pose 和 action.delta_ee_pose 一致，不引入一帧错位。
- [ ] 对跨故障、回位或重接管的包清空待配对帧，避免把两个操作段拼成一个训练动作；明确 episode 末帧没有下一包时的处理。
- [ ] 保存关节和末端中性字段、图像、schema-v2 元数据；记录输入源、臂模式、标定文件标识、代码版本、采样率、会话 ID 和结束原因。
- [ ] 检查工具 TCP 定义一致：VR IK 使用的 TCP、FrankaRobotState 实测 EE、bridge 目标 FK 与数据回放不能混用不同工具变换。
- [ ] 执行 validate_dataset.py，检查视频／Parquet 帧数、时间连续性、字段维度、关节边界和目标语义；用现有 loader 读取样本，再做 replay dry-run。

**验收：** 不连接 GELLO，完成建议至少 3 个左臂 VR episode，每个约 30–60 秒；可保存、丢弃、重新开始，数据校验通过且训练 loader 可读、回放 dry-run 通过。验收报告列出数据集路径、episode 数、帧数、异常片段策略及检查结果。此阶段即可称为“VR 已替代 GELLO 完成基础采集”。

涉及实现：[recorder](data_collection/lerobot_collection.py)、[ROS bridge](gello_software/ros2/src/franka_lerobot_data_bridge/franka_lerobot_data_bridge/lerobot_data_bridge.py)、[数据校验](data_collection/validate_dataset.py)、[轨迹元数据](utils/trajectory_metadata.py)、[回放](data_collection/replay_lerobot_episode.py)、[采集测试](test/test_lerobot_collection.py)。

### M4 — 扩展右臂、双臂与采集操作体验

- [ ] 右臂独立完成 M2/M3；启用 cam_right / cam_front，检查不会订阅或等待左臂数据。
- [ ] 双臂启用三相机，核验左右顺序、14 维 arm-only 关节字段及每臂 9 维 EE pose；禁止用零值补齐缺失臂状态。
- [ ] 验收双臂共同接管、两侧回位完成后再次接管，以及任一侧 STOP/断流时 episode 正确结束。
- [ ] 将录制中、已保存、丢弃、样本过期等状态反馈到 client；后续再考虑独立 VR 按键触发录制，定义确认／去重机制且不与 Y 冲突。
- [ ] 真实回放先完成数据校验和 dry-run，再在现场完成首帧就位及低速短片段验证；与遥操严格互斥。

**验收：** 左、右、双臂分别产生可校验的数据集，记录模式与硬件模式匹配；两机启动、停止、异常恢复均有可复现说明。

## 4. P0 稳定性：支撑真机采集的并行工作

以下工作可与 M1/M2 的离线开发并行；真机 M3/M4 验收依赖相应机械臂稳定性通过。

### S1 — FCI 启动与运行稳定性

- [ ] 将 communication_constraints_violation 按左／右／双臂、启动阶段、Y 接管前后分类，保存失败时硬件日志与完整参数快照。
- [ ] 检查 CPU 频率、实时优先级、线程绑定、网卡 IRQ、RX ring、丢包／调度延迟；先复现已成功组合，每次仅改变一个变量。
- [ ] 将性能设置整理为可检查、可重复执行的脚本；启动前显示当前值，避免重启后参数恢复而不自知。
- [ ] 核实外部 franka_ros2_ws 的调度修复来源与构建版本，并记录 gello_software 中本机 controllers.yaml 修改；消除“只在这台机器有效、提交不完整”的隐患。
- [ ] 对控制器激活阶段超时、异常退出补充明确阶段与侧别日志；保留现场停止后的诊断材料。

**建议验收：** 各模式分别完成 10 次冷启动／正常退出和一次 10 分钟连续低幅操作，报告 FCI 异常次数与运行条件；失败即保留问题，不以一次启动成功认定稳定。右臂 CPU 6 绑定已撤回，不视作现成解决方案。

### S2 — 双臂 IK 频率与跟随延迟

- [ ] 记录输入、队列等待、每侧 IK、适配器输出与 accepted target 的耗时／频率，统计 p50/p95/p99 与最大间隔。
- [ ] 现场复测当前 1e-6 优化器阈值和单线程数值库配置；保持最终 FK 残差、关节约束和异常拒绝逻辑有效。
- [ ] 优先优化连续 seed、静止目标复用和旧位姿合并；双侧串行求解总预算不足时，再评估并行计算，保留同帧配对与共同故障语义。
- [ ] 检查计算超时后目标过期／控制器 watchdog 的行为，不通过无限放宽超时或运动门限掩盖卡顿。

**建议验收：** 朝每侧接近 30 Hz 跟随推进，先验证串行双侧处理是否可进入每帧约 33 ms 总预算；15 Hz 采集时记录目标实际更新率。以实测轨迹、FK 误差和现场流畅度共同判断，不能只看发布定时器频率。

## 5. P1 功能与质量优化

- [ ] **夹爪接入：** 设计左右 Touch 的独立夹爪输入，复用现有 gripper manager/client；校验物理宽度与归一化目标的转换，再纳入 state/action。arm-only 闭环不依赖此项，抓取示教依赖此项。
- [ ] **操作体验：** 明确显示两臂就绪、跟随、回位、故障和录制状态；定位双臂回位速度差异与再次接管被拒绝的原因。
- [ ] **安全边界：** 审核当前 relaxed_motion_gates 和适配器较宽门限，建立可解释的真机参数档；评估每臂工作区、自碰撞、桌面与双臂互撞检查。现有 IK／watchdog 不等同于碰撞规划。
- [ ] **数据质量报告：** 每个 episode 输出有效时长、视频帧数、状态／目标年龄、输入丢帧、关节与 EE 跟随误差、录制结束原因。
- [ ] **追溯与诊断：** 原始 OpenVR 录制、控制日志和数据集通过会话 ID 关联；高频诊断采用限流／异步写盘，避免影响控制线程。
- [ ] **部署可复现：** 整理 client/server 环境检查、依赖锁定、子仓库提交及构建顺序；更新 README-QUICKSTART 中“双臂尚未做真机测试”的滞后表述。
- [ ] **回归测试：** 扩展输入源选择、VR accepted topic、左右模式、断流处理、跨 episode 配对、TCP 一致性和数据加载测试；保留原 GELLO 采集兼容性。

## 6. 推荐执行顺序与完成定义

| 顺序 | 工作 | 依赖／产物 |
|---|---|---|
| 1 | M1：入口与 VR bridge 配置 | 现在可开发；输出配置、路由和离线测试 |
| 2 | M2：两机通信与 schema 样本验证 | 依赖 M1 和 server 环境；先用隔离测试数据 |
| 并行 | S1/S2：FCI 与 IK 优化 | 不阻塞接口开发；为真机采集提供可重复基线 |
| 3 | M3：左臂 arm-only 采集 | M1/M2 + 左臂稳定性；交付首批有效 VR 数据 |
| 4 | M4：右臂与双臂采集 | 对应真机稳定性及协调验收 |
| 5 | P1：夹爪、体验、质量与环境完善 | 在主线闭环上增量推进 |

**主线完成标准：** 无 GELLO 输入设备，Quest 控制所选 FR3；client 数据真实到达 server；server 同步采集图像、实测状态和控制器接受的动作；episode 可开始、保存、丢弃和异常结束；产物通过校验、训练 loader 读取和回放检查；左／右／双臂分别有验收记录。夹爪抓取采集作为后续单独验收项。

**下一项最小可执行任务：** 由现场操作者启动左臂 Quest 数采入口；server 不带 `--record` 启动两路真实相机与 bridge，使用现有 M2 报告工具检查 60 秒、再 600 秒真实样本。先解决 server 仅剩约 55 GB 空间的容量规划；M3 的 FOLLOWING 门控与 episode 边界完成前，不将候选数据标为训练样本。
