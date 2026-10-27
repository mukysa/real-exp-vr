# M2：左臂跨机采集链路验证

当前 server：`pair1@192.168.52.160`，仓库 `/home/pair1/real-exp-vr`。

本轮提供环境报告、只读 ROS 话题探测、ZMQ 样本质量报告及隔离合成输入。它们不会启动机械臂或创建训练 episode。两机实测、时钟同步与真实相机运行仍需现场验收。部署方法见 [SERVER-OFFLINE-SYNC.md](SERVER-OFFLINE-SYNC.md)，主仓库和 gello_software 都有新改动，必须分别提交、打包。

2026-10-27 双机有线网卡、单播发现及现场启动环境见 [M2-NETWORK-SETUP.md](M2-NETWORK-SETUP.md)。Quest 数采入口自动加载各自 DDS XML，server 的 Quest ZMQ bridge 默认只绑定本机回环。本次网络改动只涉及主仓库，不修改 GELLO 控制逻辑。无硬件双机验证结果以本页后续记录为准。

## 1. 本机已验证的结果

2026-10-26，ROS Humble，独立 domain 173、`/m2` 话题和本机 ZMQ 15555：

- 22 项单元测试通过：入口、配置、来源标识、时间戳、过期／未来数据、缺图像、非法数值、静止目标与重复／回退时间戳。
- 真实 bridge + 合成 ROS 输入：正常链路通过；追加 schema 审计收到 121 包，121 包有效、0 包无效，观测约 15.002 Hz。
- 旧双相机配置下，运行约 7 秒后停止 action／state／cam_front 输入：分别收到 110／107／104 包，最后收到时间约 7.63／7.46／7.23 秒，随后停止输出。当前 Quest 左臂只采集 cam_left。
- 所有输入时间戳超前 5 秒：收到 0 包，bridge 拒绝输出。

本次日志及 JSON 报告保存在 `outputs/m2/20261026-local/`（本地验收产物，不随源码 bundle 自动迁移）。

这些数据仅来自短时间本机合成测试，不是跨机延迟、机器人 IK 频率或真实相机性能测量。合成 EE 姿态只用于消息／格式测试，不代表真实 FK 一致性验收。

## 2. 诊断环境

在对应机器源码根目录运行。先 source 已安装的 ROS 与 Franka overlay；server 构建完成后，再 source 本仓库 server overlay。client 不要 source server overlay 替换其控制环境。

```bash
# 示例：server 若确认安装 Jazzy；实际发行版以 /opt/ros 为准。
source /opt/ros/jazzy/setup.bash
source "$HOME/franka_ros2_ws/install/local_setup.bash"
# 首次构建前不存在此文件时跳过下面这行。
source gello_software/ros2/install/server/local_setup.bash

/usr/bin/python3 -m data_collection.diagnostics.environment \
  --output outputs/m2/server-environment.json
```

client 用 Humble 及实际 client overlay，保存为 `client-environment.json`。报告包含：提交号、子模块、ROS/RMW/domain 配置、消息展开定义及 SHA256、包加载路径、Python 模块、网卡和 NTP 状态。命令失败会记录在 JSON，不因报告文件生成成功就认定环境合格。

比较两端消息定义和配置；定义 hash 相同不等于跨发行版 DDS 一定兼容。NTP 已同步也不等于已经测得两机时钟偏差，仍需现场核对同步源／偏差。报告可能含本机网络与路径信息。

## 3. 隔离测试（本机或 server）

使用 ROS 系统 Python及 Franka overlay。测试工具直接加载当前 checkout 的 bridge 源码，不依赖该包已安装，也不能替代安装产物验证。相同测试端口只能串行运行。

```bash
export ROS_LOG_DIR=/tmp/m2-ros-logs

/usr/bin/python3 -m data_collection.diagnostics.isolated_check \
  --scenario normal --output-dir outputs/m2/normal

/usr/bin/python3 -m data_collection.diagnostics.isolated_check \
  --scenario action --output-dir outputs/m2/drop-action

/usr/bin/python3 -m data_collection.diagnostics.isolated_check \
  --scenario state --output-dir outputs/m2/drop-state

/usr/bin/python3 -m data_collection.diagnostics.isolated_check \
  --scenario camera --output-dir outputs/m2/drop-camera

/usr/bin/python3 -m data_collection.diagnostics.isolated_check \
  --scenario future --output-dir outputs/m2/future
```

工具内部强制 CycloneDDS、domain 173、XML 回环接口、`/m2` 输入和 15555/15557 测试端口，并清理其子进程，不继承外部 CYCLONEDDS_URI。只发布合成 observation／accepted 样本，不发布 raw command。普通场景约 13 秒；带 schema 的场景最多等待 45 秒。输出 result.json 和子进程日志；失败返回非零。结果保留首个失败原因及清理前后子进程退出码。

2026-10-27 本机 Humble 修复后复测：正常链路收到 190 包；schema 收到 121 包，全部有效，约 15.001 Hz；22 项既有测试通过。显式 XML 绑定 lo 时不再同时开启 ROS_LOCALHOST_ONLY，避免 Humble 重复选择接口。以上仍不是 server Jazzy 或两机验收结果。

完整 schema 检查需要 LeRobot Python。可在 normal 测试中传入该环境 Python 的绝对路径：

```bash
/usr/bin/python3 -m data_collection.diagnostics.isolated_check \
  --scenario normal --output-dir outputs/m2/schema \
  --sample-python /home/landau/anaconda3/envs/lerobot-vr/bin/python
```

server 的 Conda 路径需替换为其实际位置。schema 检查会复用 recorder 的 `bridge_packet_readiness`；单独 ROS 测试仅检查基本传输字段。

若需跨机合成测试，可手动启动 `synthetic_ros` 与 `scripts/start_m2_test_bridge.sh`，两端设置 domain 173 并允许跨机发现，清除 localhost-only 配置；这不属于上述 localhost 自动测试。首先确认没有其他系统使用该 domain。

## 4. Server 重建并启动真实数据链路

完成离线同步后在 server 构建：

```bash
bash scripts/build_data_collection_server.sh
```

保留构建输出，核对实际加载路径。先启动 client 已验证的左臂 VR 入口，再在 server 执行：

```bash
bash scripts/start_data_collection_server.sh \
  --teleop-source quest --left --arm --ros-domain-id 0
```

Quest 模式的 ZMQ bridge 和 recorder 默认在 server 本机 `127.0.0.1:5555` 通信；跨机的机器人状态和目标走有线 ROS 2 DDS `192.168.1.100 ↔ 192.168.1.50`。原 GELLO 分支的网络默认值仍可由 `DATA_COLLECTION_SERVER_IP` 覆盖。

client READY 后，server 先确认左臂实测状态、VR 状态及 cam_left 图像并报告观测链路就绪；这一步不等待 Y，也不在 server 初始化机械臂。由操作者按左 Y 进入 FOLLOWING 后，才会有正常 accepted target 和 ZMQ 样本。server 等待观测输入期间会检查相机／bridge 子进程是否退出，探测错误写入 `/tmp/quest-collection-probe.*`。收到一次状态消息并不代表录制门控已实现。

## 5. 两机真实输入与样本检查

在 server 第二个终端，source 同样的 ROS/server 环境，使用与 bridge 相同的 domain/RMW：

```bash
export ROS_DOMAIN_ID=0
/usr/bin/python3 -m data_collection.diagnostics.topics \
  --duration 30 --output outputs/m2/live-topics.json
```

该工具并行订阅左臂关节、accepted target、FrankaRobotState、VR 两类状态和 cam_left 图像，报告消息数、接收频率、最大间隔、最后接收年龄和 header 年龄。它使用 best-effort 兼容探测，收到消息不单独证明 bridge 的 reliable 订阅兼容；最终以 bridge 样本为准。输出中的最后状态可辅助区分 READY 等待 Y 与跟随中无目标。此工具不是状态机门控。

在 server 的 lerobot-vr Conda 环境中运行 ZMQ 检查：

```bash
conda activate lerobot-vr
python -m data_collection.diagnostics.samples \
  --host 127.0.0.1 --port 5555 \
  --duration 60 --silence-timeout 5 \
  --output outputs/m2/live-samples.json
```

短测通过后将 duration 改为 600 做十分钟接收测试。默认最低平均接收频率 10 Hz、最大样本年龄 0.5 秒；这是显式测试阈值，可用 `--min-rate`、`--max-age` 调整并记录理由。bridge 自身仍采用更严格的关节／相机年龄阈值，不因报告参数变更而放宽。

报告检查左臂 7 维 state/action/target、cam_left 图像、EE schema-v2 关系、Quest 来源和时间戳，输出 p50/p95/p99/max。连续 5 秒无包或收到非法包会导致失败；Ctrl+C 保存部分报告但不判通过。工具仅连接可信 bridge：现有 ZMQ 协议使用 pickle。

时间与采样解释：

- 接收间隔仅使用接收机 monotonic，不跨主机相减 monotonic。
- header 年龄与 wall-time 差包含主机时钟偏差，不能称为纯网络延迟。
- action_stamp_updates_observed 是采集端观察到的更新次数，15 Hz 下无法反推出 30 Hz 发布或 IK 频率。
- target_value_changes_observed 单独统计数值变化；静止但时间戳更新的目标仍是新鲜输入。
- bridge_diagnostics 中的 rejected_ticks 是 bridge 采样回调拒绝次数，不是网络丢包数；进程重启会清零。断流期间该字段无法随正常样本输出，需结合 bridge 日志和接收超时报告。
- 本轮 collection 模式拒绝非法、零值或超前超过 0.1 秒的输入时间戳；先修复时钟问题，不以放宽阈值掩盖它。EE 缺失 stamp 的既有回退行为仍保留，需在真实环境核对消息原始 header。

## 6. M2 仍待两机完成的验收

2026-10-27 **旧双相机配置的无硬件有线链路已通过**：server 重建 `franka_lerobot_data_bridge` 和 `franka_realsense_camera_publisher` 两包并核对安装路径；独立 domain 173 的双向字符串测试通过；server 本机合成 bridge/schema 审核 121/121 有效；跨机 client 合成关节、accepted target、FrankaRobotState、小图像 → server bridge → 本机 ZMQ 的 60 秒审核收到 **901/901 有效，0 无效，15.000 Hz**。报告在 server `outputs/m2/20261027-server-isolated/` 与 `outputs/m2/20261027-wired-crosshost-samples.json`。跨发行版 type-hash 警告仍出现，但没有阻止这些合成消息传输。这些报告保留为历史结果；当前 Quest 左臂单相机配置需要重新测试，也不是实际机器人或相机的验收结果。

仍待现场验收：

- 真实 cam_left 相机、关节、accepted action、EE 与 VR 状态的 RMW/QoS、吞吐、时间戳和两机时钟偏差；合成消息通过不代表真机话题全部兼容。
- 真实时钟偏差、十分钟连续接收统计、实际输入／IK／accepted 发布频率。
- client 退出、网络断开、相机停流、server 重启的现场诊断与恢复；停止真机输入应由现场操作者执行。
- TCP/FK 对齐及录制边界、FOLLOWING 门控、跨段清空 pending frame 属于 M3；本轮不将真实训练 episode 判为可用。
