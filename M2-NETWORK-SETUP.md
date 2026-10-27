# M2：Quest 左臂双机有线数据链路

更新日期：2026-10-27。本文只描述 Quest 数采模式；独立 VR 遥操和原 GELLO 模式仍有各自的启动入口。

## 实际通信路径

```text
Quest → client 上的 ALVR/OpenVR → 本机 IK/安全适配器 → 左 FR3
                                 │
                                 └─ ROS 2 DDS：实测关节、accepted target、VR 状态
                                    client 192.168.1.100 → server 192.168.1.50

server 本机 RealSense → ROS bridge ← 跨机 DDS
                         └─ ZMQ PUB 127.0.0.1:5555 → 本机 LeRobot recorder → server 磁盘
```

机器人 1 kHz FCI 通信只在 client 与 FR3 之间，使用 `enp114s0`（`172.16.0.100`）；不经过 server。Quest Streaming 继续使用 client 的 Wi-Fi。**server 不发布机械臂 raw command。**

原 GELLO 的双机链路也是 client 控制、跨机 ROS、server 合并相机和状态后以 ZMQ 送 recorder；本次复用 `franka_lerobot_data_bridge`、RealSense publisher 和 `lerobot_collection.py`。Quest 左臂配置在 `config/data_collection/quest_left.yaml`，action 订阅 `/left/vr/accepted_joint_states`，只采集左手相机 `cam_left`，采样率 15 Hz。Quest 专用相机配置为 `config/data_collection/quest_left_camera.yaml`；原 GELLO 相机启动配置不变。

## 网卡与配置

| 机器 | 采集 DDS 接口 | 本机地址 | 单播对端 |
|---|---|---|---|
| client | `enxf8e43bd073d7` | `192.168.1.100` | `192.168.1.50` |
| server | `enp132s0` | `192.168.1.50` | `192.168.1.100` |

双向 20 次有线 ICMP 短测均为 0 丢包，RTT 范围约 0.2–0.53 ms，链路显示 1 Gb/s 全双工。这只是连通性短测，不能代替 ROS/QoS、真实相机吞吐和长时间样本验收。两机当前 Wi-Fi 地址 `192.168.52.64`、`192.168.52.160` 可以继续用于 SSH 或 Quest 所需网络，但不作为 Quest 数采 DDS 路径；旧 server 地址 `192.168.50.13` 不可达。

仓库配置：

- client：`config/data_collection/cyclonedds-client.xml`
- server：`config/data_collection/cyclonedds-server.xml`

两份 XML 固定本机有线接口、关闭多播，并显式配置对端单播发现。两台机器的地址或网卡名变化后，先检查 `ip route get <对端有线 IP>`，再修改两份配置。不要把 client XML 复制到 server，也不要将 FR3 专用网卡用于跨机 DDS。

## 启动环境

Quest 数采入口 `scripts/start_data_collection_client.sh --teleop-source quest` 和 `scripts/start_data_collection_server.sh --teleop-source quest` 自动加载各自仓库的 XML，设置 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、`ROS_LOCALHOST_ONLY=0`，并清除继承的 `ROS_STATIC_PEERS`。server Quest 模式默认将 bridge 的 ZMQ PUB 绑定 `127.0.0.1:5555`；同机 recorder 连接这个地址。原 GELLO 模式的默认网络配置不受此变更影响。

手动运行 ROS 诊断时，需要在**各自机器**的诊断终端设置对应环境：

```bash
# client；server 将路径改为本机 checkout 内的 cyclonedds-server.xml
export CYCLONEDDS_URI="file://$PWD/config/data_collection/cyclonedds-client.xml"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=173  # 仅隔离合成测试；现场 Quest 采集使用 0
export ROS_LOCALHOST_ONLY=0
unset ROS_STATIC_PEERS
```

`README-QUICKSTART.md` 第 3 节的独立 VR 真机入口沿用已验证的原启动环境。**跨机数采请用第 5 节的 Quest 数采入口**，它自动启用这里的有线配置；不要给该入口加 `env -u CYCLONEDDS_URI`。

## 无硬件验证顺序

1. 两端核对主仓库与 `gello_software` 子模块提交、ROS overlay、`franka_msgs` 消息定义、网卡地址和时钟同步；在 server 重建 data bridge 与相机 publisher。现有构建目录存在不等于源码已更新。
2. 在独立 `ROS_DOMAIN_ID=173` 上，只使用 `/m2/...` 测试话题做双向 `std_msgs/String` 传输；不发布任何 `/<side>/vr/raw_joint_states` 或 GELLO raw-command。
3. server 使用 `scripts/start_m2_test_bridge.sh` 监听 `/m2/...` 并在本机 `127.0.0.1:15555` 发布 ZMQ；client 运行 `data_collection.diagnostics.synthetic_ros`。server 使用 `data_collection.diagnostics.samples` 审核 schema、字段和频率。合成图像只验证协议，不证明真实相机已经可用。
4. server 单独验证 `cam_left` 图像的名称、帧率和时间戳。然后在现场操作者启动左臂后，先运行不带 `--record` 的 Quest server 入口，检查 60 秒和 600 秒真实样本。

2026-10-27 已完成的无硬件结果：两机均能通过该 XML 创建 DDS 节点；`std_msgs/String` 在独立 domain 173 上双向收到。server 从当前源码重建两项 ROS 包成功，本机合成 bridge 审核 121/121 有效；client 发布跨机合成关节、末端与小图像，server 经 bridge 和本机 ZMQ 连续审核 60 秒，共 901/901 有效、0 无效、约 15.000 Hz，接收间隔最大约 67.4 ms。报告在 server 的 `outputs/m2/20261027-server-isolated/` 和 `outputs/m2/20261027-wired-crosshost-samples.json`。这组历史数据使用旧的左手加前置双相机合成配置；当前左手单相机配置需重新验收。Humble/Jazzy 的 type-hash 兼容警告仍会出现；实际合成消息与样本已经通过，真实话题/QoS 尚待验证。

第 4 步涉及真机，必须由现场操作者确认 Desk/FCI/工作空间后启动 client，且不属于本文的无硬件验证。录制 `FOLLOWING` 门控、回位/故障 episode 边界仍属 M3；在完成之前，不能把数据标为可用训练 episode。

正式录制前还需处理 server 剩余约 55 GB 空间，并通过短试录测得实际每分钟容量，再确定采集目录和预留空间。不要直接清理已有数据集。
