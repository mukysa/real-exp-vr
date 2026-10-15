# VR 数采升级：现状与待办

审查日期：2026-10-14。

快速导航：

- [一、最新工作状态](#latest-status)
- [二、工作约束与两端分工](#constraints-and-roles)
- [三、后续执行顺序与技术参考](#next-steps)

---

<a id="latest-status"></a>

# 一、最新工作状态

**当前位置：client ROS 路径与构建隔离已完成，GELLO 端到端采集尚未验收。下一步先验证“新 client + 原 server”，再迁移 server，最后进入 VR 接入阶段。**

## 已完成：GELLO client ROS 路径与构建隔离（2026-10-14）

本轮仅处理 client 的左臂、右臂、双臂 × 纯机械臂、Franka 夹爪六种组合。以下正文中的环境问题为初始审查记录，不代表本轮全部已修复。

**本轮成果是将代码加载路径和 ROS 构建产物切换到当前副本，隔离终端继承的旧路径；不是把 GELLO 运行环境迁移到 lerobot-vr。** 六种模式均使用系统 ROS Python 和 C++ 控制器，不调用 LeRobot。设置 `LEROBOT_CONDA_ENV=lerobot-vr` 在这条链路上没有实际切换 Python 的效果，不能作为 Conda 迁移完成的证据。

- 新增 `scripts/build_data_collection_client.sh`，在干净的系统 ROS 环境下构建三个 client 包，产物位于 `gello_software/ros2/{build,install,log}/client`，保留复制来的旧产物。
- `start_data_collection_client.sh` 的 arm/gripper 路径选择新的 client install，清理继承的命令搜索路径及相关环境覆盖，设置 `LEROBOT_CONDA_ENV=lerobot-vr`。
- `start_teleoperation.sh` 仅增加可选 ROS install 路径支持。直接调用它仍保持原默认 install；本轮隔离入口是 `start_data_collection_client.sh`。
- 六种 GELLO 模式实际使用系统 ROS Python，不调用 Conda 包。因此本轮没有安装、卸载或重新绑定任何 Conda 包；`lerobot-vr` 的 LeRobot editable 旧路径仍待后续涉及该包时处理，不能认为整个 Conda 环境已隔离完成。
- 未修改控制算法、配置、话题接口、Wuji 路径或 server。未修改当前项目之外的代码和任何 Python 环境。

构建：

```bash
cd /home/landau/real-exp-vr
./scripts/build_data_collection_client.sh
```

完成真实设备准备后，沿用以下入口（本轮没有实际执行机器人启动）：

```bash
./scripts/start_data_collection_client.sh --left --arm
./scripts/start_data_collection_client.sh --right --arm
./scripts/start_data_collection_client.sh --duo --arm
./scripts/start_data_collection_client.sh --left --gripper
./scripts/start_data_collection_client.sh --right --gripper
./scripts/start_data_collection_client.sh --duo --gripper
```

验证结果：三个 ROS 包构建成功；六种 robot config 的 namespace/load_gripper 检查通过；三个 ROS 包前缀及 GELLO/夹爪模块解析到当前目录；隔离后的 ROS 路径无旧 real-exp 或 Documents/gello_software 引用；Python GELLO hardware/limit 测试 10 项通过；隔离环境下 JointReferenceGenerator C++ 测试 5 项通过；Shell 语法与 git diff --check 通过。

直接从污染的调用终端运行 C++ 测试曾加载旧库并报符号错误，清理环境并加载当前 client overlay 后通过。构建脚本使用独立 bash 命令且关闭 stdin，避免首次 heredoc 调用出现的子进程等待问题。

未验证：六种模式的真实启动、fake hardware 完整联调、跨机器 DDS 接收、真实 episode 采集。下一步应在用户安排设备联调后验证这些项目。

---

<a id="constraints-and-roles"></a>

# 二、工作约束与两端分工

## 已确认的工作原则与修改边界

- **最小化修改**：只解决当前阶段的旧工程、旧环境依赖，不顺带重构或扩展功能。保留现有代码风格，优先使用已有入口和配置机制。
- **接口兼容**：保持 client/server 间 topic 名称、消息类型、关节名称及顺序、单位、坐标约定、动作语义和必要的时间行为一致。accepted target 必须仍来自控制器接受后的目标。
- **文件边界**：只修改 `/home/landau/real-exp-vr/` 内的工程文件和构建产物；原 real-exp、DexArena 和其他工程只读使用。写入前注意符号链接和生成文件，避免间接修改外部目录。
- **环境边界**：如确需改动 Conda，只允许修改 `/home/landau/anaconda3/envs/lerobot-vr/`。不得修改旧 lerobot、Conda base、系统 Python/ROS 或公共 Franka 工作空间。系统 ROS 和公共工作空间仅作为只读依赖使用。
- **当前范围**：复现 GELLO 的左臂、右臂、双臂 × arm/gripper 六种组合；不处理 Wuji、VR 接入、训练或策略部署，不修改远端 server。后续迁移和功能开发分阶段进行。
- **验证边界**：构建、静态配置检查、离线单元测试、fake hardware 联调和真实采集分别记录，不能互相替代。本阶段没有授权启动真实机器人控制，实机联调由用户另行安排。
- **按实际依赖处理**：不为使用 lerobot-vr 而把 ROS 强行迁入 Python 3.12，不整体安装 ms3 环境快照，不把 client 不使用的视频或训练依赖当作 GELLO 控制前置条件。
- **迁移与功能改造分开验收**：先建立原 GELLO 流程的隔离基线，再引入 VR，便于定位故障来源。

## Client / Server 分工与迁移约定

当前机器是 **client（机器人控制主机）**。server 地址在脚本中默认是 `192.168.50.13`。分工已核对 `scripts/start_data_collection_client.sh` 和 `scripts/start_data_collection_server.sh`。

| 职责 | Client：当前机器 | Server：数采主机 |
|---|---|---|
| GELLO 输入 | 运行 | 不承担 |
| 后续 Quest/VR 输入 | 头显连接 client，接收手柄追踪数据 | 不承担 |
| 坐标转换、离合、约束、FR3 IK | 后续主要改造位置 | 不承担 |
| FR3 和 Franka 夹爪控制 | 运行 | 不承担 |
| 机器人状态和 accepted target | 发布 | 经 ROS 2 DDS 跨机器订阅 |
| RealSense 相机发布 | 不承担 | 运行 |
| ROS → ZMQ LeRobot bridge | 不承担 | 运行 |
| LeRobot recorder、视频和数据落盘 | 不承担 | 运行 |
| 后续手柄录制按钮 | 产生录制事件 | 接收事件并控制 recorder |

```text
Client
  GELLO 或 Quest（头显追踪手柄，经 USB/ADB 或网络发送输入）
    → 输入处理 / 目标生成 → FR3 控制器 → 机器人
                               │
                 状态与 accepted target（ROS 2 DDS）
                               ▼
Server
  相机 + 机器人状态/目标 → ROS-to-ZMQ bridge → LeRobot recorder → 数据集
```

VR 替代 GELLO 的主要功能改造发生在 client。只要接口结构、数据语义和时间行为兼容，server 的相机、bridge 和数据写入核心可以大部分沿用。server 仍有两项已知的后续适配：

1. 当前 server 启动预检还要求 GELLO raw joint topic。VR 模式需按输入来源调整预检，不能仅保持 accepted target 就假定 server 无需修改，也不应伪造 GELLO raw 输入。
2. 手柄按钮控制录制时，server recorder 需要录制事件入口，以及去重、边沿和重连处理；目前主要使用本机键盘 `s/e/d/q`。

用户说明 server 理论上也有旧 real-exp 工程和 lerobot 环境，后续会将本项目迁移过去；**远端实际状态尚未检查**。源码、配置和安装说明可以迁移，ROS build/install/log 应在 server 本机重新生成，lerobot-vr 也应在 server 本机建立并绑定当地路径，避免复制绝对路径。

迁移顺序：先完成本机 client 隔离并保持兼容旧 server；用户迁移源码后，单独检查 server 环境、ROS bridge 与视频记录；以 GELLO 数据采集作为基线，再主要改造 client 的 VR 路径。server 侧环境迁移不能由本机 Conda 检查结果代替。

---

<a id="next-steps"></a>

# 三、后续执行顺序与技术参考

## 已确认的执行顺序

以下阶段按顺序验收。计划不代表已经执行，也不扩展当前已授权的修改或设备操作范围。

| 阶段 | Client | Server | 验收目标 | 当前状态 |
|---|---|---|---|---|
| A：验证本轮隔离 | 当前 real-exp-vr 独立 ROS 构建 | 保留原项目和原 lerobot 环境 | 完成短 GELLO episode，确认 client 改动没有破坏数采 | 待启动验证与联调 |
| B：完成两端迁移 | 沿用 A 阶段验证的 client | 部署新项目、本机重建 ROS，recorder 使用 lerobot-vr | 再次完成 episode，确认实际流程不再加载旧工程代码或旧 Conda 环境 | 待迁移及远端检查 |
| C：固化 GELLO 基线 | 保存代码版本、依赖与启动方式 | 保存代码版本、依赖与启动方式 | 留存通过校验的数据和两端可复现配置 | 待 A/B 完成 |
| D：VR 接入 | 按需补充 lerobot-vr 依赖并实现 VR 输入链路 | 保持已验证记录环境，按需适配事件和接口 | VR 数采可用，原 GELLO 路径仍兼容 | 尚未开始 |

**A 阶段：新 client + 原 server。** 先做无硬件启动验证，检查 launch、控制器插件与运行依赖；再由用户安排设备准备，检查 GELLO 串口、机器人网络/FCI 和姿态，验证跨机器 DDS 消息持续更新，最后保存、读回并校验短 episode。六种组合分别记录实际验证结果，不能用一个组合代替全部。

**B 阶段：两端迁移。** 用户迁移源码后，在 server 检查真实环境状态，重新构建其 ROS 包，创建或检查 lerobot-vr，绑定当地 LeRobot 源码，配置 recorder 环境，并验证视频写入、读回及数据校验。ROS 仍使用系统环境；本机视频依赖故障不是 A 阶段 client 控制的前置阻塞，也不能直接推定远端存在相同故障。

**C 阶段：保存基线。** 记录两端代码版本、依赖清单、构建与启动命令、测试结果和短 episode 的位置，作为后续 VR 改动的回归依据。

**D 阶段：开始 VR 适配。** 在上述 GELLO 基线验证后，先确定复用哪些 DexArena 模块，再检查并补充缺失依赖。当前 Quest 接收器和 NumPy/SciPy 可能已满足部分需求，不预先大幅更换包版本，也不整份合并 ms3-enviroment.yaml。

- Client 的 lerobot-vr：按需增加 VR 接收、坐标变换、IK 等能力，通过明确接口连接系统 ROS 控制进程。
- Server 的 lerobot-vr：优先保持已验证的记录环境稳定，仅在录制事件或数据接口确有需要时调整。
- 两端 Conda 环境不要求包清单完全相同，按职责分别管理。
- 每次引入依赖或功能后验证原 GELLO 路径，最终保留 GELLO/VR 输入选择。

下面保留完整目标、初始审查与分阶段技术 TODO。**初始审查中的问题不等于已修复；完成状态以文档开头及 P0 清单为准。** A–D 是整体验收顺序，后文 P0–P4 是技术任务拆分。

---

## 1. 目标与结论

将 `/home/landau/real-exp-vr/`（复制自 `/home/landau/real-exp/`）升级为结合 `/home/landau/yys/DexArena/` 的 VR 手柄数采项目，替代 GELLO 的机械臂输入，同时兼容原项目。

建议保留现有 FR3 控制、相机、LeRobot 记录、数据校验和回放链路，移植 DexArena 的 VR 输入处理、离合跟随、轨迹整形方法，并增加面向数采的录制控制与质量检查。

优先事项：

1. 让复制后的工程与环境真正独立于原目录。
2. 实现 VR 到 FR3 的目标生成与控制链路。
3. 保持动作语义，完善时间对齐、中断处理和 episode 审核。

**不建议把 `ms3-enviroment.yaml` 整份安装进 `lerobot-vr`。**

当前已确认的隔离范围是 GELLO 六种组合；后续 VR 首个功能里程碑仍建议为：**单右臂 + VR 手柄 + 现有 LeRobot 数采，完成一条可审核、可校验、可回放的 episode。** 两者属于不同阶段。

## 2. 现有项目基础

```text
GELLO → 原始关节目标 → FR3 控制器检查与接受 → 平滑参考 → 机器人

机器人状态 + 控制器接受的目标 + 相机 + 夹爪/Wuji 遥测
  → ROS 2 LeRobot bridge → ZMQ → LeRobot recorder → 数据集/校验/回放
```

### 2.1 控制器和数据桥

[configurable_joint_impedance_controller.cpp](gello_software/ros2/src/franka_fr3_arm_controllers/src/configurable_joint_impedance_controller.cpp) 会验证、限幅输入目标，再发布 accepted target。输入话题与 accepted target 话题已经参数化。

[lerobot_data_bridge.py](gello_software/ros2/src/franka_lerobot_data_bridge/franka_lerobot_data_bridge/lerobot_data_bridge.py) 已支持：

- 左臂、右臂、双臂。
- 纯机械臂、Franka 夹爪、Wuji 手。
- 实测关节、目标关节、实测末端位姿，以及目标关节 FK 对应的末端目标。
- 相机与机器人、手部数据新鲜度检查，以及相机间时间偏差检查。

**VR 应继续通过控制器 accepted target 接入采集链路。** 需要区分 VR 期望位姿、IK 输出、控制器接受的目标和机器人实测状态。

### 2.2 现有数据语义

[lerobot_collection.py](data_collection/lerobot_collection.py) 使用相邻采样包生成动作：

```text
action_arm[t] = q_target[t+1] - q_measured[t]
```

夹爪记录下一采样目标的归一化绝对开度，Wuji 手记录下一采样目标的绝对关节位置。同时保存 joint、EE 和图像等辅助字段。

更换输入设备不应静默改变这些含义。

### 2.3 已有 Quest 输入基础

[meta_quest](data_collection/meta_quest/README.md) 已有 TCP 接收、位姿解析、`grasp/clutch/record` 输入、接收时间、可视化和锚点标定基础。

当前它仍是输入检查模块，没有接通真实机器人。数据保持原始 Unity tracking frame，不能直接作为 FR3 base frame 目标。

## 3. DexArena 可复用内容与边界

| 模块（相对 DexArena 根目录） | 可复用内容 | 必须处理的适配 |
|---|---|---|
| `data_gen/teleop/io.py` | UDP 输入、协议版本、按钮、新鲜度 | 与本项目 TCP 接收统一接口 |
| `data_gen/teleop/wrist_servo.py` | 离合锚点、保持、冻结检测、二阶位姿整形 | 坐标、旋转符号、增益、频率 |
| `data_gen/teleop/arm_ik.py` | 阻尼微分 IK、零空间姿态调节、限位回避 | ManiSkill/SAPIEN agent 与 Panda 模型需替换为 FR3 |
| `data_gen/teleop/recorder.py` | 暂存、审核、保留/丢弃、导出流程 | 沿用真机 recorder 主线，借鉴工作流 |
| `data_gen/teleop/vr_native/receiver.py` | SteamVR 输入和按钮上升沿 | 依赖 VR display/SAPIEN 渲染，不是首阶段必需 |

具体发现：

- DexArena 的 `libs/VR-hand-tracking-and-controller/` 当前为空，文档引用的 `scripts/controller_bridge.py` 不存在。`.gitmodules` 指向 `https://github.com/Soo0803/VR-hand-tracking-and-controller.git` 的 `maniskill-shadowhand` 分支，需要补齐并固定版本。
- 本项目文档说明：实验室 Quest 控制器扩展不在引用的 upstream `v1.1.0` 中。需要固定可构建源码、APK 和协议版本。
- 本项目解析 Unity 原始坐标和 `xyzw`；DexArena 下游常使用已转换的仿真坐标和 `wxyz`。wrist servo 中还存在针对仿真控制器的旋转负号，不能直接移植到 FR3。
- DexArena 灵巧手路径使用 VR 控制手腕、MANUS 控制手指。本项目已有 Wuji glove/hand 链路，首阶段可只替换机械臂输入。VR 手柄本身不能提供完整的 20 关节手指动作。

## 4. lerobot-vr 检查结果

### 4.1 包版本与导入

| 项目 | 结果 |
|---|---|
| Python | 3.12.13 |
| LeRobot | 0.5.2，普通导入指向旧项目 |
| NumPy / SciPy | 2.2.6 / 1.18.0，导入成功 |
| Torch / TorchVision | 2.10.0 / 0.25.0，导入成功 |
| TorchCodec | 0.10.0，导入失败 |
| PyAV / PyArrow / ZMQ | 导入成功 |
| Pinocchio / NLOpt | pin 3.8.0 / nlopt 2.11.0，导入成功 |
| Wuji SDK / wujihandpy | 导入成功，未连接设备 |
| ManiSkill / SAPIEN / dex-retargeting | 未安装 |
| pytest | 未安装在 lerobot-vr 中 |

### 4.2 复制隔离尚未完成

`lerobot-vr` 的 editable 安装元数据仍指向：

```text
lerobot          → /home/landau/real-exp/lerobot
wuji-retargeting  → /home/landau/real-exp/libs/wuji-retargeting
```

`lerobot_collection.py` 会把当前项目的 `lerobot/src` 插入 Python 搜索路径前部，所以本次数采模块导入确实使用了新副本。但普通 Python 导入和其他入口仍可能使用旧目录。

复制的 ROS bridge egg-link 指向：

```text
/home/landau/real-exp/gello_software/ros2/build/franka_lerobot_data_bridge
```

ROS 生成的部分 setup 文件也保留旧路径。需要重新绑定本地 Python 包，在新目录构建独立 ROS 产物并核对加载位置。

### 4.3 启动环境默认值

部分启动器通过 `LEROBOT_CONDA_ENV` 选择环境，默认仍是 `lerobot`；Wuji 还支持 `WUJI_CONDA_ENV`。仅激活 `lerobot-vr` 不保证脚本使用该环境。

应显式配置 VR 项目的环境，并保留原启动方式。

### 4.4 视频与依赖问题

- 清理 ROS 路径后的 Python 和 `conda run -n lerobot-vr` 均出现 TorchCodec 加载失败，错误包含缺失 `libavutil.so.*`。该 Conda 启动环境中没有找到 `ffmpeg` 命令。
- PyAV 和 LeRobot Dataset 类可导入，因此不能据此认定完全无法录制。但本地 LeRobot 检测到 TorchCodec 包后会优先选择它，视频读取、验证或训练存在明确阻塞，需要写入与读回验证。
- `pip check` 报告 cmeel-boost 1.89.0 要求 NumPy `>=2.3,<2.4`，当前为 2.2.6。本地 LeRobot 又要求 NumPy `<2.3`，不能简单升级 NumPy，需协调 Pinocchio/cmeel 版本组合。
- `opencv-python 5.0.0.93` 与 `opencv-python-headless 4.13.0.92` 同时安装，共用 `cv2`，应按显示需求整理。

## 5. ms3-enviroment.yaml 的处理建议

[ms3-enviroment.yaml](ms3-enviroment.yaml) 是完整仿真环境快照，包含：

- Python 3.10.20、LeRobot 0.4.0。
- Torch/TorchVision 的特定 nightly 构建。
- ManiSkill、SAPIEN、渲染、训练与大量辅助包。
- 原机器 prefix：`/home/pair/miniconda/envs/ms3`。

当前 [LeRobot pyproject.toml](lerobot/pyproject.toml) 要求 Python `>=3.12`、Torch `<2.11.0`，与 YAML 的 Python、Torch 有直接冲突。

204 条 pip 固定版本与 lerobot-vr 比较：97 条版本相同、47 条版本不同、60 条未安装。**这不代表需要补装全部 60 个包。** 应按实际移植模块建立最小依赖。

建议保持环境分工：

```text
系统 ROS 2 / Python 3.10
  → FR3 控制器、ROS bridge、相机、ROS 回放

lerobot-vr / Python 3.12
  → 数据记录与处理、独立 VR 算法、Wuji 用户态进程

可选独立仿真环境
  → 完整 DexArena / ManiSkill / SAPIEN 验证
```

## 6. 推荐接入架构与兼容原则

```text
GELLO → 现有关节目标生成 ──────────────────────┐
                                             │
VR → Quest 接收 → 坐标转换/工具偏置            │
   → 离合锚点/跟随/保持 → 位姿整形与约束       │
   → FR3 IK ──────────────────────────────────┤
                                             ▼
                                      FR3 关节控制器
                                             │
                                      accepted target
                                             │
机器人状态 + 相机 + 夹爪/Wuji 遥测 ────────────┤
                                             ▼
                                      bridge → recorder
```

上图表示可选择的输入路径，运行时必须明确唯一命令源。

优先采用“VR 位姿 → FR3 IK → 现有关节控制器”，保留已有数据动作定义。[solve_fr3_ik](data_collection/move_to_target_ee.py) 可参考 FR3 模型与约束，但当前使用 `least_squares`，需评估在线延迟和连续性后才能用于周期控制。DexArena 微分 IK 需要替换模型、Jacobian、限位和工具变换。

兼容原则：

- 增加 `--input gello|vr` 一类显式选择，保留旧默认行为。
- VR 使用自己的命令话题和配置，避免与 GELLO 同时发布目标。
- bridge 按配置订阅相应 accepted target。
- 更新启动预检和 watchdog；目前部分脚本硬编码检查 GELLO raw/accepted 话题。
- 保留 arm/end-effector 组合和既有字段，新增输入来源、标定版本等元数据。
- VR 数据先使用独立目录，语义与质量验证后再考虑混合训练。

## 7. 数采重点缺口

### 7.1 录制与跟随状态分离

B 键离合、A 键录制可参考 DexArena，但保持、录制、保存、丢弃必须明确区分。按钮使用上升沿处理，重连防误触发；保留键盘 `s/e/d/q`。

### 7.2 中断与相邻帧配对

当前 recorder 在开始录制时检查新鲜度，但录制期间未见显式超时断段逻辑。断流时 `pending_packet` 可能保留到恢复后，再与新包配对，把长时间间隔当作相邻帧。

需要采样间隔检查、暂停/断段策略，禁止跨中断配对。

### 7.3 时间和来源信息

bridge 已有部分时间与相机同步信息，但 recorder 的帧转换未完整保留诊断字段。建议保存源时间、接收时间、帧序号、输入模式、标定版本、跟随状态、IK 拒绝和限幅事件。跨机器不能直接比较各自的 monotonic 时钟。

### 7.4 控制频率与采集频率

当前 bridge 配置为 15 Hz，DexArena 示例常用 20 Hz。VR 输入、目标生成、底层控制和写入可采用不同频率，但 dataset FPS 必须匹配实际采样策略，检查重复图像、丢包和时间偏差。

### 7.5 Episode 审核

借鉴 pending/review，提供预览、保留、丢弃、成功/失败与异常原因。真实任务没有通用 `env.success`，首阶段采用人工确认。

### 7.6 可执行目标与一致性

不可达目标、IK 跳变、限幅或失跟踪时，应进入明确保持与重新锚定流程。数据应能追溯实际 accepted target，不能直接使用不可执行的手柄期望作为动作。

## 8. 分阶段 TODO

### P0：工程隔离与环境修复

已完成的 client 隔离：

- [x] 在当前目录独立构建三个 GELLO client ROS 包，保留复制产物。
- [x] client arm/gripper 入口选择新 install 并清理继承路径。
- [x] 检查六种配置、ROS 包前缀和模块来源；10 项 Python 与 5 项 C++ 测试通过。
- [x] 明确系统 ROS 与 Conda 的职责：当前六种模式不调用 LeRobot，不宣称已完成 Conda 迁移。

尚未完成的联调与后续迁移：

- [ ] 六种模式的完整 fake hardware 联调。
- [ ] 用户安排后的实机启动、旧 server 跨机器接收与短 episode 验收。
- [ ] 后续实际使用 LeRobot 时修正 lerobot-vr 的 editable 旧路径；Wuji 暂不处理。
- [ ] 用户迁移项目后检查 server，独立重建其所需 ROS 包和 lerobot-vr 环境，修正 recorder 环境选择。
- [ ] 在实际记录数据的 server 检查视频链路和依赖；本机已发现的问题不能直接推定为远端问题。
- [ ] 按实际需要处理 TorchCodec/FFmpeg、NumPy/cmeel、OpenCV，完成视频写入、读回与数据校验。
- [ ] 固化 client/server 各自最小依赖与安装说明，仿真环境单独管理。

验收分开记录：client 加载当前副本并保持控制接口；server 使用迁移后的环境记录并读回数据；最终 GELLO 端到端采集通过。P0 尚未整体完成。

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

### P3：完整数采闭环

- [ ] 接入按钮录制事件，保留键盘控制。
- [ ] 增加中断检测与禁止跨间隔配对。
- [ ] 保存实际时间、来源与质量元数据。
- [ ] 对齐采样频率、dataset FPS、图像与机器人状态。
- [ ] 实现 episode 预览、审核、标签与丢弃。
- [ ] 完成一条数据的记录、校验与回放。

验收：动作可追溯；异常不会静默污染连续轨迹；episode 可审核、校验和回放。

### P4：组合扩展

- [ ] 从单臂扩展至双臂。
- [ ] 验证 Franka 夹爪与 Wuji glove/hand 组合。
- [ ] 验证 GELLO/VR 数据 schema 的兼容性与混合训练前提。
- [ ] 按需增加 DexArena 仿真对照验证。

验收：各组合有明确配置；历史数据读取、处理与回放保持兼容。

## 9. 初始只读审查的验证范围

- 已读取两个项目的相关控制、输入、数采、配置与依赖代码。
- 已检查 Conda 环境、包版本、editable 路径和 pip check。
- 已实际导入基础依赖、LeRobot Dataset 和本项目 recorder。
- 系统 Python 下选定的 Quest 协议与标定测试：**7 passed，5 deselected**。
- lerobot-vr 未安装 pytest，未运行完整数采测试套件。
- 未做相机、头显、机器人或 Wuji 设备在线验证。
- 初始审查期间未安装依赖或修改业务代码；随后已实施的 client 启动与构建改动见文档开头的已完成记录。

这些结果足以确定改造边界与优先级，但不代表环境已经达到实机采集就绪状态。
