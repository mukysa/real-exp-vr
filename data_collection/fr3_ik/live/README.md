# FR3 实时只读预览（2026-10-22）

完整日常操作顺序见 [Quest → ALVR → 网页操作流程](../../../Quest-ALVR-OpenVR-Live-Preview.md)。本文保留技术参数和后续只读反馈入口。

当前选择路线 1：实时手柄 → V4 相对映射 → 受限 IK → HTML 虚拟双臂；双臂真实 q 另一路只读显示。**系统没有发送机器人运动命令的路径**。既有 Stage11 V2/V3/V4 和 Stage12 原始录制保持不变。

## 立即查看：不需要硬件

在项目根目录执行：

```bash
bash scripts/start_fr3_live_preview.sh \
  --demo-recording recordings/quest_raw/stage12-openvr-guided-20261020.jsonl
```

浏览器打开 **http://127.0.0.1:8765/**。约第 5.8 秒第一次 B 接管，第 74.7 秒第二次 B 停止。录制结束后进入 WAITING；Ctrl+C 结束，重新运行即可重播。这里机器人反馈始终是 **MOCK 历史快照**，Quest 是 **REPLAY**，不能当真机实时数据。

## 现场阶段一：实时手柄 + 模拟双臂

终端一：

```bash
bash scripts/start_fr3_live_preview.sh
```

终端二（Quest / ALVR / SteamVR 按原流程启动后）：

```bash
bash scripts/start_meta_quest_openvr_bridge.sh --host 127.0.0.1 --port 8001
```

本服务 TCP 使用 8001，原录制器默认 8000；一个 bridge 只向一个端口发送。不要同时启动争用同一 OpenVR 输入的 bridge；先由用户停止旧 bridge，再启动上述入口。此阶段只需要戴 Quest，不需要机器人开机。

穿戴调整时不按 B，确认页面左右控制器年龄持续更新且 READY。面向机械臂前伸方向，身体朝向与机械臂前方大致平行，可站右后方。**右手 B 按一次松开 → 接管；再按一次 → 停止**。左手对应 left/.3，右手对应 right/.2。先依次做左手、右手小幅前/左/上，随后停止并重新接管。不要求重新录制固定六段离线数据。

## 现场阶段二：真机只读反馈 + 实时手柄

**本轮没有执行设备连接。需要用户明确确认允许连接 .3/.2 读取状态，并核对现场 FCI 与已有进程占用。** 不自动启动控制器、不恢复机器人错误、不修改机器人参数。

只读程序已使用现有 libfranka 编译。重新编译（不连接设备）：

```bash
bash scripts/build_fr3_readonly_preview.sh
```

确认后替换终端一为：

```bash
bash scripts/start_fr3_live_preview.sh --robot-source franka --connect-readonly
```

终端二仍为上述 bridge。实测模型始终更新，虚拟模型仅接管后响应手柄。默认只绑定 localhost；不变更网络配置。关闭时 Ctrl+C，会结束两侧只读子进程。

## 左臂单臂模式（当前真机准备入口）

仅左臂时使用两个终端：

```bash
bash scripts/start_fr3_live_preview.sh --single-left --robot-source franka --connect-readonly --position-scale 0.5
bash scripts/start_meta_quest_openvr_bridge.sh --single-left --host 127.0.0.1 --port 8001
```

该组合只连接 `172.16.0.3`，不会打开 `172.16.0.2`。左手柄 X 键上升沿第一次接管，再次按下后先受限减速，再平滑归位到本次接管起点；松开 X 不停止。归位期间 X 锁定，结束后必须释放再按才能重新接管。归位只作用于虚拟预览参考，不会向真机发送命令。接管首帧锚定新鲜 measured q；平移目标使用 `position_scale=0.5`，即手柄 10 mm 只生成约 5 mm 的 TCP 平移目标，姿态保持当前锚定姿态。

Y 键现场排查（2026-10-22）：SteamVR 报告 `oculus_touch` / `touch_profile.json`，但用户按 Y 时原始 OpenVR `ulButtonPressed` 一直为 0，仅 `ulButtonTouched` 变化。ALVR 自定义 `Quest2Touch` 的 `button_set` 指向左手 `/input/a/click`，与左手 X 的语义相符。现将单臂离合改用 X；SteamVR Touch profile 将左 X 映射到 `left_a_press`（位 7）。需要重启 bridge 并实际读到 pressed 位，才能确认有效。服务提示会随输入 freshness 更新为“按 X 接管”。

当前服务仍是 `control_eligible=false`、`commands=DISABLED`，只显示和记录真机反馈；它不会向 ROS controller 或 libfranka 发布运动目标。真实执行需要后续单独启用并确认左臂 controller。

不连接硬件的单臂预览可使用：

```bash
bash scripts/start_fr3_live_preview.sh --single-left --robot-source mock --position-scale 0.5
```

网页按实际模式显示左臂、X 键、平移缩放和归位说明。单臂日志每个服务循环追加 `type=sample`，包含 measured q/TCP、数据年龄、原始平移增量、0.5 缩放增量、requested TCP、整形后 TCP 及 preview q；首次接管前没有 targets。`accepted_q=null` 表示没有执行控制器的接受反馈，不能将 preview q 当成 accepted target。记录仍为服务循环采样，并非 lossless Quest 原始包录制。

单臂 HTTP mock 烟测已通过，143 帧状态均只含 left、缩放 0.5、无 fatal error、真机命令禁用。该次临时日志已在 2026-10-23 输出整理时删除；行为由自动测试覆盖。单臂 X 停止后虚拟平滑归位由后续改动加入。本次未连接任一真实机械臂，未启动 ROS 控制器；左臂真实 IK → ROS/fake 联调仍待完成。

## 行为与实现边界（双臂模式）

- 默认模拟源；真机必须同时明确指定 `--robot-source franka --connect-readonly`。录制回放禁止配真机源。
- 右 B 只触发双臂模式上升沿，启动时已按住 B 不接管，去抖 150 ms。接管锚定两侧最新实测 q，首个计算帧不改变参考。V4 spatial-relative 映射，无自动偏航补偿。
- 右 B 停止先按受限参考减速，再用双臂共同时间的五次平滑关节轨迹返回本次接管起点。归位时长至少 1 秒，并根据最大关节位移、现有速度/加速度上限计算。STOPPING / RETURNING 期间拒绝 B，完成后需松开再按 B。再次接管从当时实测 q 重新锚定。轨迹仅供虚拟显示，未做碰撞规划。
- 单侧输入/反馈超过 250 ms、丢跟踪、输入跳变、重连均停止双方，恢复后需要重新释放并按 B。服务循环停顿超过 100 ms 也停止。队列溢出/积压或计算异常锁定 FAULT，需重启服务。
- 真实 reader 只调用 libfranka `Robot::read`；检查 q 有限且在模型关节范围、F_T_EE 与模型工具一致。reader 异常会失效该侧反馈并停止；不自动重连。实际 FCI 可用性、延迟和两侧长期并行读取尚待现场验证。
- 页面粗虚线是实测，细实线是虚拟，显示各侧输入年龄与两模型 TCP 距离。此距离不是 IK 目标残差。模型是连杆线图，不是碰撞或动力学仿真。
- 限速/限加速度沿用已验证的离线参考；仍 `control_eligible=false`，没有 ROS publisher、机器人 control 或 gripper 接口。不能把预览通过当真机控制验收。
- 实时状态 `/api/state`；网页 HTTP 只有 GET，不提供动作接口。JSONL 日志在 `outputs/fr3_ik/live-sessions/`，记录来源与状态转换快照，**不是完整原始录制**。原始录制应沿用原 recorder 流程。
- 本轮验证：Python 状态/约束测试、Node 三视角绘制烟测、只读 helper 编译、Stage12 实时节奏 localhost 回放。未进行真实浏览器截图验收、Quest 现场输入或真机连接。

代码入口：`engine.py` 状态与 IK；`server.py` 数据接入与 HTTP；`viewer.html/viewer.js` 页面；`scripts/franka_readonly_preview.cpp` 独立只读采集器。

### MobaXterm / SSH 启动注意

2026-10-22 更正：实时输入使用与 Stage12 原录制一致的 **直接 OpenVR bridge**：`start_meta_quest_openvr_bridge.sh`，不使用旧 SAPIEN `start_meta_quest_steamvr_bridge.sh`。后者先遭遇 SSH X11 认证失败，修正后仍报 `VR is not supported`；不再作为本实时系统入口。直接 OpenVR 不创建 SAPIEN 显示，不需要为本链路修改 X11 认证。已通过现有依赖 dry-run；实际 Streaming 输入待用户现场复试。

从 Windows 浏览器访问网页还需要 SSH 本地转发：MobaXterm 新建 Local port forwarding，将本地 8765 转到 SSH 服务器的 `127.0.0.1:8765`，再在 Windows 打开 `http://127.0.0.1:8765/`；服务保持只绑定 localhost。

### Firefox 显示 SSL_ERROR_RX_RECORD_TOO_LONG

本服务是普通 HTTP，地址必须是 `http://127.0.0.1:8765/`，不是 HTTPS。Firefox 若自动升级到 HTTPS，可在该站点的 HTTPS-Only 模式面板选择对此站点关闭/允许 HTTP（界面名称随版本变化），再重新输入完整 HTTP 地址；不需要全局关闭安全功能或安装证书。Windows 浏览器仍须上述 SSH 本地转发。2026-10-22 用户日志已确认直接 OpenVR 识别左右控制器并持续约 60 pose/s，总计两手约各 30 Hz；网页实际显示尚待确认。

### 2026-10-22 接管后误停修复

现场日志出现 `input or feedback stale`，但 pose_age 为约 −1～−5 ms，并非真实断流。根因：服务在队列读取前获取循环 now，接收线程在这之后写入的新样本被用旧 now 检查，负年龄误触发 watchdog。现改为实时消息处理时重新采样 monotonic，并在完成队列处理后再次采样 tick 时间。保留非负年龄检查与 250 ms 真断流保护。修改需要重启网页服务进程才生效；无需重启 ALVR/SteamVR。该条记载时间竞争修复时的行为；停止归位现已升级，见下节。

### 2026-10-22 虚拟平滑归位

用户明确要求替换 B 停止后的瞬间重置。现为 FOLLOWING → STOPPING → RETURNING → READY/WAITING。归位目标是本次接管时保存的左右 q，制动后从静止开始，同步五次插值（端点速度/加速度为零），按既有速度/加速度上限确定持续时间。归位过程中 B 不排队、不重启；完成后要求释放再按。仅显式 B 停止启动归位，异常停止仍走故障处理，不自动启动归位。已通过 12 项引擎测试和 Node 三视角烟测；现场动画观感待重启服务后验收。

## 虚拟阶段收尾确认（2026-10-22）

用户已现场认可：实时动作方向基本正确，不同朝向接管无跳变，B 停止后的平滑归位动画合理。归位速度略快，但当前接受，暂不调整。固定方向、脑门佩戴维持 Streaming、B 接管/停止与 RETURNING 锁定继续保留。虚拟工作至此收尾。

当前进入 **左臂 left / 172.16.0.3 真机准备阶段**，右臂由他人使用，不操作。现已有上文的显式 `--single-left` 入口；默认双臂 CLI 不可用于左臂单独接入。单臂 X 停止后的虚拟归位与双臂 B 行为一致，但真机依然没有命令路径；该预览归位不代表也不会执行真机归位。此前左臂只读采样证据见 `recordings/robot_state/readonly-left-20261022/README.md`；本次仅改动和验证预览逻辑，没有连接或移动真机。后续仍需完成左臂执行接口与 ROS/fake 联调，再根据新鲜现场状态提出具体运动方案。
