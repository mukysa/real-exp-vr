# Quest 3 → ALVR / OpenVR → 双臂与左臂虚拟预览流程

更新日期：2026-10-22。参考 [原始手柄录制复现](Quest-ALVR-OpenVR-Reproduction.md)，用于复现网页虚拟双臂流程和新增的虚拟左臂单臂流程。下文有相同步骤时先列双臂默认方式，再列左臂单臂命令。

本文提供两种互斥的虚拟预览模式：**默认双臂模式使用右手 B 接管/停止；左臂单臂模式使用左手 X 接管/停止**。两者均使用 MOCK 机器人反馈，不连接机器人，不开启 FCI，不发送机器人运动命令。固定操作方向；头显可按当前方法扣在脑门上以保持 Streaming 并观察屏幕，头显朝向不参与映射。

```text
Quest 左右 Touch 手柄
  → Quest ALVR client → 电脑 ALVR Dashboard + SteamVR
  → 直接 OpenVR bridge → TCP 127.0.0.1:8001
  → 实时预览服务（MOCK 历史机器人快照 + 相对映射 + 受限 IK；双臂或仅 left）
  → HTTP 127.0.0.1:8765 → 服务器本地 Firefox
```

## 0. 启动前约定

- 下列命令都在服务器 `/home/landau/real-exp-vr` 执行。图形程序和 Firefox 在服务器本地图形桌面运行最直接。
- ALVR、SteamVR 已在运行时，不重复启动。网页服务和输入 bridge 各占一个终端，运行期间保持打开。
- 使用现有脚本和环境，无需安装、升级依赖或手动切换 Conda 环境。脚本自行选择所需解释器。
- 本文不启动 raw recorder；实时会话日志不是完整手柄原始录制。
- 左手对应 left / `.3`，右手对应 right / `.2`。这些只是当前模型的实体标签，MOCK 模式不会连接这些 IP。
- 双臂是默认模式；仅左臂须在网页服务和 OpenVR bridge 两边都指定 `--single-left`，否则两个进程输入侧不匹配。

可先检查是否已有旧进程：

```bash
cd /home/landau/real-exp-vr
pgrep -af 'data_collection.fr3_ik.live.server|data_collection.meta_quest.openvr_bridge|data_collection.meta_quest.steamvr_bridge|raw_recorder'
ss -ltnp | rg ':8001|:8765'
```

如已有本次要用的正常进程，直接复用。需要重新启动时，回到对应终端按 Ctrl+C 正常退出。不要使用宽泛的 pkill，也不要删除旧录制或输出。

## 1. 启动 ALVR Dashboard

在服务器本地图形桌面打开终端：

```bash
cd /home/landau/real-exp-vr
./tools/alvr-v20.8.1/bin/alvr_dashboard
```

保持终端和 Dashboard 打开。头显未连接时显示 Disconnected 正常。

## 2. 启动 SteamVR

另开服务器本地图形终端：

```bash
/home/landau/.local/share/Steam/steam.sh -applaunch 250820
```

等待 SteamVR 状态窗口出现。必要时确认：

```bash
pgrep -af 'vrserver|vrmonitor|vrcompositor'
```

应看到 SteamVR runtime 相关进程。保持现有 ALVR 编码和网络配置，不需要每次重新注册 driver。仅当 SteamVR 完全看不到 ALVR driver 时，按原始复现文档的故障处理执行，不作为日常启动步骤。

## 3. 在 Quest 中建立 Streaming

1. 戴上并唤醒 Quest，确认左右手柄均可跟踪。
2. 打开 Quest 内的 ALVR。
3. 选择已信任的电脑（原流程名称 `5056.client`，以现场显示为准）；如出现信任提示，在电脑端确认。
4. 保持 ALVR 前台，等电脑 Dashboard 显示 **Streaming**。
5. 按当前已验证的方式调整头显到脑门，露出视线观察屏幕/机器人，同时确保 Streaming 不断开。避免遮挡跟踪摄像头。

如果头显休眠或 Streaming 断开，先恢复头显和 ALVR，再检查 bridge。不能只凭 Dashboard 图像判断手柄数据正常，后面还要看输入计数及网页状态。

## 4. 启动网页服务（终端 A）

```bash
cd /home/landau/real-exp-vr
bash scripts/start_fr3_live_preview.sh
```

正常输出类似：

```text
Preview: http://127.0.0.1:8765 | robot=mock | commands=DISABLED | log=...
```

**保持终端 A 运行。** 默认启动即为模拟反馈，不要添加真机选项。尚未收到手柄时网页显示 WAITING 是正常的。

左臂单臂虚拟预览时，终端 A 改用以下命令；MOCK 仍是历史快照，不是真机：

```bash
bash scripts/start_fr3_live_preview.sh --single-left --robot-source mock --position-scale 0.5
```

单臂网页只显示 left，位置缩放为 0.5，手柄旋转不改变目标姿态。运行期间看见 MOCK 和“真机命令：禁用”才是预期状态。

## 5. 启动直接 OpenVR 输入（终端 B）

```bash
cd /home/landau/real-exp-vr
bash scripts/start_meta_quest_openvr_bridge.sh --host 127.0.0.1 --port 8001
```

注意脚本名是 **openvr**，不使用旧的 `start_meta_quest_steamvr_bridge.sh`；旧 SAPIEN 路径在本次现场报 VR is not supported。

正常输出类似：

```text
Connecting direct OpenVR input to PoseReceiver at 127.0.0.1:8001; robot_output_enabled=false
OpenVR controller ids: {} -> {'left': 1, 'right': 2}
OpenVR frames=... sent=... rate=60.0 pose/s ids={'left': 1, 'right': 2}
```

左右设备编号可能变化，以实际角色识别结果为准。计数应持续增长。现场约 60 pose/s 是左右手总记录速率，每手约 30 Hz；不要求恰好等于这个值。

**保持终端 B 运行。** 此入口连接 8001，不是原录制流程的 8000。不要另起第二个 bridge 或原录制启动脚本来争用输入。

左臂单臂模式使用同一个 bridge，但须加 `--single-left`，只发送左手控制器数据：

```bash
bash scripts/start_meta_quest_openvr_bridge.sh --single-left --host 127.0.0.1 --port 8001
```

日志应持续显示 left controller 和 `sent` 计数；按下或松开左手 X 时，bridge 应分别打印 `Left X clutch=pressed/released`。左手 X 使用 ALVR 自定义左手 `/input/a/click` 对应的按键位。只有两个进程都使用单臂参数时，才能按本节预期操作。

## 6. 在服务器本地 Firefox 打开页面

在地址栏完整输入：

```text
http://127.0.0.1:8765/
```

必须为 **http**，不是 https。服务器本地浏览器不需要 SSH 转发。

页面应显示：

- 机器人来源：**MOCK 模拟快照（不是真机实时反馈）**。
- Quest：LIVE TCP；真机命令：禁用。
- 输入就绪后状态为 **READY**，两侧 Quest 数据年龄持续更新。
- 默认右后方视角；可切换正后方和俯视。
- 青色为左臂，橙色为右臂；粗虚线为反馈，细实线为虚拟臂。尚未运动时两者重合正常。

单臂模式下页面应只显示 left、显示 MOCK 与真机命令禁用；Ready 后提示按左手 X 接管。行内 X 按下/松开状态应随按键变化，反馈和 Quest 年龄持续更新。

### Firefox 出现 SSL_ERROR_RX_RECORD_TOO_LONG

这表示浏览器可能用 HTTPS 访问了普通 HTTP 服务。先确认地址前缀确为 `http://`。如仍自动升级，在 Firefox 打开 `about:config`，检查 `dom.security.https_only_mode` 和 `dom.security.https_first`；本次排查曾指导将存在且为 true 的项改为 false（不要新建缺失项）。这会改变该 Firefox 配置的 HTTPS 自动升级策略，优先使用站点例外；如已有可用配置，保持即可，不必反复调整。

检查服务可使用 GET：

```bash
curl --noproxy '*' --max-time 5 -v http://127.0.0.1:8765/ -o /tmp/fr3-live-page.html
```

正常应返回 `HTTP/1.0 200 OK`。Failed to connect 时检查终端 A；首页正常但浏览器报 SSL 错误时检查浏览器实际地址与升级设置。不要用 `curl -I` 判断页面功能，该服务未实现 HEAD。

## 7. 按 B 操作虚拟双臂

1. 暂时不按 B，调整手柄到舒适位置，确认页面 READY。
2. 站在当前方便的右后方位置。为方便理解动作，身体尽量朝机械臂向工作区伸出的方向。
3. **右手 B 短按一次并松开**，页面应变为 FOLLOWING。启动时已按住 B 不会接管，需要先松开再按。
4. 接管瞬间应保持原位，不跳变。之后移动手柄才会产生相对运动。
5. 先只用左手做 5～10 cm 的慢速前移、左移、抬高，再测试右手。左手只控制左臂，右手只控制右臂。
6. 松开 B 后保持跟随，**不用一直按住**。
7. 再次短按右手 B，停止双方跟随。先显示 STOPPING 制动，再显示 RETURNING，平滑归位到本次接管的起始姿态。
8. 可以重新摆放双手，再按右 B 接管。每次都重新锚定，接管首帧不应跳变。制动和归位期间接管均被拒绝，等 READY 后先松开 B，再按 B。

### 固定方向是当前约定

- +X：机械臂向工作区伸出的前方；+Y：左；+Z：上。页面显示共同轴，当前实际映射仍沿用已验证的跟踪空间方向，未新增物理偏航校准。
- B 重设相对运动起点，**不会按身体或头显朝向旋转坐标轴**。
- 因此你转向左侧后接管，身体右侧可能对应程序前方，这是当前预期行为。
- 头显扣在脑门上朝前上方不会旋转当前映射；头显朝向没有参与计算。暂不加入头显朝向对齐或手柄“前方校准”。

### 停止后的“回到快照”

MOCK 的反馈姿态固定不动。再次按 B 后，虚拟模型先制动，再按平滑关节轨迹返回本次接管起点（当前 MOCK 下即固定快照姿态），不会瞬间跳回。归位不是真机归位命令，也不代表碰撞检查通过。接管中不应无故归位。

## 7A. 按 X 操作虚拟左臂

双臂模式继续使用上一节的右手 B。若本次只验收左臂虚拟预览，确认两个进程均以 `--single-left` 启动，再按以下顺序操作：

1. 暂时不按 X，等待页面只显示 left 且状态为 READY。
2. **左手 X 短按一次并松开**，页面应进入 FOLLOWING；启动时已按住 X 不会接管，先松开再按。
3. 接管首帧应保持原位；随后小幅平移左手柄，虚拟左臂跟随。手柄输入平移按 `0.5` 缩放，例如输入 10 mm 生成约 5 mm 目标；旋转手柄不会改变目标姿态。
4. 松开 X 后继续跟随，不需要持续按住。
5. 再短按左手 X 停止。虚拟目标先进入 STOPPING 制动，再进入 RETURNING，平滑回到本次接管起点。
6. STOPPING / RETURNING 期间 X 锁定；归位完成后先释放 X，再按一次重新接管。每次新接管都从当前 measured/mock 状态重新锚定。

单臂下的“回到快照”与双臂 B 行为相同，但仅作用于虚拟 left 模型。MOCK 反馈固定，因此归位终点就是固定快照姿态；这不是机器人运动，也不是实机归位方案。断流、丢跟踪、位姿跳变、重连或计算异常会进入 HOLD/FAULT，不自动触发归位。

## 8. 状态与故障处理

| 状态/现象 | 处理 |
|---|---|
| READY | 输入就绪，可按右 B |
| FOLLOWING | 正在跟随，松开 B 不停止 |
| STOPPING / RETURNING | 正在制动 / 平滑归位，B 锁定；READY 后松开再按 B |
| WAITING | 看两侧数据年龄，检查 Streaming、bridge 和手柄跟踪 |
| FAULT | 记录页面原因，停止 bridge 后重启网页服务，再启动 bridge |
| 按 B 不动 | 先松开 B；确认 READY、左右数据齐全，使用右手 B |
| bridge Connection refused | 先启动终端 A，确认 TCP 8001；不要改到录制端口 8000 |
| Address already in use | 找到对应旧终端，复用或 Ctrl+C 退出，不重复运行 |
| 页面显示服务断开 | 检查终端 A 是否退出；页面最后一帧不代表数据仍有效 |
| 接管后自行停止 | 看页面 reason 和会话日志；真实断流、丢跟踪、跳变、重连或循环停顿均可能触发停止 |

仅左臂时，READY 要求 left 输入与 MOCK feedback 都新鲜；按左手 X 不接管时，先看网页 X 按下/松开状态及 bridge 的 `Left X clutch` 输出。STOPPING / RETURNING 时等待归位结束，再释放后按 X；故障停止不会自动归位。

早期现场曾因队列接收与循环时间竞争产生负数据年龄，误触发 stale 并回到快照；2026-10-22 已修复。运行旧进程不会自动加载修改，需按下面顺序重启。不要为避免停止而关闭真实断流保护。

重启：终端 B Ctrl+C → 终端 A Ctrl+C → 启动 A → 启动 B → 刷新网页 → READY 后重新按 B。

需要助手排查时，把报错/页面状态写到 `/home/landau/real-exp-vr/a_info.txt` 并通知即可。服务状态转换日志在 `outputs/fr3_ik/live-sessions/`，助手可直接读取，不必截图或复制全部输出。

## 9. 正常结束

1. 双臂用右 B 停止虚拟跟随；单臂用左 X 停止并等待虚拟归位完成。
2. 终端 B 按 Ctrl+C，等待 `Stopping OpenVR pose bridge cleanly.` 并返回提示符。
3. 终端 A 按 Ctrl+C，等待服务退出。
4. 不再操作时，从图形界面退出 SteamVR 和 ALVR Dashboard，再退出 Quest 内 ALVR。

该流程没有启动原始 recorder，不会生成新的完整 quest_raw 录制。旧录制和回放保持不变。

## 10. 当前验收与后续边界

2026-10-22 用户现场确认：网页可打开，实时动作基本符合预期；面向前、左、右启动均未发现接管跳变。当前固定方向模式予以保留。时间竞争修复后用户反馈基本正常，长期稳定性仍需持续观察。

本轮不要求真机开机或开启 FCI。真机只读反馈接入是后续独立步骤，需要明确授权后按 [技术说明](data_collection/fr3_ik/live/README.md) 执行；真机运动另行讨论。

## 快速启动清单（已有环境配置可用时）

1. 本地图形终端启动 ALVR Dashboard。
2. 启动 SteamVR。
3. Quest 打开 ALVR，保持电脑端 Streaming。
4. 双臂终端 A：`bash scripts/start_fr3_live_preview.sh`；单臂改用 `bash scripts/start_fr3_live_preview.sh --single-left --robot-source mock --position-scale 0.5`。
5. 双臂终端 B：`bash scripts/start_meta_quest_openvr_bridge.sh --host 127.0.0.1 --port 8001`；单臂改用 `bash scripts/start_meta_quest_openvr_bridge.sh --single-left --host 127.0.0.1 --port 8001`。
6. Firefox：`http://127.0.0.1:8765/`。
7. 双臂：确认 MOCK + READY → 右 B 短按松开 → 小幅操作 → 右 B 停止。单臂：确认仅 left + MOCK + READY → 左 X 短按松开 → 小幅平移/旋转输入 → 左 X 停止并等待虚拟归位。

## 虚拟阶段收尾确认（2026-10-22）

用户已现场认可：实时动作方向基本正确，不同朝向接管无跳变，B 停止后的平滑归位动画合理。归位速度略快，但当前接受，暂不调整。固定方向、脑门佩戴维持 Streaming、B 接管/停止与 RETURNING 锁定继续保留。虚拟工作至此收尾。

双臂和左臂单臂虚拟预览均使用 MOCK，不会连接真机。左臂单臂的 X 停止后会先制动再平滑归位；这只改变虚拟模型的参考轨迹，不是实机归位功能。左臂真实阶段仍需单独核对只读反馈和执行接口，再讨论具体运动范围；右臂由他人使用，不操作。
