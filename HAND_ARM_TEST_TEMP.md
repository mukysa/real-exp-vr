# --hand-arm 临时操作说明

日期：2026-11-02。

启动流程与原来相同：client 控制机械臂；server 用两个终端分别运行接收脚本和录制器。三个入口均启用 `--hand-arm`。

如果原命令使用 `--gripper`，将它替换成 `--hand-arm`，不能同时使用。`--hand-arm` 自动选择 arm-only，保留 `--arm` 也可以。本轮只控制左臂，不启动 Franka gripper 或 Wuji 手。

## 1. 启动前

- client 和 server 均使用包含本次修改的代码。
- SteamVR、ALVR 正常运行，Quest 显示 Streaming，左 Touch 位姿有效，右 Touch 按钮可读。
- 两端 ROS_DOMAIN_ID 一致，以下命令使用 0；两机系统时间需同步，远程按键请求有效期为 1 秒。
- 左臂 Desk/FCI 正常，没有其他机械臂控制或回放进程占用左臂。
- 启动时松开右侧 B、A 和食指扳机；已经按住的输入需要先释放再按。

## 2. client：启动左臂控制

在 client 终端执行：

```bash
cd /home/landau/real-exp-vr
export ROS_DOMAIN_ID=0

bash scripts/start_data_collection_client.sh \
  --teleop-source quest --left --hand-arm --enable-hardware
```

如需先做不连接机械臂的入口检查，将最后的 `--enable-hardware` 改为 `--check-only`。

先等待 READY，再用右 B 验证接管、小幅位姿跟随和回位。此时可以暂不启动 server；录制器尚未启动时，A/食指扳机不执行录制操作。

## 3. server 终端 1：接收、相机和桥接

不要加 `--record`，保留原来的独立录制方式：

```bash
cd ~/real-exp-vr

bash scripts/start_data_collection_server.sh \
  --teleop-source quest --left --hand-arm \
  --ros-domain-id 0
```

保持这个终端运行。它负责相机、client 状态接收、采集样本发布，以及新模式的录制按键转发。

## 4. server 终端 2：独立录制器

```bash
cd ~/real-exp-vr
conda activate lerobot-vr

python data_collection/lerobot_collection.py \
  --host 127.0.0.1 --port 5555 \
  --hand-arm \
  --repo-id local/franka_quest_left_hand_arm_teleop \
  --local-dir ./lerobot_data/quest_left_hand_arm_test1
```

如需另建测试数据集，修改 `--local-dir`。只启动一个录制器。

## 5. 按键映射

| 输入 | 行为 |
|---|---|
| 左 Touch 位姿 | 驱动左臂位置和朝向 |
| 左 Touch 全部按键、扳机 | 忽略 |
| 右 B 短按 | 接管左臂；再次短按返回本轮接管点 |
| 右 A 短按 | 空闲时开始录制；录制中结束并保存 |
| 右食指扳机 | 放弃当前未保存 episode，不删除已保存历史 |
| server 终端 2 输入 s/e/d/q + Enter | 保留原键盘操作 |

食指扳机不是侧面的 grip。右手柄位姿不控制机械臂。长按只触发一次，释放后才能再次触发。

## 6. 建议测试顺序

1. 先验证臂：右 B 接管 → 确认 FOLLOWING → 小幅移动左 Touch → 右 B 回位 → 等待 READY → 松开并再次按 B，检查再次接管。
2. 验证保存：右 B 接管 → 确认 FOLLOWING 和录制器可用 → 右 A 开始 → 移动左 Touch → 右 A 保存 → 等待保存完成 → 右 B 回位。
3. 验证丢弃：右 B 接管 → 右 A 开始 → 移动左 Touch → 右食指扳机丢弃 → 确认丢弃完成 → 右 B 回位。

全程不需要按左手柄按键。开始录制需要有效新鲜样本和 FOLLOWING 状态；开始被拒后，下一次 A 仍尝试开始。B 不会自动结束录制，先保存或丢弃，再回位。

client 的 LEFT_SOURCE_STATUS 中可查看 recorder 状态，server 终端 2 可查看开始、保存、丢弃的结果。

## 7. 退出

在 server 终端 2 输入 `q` 并回车，只退出录制器；有未保存帧时按原流程保存。若要放弃当前条目，先输入 `d` 回车，再输入 `q` 回车。

server 终端 1 的接收和相机进程继续运行；需要停止时按 Ctrl+C。client 臂控制也需在其启动终端按 Ctrl+C 停止。

省略 `--hand-arm` 继续使用原有模式。当前软件构建与本地验证已完成，本文件用于现场验收。
