# Quest 3 控制 FR3：快速启动

适用于已配置好的 client（`/home/landau/real-exp-vr`）。`--left` 控制左 FR3（`172.16.0.3`），`--right` 控制右 FR3（`172.16.0.2`），`--duo` 控制双臂。左臂已完成现场真机验证；右臂与双臂已完成离线检查，仍待现场真机测试。详细记录和注意事项见 [README.md](README.md)。

## 1. client 开机后设置性能

```bash
cd /home/landau/real-exp-vr
sed -i 's/cpu_affinity: \[4,5\]/cpu_affinity: [4]/' \
  gello_software/ros2/src/franka_fr3_arm_controllers/config/controllers.yaml
sudo powerprofilesctl set performance
printf 'performance\n' | sudo tee /sys/devices/system/cpu/cpu{4,5}/cpufreq/scaling_governor
sudo ethtool -G enp114s0 rx 4096
sudo systemctl stop irqbalance
awk -F: '/enp114s0($|-)/ {gsub(/[[:space:]]/, "", $1); print $1}' /proc/interrupts |
  while read -r irq; do printf '5\n' | sudo tee "/proc/irq/${irq}/smp_affinity_list" >/dev/null; done
```

## 2. 启动 Steam、ALVR Dashboard、SteamVR

分别在 client 图形桌面终端执行；已运行的程序无需重复启动。

```bash
env -u WAYLAND_DISPLAY DISPLAY=:0 /home/landau/.local/share/Steam/steam.sh -silent
```

```bash
cd /home/landau/real-exp-vr
env -u WAYLAND_DISPLAY DISPLAY=:0 ./tools/alvr-v20.14.1/bin/alvr_dashboard
```

```bash
env -u WAYLAND_DISPLAY DISPLAY=:0 /home/landau/.local/share/Steam/steam.sh -applaunch 250820
```

戴上 Quest，打开 ALVR 20.14.1，通过 WiFi 连接电脑，保持 `Streaming`，握住并抬起两只实体 Touch 手柄，使两只手柄都处于头显可追踪范围内；即使只控制左臂也需要如此。仅显示 `Streaming` 不代表手柄追踪有效，未抬起两只手柄时可能出现 `0/30 valid OpenVR poses`。关闭 Quest 系统菜单，使 ALVR 保持前台。三种模式都会先检查并持续读取双手柄；实体左手柄 Y 是唯一接管／回位键。

## 3. 选择机械臂并启动

确认所选机械臂 Desk 绿色、FCI 开启、工作空间空旷且急停可触及；双臂模式要分别确认左右臂。在新的 client 终端先执行：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
```

然后按目标**只执行一条**：

左臂：

```bash
env -u CYCLONEDDS_URI \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ROS_DOMAIN_ID=0 \
  bash scripts/start_quest3_hardware.sh --left --enable-hardware
```

右臂：

```bash
env -u CYCLONEDDS_URI \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ROS_DOMAIN_ID=0 \
  bash scripts/start_quest3_hardware.sh --right --enable-hardware
```

双臂：

```bash
env -u CYCLONEDDS_URI \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ROS_DOMAIN_ID=0 \
  bash scripts/start_quest3_hardware.sh --duo --enable-hardware
```

脚本先报告左右实体 Touch 均为 `30/30 valid OpenVR poses`，再启动所选机械臂。等待 `Hardware chain ready` 和所选侧的 `LEFT_SOURCE_STATUS`／`RIGHT_SOURCE_STATUS` 显示 `"state":"READY"`；双臂模式必须**两侧都 READY**，此时才按左 Y。

第一次短按左 Y 接管所选机械臂：`--left` 跟随左手柄，`--right` 跟随右手柄，`--duo` 同时跟随两只手柄。第二次短按左 Y 返回本轮各自的接管点；回位完成并松开 Y 后可再次接管。右手柄 B 不触发接管。若双臂中任一侧显示适配器 `STOP`，在启动终端按 `Ctrl+C`，排除故障后重启会话。

## 4. 结束

在真机启动终端按 `Ctrl+C`，等待 `Stopped`；再关闭 FCI、SteamVR 和 ALVR Dashboard。

## 5. 左臂 VR 双机采集验证

完成第 1、2 节的准备后，用下面的 **client 数采入口替代第 3 节的独立遥操命令**；不要同时启动两个真机入口。Quest 数采目前只支持左臂 arm-only。client 负责左 FR3 控制，机器人状态和 accepted target 经有线 ROS 2 DDS 到 server；server 合并两路相机后在本机 `127.0.0.1:5555` 发布 ZMQ 样本，录制文件保存在 server。

### 5.1 client：检查并启动左臂

先确认左臂 Desk 绿色、FCI 开启、工作空间安全，两只实体 Touch 都在 ALVR 中有效追踪。在 client 终端执行：

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
bash scripts/start_data_collection_client.sh --teleop-source quest --left --arm --check-only
bash scripts/start_data_collection_client.sh --teleop-source quest --left --arm --enable-hardware
```

第一个命令不连接机械臂；第二个命令连接左 FR3。等待 `Hardware chain ready` 和左臂 `READY`。这个入口自动加载 client 的有线 DDS 配置，不要加第 3 节的 `env -u CYCLONEDDS_URI`。

### 5.2 server：启动相机和数据桥

在 server（`/home/pair1/real-exp-vr`）的另一个终端执行：

```bash
cd /home/pair1/real-exp-vr
bash scripts/start_data_collection_server.sh \
  --teleop-source quest --left --arm --ros-domain-id 0
```

server 当前的 ROS 采集包已按源码构建。若之后改动了 bridge 或相机源码，先在 server 执行 `bash scripts/build_data_collection_server.sh`。当 server 提示等待左臂 VR 输入时，短按一次左 Y 接管并移动左手柄；server 收到 `/left/vr/accepted_joint_states` 后会显示 `Data-server stack is running`。测试期间保持左臂跟随。

### 5.3 server：检查并录制数据

先在 server 的第二个终端检查 60 秒 ZMQ 样本：

```bash
cd /home/pair1/real-exp-vr
conda activate lerobot-vr
python -m data_collection.diagnostics.samples \
  --host 127.0.0.1 --port 5555 \
  --duration 60 --silence-timeout 5 \
  --output outputs/m2/live-samples.json
```

收到有效样本后，在 server 的另一个终端启动**交互式录制器**：

```bash
cd /home/pair1/real-exp-vr
env -u PYTHONHOME -u PYTHONPATH -u LD_LIBRARY_PATH \
  /home/pair1/miniconda3/envs/lerobot-vr/bin/python \
  -m data_collection.lerobot_collection \
  --host 127.0.0.1 --port 5555 \
  --repo-id local/franka_quest_left_teleop \
  --local-dir ./lerobot_data/quest_left
```

左臂处于 `FOLLOWING` 且录制器收到新样本后，在**录制器终端**输入 `s` 回车开始一个 episode；完成动作后输入 `e` 回车保存，再按第二次左 Y 回位。需要丢弃当前 episode 时输入 `d` 回车；结束录制输入 `q` 回车。先停录制器，再分别在 server 和 client 启动终端按 `Ctrl+C`。

双机真实相机、真实状态和录制结果仍需这次现场验证。左 Y 不会自动开始或结束 episode；目前也没有回位／故障自动切断录制的门控，保存的数据先作为链路验证材料，不直接视为可用训练数据。省略 `--teleop-source` 继续使用原 GELLO 模式。
