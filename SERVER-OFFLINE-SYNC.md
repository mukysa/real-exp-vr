# Server 无法联网时的代码同步

更新日期：2026-10-26。

本文记录已验证的 Git bundle 同步方式：client 导出已提交代码，通过局域网传给 server，再由 server 从本地文件导入。server 不需要连接 GitHub；保留 Git 历史、提交号和现有数据集。

## 1. 机器与仓库

| 项目 | 地址或路径 |
|---|---|
| client 工作目录 | `/home/landau/real-exp-vr` |
| server SSH 地址 | `pair1@192.168.52.160` |
| server 工作目录 | `/home/pair1/real-exp-vr` |
| 主仓库 GitHub | `https://github.com/mukysa/real-exp-vr.git` |
| gello_software GitHub | `https://github.com/mukysa/gello_software.git` |
| client 同步分支 | `quest-left-collection` |
| server 工作分支 | `server/quest-left` |

**当前 server IP 是 `192.168.52.160`，不要沿用旧地址 `192.168.50.13`。** 本文直接使用 IP，避免本机 `ssh server` 别名仍指向旧地址。若要使用别名，先检查并更新 `~/.ssh/config`。

主仓库与 `gello_software` 是两个独立 Git 仓库。主仓库仅记录子模块提交号，所以两者需要分别打包。主仓库 GitHub 地址保留不变，离线导入无需重设 origin。

## 2. Client：保存提交并打包

先停止正在修改源码的任务，检查版本和工作区：

```bash
cd /home/landau/real-exp-vr

git status -sb
git log -1 --oneline quest-left-collection

git -C gello_software status -sb
git -C gello_software log -1 --oneline quest-left-collection

git ls-tree quest-left-collection gello_software
```

打包前必须先提交需要同步的修改：先提交子模块，再在主仓库 `git add gello_software` 并提交其版本指针及相关代码。确认上面 `git ls-tree` 显示的子模块提交包含在要导出的子模块分支历史中。若使用其他分支，请相应替换本文命令中的分支名。

**Bundle 只包含已提交内容，不包含未提交修改、未跟踪文件、数据集或本机构建环境。** 不要为打包而直接执行 `git add .`；应选择需要同步的代码文件。

生成带完整分支历史的 bundle：

```bash
git bundle create /tmp/real-exp-vr-quest.bundle \
  quest-left-collection

git -C gello_software bundle create /tmp/gello-quest.bundle \
  quest-left-collection

git bundle verify /tmp/real-exp-vr-quest.bundle
git -C gello_software bundle verify /tmp/gello-quest.bundle
```

通过局域网传给 server：

```bash
scp /tmp/real-exp-vr-quest.bundle /tmp/gello-quest.bundle \
  pair1@192.168.52.160:/home/pair1/
```

这一步要求 client 能通过 SSH 连接 server，但不要求 server 能访问互联网。上传 GitHub 可以单独完成，不是 bundle 同步的前提。

## 3. Server：保存本地改动

登录并检查：

```bash
ssh pair1@192.168.52.160
cd /home/pair1/real-exp-vr

git status -sb
git -C gello_software status -sb
```

首次迁移时，server 原有改动是 `data_collection/DATA_COLLECTION_README.md`，未跟踪内容为 `.claude/` 和 `dataset/`。README 已经备份并 stash；后续同步无需重复处理，除非再次修改。

如这份 README 仍有未提交改动，可执行：

```bash
cp data_collection/DATA_COLLECTION_README.md \
  "$HOME/DATA_COLLECTION_README.before-quest-$(date +%Y%m%d-%H%M%S).md"

git stash push -m "Server README before Quest update" \
  -- data_collection/DATA_COLLECTION_README.md
```

如有其他本地代码改动，应先分别备份或提交，尤其是子模块内的修改；主仓库 stash 不会自动保存子模块工作区的文件修改。

保留当前主仓库版本位置：

```bash
git branch "backup/server-before-quest-$(date +%Y%m%d-%H%M%S)"
```

该备份分支只保存提交位置，不是数据备份。需要备份数据时，另行复制或归档 `dataset/`。不要执行 `git clean`、强制 checkout 或 reset 来消除本地文件。

## 4. Server：导入主仓库

验证文件：

```bash
git bundle verify ~/real-exp-vr-quest.bundle
git -C gello_software bundle verify ~/gello-quest.bundle
```

导入主仓库，**显式关闭递归子模块获取**：

```bash
git fetch --no-recurse-submodules ~/real-exp-vr-quest.bundle \
  refs/heads/quest-left-collection
```

首次同步、尚不存在 `server/quest-left` 分支时：

```bash
git -c submodule.recurse=false switch -c server/quest-left FETCH_HEAD
```

后续同步、该分支已存在时，改用下面两条，勿重复创建分支：

```bash
git -c submodule.recurse=false switch server/quest-left
git -c submodule.recurse=false merge --ff-only FETCH_HEAD
```

执行上述切换或合并前，不要在主仓库额外执行其他 fetch，以免改变 `FETCH_HEAD`。若出现本地文件会被覆盖或无法快进的提示，停止并检查差异，不要强制覆盖。

## 5. Server：导入并检出子模块

```bash
git -C gello_software fetch --no-recurse-submodules \
  ~/gello-quest.bundle \
  refs/heads/quest-left-collection

git submodule sync -- gello_software

git -c submodule.recurse=false submodule update \
  --init --no-fetch -- gello_software
```

此流程以 server 已有、已初始化的 `gello_software` 仓库为前提；本次迁移满足该条件。全新机器若没有该子模块目录，需要先从 bundle 建立仓库，不能直接假定 `--init --no-fetch` 可以完成离线克隆。

- `fetch --no-recurse-submodules`：只导入当前 bundle，避免尝试联网获取其他子模块。
- `submodule sync`：把新 `.gitmodules` 中的地址同步到本地配置，本身不联网。
- `submodule update --no-fetch`：用已导入的本地对象检出主仓库记录的确切提交，不联网下载。
- 不需要进入子模块执行 `git pull` 或切换到分支；`HEAD (no branch)` 是正常状态。

如果提示缺少目标提交，检查主仓库记录的子模块版本与 bundle 是否匹配；不要删除 `--no-fetch` 来盲目重试。

## 6. 同步后检查

```bash
git status -sb
git log -1 --oneline

git -C gello_software status -sb
git -C gello_software log -1 --oneline

git submodule status -- gello_software lerobot

ls -l config/data_collection/quest_left.yaml
bash scripts/start_data_collection_server.sh --help
```

2026-10-26 首次成功迁移的检查点：

| 项目 | 结果 |
|---|---|
| 主仓库 | `2acc5a15d1ae74d7bd75f887a24cff30543f6dc2` |
| server 分支 | `server/quest-left` |
| gello_software | `135d3bf32ea275f8bd257dcf0e7eef2384991fc6` |
| lerobot | `4eecbad32b0380c64bd5574bbf74e56e5255bdea` |
| Quest 左臂配置 | `config/data_collection/quest_left.yaml` 存在 |
| server help | 包含 `--teleop-source` |

上述提交号是历史检查点，后续同步应以 client 本次导出的版本为准，不要强制回退到这些提交。

`git submodule status` 前导空格表示与主仓库记录一致；`+` 表示版本不同，`-` 表示尚未初始化。若 `lerobot` 或其他依赖也发生版本变化，需为对应仓库单独制作 bundle 并导入，不能假定只同步 gello_software 就足够。

主仓库仅显示 `?? .claude/`、`?? dataset/` 时，说明原有未跟踪文件仍保留；这不影响已提交代码同步。同步不会自动恢复之前 stash 的 README，恢复前应先比较新旧内容。

## 7. 常见问题

### fetch bundle 后出现 GitHub publickey 错误

现象：主仓库对象已接收，随后输出 `Fetching submodule gello_software` 和 `Permission denied (publickey)`。

原因：Git 自动递归获取子模块。主仓库对象可能已经成功导入，不表示 bundle 损坏，也不必为离线同步创建 SSH Key。重新按第 4、5 节使用 `--no-recurse-submodules` 导入，再使用 `--no-fetch` 更新子模块即可。

### GitHub 超时，或 localhost:7890 连接被拒绝

- `timeout` 返回 `124`：命令超过等待时间，不能据此认定远端分支不存在。
- GitHub 443 连接超时：需处理网络，Token 无法修复网络连接。
- `127.0.0.1:7890` 被拒绝：server 该端口没有代理；client 的本地代理不会自动成为 server 的代理。

使用本文 bundle 流程可以绕开 server 到 GitHub 的连接要求。

### 终端出现中文说明的 command not found

只复制 Markdown 代码块里的命令，不要复制正文说明。如果仅正文被当作命令执行失败，应检查后续命令的实际结果，不要因此重复执行破坏性操作。

## 8. 同步完成后再构建与验证

源码同步不等于安装产物更新，也不等于 M2 或真实采集验收通过。应在 server 本机重新构建，不要复制 client 的 `build/`、`install/` 或 Conda 环境覆盖 server。

完成环境核对后，server 构建命令为：

```bash
cd /home/pair1/real-exp-vr
bash scripts/build_data_collection_server.sh
```

M2 联调时，client 需要运行左臂 VR 控制；server 启动 bridge，暂不录制。**现有入口默认 bridge IP 仍可能是旧地址，必须显式指定当前地址：**

```bash
bash scripts/start_data_collection_server.sh \
  --teleop-source quest --left --arm \
  --bridge-host 192.168.52.160 \
  --ros-domain-id 0
```

`--bridge-host` 设置 ZMQ 绑定地址，不负责配置 DDS 网卡或发现机制；跨机 ROS 消息接收仍需独立验证。后续单独启动 recorder 时，也应使用 `--host 192.168.52.160`；由 server 入口加 `--record` 启动时会自动传递其 bridge 地址。正式录制前仍需完成 M2 和录制边界处理。
