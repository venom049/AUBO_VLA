# 环境配置与虚拟机同步

当前开发基线：Ubuntu 22.04、Python 3.10、AUBO Sim 0.32.1。
机器人后端通过 HTTP JSON-RPC 控制虚拟机中的 `rob1`。

## 安装与验证

在宿主机和虚拟机各自的项目目录中配置环境。不要直接复制宿主机的 `.venv`。

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements_robot.txt
python -m pip check
python -m unittest discover -s tests -v
```

`requirements_robot.txt` 固定了 Python 3.10 可用的 NumPy、SciPy、OpenCV 和 Pillow 版本。
SciPy 用于旋转组合与到位误差计算；JSON-RPC 使用 Python 标准库。
`sdk_test/test_aubo_control.py` 是独立的 SDK 示例，需另行安装与控制器兼容的
`pyaubo_sdk`；它不属于当前 JSON-RPC 后端的依赖或离线测试入口。

如果当前终端已经加载 ROS，`PYTHONPATH` 可能把系统 ROS 包带入虚拟环境，
导致 `pip check` 报告与本项目无关的缺失依赖。可仅对当前命令清除它：

```bash
env -u PYTHONPATH .venv/bin/python -m pip check
env -u PYTHONPATH .venv/bin/python -m unittest discover -s tests -v
```

## 共享配置

默认配置位于 `config/robot_sim.json`，从代码文件的位置解析，运行目录不影响加载。
支持 `--config /absolute/path/robot.json` 或 `AUBO_CONFIG` 选择配置；
`AUBO_RPC_URL`、`AUBO_ROBOT_NAME` 可覆盖文件里的连接地址和机器人名称。

```bash
python tests/test_motion.py --config config/robot_sim.json --status
```

关键默认值：

| 设置 | 默认值 | 含义 |
|---|---|---|
| `rpc_url` | `http://127.0.0.1:9012/jsonrpc` | 当前机器的控制服务 |
| `speed_fraction` | 0.1 | 全局速度比例 |
| `motion_timeout` | 30 s | 等待到位的期限 |
| `rpc_timeout` | 5 s | 单次通信期限；等待时限制为剩余总时间 |
| `stop_timeout` | 5 s | 停止命令后的状态确认期限 |
| `joint_tolerance_rad` | 0.005 rad | 最大单关节误差 |
| `position_tolerance_m` | 0.001 m | TCP 位置的欧氏距离误差 |
| `orientation_tolerance_rad` | 0.01 rad | 两个旋转之间的最短角距离 |
| `settled_samples` | 2 | 连续满足到位条件的采样数 |
| `action_frame` | `base` | 策略增量使用的坐标系，可选 `tool` |

默认地址的 `127.0.0.1` 指运行 Python 的机器。因此使用默认配置时，
运动脚本应在 AUBO 虚拟机里运行。若配置了可访问的控制器地址或端口转发，
也可从宿主机连接；仅修改地址并不能建立虚拟机网络连接。

## 虚拟机中的验证顺序

先启动 AUBO Sim，再执行只读检查：

```bash
source .venv/bin/activate
python tests/test_motion.py --status
```

在 AUBO 界面将机器人上电、启动到 `Running` 后，执行运动验证：

```bash
python tests/test_motion.py
python tests/test_vla_pipeline.py
python tests/test_observation_pipeline.py
```

第一个脚本将 J1 在当前位置增加 5°，确认到位后返回起始关节位置。
后两个脚本分别执行 FakeVLA 的 2 cm 和 1 cm 相对平移。
它们仍是单次集成演示，图像为占位数据，策略为固定动作。

驱动不会自动上电或解除异常状态。状态不满足条件、控制器拒绝指令、
通信失败、运动超时均会使脚本以非零状态退出。
只有运动队列为空、机器人静止、测量值在容差内，并连续满足配置的采样数，
才报告运动完成。没有 API 时会明确失败，不会用固定 sleep 代替状态确认。

驱动接受运动后，等待期间的异常或 Ctrl+C 会尝试 `stopJoint` / `stopLine`，
并确认机器人静止且运动队列为空。停止也失败时，错误会包含两个失败原因；
软件停止依赖控制器通信，不能替代控制器自身的停止功能。

## 宿主机与虚拟机保持同一版本

可联网时，在虚拟机中使用 Git，并确保运行前工作区没有未提交修改：

```bash
git clone --branch main https://github.com/venom049/AUBO_VLA.git AUBO_VLA_Project
cd AUBO_VLA_Project
git pull --ff-only
git rev-parse HEAD
git status --short
```

无法联网时，在宿主机的干净 Git 工作区导出完整已提交源码：

```bash
python3 scripts/export_vm.py /tmp/AUBO_VLA_Project.zip
python3 -m http.server 8000 --bind 0.0.0.0 --directory /tmp
```

在使用现有 QEMU NAT 网络的虚拟机内下载，解压到新的目录：

```bash
mkdir -p /root/aubo_runs
cd /root/aubo_runs
wget -O AUBO_VLA_Project.zip http://10.0.2.2:8000/AUBO_VLA_Project.zip
unzip AUBO_VLA_Project.zip -d run_latest
cd run_latest/AUBO_VLA_Project
cat REVISION.txt
```

随后在解压后的项目里按本文建立 `.venv`。导出包不包含虚拟环境、
Git 数据库和虚拟机磁盘；它自带 `REVISION.txt`，导出时若源码有未提交修改会拒绝。
三个演示脚本都会打印版本和实际配置，便于与宿主机提交核对。
导出包中的版本标记代表导出时的提交；若在虚拟机直接修改文件，应重新导出或使用 Git。
