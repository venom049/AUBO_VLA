# AUBO_VLA_Project

基于 AUBO 的视觉、语言与动作机器人控制研究项目。
当前已建立 AUBO 仿真控制后端和策略接口原型，目标是为后续
UMI 示教数据、学习策略与真实执行器提供部署基础。

当前链路：

```text
占位 RGB 图像 + 机器人状态
            ↓
      FakeVLA 固定策略
            ↓
      RobotAction → ActionDecoder
            ↓
     AuboRobot → JSON-RPC → rob1
            ↓
      状态反馈与测量到位确认
```

## 当前完成程度

- AUBO JSON-RPC 通信、关节与 TCP 状态读取。
- 关节运动、TCP 直线运动、返回码检查、到位确认与超时停止。
- 图像和机器人状态的 Observation 接口；图像仍为全黑占位数组。
- 六维末端增量和夹爪预留量的动作接口；夹爪量尚未执行。
- 两条 FakeVLA 单次运动演示；尚无持续控制循环、真实模型或语言输入。
- 离线回归测试、共享机器人配置与带提交版本的虚拟机源码导出。

## 项目结构

```text
config/robot_sim.json       连接、运动、容差与动作坐标系配置
robot/                     RPC、状态、驱动、配置与演示 CLI
control/                   动作表示、坐标变换和位姿误差
observation/               占位图像与机器人状态输入
vla/                       固定输出的 FakeVLA 策略
tests/test_backend.py      离线自动测试（不联系机器人）
tests/test_motion.py       J1 小幅运动并返回的集成演示
tests/test_vla_pipeline.py FakeVLA 动作集成演示
tests/test_observation_pipeline.py 观测与策略集成演示
scripts/export_vm.py       导出已提交源码及 REVISION.txt
sim_control/               原有独立仿真控制脚本
sdk_test/                  独立 pyaubo_sdk 示例
```

安装、配置、宿主机与虚拟机同步方法见 [ENV_SETUP.md](ENV_SETUP.md)。

## 快速验证

Ubuntu 22.04 / Python 3.10：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements_robot.txt
python -m pip check
python -m unittest discover -s tests -v
```

导入测试模块不会触发运动。三个集成演示均支持 `--status` 只读状态检查
和 `--config` 指定配置文件。使用默认的本地控制器地址时，在 AUBO 虚拟机内运行：

```bash
python tests/test_motion.py --status
# 在 AUBO 界面上电、启动到 Running 后：
python tests/test_motion.py
python tests/test_vla_pipeline.py
python tests/test_observation_pipeline.py
```

## 机器人接口与运动完成条件

```python
from robot.aubo_driver import AuboRobot

robot = AuboRobot()
robot.connect()                       # 只连接，不上电
state = robot.get_state()             # 关节 rad、TCP m/rad
robot.move_joint_deg(joints_deg)      # 默认阻塞，确认关节到位
robot.move_pose(target_pose)          # 默认阻塞，确认 TCP 到位
robot.stop()                         # 根据待执行动作选择 stopJoint / stopLine
```

`move_joint_deg()` / `move_pose()` 保留成功返回 `0` 的接口，但现在默认等待实际到位。
完成条件包括执行 ID 为 `-1`、队列为空、`isSteady` 为真，以及测量误差在容差内。
必须连续满足这些条件，避免将指令入队之前的静止状态当作完成。

使用 `wait=False` 可以仅提交命令，随后必须调用 `wait_until_complete()` 或 `stop()`；
同一个驱动实例有待执行动作时会拒绝下一条动作。控制器非零返回码会抛出错误。
已提交运动的等待发生异常、超时或中断时会尝试停止并确认状态；运动回复丢失时
也会尝试停止，避免重复发送可能已经接受的命令。

## 动作坐标与姿态约定

机器人绝对 TCP 位姿为 `[x, y, z, rx, ry, rz]`：

- 位置属于基坐标系，单位为米。
- 姿态为 AUBO SDK 定义的 ZYX 欧拉角，单位为弧度。
- 旋转矩阵为 `Rz(rz) @ Ry(ry) @ Rx(rx)`。

策略动作是 `[dx, dy, dz, dRx, dRy, dRz, gripper]`。
解码器只接收前六维；`gripper` 当前仅预留，不代表夹爪已实现。
旋转增量是同一约定下的相对旋转，不能将欧拉角直接逐项相加。

```python
from control.action_decoder import ActionDecoder

base_decoder = ActionDecoder(frame="base")  # 默认，与原有纯平移演示一致
# p_target = p_current + dp
# R_target = R_delta @ R_current

tool_decoder = ActionDecoder(frame="tool")
# p_target = p_current + R_current @ dp
# R_target = R_current @ R_delta
```

这是旧版逐项相加姿态语义的修正。接入策略或示教数据时，应明确训练动作的坐标系
与旋转表示；旋转向量、四元数或角度单位的数据需要先转换。
姿态接近欧拉角奇异点时 SciPy 会提示并返回等价表示，到位判断使用旋转距离，
不会直接比较欧拉角数值。所有关节、位姿和增量输入必须是六个有限数值。

## 下一阶段

1. 在虚拟机验证当前运动、停止和异常路径，记录提交版本与配置。
2. 实现基于最新状态的持续反馈控制、任务停止条件与运行日志。
3. 接入有效视觉场景，建立同步示教记录、回放与评估数据。
4. 训练 ACT 或 Diffusion Policy 基线；UMI 主线需处理相对轨迹与延迟对齐。
5. 增加语言条件策略、实际夹爪或软体执行器，再推进真实 AUBO 部署。
