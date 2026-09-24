# AUBO_VLA_Project

## 项目简介

本项目面向基于AUBO机械臂的视觉-语言-动作（VLA）机器人控制研究。

当前阶段主要目标是在缺少真实UMI设备和软体执行器硬件的情况下，先完成：

    Observation
        ↓
    Policy
        ↓
    Action
        ↓
    Robot Controller
        ↓
    AUBO Robot

的软件框架搭建，为后续接入UMI多模态示教数据和真实VLA模型提供基础。

当前运行环境：

-   Ubuntu 22.04
-   Python 3.10
-   AUBO Sim
-   AUBO JSON-RPC控制接口

------------------------------------------------------------------------

# 项目结构

    AUBO_VLA_Project/

    ├── robot/
    │   ├── aubo_rpc.py
    │   ├── robot_state.py
    │   └── aubo_driver.py
    │
    ├── observation/
    │   ├── camera.py
    │   ├── robot_state_obs.py
    │   └── observation.py
    │
    ├── control/
    │   ├── action.py
    │   └── action_decoder.py
    │
    ├── vla/
    │   ├── fake_vla.py
    │   └── fake_vla_obs.py
    │
    ├── tests/
    │   ├── test_motion.py
    │   ├── test_vla_pipeline.py
    │   └── test_observation_pipeline.py
    │
    ├── sim_control/
    │   └── aubo_sim_control.py
    │
    ├── requirements_robot.txt
    └── ENV_SETUP.md

------------------------------------------------------------------------

# 模块说明

## robot

机器人底层控制模块。

主要功能：

-   JSON-RPC通信
-   AUBO机器人连接
-   机器人状态读取
-   关节运动控制
-   TCP位姿控制

核心接口：

``` python
AuboRobot()
```

提供：

``` python
connect()

get_state()

get_tcp_pose()

move_joint_deg()

move_pose()
```

------------------------------------------------------------------------

## observation

观测输入模块。

负责构建VLA输入。

当前包含：

### Camera

当前：

-   模拟RGB图像输入

未来：

-   RealSense
-   GoPro
-   工业相机

### Robot State

提供：

-   Joint state
-   TCP pose

输出格式：

``` python
{
    "image": image,
    "state": {
        "joint": [],
        "tcp_pose": []
    }
}
```

------------------------------------------------------------------------

## control

动作表示和转换模块。

统一动作格式：

    [dx,dy,dz,dRx,dRy,dRz,gripper]

其中：

-   前6维：机械臂末端动作
-   第7维：执行器控制量

Action Decoder负责：

    相对动作

    ↓

    目标TCP位姿

------------------------------------------------------------------------

## vla

策略模型接口。

当前：

-   FakeVLA

用于验证：

    Observation

    ↓

    Policy

    ↓

    Action

未来替换：

-   ACT
-   Diffusion Policy
-   OpenVLA

------------------------------------------------------------------------

# 当前测试方式

进入项目：

``` bash
cd ~/AUBO_VLA_Project
```

激活环境：

``` bash
source .venv/bin/activate
```

------------------------------------------------------------------------

## 测试机器人控制

``` bash
python3 tests/test_motion.py
```

验证：

Python → AUBO → rob1

------------------------------------------------------------------------

## 测试VLA闭环

``` bash
python3 tests/test_vla_pipeline.py
```

流程：

    FakeVLA
     ↓
    Action
     ↓
    Decoder
     ↓
    AuboDriver
     ↓
    Robot

------------------------------------------------------------------------

## 测试Observation闭环

``` bash
python3 tests/test_observation_pipeline.py
```

流程：

    Camera
    +
    Robot State

    ↓

    Observation

    ↓

    FakeVLA

    ↓

    AUBO

------------------------------------------------------------------------

# 后续开发方向

## Stage 1（已完成）

AUBO机器人后端：

-   控制接口
-   状态读取
-   TCP运动控制

## Stage 2（已完成）

Observation框架：

-   图像输入接口
-   Robot State输入接口

## Stage 3（下一步）

真实策略模型接入：

优先：

1.  ACT
2.  Diffusion Policy

最终：

3.  OpenVLA

## Stage 4

加入UMI多模态示教数据：

数据包括：

-   RGB图像
-   机器人状态
-   末端位姿
-   执行器状态
-   触觉信息

## Stage 5

真实AUBO部署：

    UMI Dataset

    ↓

    VLA Policy

    ↓

    AUBO Robot

    ↓

    Real Task
