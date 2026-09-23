# AUBO_VLA_Project

## 项目简介

本项目用于搭建基于AUBO机械臂的机器人学习部署框架。

当前阶段目标：

实现：

策略模型 ↓ 动作表示 ↓ 动作解析 ↓ AUBO执行

当前使用：

-   AUBO Sim虚拟机器人
-   JSON-RPC控制接口
-   Python机器人控制框架

------------------------------------------------------------------------

## 项目结构

    AUBO_VLA_Project/

    ├── robot/
    │   ├── aubo_rpc.py
    │   ├── robot_state.py
    │   └── aubo_driver.py
    │
    ├── control/
    │   ├── action.py
    │   └── action_decoder.py
    │
    ├── vla/
    │   └── fake_vla.py
    │
    ├── tests/
    │   ├── test_motion.py
    │   └── test_vla_pipeline.py
    │
    ├── sim_control/
    │   └── aubo_sim_control.py
    │
    └── .venv/

------------------------------------------------------------------------

## 环境要求

-   Ubuntu 22.04
-   Python 3.10
-   AUBO Sim
-   AUBO JSON-RPC接口

------------------------------------------------------------------------

## 环境启动

进入项目：

``` bash
cd ~/AUBO_VLA_Project
```

激活虚拟环境：

``` bash
source .venv/bin/activate
```

------------------------------------------------------------------------

## 测试机器人连接

启动AUBO Sim后：

``` bash
python3 tests/test_motion.py
```

预期输出：

    Connected: rob1

------------------------------------------------------------------------

## 测试VLA闭环

运行：

``` bash
python3 tests/test_vla_pipeline.py
```

执行流程：

    FakeVLA
       |
    RobotAction
       |
    ActionDecoder
       |
    AuboDriver
       |
    rob1

------------------------------------------------------------------------

## 模块说明

### robot

机器人底层控制模块。

负责：

-   JSON-RPC通信
-   状态读取
-   运动控制

### control

动作接口模块。

负责：

-   VLA动作格式定义
-   动作转换

### vla

策略模型接口。

当前：

FakeVLA

未来替换：

-   ACT
-   Diffusion Policy
-   OpenVLA

------------------------------------------------------------------------

## 后续开发路线

### Stage 2

建立Observation模块：

    Camera
    +
    Robot State

    ↓

    VLA

### Stage 3

加入UMI多模态示教数据：

    UMI设备

    ↓

    Dataset

    ↓

    Policy Training

### Stage 4

真实机器人部署：

    VLA模型

    ↓

    AUBO机械臂

    ↓

    任务执行
