#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AUBO SDK First Test
Function:
1. Connect to AUBO simulator
2. Login
3. Read robot state
4. Execute a joint motion
5. Disconnect

User: venom
"""


import time
import math


# AUBO SDK Python library path


import pyaubo_sdk


ROBOT_IP = "127.0.0.1"
ROBOT_PORT = 30004

USERNAME = "aubo"
PASSWORD = "123456"


def wait_arrival(robot_interface):
    """
    Wait until robot finishes current motion
    """
    while robot_interface.getMotionControl().getExecId() != -1:
        time.sleep(0.05)


def main():

    print("====== AUBO Test Start ======")

    rpc_client = pyaubo_sdk.RpcClient()

    rpc_client.setRequestTimeout(1000)

    print("Connecting...")

    rpc_client.connect(
        ROBOT_IP,
        ROBOT_PORT
    )

    if not rpc_client.hasConnected():
        print("RPC connection failed")
        return

    print("RPC connected")


    rpc_client.login(
        USERNAME,
        PASSWORD
    )

    if not rpc_client.hasLogined():
        print("Login failed")
        return

    print("Login successful")


    robot_name = rpc_client.getRobotNames()[0]

    print("Robot name:", robot_name)


    robot_interface = rpc_client.getRobotInterface(
        robot_name
    )


    # Read current state

    print("\nCurrent TCP pose:")
    pose = robot_interface.getRobotState().getTcpPose()
    print(pose)


    print("\nCurrent joint positions:")
    joint = robot_interface.getRobotState().getJointPositions()
    print(joint)


    # Motion test

    print("\nMoving robot...")

    motion = robot_interface.getMotionControl()

    motion.setSpeedFraction(0.2)


    target_joint = [
        0,
        -15 * math.pi / 180,
        80 * math.pi / 180,
        0,
        90 * math.pi / 180,
        0
    ]


    motion.moveJoint(
        target_joint,
        50 * math.pi / 180,
        50 * math.pi / 180,
        0,
        0
    )


    wait_arrival(robot_interface)


    print("Motion finished")


    rpc_client.logout()
    rpc_client.disconnect()


    print("====== Finished ======")


if __name__ == "__main__":
    main()
