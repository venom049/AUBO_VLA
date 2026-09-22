#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

from .aubo_rpc import AuboRPC
from .robot_state import RobotState


class AuboRobot:

    def __init__(self,
                 robot_name="rob1",
                 rpc_url="http://127.0.0.1:9012/jsonrpc"):

        self.robot_name = robot_name
        self.rpc = AuboRPC(rpc_url)
        self.state = RobotState(self.rpc, robot_name)

    def connect(self):
        names = self.rpc.call("getRobotNames")
        if self.robot_name not in names:
            raise RuntimeError(
                f"{self.robot_name} not found: {names}"
            )
        print("Connected:", self.robot_name)

    def get_state(self):
        return self.state.get_all()

    def power_on(self):
        return self.rpc.call(
            f"{self.robot_name}.RobotControl.powerOn"
        )

    def startup(self):
        return self.rpc.call(
            f"{self.robot_name}.RobotControl.startup"
        )

    def move_joint_deg(self, joints_deg):
        joints_rad = [
            x * math.pi / 180.0
            for x in joints_deg
        ]

        return self.rpc.call(
            f"{self.robot_name}.MotionControl.moveJoint",
            [
                joints_rad,
                50 * math.pi / 180,
                50 * math.pi / 180,
                0,
                0
            ]
        )

    def wait(self, seconds=1):
        time.sleep(seconds)
