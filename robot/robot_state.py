#!/usr/bin/env python3
# -*- coding: utf-8 -*-


class RobotState:

    def __init__(self, rpc, robot_name="rob1"):
        self.rpc = rpc
        self.robot_name = robot_name

    def get_joint_positions(self):
        method = f"{self.robot_name}.RobotState.getJointPositions"
        return self.rpc.call(method)

    def get_tcp_pose(self):
        method = f"{self.robot_name}.RobotState.getTcpPose"
        return self.rpc.call(method)

    def get_all(self):
        return {
            "robot": self.robot_name,
            "joint": self.get_joint_positions(),
            "tcp_pose": self.get_tcp_pose()
        }
