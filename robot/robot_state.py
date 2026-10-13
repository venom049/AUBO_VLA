#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from control.pose import vector6


class RobotState:

    def __init__(self, rpc, robot_name="rob1"):
        self.rpc = rpc
        self.robot_name = robot_name

    def get_joint_positions(self, *, timeout=None):
        method = f"{self.robot_name}.RobotState.getJointPositions"
        return vector6(self.rpc.call(method, timeout=timeout), "joint positions").tolist()

    def get_tcp_pose(self, *, timeout=None):
        method = f"{self.robot_name}.RobotState.getTcpPose"
        return vector6(self.rpc.call(method, timeout=timeout), "TCP pose").tolist()

    def get_all(self):
        return {
            "robot": self.robot_name,
            "joint": self.get_joint_positions(),
            "tcp_pose": self.get_tcp_pose()
        }
