#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class RobotStateObservation:
    """
    Robot proprioception observation.

    Used as VLA input:
    joint state + tcp pose
    """

    def __init__(self, robot):
        self.robot = robot

    def get(self):
        state = self.robot.get_state()

        return {
            "joint": state.get("joint"),
            "tcp_pose": state.get("tcp_pose")
        }
