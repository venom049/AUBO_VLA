#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
sys.path.append(".")

from robot.aubo_driver import AuboRobot


robot = AuboRobot()

robot.connect()

print("Current state:")
print(robot.get_state())

print("Move test...")
robot.move_joint_deg(
    [5, -14.61, 100, 25, 90, 0]
)

robot.wait(3)

print("Finished")
