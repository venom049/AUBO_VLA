#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
sys.path.append(".")

from robot.aubo_driver import AuboRobot
from observation.observation import Observation
from vla.fake_vla_obs import FakeVLAObservation
from control.action_decoder import ActionDecoder


robot = AuboRobot()

robot.connect()


# 1. get observation

obs = Observation(robot)

observation = obs.get()

print("Observation:")
print(observation["state"])


# 2. policy inference

policy = FakeVLAObservation()

action = policy.predict(observation)


print("Action:")
print(action.vector())


# 3. action decode

decoder = ActionDecoder()

current_pose = robot.get_tcp_pose()

target_pose = decoder.delta_to_pose(
    current_pose,
    action.arm
)


print("Target pose:")
print(target_pose)


# 4. execute

robot.move_pose(target_pose)

print("Observation pipeline finished")
