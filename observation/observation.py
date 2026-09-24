#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from .camera import Camera
from .robot_state_obs import RobotStateObservation


class Observation:

    def __init__(self, robot):

        self.camera = Camera()
        self.robot_state = RobotStateObservation(robot)


    def get(self):

        return {
            "image": self.camera.get_image(),
            "state": self.robot_state.get()
        }
