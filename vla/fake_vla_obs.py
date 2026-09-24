#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from control.action import RobotAction


class FakeVLAObservation:

    def predict(self, observation):

        print("Receive observation:")
        print(observation.keys())

        # simulate policy output

        return RobotAction(
            dx=0.01,
            dy=0,
            dz=0,
            drx=0,
            dry=0,
            drz=0,
            gripper=0
        )
