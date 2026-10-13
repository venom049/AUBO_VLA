#!/usr/bin/env python3
"""Simulator integration demo. Use --status for a read-only connection check."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot.demo import run_demo

from control.action_decoder import ActionDecoder
from observation.observation import Observation
from vla.fake_vla_obs import FakeVLAObservation


def pipeline_demo(robot):
    observation = Observation(robot).get()
    action = FakeVLAObservation().predict(observation)
    decoder = ActionDecoder(frame=robot.config.action_frame)
    target_pose = decoder.delta_to_pose(robot.get_tcp_pose(), action.arm)
    print("Observation state:", observation["state"])
    print("Policy action:", action.vector())
    print("Target TCP:", target_pose)
    robot.move_pose(target_pose)
    print("Observation pipeline completed; measured TCP:", robot.get_tcp_pose())


def main():
    return run_demo(pipeline_demo, "Placeholder image + robot state + FakeVLA motion demo")


if __name__ == "__main__":
    raise SystemExit(main())
