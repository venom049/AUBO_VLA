#!/usr/bin/env python3
"""Simulator integration demo. Use --status for a read-only connection check."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot.demo import run_demo

from control.action_decoder import ActionDecoder
from vla.fake_vla import FakeVLA


def pipeline_demo(robot):
    current_pose = robot.get_tcp_pose()
    action = FakeVLA().predict()
    decoder = ActionDecoder(frame=robot.config.action_frame)
    target_pose = decoder.delta_to_pose(current_pose, action.arm)
    print("Current TCP:", current_pose)
    print("Policy action:", action.vector())
    print("Target TCP:", target_pose)
    robot.move_pose(target_pose)
    print("Motion completed; measured TCP:", robot.get_tcp_pose())


def main():
    return run_demo(pipeline_demo, "FakeVLA action decoding and measured TCP arrival")


if __name__ == "__main__":
    raise SystemExit(main())
