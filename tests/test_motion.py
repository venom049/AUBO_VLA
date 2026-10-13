#!/usr/bin/env python3
"""Simulator integration demo. Use --status for a read-only connection check."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot.demo import run_demo


def motion_demo(robot):
    initial = robot.get_state()["joint"]
    target = [math.degrees(j) for j in initial]
    target[0] += 5.0
    print("Current state:", robot.get_state())
    print("Move J1 +5 degrees, then return")
    robot.move_joint_deg(target)
    print("Measured joints:", robot.get_state()["joint"])
    robot.move_joint_deg([math.degrees(j) for j in initial])
    print("Finished; measured return joints:", robot.get_state()["joint"])


def main():
    return run_demo(motion_demo, "AUBO joint motion and measured return test")


if __name__ == "__main__":
    raise SystemExit(main())
