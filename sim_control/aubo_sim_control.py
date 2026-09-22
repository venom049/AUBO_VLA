#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AUBO Sim local controller (JSON-RPC over HTTP)

This script is intentionally locked to:
    http://127.0.0.1:9012/jsonrpc

It is meant to control the AUBO simulator running INSIDE the same virtual
machine, not a physical robot on the network.

Examples:
    python3 aubo_sim_control.py status
    python3 aubo_sim_control.py demo
    python3 aubo_sim_control.py move 0 -15 100 25 90 0

Joint angles entered on the command line are in DEGREES.
"""

import json
import math
import sys
import time
import urllib.error
import urllib.request

RPC_URL = "http://127.0.0.1:9012/jsonrpc"
SPEED_FRACTION = 0.10     # 10% global speed
MOVE_ACCEL = 0.30         # rad/s^2
MOVE_VELOCITY = 0.30      # rad/s
WAIT_TIMEOUT = 30.0       # seconds

_rpc_id = 0


class AuboError(RuntimeError):
    pass


def rpc(method, params=None, timeout=5.0):
    """Call one AUBO JSON-RPC method."""
    global _rpc_id
    _rpc_id += 1

    payload = {
        "jsonrpc": "2.0",
        "method": method,
        "params": [] if params is None else params,
        "id": _rpc_id,
    }

    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        RPC_URL,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    try:
        with urllib.request.urlopen(req, timeout=timeout) as response:
            raw = response.read().decode("utf-8")
    except urllib.error.URLError as exc:
        raise AuboError(
            "Cannot connect to AUBO Sim at 127.0.0.1:9012.\n"
            "Make sure AUBO Sim/aubo_control is running in this virtual machine.\n"
            f"Original error: {exc}"
        ) from exc

    try:
        result = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise AuboError(f"Invalid JSON response from controller: {raw!r}") from exc

    if "error" in result:
        raise AuboError(f"RPC error from {method}: {result['error']}")

    return result.get("result")


def command(method, params=None):
    """Run a command whose normal successful return value is 0."""
    result = rpc(method, params)
    if result != 0:
        raise AuboError(f"{method} failed, return value: {result}")
    return result


def get_robot():
    names = rpc("getRobotNames")
    if not names:
        raise AuboError("No robot was found by getRobotNames().")
    return names[0]


def get_mode(robot):
    return rpc(f"{robot}.RobotState.getRobotModeType")


def get_joints(robot):
    q = rpc(f"{robot}.RobotState.getJointPositions")
    if not isinstance(q, list) or len(q) != 6:
        raise AuboError(f"Unexpected joint position data: {q!r}")
    return [float(x) for x in q]


def fmt_deg(q_rad):
    return "[" + ", ".join(f"{math.degrees(x):.2f}" for x in q_rad) + "] deg"


def wait_for_mode(robot, accepted, timeout=WAIT_TIMEOUT):
    start = time.time()
    while time.time() - start < timeout:
        mode = get_mode(robot)
        if mode in accepted:
            return mode
        time.sleep(0.25)
    raise AuboError(
        f"Timed out waiting for robot mode {sorted(accepted)}. "
        f"Current mode: {get_mode(robot)}"
    )


def ensure_running(robot):
    """Power on and release brakes if needed."""
    mode = get_mode(robot)
    print(f"Robot mode: {mode}")

    if mode == "PowerOff":
        print("Powering on...")
        command(f"{robot}.RobotManage.poweron")
        mode = wait_for_mode(robot, {"Idle", "PowerOn", "Running"})

    if mode in {"Booting", "PowerOn"}:
        mode = wait_for_mode(robot, {"Idle", "Running"})

    if mode == "Idle":
        print("Starting robot / releasing brakes...")
        command(f"{robot}.RobotManage.startup")
        mode = wait_for_mode(robot, {"Running"})

    if mode == "BrakeReleasing":
        mode = wait_for_mode(robot, {"Running"})

    if mode != "Running":
        raise AuboError(
            f"Robot is not ready for motion. Current mode: {mode}"
        )

    print("Robot is Running.")


def optional_bool(robot, method):
    """Read an optional boolean state. Return None if the version lacks it."""
    try:
        value = rpc(f"{robot}.RobotState.{method}")
        return bool(value)
    except Exception:
        return None


def wait_until_steady(robot, timeout=WAIT_TIMEOUT):
    start = time.time()
    while time.time() - start < timeout:
        try:
            if rpc(f"{robot}.RobotState.isSteady"):
                return
        except Exception:
            # If this firmware does not expose isSteady, fall back to a short wait.
            time.sleep(2.0)
            return
        time.sleep(0.10)
    raise AuboError("Timed out waiting for the robot to stop.")


def move_joint_rad(robot, target):
    if len(target) != 6:
        raise AuboError("Exactly 6 joint angles are required.")

    # Refuse motion if the controller explicitly reports a safety-limit violation.
    within_limits = optional_bool(robot, "isWithinSafetyLimits")
    if within_limits is False:
        raise AuboError("Controller reports the robot is outside safety limits; motion aborted.")

    collision = optional_bool(robot, "isCollisionOccurred")
    if collision is True:
        raise AuboError("Controller reports a collision state; motion aborted.")

    command(f"{robot}.MotionControl.setSpeedFraction", [SPEED_FRACTION])

    print(f"Target joints: {fmt_deg(target)}")
    command(
        f"{robot}.MotionControl.moveJoint",
        [target, MOVE_ACCEL, MOVE_VELOCITY, 0, 0],
    )

    wait_until_steady(robot)
    final_q = get_joints(robot)
    print(f"Current joints: {fmt_deg(final_q)}")


def move_joint_deg(robot, target_deg):
    target_rad = [math.radians(float(x)) for x in target_deg]
    move_joint_rad(robot, target_rad)


def show_status(robot):
    mode = get_mode(robot)
    q = get_joints(robot)
    print(f"RPC URL: {RPC_URL}")
    print(f"Robot: {robot}")
    print(f"Mode: {mode}")
    print(f"Joint positions: {fmt_deg(q)}")

    steady = optional_bool(robot, "isSteady")
    if steady is not None:
        print(f"Steady: {steady}")

    within_limits = optional_bool(robot, "isWithinSafetyLimits")
    if within_limits is not None:
        print(f"Within safety limits: {within_limits}")

    collision = optional_bool(robot, "isCollisionOccurred")
    if collision is not None:
        print(f"Collision detected: {collision}")


def run_demo(robot):
    ensure_running(robot)
    q0 = get_joints(robot)

    print(f"Start joints: {fmt_deg(q0)}")
    print("Demo: J1 +5 degrees, then return to the starting position.")

    q1 = q0.copy()
    q1[0] += math.radians(5.0)

    move_joint_rad(robot, q1)
    time.sleep(0.5)
    move_joint_rad(robot, q0)

    print("Demo finished.")


def usage():
    print(
        """
AUBO Sim local controller

Usage:
  python3 aubo_sim_control.py status

      Connect only; print robot mode and joint positions.
      This command does NOT move the robot.

  python3 aubo_sim_control.py demo

      Prepare the simulated robot if necessary, move J1 by +5 degrees,
      then return to the starting position.

  python3 aubo_sim_control.py move J1 J2 J3 J4 J5 J6

      Move to six absolute joint angles. Angles are in DEGREES.

Example:
  python3 aubo_sim_control.py move 0 -15 100 25 90 0

Important:
  The controller address is hard-coded to 127.0.0.1 so this script is
  intended for the AUBO simulator in the same virtual machine.
"""
    )


def main():
    if len(sys.argv) < 2 or sys.argv[1] in {"-h", "--help", "help"}:
        usage()
        return 0

    action = sys.argv[1].lower()
    robot = get_robot()

    if action == "status":
        show_status(robot)
        return 0

    if action == "demo":
        run_demo(robot)
        return 0

    if action == "move":
        if len(sys.argv) != 8:
            raise AuboError(
                "move requires exactly 6 joint angles.\n"
                "Example: python3 aubo_sim_control.py move 0 -15 100 25 90 0"
            )
        target = [float(x) for x in sys.argv[2:8]]
        ensure_running(robot)
        move_joint_deg(robot, target)
        return 0

    raise AuboError(f"Unknown command: {action!r}. Run with --help for usage.")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nInterrupted.")
        sys.exit(130)
    except Exception as exc:
        print("\nERROR:")
        print(exc)
        sys.exit(1)
