"""Blocking AUBO motions with controller status and measured arrival checks."""

import math
import time
from dataclasses import replace

from control.pose import pose_error, vector6
from .aubo_rpc import AuboRPC
from .config import RobotConfig
from .robot_state import RobotState


class MotionError(RuntimeError):
    pass


class AuboRobot:
    def __init__(self, robot_name=None, rpc_url=None, *, config=None):
        config = config or RobotConfig.from_file()
        overrides = {}
        if robot_name is not None:
            overrides["robot_name"] = robot_name
        if rpc_url is not None:
            overrides["rpc_url"] = rpc_url
        self.config = replace(config, **overrides)
        self.robot_name = self.config.robot_name
        self.rpc = AuboRPC(self.config.rpc_url, self.config.rpc_timeout)
        self.state = RobotState(self.rpc, self.robot_name)
        self._pending_motion = None

    def connect(self):
        names = self.rpc.call("getRobotNames")
        if not isinstance(names, list) or self.robot_name not in names:
            raise RuntimeError(f"Robot {self.robot_name!r} not found: {names!r}")
        print("Connected:", self.robot_name)

    def get_state(self):
        return self.state.get_all()

    def get_tcp_pose(self):
        return self.state.get_tcp_pose()

    def get_status(self):
        status = self.get_state()
        for key, method in (("mode", "getRobotModeType"),
                            ("safety_mode", "getSafetyModeType"),
                            ("steady", "isSteady")):
            status[key] = self._call("RobotState", method)
        return status

    def _timeout(self, deadline=None):
        if deadline is None:
            return self.config.rpc_timeout
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("Motion deadline exceeded")
        return min(self.config.rpc_timeout, remaining)

    def _call(self, section, method, params=None, *, deadline=None):
        return self.rpc.call(
            f"{self.robot_name}.{section}.{method}", params,
            timeout=self._timeout(deadline),
        )

    def _command(self, method, params):
        result = self._call("MotionControl", method, params)
        if type(result) is not int:
            raise RuntimeError(f"{method}: invalid controller return code {result!r}")
        if result != 0:
            raise MotionError(f"{method} rejected by controller: return code {result!r}")
        return result

    def _check_ready(self, *, deadline=None):
        mode = self._call("RobotState", "getRobotModeType", deadline=deadline)
        if mode != "Running":
            raise MotionError(f"Robot must be Running; current mode: {mode!r}")
        safety = self._call("RobotState", "getSafetyModeType", deadline=deadline)
        if safety not in {"Normal", "ReducedMode"}:
            raise MotionError(f"Robot cannot move in safety mode {safety!r}")
        if self._call("RobotState", "isWithinSafetyLimits", deadline=deadline) is not True:
            raise MotionError("Controller does not confirm robot is within safety limits")
        if self._call("RobotState", "isCollisionOccurred", deadline=deadline) is not False:
            raise MotionError("Controller reports collision or invalid collision state")

    def _idle(self, *, deadline=None):
        exec_id = self._call("MotionControl", "getExecId", deadline=deadline)
        queue_size = self._call("MotionControl", "getQueueSize", deadline=deadline)
        steady = self._call("RobotState", "isSteady", deadline=deadline)
        if (type(exec_id) is not int or exec_id < -1 or
                type(queue_size) is not int or queue_size < 0 or type(steady) is not bool):
            raise MotionError("Invalid controller motion status")
        return exec_id == -1 and queue_size == 0 and steady

    def _move(self, kind, target, *, wait):
        if self._pending_motion is not None:
            raise MotionError("Wait for or stop the pending motion before submitting another")
        self._check_ready()
        if not self._idle():
            raise MotionError("Robot is busy; existing motion must finish first")
        self._command("setSpeedFraction", [self.config.speed_fraction])
        if kind == "joint":
            method = "moveJoint"
            acceleration, velocity = self.config.joint_acceleration, self.config.joint_velocity
        else:
            method = "moveLine"
            acceleration, velocity = self.config.tcp_acceleration, self.config.tcp_velocity
        self._pending_motion = (kind, target)
        try:
            self._command(method, [target, acceleration, velocity, 0, 0])
        except MotionError:
            # An explicit controller rejection did not accept this motion.
            self._pending_motion = None
            raise
        except (Exception, KeyboardInterrupt) as exc:
            # A lost reply may follow acceptance: attempt a stop, never retry.
            self._abort_motion(exc, "motion submission was not confirmed")
        if wait:
            self.wait_until_complete()
        return 0

    def _abort_motion(self, exc, detail):
        try:
            self.stop()
        except Exception as stop_error:
            raise MotionError(
                f"Motion failed ({exc}; {detail}); stopping also failed ({stop_error})"
            ) from exc
        raise exc

    def move_joint_deg(self, joints_deg, *, wait=True):
        target = vector6(joints_deg, "joints_deg")
        return self._move("joint", [math.radians(j) for j in target], wait=wait)

    def move_pose(self, pose, *, wait=True):
        """Absolute base-frame [x,y,z,rx,ry,rz], metres / ZYX Euler radians."""
        return self._move("pose", vector6(pose, "pose").tolist(), wait=wait)

    def wait_until_complete(self, timeout=None):
        """Confirm idle AND measured arrival on consecutive samples.

        An initially idle controller is not treated as proof of completion.
        Communication failures, faults, timeouts and interrupts attempt a stop.
        """
        if self._pending_motion is None:
            raise MotionError("No pending motion to wait for")
        timeout = self.config.motion_timeout if timeout is None else timeout
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("Motion timeout must be finite and positive")
        kind, target = self._pending_motion
        deadline = time.monotonic() + timeout
        settled = 0
        error = "no feedback yet"
        try:
            while True:
                self._check_ready(deadline=deadline)
                idle = self._idle(deadline=deadline)
                if kind == "joint":
                    actual = self.state.get_joint_positions(timeout=self._timeout(deadline))
                    distance = max(abs(a - b) for a, b in zip(actual, target))
                    arrived = distance <= self.config.joint_tolerance_rad
                    error = f"joint error {distance:.6f} rad"
                else:
                    actual = self.state.get_tcp_pose(timeout=self._timeout(deadline))
                    position, orientation = pose_error(actual, target)
                    arrived = (position <= self.config.position_tolerance_m and
                               orientation <= self.config.orientation_tolerance_rad)
                    error = f"position error {position:.6f} m, rotation error {orientation:.6f} rad"
                self._timeout(deadline)
                settled = settled + 1 if idle and arrived else 0
                if settled >= self.config.settled_samples:
                    self._pending_motion = None
                    return actual
                time.sleep(min(self.config.poll_interval, self._timeout(deadline)))
        except (Exception, KeyboardInterrupt) as exc:
            if isinstance(exc, TimeoutError):
                exc = TimeoutError(f"Motion did not complete within {timeout}s: {error}")
            self._abort_motion(exc, error)

    def stop(self, mode=None):
        """Stop using the SDK's stopJoint(acc) or stopLine(acc, acc_rot).

        With no pending motion, the default is joint-space stop. Call mode='pose'
        explicitly when stopping Cartesian motion started outside this driver.
        """
        mode = mode or (self._pending_motion[0] if self._pending_motion else "joint")
        if mode == "joint":
            self._command("stopJoint", [self.config.stop_joint_acceleration])
        elif mode == "pose":
            self._command("stopLine", [
                self.config.stop_linear_acceleration, self.config.stop_angular_acceleration,
            ])
        else:
            raise ValueError("Stop mode must be 'joint' or 'pose'")
        deadline = time.monotonic() + self.config.stop_timeout
        settled = 0
        while True:
            idle = self._idle(deadline=deadline)
            self._timeout(deadline)
            settled = settled + 1 if idle else 0
            if settled >= self.config.settled_samples:
                self._pending_motion = None
                return 0
            time.sleep(min(self.config.poll_interval, self._timeout(deadline)))

    def sleep(self, seconds):
        """Delay only; use wait_until_complete() to confirm motion completion."""
        time.sleep(seconds)
