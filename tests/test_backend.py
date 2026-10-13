"""Offline regressions; no simulator or physical robot is contacted."""

import contextlib
import io
import json
import math
import os
import runpy
import tempfile
import unittest
import urllib.error
from pathlib import Path
from unittest.mock import patch

import numpy as np

from control.action_decoder import ActionDecoder
from control.pose import pose_error, pose_rotation
from robot.aubo_driver import AuboRobot, MotionError
from robot.aubo_rpc import AuboRPC, AuboRPCError
from robot.config import DEFAULT_CONFIG, RobotConfig
from robot.robot_state import RobotState


ROOT = Path(__file__).resolve().parents[1]


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Controller:
    """Emulates delayed execution and measured state independently of commands."""

    def __init__(self):
        self.calls = []
        self.mode = "Running"
        self.safety = "Normal"
        self.collision = False
        self.within_limits = True
        self.busy = False
        self.started = False
        self.kind = None
        self.target = None
        self.feedback_reads = 0
        self.frames = None
        self.return_codes = {}
        self.submit_error = None
        self.feedback_error = None
        self.fault_after_move = False
        self.stop_steady = True
        self.joints = [0.0] * 6
        self.pose = [0.3, 0.1, 0.4, 0.0, 0.0, 0.0]

    def frame(self):
        frames = self.frames or [
            {"actual": self.joints if self.kind == "joint" else self.pose},
            {"exec": 7, "steady": False, "actual": self.target},
            {"actual": self.target},
        ]
        return frames[min(self.feedback_reads, len(frames) - 1)]

    def call(self, method, params=None, *, timeout=None):
        self.calls.append((method, params, timeout))
        name = method.rsplit(".", 1)[-1]
        if name == "getRobotNames":
            return ["rob1"]
        if name == "getRobotModeType":
            return "ProtectiveStop" if self.started and self.fault_after_move else self.mode
        if name == "getSafetyModeType":
            return self.safety
        if name == "isCollisionOccurred":
            return self.collision
        if name == "isWithinSafetyLimits":
            return self.within_limits
        if name in {"getExecId", "getQueueSize", "isSteady"}:
            if self.started:
                frame = self.frame()
                return frame.get({"getExecId": "exec", "getQueueSize": "queue", "isSteady": "steady"}[name],
                                 {"getExecId": -1, "getQueueSize": 0, "isSteady": True}[name])
            return {"getExecId": 9 if self.busy else -1,
                    "getQueueSize": 1 if self.busy else 0, "isSteady": self.stop_steady}[name]
        if name in {"getJointPositions", "getTcpPose"}:
            if self.started and name == ("getJointPositions" if self.kind == "joint" else "getTcpPose"):
                if self.feedback_error:
                    raise self.feedback_error
                actual = self.frame()["actual"]
                self.feedback_reads += 1
                return actual
            return self.joints if name == "getJointPositions" else self.pose
        if name == "setSpeedFraction":
            return self.return_codes.get(name, 0)
        if name in {"moveJoint", "moveLine"}:
            self.kind = "joint" if name == "moveJoint" else "pose"
            self.target = params[0]
            self.started = self.return_codes.get(name, 0) == 0
            if self.submit_error:
                self.started = True
                raise self.submit_error
            return self.return_codes.get(name, 0)
        if name in {"stopJoint", "stopLine"}:
            result = self.return_codes.get(name, 0)
            if result == 0:
                self.started = False
            return result
        raise AssertionError(f"Unexpected RPC method {method}")

    def commands(self, name):
        return [params for method, params, _ in self.calls if method.endswith("." + name)]


class DriverTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.controller = Controller()
        self.config = RobotConfig(poll_interval=0.01, motion_timeout=0.1, stop_timeout=0.05)
        self.robot = AuboRobot(config=self.config)
        self.robot.rpc = self.controller
        self.robot.state = RobotState(self.controller)
        self.addCleanup(patch.stopall)
        patch("robot.aubo_driver.time.monotonic", self.clock.monotonic).start()
        patch("robot.aubo_driver.time.sleep", self.clock.sleep).start()

    def test_joint_arrival_waits_through_initial_idle_and_active_motion(self):
        self.assertEqual(self.robot.move_joint_deg([5, 0, 0, 0, 0, 0]), 0)
        self.assertEqual(self.controller.feedback_reads, 4)
        target = self.controller.commands("moveJoint")[0][0]
        self.assertAlmostEqual(target[0], math.radians(5))
        self.assertEqual(self.controller.commands("setSpeedFraction"), [[0.1]])
        self.assertIsNone(self.robot._pending_motion)

    def test_queue_must_be_empty_even_when_feedback_is_at_target(self):
        target = [0.31, 0.1, 0.4, 0, 0, 0]
        self.controller.frames = [{"queue": 1, "actual": target}, {"actual": target}]
        self.robot.move_pose(target)
        self.assertEqual(self.controller.feedback_reads, 3)

    def test_equivalent_wrapped_orientation_counts_as_arrived(self):
        target = [0.31, 0.1, 0.4, 0, 0, math.pi]
        self.controller.frames = [{"actual": target[:5] + [-math.pi]}]
        self.robot.move_pose(target)
        self.assertEqual(self.controller.feedback_reads, 2)

    def test_wrong_orientation_times_out_and_stops_cartesian_motion(self):
        target = [0.31, 0.1, 0.4, 0, 0, 0]
        self.controller.frames = [{"actual": target[:5] + [0.2]}]
        with self.assertRaisesRegex(TimeoutError, "rotation error"):
            self.robot.move_pose(target)
        self.assertEqual(self.controller.commands("stopLine"), [[0.25, 0.5]])
        self.assertIsNone(self.robot._pending_motion)

    def test_idle_without_joint_arrival_is_not_success(self):
        self.controller.frames = [{"actual": [0.0] * 6}]
        with self.assertRaisesRegex(TimeoutError, "joint error"):
            self.robot.move_joint_deg([5, 0, 0, 0, 0, 0])
        self.assertEqual(self.controller.commands("stopJoint"), [[0.3]])

    def test_nonblocking_motion_requires_explicit_wait(self):
        self.robot.move_pose([0.31, 0.1, 0.4, 0, 0, 0], wait=False)
        self.assertEqual(self.controller.feedback_reads, 0)
        with self.assertRaisesRegex(MotionError, "pending motion"):
            self.robot.move_pose([0.32, 0.1, 0.4, 0, 0, 0])
        self.robot.wait_until_complete()
        self.assertIsNone(self.robot._pending_motion)

    def test_state_and_busy_rejections_do_not_submit_motion(self):
        for attribute, value in (("mode", "PowerOff"), ("safety", "ProtectiveStop"),
                                 ("collision", True), ("within_limits", False), ("busy", True)):
            with self.subTest(attribute=attribute):
                setattr(self.controller, attribute, value)
                with self.assertRaises(MotionError):
                    self.robot.move_joint_deg([0] * 6)
                self.assertEqual(self.controller.commands("moveJoint"), [])
                setattr(self.controller, attribute, getattr(Controller(), attribute))

    def test_nonzero_return_code_is_reported_as_rejection(self):
        for result in [1, -5]:
            with self.subTest(result=result):
                self.controller.return_codes["moveJoint"] = result
                with self.assertRaisesRegex(MotionError, "return code"):
                    self.robot.move_joint_deg([5, 0, 0, 0, 0, 0])
                self.assertIsNone(self.robot._pending_motion)

    def test_failed_speed_setting_prevents_motion_submission(self):
        self.controller.return_codes["setSpeedFraction"] = 3
        with self.assertRaises(MotionError):
            self.robot.move_pose([0.31, 0.1, 0.4, 0, 0, 0])
        self.assertEqual(self.controller.commands("moveLine"), [])

    def test_lost_submission_reply_attempts_stop_without_retry(self):
        self.controller.submit_error = RuntimeError("connection lost after submission")
        with self.assertRaisesRegex(RuntimeError, "connection lost"):
            self.robot.move_pose([0.31, 0.1, 0.4, 0, 0, 0])
        self.assertEqual(len(self.controller.commands("moveLine")), 1)
        self.assertEqual(len(self.controller.commands("stopLine")), 1)

    def test_malformed_submission_reply_attempts_stop(self):
        self.controller.return_codes["moveJoint"] = None
        with self.assertRaisesRegex(RuntimeError, "invalid controller return"):
            self.robot.move_joint_deg([5, 0, 0, 0, 0, 0])
        self.assertEqual(len(self.controller.commands("stopJoint")), 1)

    def test_feedback_error_and_interrupt_attempt_stop(self):
        for error in [RuntimeError("feedback unavailable"), KeyboardInterrupt()]:
            with self.subTest(error=type(error)):
                self.controller.feedback_error = error
                with self.assertRaises(type(error)):
                    self.robot.move_pose([0.31, 0.1, 0.4, 0, 0, 0])
                self.assertIsNone(self.robot._pending_motion)
        self.assertEqual(len(self.controller.commands("stopLine")), 2)

    def test_fault_during_motion_attempts_stop(self):
        self.controller.fault_after_move = True
        with self.assertRaisesRegex(MotionError, "ProtectiveStop"):
            self.robot.move_pose([0.31, 0.1, 0.4, 0, 0, 0])
        self.assertEqual(len(self.controller.commands("stopLine")), 1)

    def test_failed_stop_is_reported_and_pending_target_is_retained(self):
        self.controller.frames = [{"actual": self.controller.pose}]
        self.controller.return_codes["stopLine"] = -1
        with self.assertRaisesRegex(MotionError, "stopping also failed"):
            self.robot.move_pose([0.31, 0.1, 0.4, 0, 0, 0])
        self.assertIsNotNone(self.robot._pending_motion)

    def test_stop_requires_controller_to_confirm_idle(self):
        self.controller.stop_steady = False
        with self.assertRaises(TimeoutError):
            self.robot.stop()

    def test_bad_dimensions_and_nonfinite_targets_never_reach_rpc(self):
        for target in [[0] * 5, [0] * 7, [math.nan] * 6, [math.inf] * 6]:
            for method in [self.robot.move_pose, self.robot.move_joint_deg]:
                with self.subTest(target=target, method=method):
                    with self.assertRaises(ValueError):
                        method(target)
        self.assertEqual(self.controller.calls, [])

    def test_each_wait_rpc_uses_remaining_deadline(self):
        self.robot.move_pose([0.31, 0.1, 0.4, 0, 0, 0])
        timeouts = [timeout for _, _, timeout in self.controller.calls if timeout < 5]
        self.assertTrue(timeouts)
        self.assertTrue(all(0 < timeout <= self.config.motion_timeout for timeout in timeouts))


class PoseTests(unittest.TestCase):
    def test_translation_frame_is_explicit(self):
        current = [0.3, 0.1, 0.4, 0, 0, math.pi / 2]
        delta = [0.01, 0, 0, 0, 0, 0]
        np.testing.assert_allclose(ActionDecoder("base").delta_to_pose(current, delta),
                                   [0.31, 0.1, 0.4, 0, 0, math.pi / 2])
        np.testing.assert_allclose(ActionDecoder("tool").delta_to_pose(current, delta),
                                   [0.3, 0.11, 0.4, 0, 0, math.pi / 2])

    def test_rotation_composition_is_noncommutative(self):
        current = [0, 0, 0, 0, 0, math.pi / 2]
        delta = [0, 0, 0, math.pi / 2, 0, 0]
        with self.assertWarnsRegex(UserWarning, "Gimbal lock"):
            base = ActionDecoder("base").delta_to_pose(current, delta)
        tool = ActionDecoder("tool").delta_to_pose(current, delta)
        # Analytic Rx(90) @ Rz(90) and Rz(90) @ Rx(90), respectively.
        np.testing.assert_allclose(pose_rotation(base).as_matrix(),
                                   [[0, -1, 0], [0, 0, -1], [1, 0, 0]], atol=1e-12)
        np.testing.assert_allclose(pose_rotation(tool).as_matrix(),
                                   [[0, 0, 1], [1, 0, 0], [0, 1, 0]], atol=1e-12)

    def test_orientation_error_uses_rotation_distance(self):
        position, angle = pose_error([1, 0, 0, 0, 0, -math.pi], [0, 0, 0, 0, 0, math.pi])
        self.assertEqual(position, 1)
        self.assertAlmostEqual(angle, 0)

    def test_bad_frames_and_actions_are_rejected(self):
        with self.assertRaises(ValueError):
            ActionDecoder("camera")
        for bad in [[0] * 7, [0] * 5, [math.nan] * 6, [[0] * 6]]:
            with self.assertRaises(ValueError):
                ActionDecoder().delta_to_pose([0] * 6, bad)


class RPCTests(unittest.TestCase):
    def response(self, payload):
        return io.BytesIO(json.dumps(payload).encode())

    def test_request_and_timeout_and_valid_response(self):
        with patch("urllib.request.urlopen", return_value=self.response(
                {"jsonrpc": "2.0", "id": 1, "result": ["rob1"]})) as opener:
            rpc = AuboRPC()
            self.assertEqual(rpc.call("getRobotNames", timeout=0.2), ["rob1"])
            request = opener.call_args.args[0]
            self.assertEqual(json.loads(request.data),
                             {"jsonrpc": "2.0", "id": 1, "method": "getRobotNames", "params": []})
            self.assertEqual(opener.call_args.kwargs["timeout"], 0.2)

    def test_invalid_protocol_responses_do_not_silently_return_none(self):
        for payload in [[], {"jsonrpc": "2.0", "id": 2, "result": 0},
                        {"jsonrpc": "2.0", "id": True, "result": 0},
                        {"jsonrpc": "2.0", "id": 1},
                        {"jsonrpc": "2.0", "id": 1, "result": 0, "error": {}},
                        {"jsonrpc": "2.0", "id": 1, "error": "broken"}]:
            with self.subTest(payload=payload), patch("urllib.request.urlopen", return_value=self.response(payload)):
                with self.assertRaises(RuntimeError):
                    AuboRPC().call("test")

    def test_rpc_error_preserves_error_code(self):
        payload = {"jsonrpc": "2.0", "id": 1,
                   "error": {"code": -32601, "message": "Method not found"}}
        with patch("urllib.request.urlopen", return_value=self.response(payload)):
            with self.assertRaises(AuboRPCError) as caught:
                AuboRPC().call("missing")
            self.assertEqual(caught.exception.code, -32601)

    def test_transport_and_json_errors_include_method_context(self):
        with patch("urllib.request.urlopen", side_effect=urllib.error.URLError("refused")):
            with self.assertRaisesRegex(RuntimeError, "getRobotNames.*cannot reach"):
                AuboRPC().call("getRobotNames")
        with patch("urllib.request.urlopen", return_value=io.BytesIO(b"not json")):
            with self.assertRaisesRegex(RuntimeError, "invalid JSON response"):
                AuboRPC().call("getRobotNames")


class ConfigTests(unittest.TestCase):
    def test_checked_in_config_matches_defaults(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(RobotConfig.from_file(DEFAULT_CONFIG), RobotConfig())

    def test_invalid_settings_are_rejected(self):
        for settings in [{"motion_timeout": 0}, {"poll_interval": math.nan},
                         {"speed_fraction": 2}, {"settled_samples": 1},
                         {"rpc_url": "localhost"}, {"action_frame": "camera"}]:
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                RobotConfig(**settings)

    def test_environment_overrides_selected_config(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot.json"
            path.write_text('{"robot_name": "rob2", "motion_timeout": 12}')
            with patch.dict(os.environ, {"AUBO_CONFIG": str(path), "AUBO_ROBOT_NAME": "rob3"}):
                config = RobotConfig.from_file()
            self.assertEqual(config.robot_name, "rob3")
            self.assertEqual(config.motion_timeout, 12)


class DemoTests(unittest.TestCase):
    SCRIPTS = ["test_motion.py", "test_vla_pipeline.py", "test_observation_pipeline.py"]

    def test_importing_demos_never_connects(self):
        with patch.object(AuboRobot, "connect", side_effect=AssertionError("must not connect")):
            for filename in self.SCRIPTS:
                runpy.run_path(str(ROOT / "tests" / filename))

    def run_script(self, filename, controller, flags=()):
        clock = Clock()
        output = io.StringIO()

        def transport(request, **kwargs):
            payload = json.loads(request.data)
            result = controller.call(payload["method"], payload["params"], timeout=kwargs["timeout"])
            return io.BytesIO(json.dumps({"jsonrpc": "2.0", "id": payload["id"], "result": result}).encode())

        with patch("urllib.request.urlopen", side_effect=transport), \
                patch("robot.aubo_driver.time.monotonic", clock.monotonic), \
                patch("robot.aubo_driver.time.sleep", clock.sleep), \
                patch.object(__import__('sys'), "argv", [filename, *flags]), \
                contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
            with self.assertRaises(SystemExit) as exit_result:
                runpy.run_path(str(ROOT / "tests" / filename), run_name="__main__")
        return exit_result.exception.code, output.getvalue()

    def test_all_status_entries_are_read_only(self):
        for filename in self.SCRIPTS:
            controller = Controller()
            code, output = self.run_script(filename, controller, ["--status"])
            self.assertEqual(code, 0, output)
            self.assertEqual(controller.commands("moveJoint"), [])
            self.assertEqual(controller.commands("moveLine"), [])
            self.assertIn("Project revision:", output)

    def test_pipeline_demos_confirm_measured_arrival(self):
        for filename in self.SCRIPTS[1:]:
            controller = Controller()
            code, output = self.run_script(filename, controller)
            self.assertEqual(code, 0, output)
            self.assertIn("measured TCP", output)
            # Four completion samples plus the final measured-pose log read.
            self.assertEqual(controller.feedback_reads, 5)

    def test_joint_demo_waits_for_outbound_and_return_motion(self):
        code, output = self.run_script(self.SCRIPTS[0], Controller())
        self.assertEqual(code, 0, output)
        self.assertIn("measured return joints", output)

    def test_failed_pipeline_exits_without_success_message(self):
        controller = Controller()
        controller.return_codes["moveLine"] = -5
        code, output = self.run_script(self.SCRIPTS[1], controller)
        self.assertEqual(code, 1)
        self.assertNotIn("Motion completed", output)
        self.assertIn("return code -5", output)


if __name__ == "__main__":
    unittest.main()
