"""Shared settings for the simulator driver and demo scripts."""

import json
import math
import os
from dataclasses import dataclass, fields
from pathlib import Path
from urllib.parse import urlparse

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "robot_sim.json"


@dataclass(frozen=True)
class RobotConfig:
    robot_name: str = "rob1"
    rpc_url: str = "http://127.0.0.1:9012/jsonrpc"
    rpc_timeout: float = 5.0
    motion_timeout: float = 30.0
    stop_timeout: float = 5.0
    poll_interval: float = 0.1
    settled_samples: int = 2
    speed_fraction: float = 0.1
    joint_acceleration: float = 0.3
    joint_velocity: float = 0.3
    tcp_acceleration: float = 0.25
    tcp_velocity: float = 0.25
    joint_tolerance_rad: float = 0.005
    position_tolerance_m: float = 0.001
    orientation_tolerance_rad: float = 0.01
    stop_joint_acceleration: float = 0.3
    stop_linear_acceleration: float = 0.25
    stop_angular_acceleration: float = 0.5
    action_frame: str = "base"

    def __post_init__(self):
        if not isinstance(self.robot_name, str) or not self.robot_name.strip():
            raise ValueError("robot_name must be a nonempty string")
        if not isinstance(self.rpc_url, str):
            raise ValueError("rpc_url must be an HTTP(S) URL")
        parsed = urlparse(self.rpc_url)
        if parsed.scheme not in {"http", "https"} or not parsed.hostname:
            raise ValueError("rpc_url must be an HTTP(S) URL")
        if self.action_frame not in {"base", "tool"}:
            raise ValueError("action_frame must be 'base' or 'tool'")
        if type(self.settled_samples) is not int or self.settled_samples < 2:
            raise ValueError("settled_samples must be an integer >= 2")
        for field in fields(self):
            if field.name in {"robot_name", "rpc_url", "action_frame", "settled_samples"}:
                continue
            value = getattr(self, field.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{field.name} must be a finite positive number")
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{field.name} must be a finite positive number")
        if self.speed_fraction > 1:
            raise ValueError("speed_fraction must be <= 1")

    @classmethod
    def from_file(cls, path=None):
        path = Path(path or os.environ.get("AUBO_CONFIG", DEFAULT_CONFIG))
        settings = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(settings, dict):
            raise ValueError("Robot configuration must be a JSON object")
        for key, env_name in (("rpc_url", "AUBO_RPC_URL"), ("robot_name", "AUBO_ROBOT_NAME")):
            if env_name in os.environ:
                settings[key] = os.environ[env_name]
        return cls(**settings)
