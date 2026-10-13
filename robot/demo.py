"""CLI shared by the simulator verification scripts; importing never moves."""

import argparse
import json
import subprocess
import sys
from dataclasses import asdict
from pathlib import Path

from .aubo_driver import AuboRobot
from .config import RobotConfig


ROOT = Path(__file__).resolve().parents[1]


def repository_revision():
    exported = ROOT / "REVISION.txt"
    if exported.exists():
        return exported.read_text(encoding="utf-8").strip()
    if (ROOT / ".git").exists():
        try:
            revision = subprocess.check_output(
                ["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True,
            ).strip()
            dirty = subprocess.check_output(
                ["git", "-C", str(ROOT), "status", "--porcelain"], text=True,
            ).strip()
            return revision + (" (uncommitted changes)" if dirty else "")
        except (OSError, subprocess.CalledProcessError):
            pass
    return "unknown (use scripts/export_vm.py to include the revision)"


def run_demo(callback, description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--config", help="Robot JSON config; defaults to AUBO_CONFIG or config/robot_sim.json")
    parser.add_argument("--status", action="store_true", help="Read state only; do not move")
    args = parser.parse_args()
    try:
        config = RobotConfig.from_file(args.config)
        print("Project revision:", repository_revision())
        print("Robot configuration:", json.dumps(asdict(config), ensure_ascii=False))
        robot = AuboRobot(config=config)
        robot.connect()
        if args.status:
            print("Status:", robot.get_status())
        else:
            callback(robot)
        return 0
    except KeyboardInterrupt:
        print("Interrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
