#!/usr/bin/env python3
"""Export committed source and its revision for offline VM transfers."""

import argparse
import io
import subprocess
import zipfile
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Output ZIP path")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    dirty = subprocess.check_output(
        ["git", "-C", str(root), "status", "--porcelain"], text=True,
    ).strip()
    if dirty:
        parser.error("Commit your source changes before exporting the VM package")
    revision = subprocess.check_output(
        ["git", "-C", str(root), "rev-parse", "HEAD"], text=True,
    ).strip()
    archive = subprocess.check_output([
        "git", "-C", str(root), "archive", "--format=zip",
        "--prefix=AUBO_VLA_Project/", revision,
    ])
    buffer = io.BytesIO(archive)
    with zipfile.ZipFile(buffer, "a") as package:
        package.writestr("AUBO_VLA_Project/REVISION.txt", revision + "\n")
    args.output.write_bytes(buffer.getvalue())
    print(f"Exported {revision} to {args.output.resolve()}")


if __name__ == "__main__":
    main()
