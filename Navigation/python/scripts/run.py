#!/usr/bin/env python3
"""Build the C++ template if requested, then run the Python simulator."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[2]
BUILD_SCRIPT = ROOT_DIR / "python" / "scripts" / "build.py"
MAIN_SCRIPT = ROOT_DIR / "python" / "main.py"


def run_command(command: list[str], cwd: Path) -> None:
    print(f"+ {' '.join(command)}", flush=True)
    try:
        subprocess.run(command, cwd=cwd, check=True)
    except subprocess.CalledProcessError as error:
        raise SystemExit(error.returncode) from None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run PyRobo from the repository root."
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Start the simulator without compiling the C++ interface first.",
    )
    parser.add_argument(
        "--display",
        action="store_true",
        help="Only display the simulator; do not build, plan, or control the robot.",
    )
    parser.add_argument(
        "--scenario",
        choices=("ex", "unknown"),
        default="ex",
        help="Scenario configuration module under python/.",
    )
    parser.add_argument(
        "--config",
        default="Release",
        choices=("Debug", "Release", "RelWithDebInfo", "MinSizeRel"),
        help="CMake build configuration used when building first.",
    )
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Clean the C++ build directory before building.",
    )
    parser.add_argument(
        "--python",
        default=sys.executable,
        help="Python executable used to run python/main.py.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if not args.skip_build and not args.display:
        build_command = [args.python, str(BUILD_SCRIPT), "--config", args.config]
        if args.clean:
            build_command.append("--clean")
        run_command(build_command, ROOT_DIR)

    main_command = [args.python, str(MAIN_SCRIPT)]
    if args.display:
        main_command.append("--display")
    main_command.extend(["--scenario", args.scenario])
    run_command(main_command, ROOT_DIR)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
