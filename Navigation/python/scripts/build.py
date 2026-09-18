#!/usr/bin/env python3
"""Configure and build the PyRobo C++ candidate interface."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[2]
CPP_DIR = ROOT_DIR / "cpp"
DEFAULT_BUILD_DIR = CPP_DIR / "build"
HOT_RELOAD_SCRIPT = ROOT_DIR / "python" / "scripts" / "hot_reload.py"


def run_command(command: list[str], cwd: Path) -> None:
    print(f"+ {' '.join(command)}", flush=True)
    try:
        subprocess.run(command, cwd=cwd, check=True)
    except subprocess.CalledProcessError as error:
        raise SystemExit(error.returncode) from None


def require_command(name: str) -> str:
    path = shutil.which(name)
    if path is None:
        raise SystemExit(
            f"error: required command '{name}' was not found in PATH. "
            "Please install CMake and a C++17 compiler."
        )
    return path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build the C++ interface/template with CMake."
    )
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=DEFAULT_BUILD_DIR,
        help="CMake build directory. Defaults to cpp/build.",
    )
    parser.add_argument(
        "--config",
        default="Release",
        choices=("Debug", "Release", "RelWithDebInfo", "MinSizeRel"),
        help="CMake build configuration.",
    )
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Delete the build directory before configuring.",
    )
    parser.add_argument(
        "--generator",
        help="Optional CMake generator, for example Ninja or Unix Makefiles.",
    )
    parser.add_argument(
        "--target",
        help="Optional build target.",
    )
    parser.add_argument(
        "--parallel",
        type=int,
        default=os.cpu_count() or 1,
        help="Number of parallel build jobs.",
    )
    parser.add_argument(
        "--watch",
        action="store_true",
        help="After building, start the simulator and rebuild when contestant.cpp changes.",
    )
    parser.add_argument(
        "--scenario",
        choices=("ex", "unknown"),
        default="ex",
        help="Scenario configuration module under python/.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cmake = require_command("cmake")

    build_dir = args.build_dir
    if not build_dir.is_absolute():
        build_dir = ROOT_DIR / build_dir

    if args.clean and build_dir.exists():
        shutil.rmtree(build_dir)

    configure_command = [
        cmake,
        "-S",
        str(CPP_DIR),
        "-B",
        str(build_dir),
        f"-DCMAKE_BUILD_TYPE={args.config}",
    ]
    if args.generator:
        configure_command.extend(["-G", args.generator])

    build_command = [
        cmake,
        "--build",
        str(build_dir),
        "--config",
        args.config,
        "--parallel",
        str(args.parallel),
    ]
    if args.target:
        build_command.extend(["--target", args.target])

    run_command(configure_command, ROOT_DIR)
    run_command(build_command, ROOT_DIR)

    if args.watch:
        watch_command = [
            sys.executable,
            str(HOT_RELOAD_SCRIPT),
            "--skip-build",
            "--config",
            args.config,
            "--build-dir",
            str(build_dir),
            "--parallel",
            str(args.parallel),
            "--scenario",
            args.scenario,
        ]
        run_command(watch_command, ROOT_DIR)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
