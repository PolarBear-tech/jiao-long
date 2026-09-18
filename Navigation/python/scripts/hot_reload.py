#!/usr/bin/env python3
"""Rebuild and restart the simulator when the C++ answer changes."""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[2]
BUILD_SCRIPT = ROOT_DIR / "python" / "scripts" / "build.py"
RUN_SCRIPT = ROOT_DIR / "python" / "scripts" / "run.py"
WATCH_FILES = (ROOT_DIR / "cpp" / "src" / "contestant.cpp",)


def snapshot() -> tuple[tuple[str, int, int], ...]:
    result = []
    for path in WATCH_FILES:
        try:
            stat = path.stat()
        except FileNotFoundError:
            result.append((str(path), 0, 0))
        else:
            result.append((str(path), stat.st_mtime_ns, stat.st_size))
    return tuple(result)


def run_build(
    python: str, config: str, build_dir: Path, parallel: int, scenario: str
) -> bool:
    command = [
        python,
        str(BUILD_SCRIPT),
        "--config",
        config,
        "--build-dir",
        str(build_dir),
        "--parallel",
        str(parallel),
        "--scenario",
        scenario,
    ]
    print(f"+ {' '.join(command)}", flush=True)
    return subprocess.run(command, cwd=ROOT_DIR).returncode == 0


def stop_process(process: subprocess.Popen[bytes] | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def start_process(python: str, config: str, scenario: str) -> subprocess.Popen[bytes]:
    command = [
        python,
        str(RUN_SCRIPT),
        "--skip-build",
        "--config",
        config,
        "--scenario",
        scenario,
    ]
    print(f"+ {' '.join(command)}", flush=True)
    return subprocess.Popen(command, cwd=ROOT_DIR)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Rebuild and restart PyRobo when contestant.cpp changes."
    )
    parser.add_argument(
        "--config",
        default="Release",
        choices=("Debug", "Release", "RelWithDebInfo", "MinSizeRel"),
    )
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=ROOT_DIR / "cpp" / "build",
        help="CMake build directory. Defaults to cpp/build.",
    )
    parser.add_argument(
        "--parallel",
        type=int,
        default=1,
        help="Number of parallel build jobs used after source changes.",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Use the existing build and start watching immediately.",
    )
    parser.add_argument("--interval", type=float, default=0.5, help="Polling interval in seconds.")
    parser.add_argument(
        "--scenario",
        choices=("ex", "unknown"),
        default="ex",
        help="Scenario configuration module under python/.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.interval <= 0:
        raise SystemExit("--interval must be positive")

    if args.parallel <= 0:
        raise SystemExit("--parallel must be positive")
    if not args.skip_build and not run_build(
        sys.executable, args.config, args.build_dir, args.parallel, args.scenario
    ):
        raise SystemExit("initial C++ build failed")

    process = start_process(sys.executable, args.config, args.scenario)
    last_snapshot = snapshot()
    try:
        while True:
            time.sleep(args.interval)
            current_snapshot = snapshot()
            if current_snapshot == last_snapshot:
                if process.poll() is not None:
                    return process.returncode or 0
                continue

            last_snapshot = current_snapshot
            print("contestant.cpp changed; rebuilding and restarting...", flush=True)
            stop_process(process)
            if run_build(
                sys.executable,
                args.config,
                args.build_dir,
                args.parallel,
                args.scenario,
            ):
                process = start_process(sys.executable, args.config, args.scenario)
            else:
                print("build failed; waiting for the next source change", flush=True)
    except KeyboardInterrupt:
        return 0
    finally:
        stop_process(process)


if __name__ == "__main__":
    raise SystemExit(main())
