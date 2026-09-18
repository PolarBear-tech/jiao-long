#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

if command -v python3 >/dev/null 2>&1; then
    exec python3 "$ROOT_DIR/python/scripts/run.py" "$@"
fi

exec python "$ROOT_DIR/python/scripts/run.py" "$@"
