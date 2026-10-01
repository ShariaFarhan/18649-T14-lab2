#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPO_DIR=$(dirname -- "$SCRIPT_DIR")

exec "$REPO_DIR/.venv-wheel/bin/python" \
    "$REPO_DIR/pi/wheel_dual.py" pc \
    --pi 172.26.116.157 \
    --web-port 18866 \
    --steer-axis 0 \
    --throttle-axis 1 \
    --brake-axis 2 \
    "$@"
