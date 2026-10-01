#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPO_DIR=$(dirname -- "$SCRIPT_DIR")

exec "$REPO_DIR/.venv-wheel/bin/python" \
    "$REPO_DIR/logitech-wheel-dev-updated-f26/mac_proxy.py" --probe
