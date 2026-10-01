#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPO_DIR=$(dirname -- "$SCRIPT_DIR")
PI_USER=${PI_USER:-sfarhan}
PI_HOST=${PI_HOST:-172.26.116.157}
MAC_IP=${MAC_IP:-172.26.51.10}
PI_UART=${PI_UART:-/dev/serial0}

echo "Deploying dual-wheel relay to $PI_USER@$PI_HOST."
echo "Enter the Raspberry Pi password when SSH prompts."

COPYFILE_DISABLE=1 tar --no-xattrs -C "$REPO_DIR/pi" -cf - \
    wheel_dual.py \
    wheel_dual_protocol.py \
    wheel_protocol.py \
    self_test_button.py \
    link_health.py \
    test_points.py |
ssh "$PI_USER@$PI_HOST" \
    "for pid in \$(fuser '$PI_UART' 2>/dev/null || true); do \
         command=\$(ps -p \"\$pid\" -o args=); \
         case \"\$command\" in \
             *wheel_dual.py*' pi '*) kill \"\$pid\" ;; \
             *) echo \"Refusing to stop non-wheel process \$pid: \$command\" >&2; exit 1 ;; \
         esac; \
     done; \
     sleep 1; \
     mkdir -p ~/lab2-dual && \
     tar -xf - -C ~/lab2-dual && \
     cd ~/lab2-dual && \
     exec python3 wheel_dual.py pi --uart '$PI_UART' --bind '$PI_HOST' --pc-ip '$MAC_IP'"
