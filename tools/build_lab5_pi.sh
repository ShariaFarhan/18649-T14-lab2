#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
python3 tools/lab5_config.py --check
mkdir -p build-lab5
gcc -std=c11 -Wall -Wextra -Werror -O2 -pthread -DLAB5_ENABLED=1 -Ilab5 -Ishared -Ipi \
    pi/cockpit.c pi/lab5_agent.c lab5/raft.c lab5/network.c lab5/time_sync.c \
    lab5/wire.c lab5/node.c lab5/block_log.c lab5/posix_store.c lab5/event_display.c -o build-lab5/cockpit
