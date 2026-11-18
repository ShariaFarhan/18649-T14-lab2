#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
python3 tools/lab5_config.py --check
mkdir -p build-lab5-tests
gcc -std=c11 -Wall -Wextra -Werror -Ilab5 tests/test_lab5_core.c lab5/raft.c lab5/network.c lab5/time_sync.c lab5/block_log.c -o build-lab5-tests/test_core
./build-lab5-tests/test_core
gcc -std=c11 -Wall -Wextra -Werror -Ilab5 tests/test_lab5_durability.c lab5/network.c lab5/block_log.c -o build-lab5-tests/test_durability
./build-lab5-tests/test_durability
gcc -std=c11 -Wall -Wextra -Werror -Ilab5 -Ishared tests/test_lab5_transport.c lab5/raft.c lab5/network.c lab5/time_sync.c lab5/node.c lab5/wire.c -o build-lab5-tests/test_transport
./build-lab5-tests/test_transport
gcc -std=c11 -Wall -Wextra -Werror -Ilab5 -Ishared tests/test_lab5_can_arbitration.c lab5/raft.c lab5/network.c lab5/time_sync.c lab5/node.c lab5/wire.c -o build-lab5-tests/test_can_arbitration
./build-lab5-tests/test_can_arbitration
gcc -std=c11 -Wall -Wextra -Werror -Ilab5 tests/test_lab5_posix.c lab5/block_log.c lab5/network.c lab5/posix_store.c lab5/event_display.c -o build-lab5-tests/test_posix
./build-lab5-tests/test_posix
sh tools/build_lab5_pi.sh
python3 -m py_compile tools/lab5_config.py tools/lab5_display.py tools/lab5_capture.py
