#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
mkdir -p build-checks
for name in test_lab3_can_rx test_lab3_health test_lab3_delayed_bus test_lab3_relay test_lab3_steer_mapping test_lab3_drive_policy; do
    gcc -std=c11 -Wall -Wextra -Werror -Ipi -Ishared "tests/$name.c" -o "build-checks/$name"
    "build-checks/$name"
done
gcc -std=c11 -Wall -Wextra -Werror -Ipi -Ishared -Istm32/logic/system -Istm32/logic/drive tests/test_lab3_completion.c stm32/logic/system/system_state.c stm32/logic/drive/dual_speed_pid.c -o build-checks/test_lab3_completion
./build-checks/test_lab3_completion
gcc -std=c11 -Wall -Wextra -Werror -pthread -Itests/fakes -Ipi -Ishared tests/test_lab3_transport.c -o build-checks/test_lab3_transport
./build-checks/test_lab3_transport
gcc -std=c11 -Wall -Wextra -Werror -Ipi tests/test_cockpit_math.c -lm -o build-checks/test_cockpit_math
./build-checks/test_cockpit_math
gcc -std=c11 -Wall -Wextra -Werror -O2 -Ipi pi/cockpit.c -o build-checks/cockpit-complete
python3 tests/test_windows_packet.py
python3 -m py_compile tools/windows_wheel_udp.py tools/lab3_bad_input.py tools/lab3_capture.py
