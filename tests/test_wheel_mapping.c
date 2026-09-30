#include <assert.h>
#include <stddef.h>
#include "wheel_mapping.h"
#include "link_protocol.h"
int main(void)
{
    _Static_assert(sizeof(cmd_payload_t) == 15, "raw UART payload size");
    _Static_assert(offsetof(cmd_payload_t, flags) == 14, "raw UART flag offset");
    int16_t out, previous = -100;
    for (int32_t raw = 0; raw <= 65535; ++raw) {
        assert(wheel_steering_percent(raw, &out) == 0);
        assert(out >= previous && out >= -100 && out <= 100);
        previous = out;
    }
    assert(wheel_steering_percent(0, &out) == 0 && out == -100);
    assert(wheel_steering_percent(WHEEL_RAW_CENTER, &out) == 0 && out == 0);
    assert(wheel_steering_percent(65535, &out) == 0 && out == 100);
    assert(wheel_steering_percent(-1, &out) != 0);
    assert(wheel_steering_percent(65536, &out) != 0);
    assert(wheel_steering_percent(0, NULL) != 0);
    return 0;
}
