/* Bench UART check only: incoming commands never actuate the vehicle. */
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include "pi_link.h"

void pi_link_on_command_received(const cmd_payload_t *cmd, bool plausible)
{
    printk("UART RX: steer=%d throttle=%d brake=%d buttons=%u flags=%u valid=%d (test only)\n",
           cmd->steering_raw, cmd->throttle_raw, cmd->brake_raw,
           cmd->buttons, cmd->flags, plausible);
}

void pi_link_get_currents(uint16_t *left, uint16_t *right, uint16_t *servo)
{
    /* No sensors connected: do not report zero as a measurement. */
    *left = *right = *servo = UINT16_MAX;
}

static int status_command(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    shell_print(sh, "UART test only: alive=%d received=%u rejected=%u seq_gaps=%u",
                pi_link_is_alive(), pi_link_rx_valid_count(),
                pi_link_rx_crc_fail_count(), pi_link_rx_seq_gap_count());
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(link_commands,
    SHELL_CMD(status, NULL, "UART transport counters (no actuation)", status_command),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(link, &link_commands, "Pi UART diagnostics", NULL);
