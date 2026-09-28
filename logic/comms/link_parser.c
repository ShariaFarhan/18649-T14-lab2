/*
 * link_parser.c -- see link_parser.h for why this file has no Zephyr
 * dependency. State machine ported 1:1 from link_forwarder.c's
 * uart_rx_thread() parser on the Pi (same states, same transitions,
 * same "CRC fail or size mismatch -> silently drop, no NACK" behavior),
 * so this side of the link runs provably matching logic rather than a
 * hand-retyped lookalike.
 */

#include "link_parser.h"
#include <string.h>

/* --- CRC ------------------------------------------------------------ */

uint16_t link_crc16_ccitt(uint16_t seed, const uint8_t *data, size_t len)
{
    uint16_t crc = seed;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408)
                             : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

/* --- Encoder ---------------------------------------------------------- */

size_t link_frame_encode(uint8_t *out_buf, size_t out_buf_cap,
                          uint8_t seq, const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len > MAX_PAYLOAD) {
        return 0;
    }
    size_t needed = 2 + 1 + 1 + (size_t)payload_len + 2;
    if (out_buf_cap < needed) {
        return 0;
    }

    uint8_t crc_region[1 + 1 + MAX_PAYLOAD]; /* LEN | SEQ | PAYLOAD */
    size_t idx = 0;

    out_buf[idx++] = FRAME_START0;
    out_buf[idx++] = FRAME_START1;
    out_buf[idx++] = payload_len;
    out_buf[idx++] = seq;
    memcpy(&out_buf[idx], payload, payload_len);
    idx += payload_len;

    crc_region[0] = payload_len;
    crc_region[1] = seq;
    memcpy(&crc_region[2], payload, payload_len);
    uint16_t crc = link_crc16_ccitt(0xFFFF, crc_region, 2 + (size_t)payload_len);

    out_buf[idx++] = (uint8_t)(crc & 0xFF);
    out_buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    return idx;
}

/* --- Parser ------------------------------------------------------------ */

void link_parser_init(link_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = LP_SCAN_START0;
}

void link_parser_reset(link_parser_t *p)
{
    p->state = LP_SCAN_START0;
    p->payload_idx = 0;
}

bool link_parser_in_progress(const link_parser_t *p)
{
    return p->state != LP_SCAN_START0;
}

link_frame_result_t link_parser_feed_byte(link_parser_t *p, uint8_t byte,
                                           uint8_t *out_seq, uint8_t *out_payload,
                                           uint8_t *out_payload_len)
{
    switch (p->state) {
    case LP_SCAN_START0:
        if (byte == FRAME_START0) {
            p->state = LP_SCAN_START1;
        }
        return LP_INCOMPLETE;

    case LP_SCAN_START1:
        if (byte == FRAME_START1) {
            p->state = LP_LEN;
        } else {
            /* False positive on START0 -- resync, but don't lose a
             * genuine START0 that happens to be this byte (mirrors
             * link_forwarder.c's uart_rx_thread exactly). */
            p->state = (byte == FRAME_START0) ? LP_SCAN_START1 : LP_SCAN_START0;
        }
        return LP_INCOMPLETE;

    case LP_LEN:
        if (byte > MAX_PAYLOAD) {
            /* Implausible length -- this wasn't a real frame, not a
             * "bad" one; go back to scanning without counting it as a
             * rejection. */
            p->state = LP_SCAN_START0;
            return LP_INCOMPLETE;
        }
        p->len = byte;
        p->payload_idx = 0;
        p->state = LP_SEQ;
        return LP_INCOMPLETE;

    case LP_SEQ:
        p->seq = byte;
        p->state = (p->len > 0) ? LP_PAYLOAD : LP_CRC0;
        return LP_INCOMPLETE;

    case LP_PAYLOAD:
        p->payload_buf[p->payload_idx++] = byte;
        if (p->payload_idx == p->len) {
            p->state = LP_CRC0;
        }
        return LP_INCOMPLETE;

    case LP_CRC0:
        p->crc_rx = byte;
        p->state = LP_CRC1;
        return LP_INCOMPLETE;

    case LP_CRC1: {
        p->crc_rx |= ((uint16_t)byte << 8);

        uint8_t crc_region[1 + 1 + MAX_PAYLOAD];
        crc_region[0] = p->len;
        crc_region[1] = p->seq;
        memcpy(&crc_region[2], p->payload_buf, p->len);
        uint16_t crc_calc = link_crc16_ccitt(0xFFFF, crc_region, 2 + (size_t)p->len);

        bool valid = (crc_calc == p->crc_rx);

        if (valid) {
            *out_seq = p->seq;
            *out_payload_len = p->len;
            memcpy(out_payload, p->payload_buf, p->len);
        }
        /* CRC fail -> silently drop, no NACK (no ARQ on this link). */

        p->state = LP_SCAN_START0;
        return valid ? LP_VALID : LP_REJECTED;
    }
    }

    /* Unreachable, but keeps -Wall happy on compilers that don't see
     * the switch as exhaustive. */
    p->state = LP_SCAN_START0;
    return LP_INCOMPLETE;
}
