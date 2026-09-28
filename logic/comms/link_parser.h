

#ifndef LINK_PARSER_H
#define LINK_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "link_protocol.h"

//CRC
uint16_t link_crc16_ccitt(uint16_t seed, const uint8_t *data, size_t len);

// --- Encoder ---------------------------------------------------------- 

/* Builds a full frame (START0 START1 LEN SEQ PAYLOAD CRC_LO CRC_HI) into
 * out_buf. out_buf must be at least 2+1+1+payload_len+2 bytes
 * (LINK_FRAME_MAX_SIZE below is that worst case for MAX_PAYLOAD).
 * Returns the number of bytes written, or 0 if payload_len > MAX_PAYLOAD.
 * This is used by the STM32's heartbeat sender, and mirrors
 * link_forwarder.c's send_frame() exactly (same byte layout, same CRC
 * region) so a byte capture of either side's output should be
 * indistinguishable in format. */
#define LINK_FRAME_MAX_SIZE (2 + 1 + 1 + MAX_PAYLOAD + 2)
size_t link_frame_encode(uint8_t *out_buf, size_t out_buf_cap,
                          uint8_t seq, const uint8_t *payload, uint8_t payload_len);

/* --- Parser ------------------------------------------------------------ */

typedef enum {
    LP_SCAN_START0,
    LP_SCAN_START1,
    LP_LEN,
    LP_SEQ,
    LP_PAYLOAD,
    LP_CRC0,
    LP_CRC1,
} link_parse_state_t;

typedef struct {
    link_parse_state_t state;
    uint8_t  payload_buf[MAX_PAYLOAD];
    uint8_t  len;
    uint8_t  seq;
    uint8_t  payload_idx;
    uint16_t crc_rx;
} link_parser_t;

/* LP_INCOMPLETE: byte consumed, frame not finished yet (or byte was
 *   discarded while resyncing -- e.g. junk before START0).
 * LP_VALID: a full frame passed length-plausibility AND CRC; the
 *   out_* params are filled.
 * LP_REJECTED: a frame's CRC didn't check out (length was already
 *   filtered earlier, silently, same as link_forwarder.c's LEN>MAX_PAYLOAD
 *   check -- that case reports LP_INCOMPLETE, matching "wasn't a real
 *   frame" rather than "was a frame, but bad," since a length that big
 *   couldn't have been a genuine header in the first place).
 * Distinguishing REJECTED from INCOMPLETE lets a caller count CRC
 * failures for diagnostics without adding any state the Pi's own
 * parser doesn't already have -- link_forwarder.c's C-side equivalent
 * just silently drops in both cases, no ARQ either side. */
typedef enum {
    LP_INCOMPLETE,
    LP_VALID,
    LP_REJECTED,
} link_frame_result_t;

void link_parser_init(link_parser_t *p);

/* Call on a mid-frame timeout (inter-byte gap, or total-frame-duration
 * budget exceeded -- your call which; see pi_link.c for the semaphore-
 * timeout-driven version used in firmware, and the design-choice note
 * there about how it differs slightly from link_forwarder.c's own
 * "elapsed since START seen" version). Drops any partial frame and
 * goes back to scanning for START0. Safe to call even when idle. */
void link_parser_reset(link_parser_t *p);

/* True once a START byte has been matched and we're mid-frame -- use
 * this to decide whether the caller should apply a mid-frame timeout
 * at all (don't time out while idle scanning for START0). */
bool link_parser_in_progress(const link_parser_t *p);

/* Feed one received byte to the parser. See link_frame_result_t above.
 * On LP_VALID, out_seq and out_payload_len are filled and out_payload
 * (caller-supplied buffer, >= MAX_PAYLOAD bytes) holds the payload.
 * The parser automatically resets itself to scan for the next frame
 * after every terminal outcome (LP_VALID, LP_REJECTED, or an
 * implausible LEN) -- callers don't need to call link_parser_reset()
 * themselves except on an external mid-frame timeout. No NACK is ever
 * sent on rejection -- this link does not use ARQ. */
link_frame_result_t link_parser_feed_byte(link_parser_t *p, uint8_t byte,
                                           uint8_t *out_seq, uint8_t *out_payload,
                                           uint8_t *out_payload_len);

#endif /* LINK_PARSER_H */
