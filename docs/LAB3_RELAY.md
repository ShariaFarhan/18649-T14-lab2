# Lab3 N2 relay

This is the deployed relay-stage record. The newer unflashed candidate in
LAB3_COMPLETION.md adds the formerly pending FORCE, PID, watchdog, buttons
and timing headers; its local self-test also pauses this relay.

Implemented against handout v1.0, Part 4.2-4.3 (pages 14-15), Part 5
(page 16), test points (page 17), and checkoff item 7 (page 19).
Local code is not committed or pushed. `front-relay.bin` was flashed to Front
N2 (D:/) on 2026-10-22. A five-second passive CAN check observed additional
identical same-sequence copies of CTRL, HB_N1, HB_N3 and BLINK, confirming
forwarding on the connected topology. Front and Rear heartbeats reported
NORMAL with no degraded flags. COM10 returned no bytes after flashing;
serial relay counters were not verified. Physical crossed-isolation and
scope timing checks remain pending.

## Routing and state

Only Front/N2 relays. Its independent preemptible RELAY thread has priority 4
and a 1536-byte stack, above STEER(5) and dispatch(6), below the cooperative
MCP2515 workers. The ISR still only enqueues received frames.

| Frame | Action on N2 |
|---|---|
| CTRL 0x010, BLINK 0x200 | Forward a valid NEW frame onto the other bus |
| HB_N1 0x100, HB_N3 0x102 | Same, to preserve peer visibility across isolation |
| FAULT 0x020 from N1 or N3 | Same; independent sequence state per producer |
| HB_N2 and N2's own FAULT | Original dual-bus transmission, never relay |
| FORCE 0x300 | N2 is its source; eventual producer must transmit on both buses |

The FORCE producer remains a separate pending feature. No extra sequence,
relay flag, CRC rewrite or new CAN ID is introduced. Original eight bytes and
the original four-bit sequence are preserved.

N2 offers forwarding immediately after its existing validation/deduplication;
it does not wait 60 ms for a degraded flag. A second copy observed on the
destination bus cancels pending forwarding. Same-bus repeats, OLD frames,
conflicts and malformed payloads cannot start forwarding. N1 and N3 do not
relay, so copies cannot circulate through another bridge. Late copies remain
subject to the original receive-side deduplication.

The relay holds one latest pending frame per message/source slot. A newer
generation supersedes old pending work; a completion for the older generation
cannot clear the newer one. Each accepted generation is queued at most once
to the other bus. Pending software work expires after 20 ms. This expiration
does not cancel a frame already queued in MCP2515 hardware; existing controller
recovery handles stalled TX. TX errors and such recovery must be measured on
the bench.

Both buses are tried with `K_NO_WAIT`. A blocked destination cannot hold a TX
wait across the healthy bus. Pending work is revisited at 1 ms intervals and
on notifications. The policy/state mutex is never held over SPI or printing.
Relay TX completions contribute to the existing per-controller progress
counter, avoiding a false stalled-controller reset while it is forwarding.

Health only uses actual received, advancing heartbeats. Forwarding does not
fabricate peer liveness or clear a degraded flag. No heartbeat thresholds were
relaxed. The separately configured 250 ms laptop UDP timeout remains unchanged.

Always-offered forwarding can add traffic even when both direct paths work:
the worker can enqueue before the original other-bus copy reaches dispatch.
Account for these copies and extra RX/SPI work in bus utilization and RTA.
This design does not promise unchanged healthy-bus load.

## Test point and firmware

- Front **D6 (PB10)** = RELAY_TX. It toggles after each successfully queued
  forwarded frame, irrespective of which destination bus. It is a queue marker,
  not a transmission-complete marker.
- Front **D7** remains the 50 Hz servo output.
- Rear **D6/D7** remain motor scope PWM; these are on a different board.
- Rear **A4** is validated/deduplicated CTRL_RX and **D8** is PWM_SET.

Build with NODE_ID=2 as documented in `LAB3_CTRL_BLINK.md`. The relay image is
`build-checks/firmware/front-relay.bin`. Only Front needs the new image; the
currently deployed Pi and Rear already accept the unchanged frame format.

## Complete bench sequence

Keep real actuator power disconnected for this scope-output demonstration.
Keep module power, SPI, grounds and both bus termination resistors connected.
Fault injection below removes only the selected module's H/L branch wires.

1. With all branches connected, confirm all nodes NORMAL, all DEG lamps off,
   steering on Front D7, left/right lamps and Rear pedal outputs working.
   A healthy-network relay may already toggle D6; this is expected.
2. Remove **Pi/N1 CAN B** H/L. Pi DEG_B should light; A remains operational.
3. Also remove **Rear/N3 CAN A** H/L. N2 stays connected to both buses. This
   is checkoff's hop scenario: Pi -> A -> Front relay -> B -> Rear, and
   Rear heartbeat -> B -> Front relay -> A -> Pi.
4. Confirm only Pi DEG_B and Rear DEG_A are on, all zones return/remain NORMAL,
   Rear CTRL_RX/PWM_SET keep advancing, throttle/brake outputs still respond,
   and front/rear lamps follow the same commands. Front logs should show
   queued_to_B increasing for N1 traffic and queued_to_A for HB_N3.
5. Measure the brake chain with Pi UDP_RX, Front D6 RELAY_TX and Rear DIR_A
   (A0) / PWM_SET (D8). Identify the CTRL frame with CAN decoding: RELAY_TX
   also marks forwarded heartbeats/BLINK/FAULT and is not CTRL-specific.
   Record the maximum across repeated command changes, not the typical value.
6. Restore Rear A and Pi B. Confirm degraded lamps clear and no reset is needed.
7. Repeat the mirror topology: Pi A disconnected, Rear B disconnected.
8. Restore all wires, then remove both branches from Rear. It must fail safe
   and the remaining nodes must enter system error. Restore both and check
   automatic recovery. This ensures relay is not masking a truly missing node.

Handout acceptance targets still require physical measurements: DEG on <=60 ms,
DEG off <=100 ms, isolation system error <=100 ms, restoration <=500 ms,
front/rear blinker edges <=1 ms, steering <=50 ms, brake/throttle <=2 ms.
Part 4.3 explicitly allows an analytically justified report of a longer relay
brake path; do not claim a 2 ms pass without measurement and the revised RTA.
The extra receive/SPI/RELAY/second-bus costs belong in that analysis.

## Verification completed

Host tests simulate 40 cross-isolation heartbeat/CTRL periods through the
actual receive and relay policy, covering two sequence wraps, both routing
directions, per-node health, duplicate cancellation, no re-forward loop,
software expiry, newer-frame replacement, foreign FAULT producers, invalid
input rejection, and genuine peer timeout. Existing dedup, delayed-bus and
health regressions pass. Front and Rear compile successfully.

These are software checks. Physical hop, transient jumper-pull behavior,
bus load and timing are not yet measured. This change does not implement
the remaining FORCE producer, 10 ms PID task, STM32 self-test button,
independent watchdog, or all per-bus timing headers.
