# Independent software review — 2026-10-22

Reviewer: independent agent `lab3_completion_xhigh_review`, requested **xhigh**
reasoning effort. Review was read-only; the reviewer did not edit files, flash
boards, operate live services, commit or push.

## Final verdict

**PASS — software candidate approved for bench verification.**
The reviewer reported no remaining release-blocking findings after re-review.
This is not a Lab3 timing/checkoff certification.

## Subsequent deployment finding and targeted re-review

The initial approved build exposed a driver-default conflict on hardware:
Zephyr's IWDG init preinstalled a 100 ms timeout, so the application's second
installation was rejected and the boards repeatedly reset with OUTPUT_FAULT.
The initial software review missed that configuration dependency.

The fix is `CONFIG_WDT_DISABLE_AT_BOOT=y`, plus a compile-time assertion in
`runtime.c`. Watchdog support remains enabled; the application still installs,
starts and supervises its 1000 ms IWDG. The same independent xhigh reviewer
checked the driver, rebuilt configurations and disassembly and returned
**PASS for the targeted fix** before the corrected images were flashed.

The Pi application and Windows sender were also updated. A subsequent
60-second CAN observation included user X-button tests and recovery to NORMAL
on all three nodes, approximately 3000 heartbeats from each STM32 and 1199
FORCE frames spanning -100..100. This is functional evidence, not a scope
latency pass or a deliberate watchdog-stall/reset measurement.

## First review and fixes

| Finding | Severity | Resolution |
|---|---|---|
| A queued pre-failure NORMAL heartbeat could transmit after reconnect during local self-test | P1 | Local failure drains controller TX with stop/start under the same lifecycle gate as all sends. Only the final own SELF_TEST FAULT is permitted while failed. |
| Cached hazard phase could remain solid/dark for750 ms after losing the master | P2 | Following master phases now requires fresh HB_N1 on either bus; otherwise local hazards run. |
| FORCE/RELAY sends could race MCP2515 stop/start | P2 | Per-controller gate covers the complete send and lifecycle operation. Sending uses nonblocking acquisition so a busy A does not block attempts on B. |

The first review was NOT PASS. After fixes, the same reviewer independently
reran the host suite and Front/Rear build checks and returned PASS.

## Verification

- Nine C test executables: dedup, health, delayed bus, relay, steering mapping,
  drive policy, completion/button/PID/watchdog policy, transport concurrency,
  and cockpit protocol/calibration/control socket.
- Two Python protocol tests plus Python syntax checks.
- The production TX-gate implementation is tested against deterministic
  pthread/CAN fakes: started/queue-load interleaving, recovery exclusion,
  healthy-other-bus progress, self-test drain, and reconnect rejection of old
  normal traffic. These are software regressions, not physical driver tests.
- Zephyr4.1 Front/Rear builds succeeded. Pi C also compiled natively as ARM
  aarch64 in `/home/sfarhan/lab3-review-candidate`; it was not started.
- Candidate images and SHA-256 values are in ignored `build-checks/firmware/`
  and `build-checks/release-candidate.json`.

The documented nominal20 ms UDP-driven CTRL plus40 ms delayed-input fallback
was not judged a blocker. Real burst/jitter and fallback releases must be
included in the measured bus/RTA analysis. Laptop UDP timeout remains the
user-approved250 ms.

## Unverified on hardware

GPIO timing, physical fault transitions, independent watchdog reset behavior,
all scope deadlines and the report/RTA remain pending. The PID feedback is an
explicit software plant, not real encoder RPM. No physical closed-loop motor
claim is made. Use LAB3_COMPLETION.md and LAB3_MEASUREMENTS.csv for the next
bench phase. The review itself did not replace running firmware/services.
Deployment and the configuration correction happened afterward; the user
then authorized publication on a new branch.
