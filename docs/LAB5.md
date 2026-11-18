# Lab 5 additions on the team's `lab3-complete`

Start with [Run Lab 5](#run-lab-5). N1 is the Pi cockpit, N2 is the Front
steering board, and N3 is the Rear drive board. All three must run the same
Lab 5 revision. The base firmware build defaults to Lab 3.

## Scope and implementation

`lab5/` contains the portable Raft agent, state machine, client, clock model,
CAN framing and durable log. `pi/lab5_agent.c` and the optional STM32
`src/lab5_agent.c` connect it to the existing application. The application
selects outgoing business routes from its committed graph. Consensus and REF
traffic retain bounded duplicate-suppressed discovery forwarding on both
buses, so agreeing on a changed route does not depend on already knowing it.
All three nodes can forward in Lab5; this is not the old N2-only relay.

Legacy CTRL/BLINK/HB/FORCE/FAULT frames keep their original CAN IDs. They are
forwarded only when the committed graph shows that the third node needs a
hop. Flooding those frames from multiple bridges can leave both remaining
controllers transmitting the same ID, with no receiving node to acknowledge
it. Consensus and clock discovery retain their separate last-hop IDs and do
not depend on this application-forwarding decision.

Implemented requirements:

- Three fixed voters (majority=2), predefined cockpit leader on a fresh term1
  log without a boot election; random-election mode; RequestVote,
  AppendEntries/ACK, current-term majority commit, conflicting suffix repair,
  persistent term/vote and client request reservations. Pre-vote prevents an
  isolated minority from repeatedly increasing the majority's term.
- Entries have term/index/event. NODE_UP/DOWN, CANLINK_UP/DOWN and
  COCKPIT_OK/ERROR update a replicated graph. `l5_addLink`, `l5_dropLink`,
  `l5_addNode`, `l5_dropNode` implement the four mutators; `l5_get_network`
  exposes the committed graph. Removing a graph node does not shrink the
  Raft voter set or make a single remaining node a majority.
- Clients send to the known leader and handle follower redirects. Reports
  include the graph commit index they observed; a delayed observation cannot
  undo a newer topology transition. The state-reporting client rereads the
  graph and retries current observations after a stale-report rejection.
- Leader REF every20 ms, empty APPEND heartbeat every100 ms. N2 produces an
  EVENT once a second for the held-events/replay demonstration. This event's
  counter is a demonstration value, not a new sensor requirement.
- A newly elected node first obtains a shared epoch if it lacks a recent
  synchronized holdover. While `time_pending=1`, the one-frame consensus
  heartbeat runs every 20 ms. REF/LEADER/SYNC wait for a valid time master.
- Leader is time master. Followers compensate the configured hop delay and
  relay residence time, estimate relative drift and expose
  `l5_global_gettime()`. Zephyr followers update CLOCK_REALTIME; scheduling and
  watchdog time remain monotonic. Pi's operating-system wall clock is not
  changed. The shared epoch is the time master's boot-based monotonic time,
  not an externally calibrated UTC source.
- PPS at global whole seconds, LEADER/SYNC/REF_RX/HZ outputs, periodic global/
  local-time, drift-ratio and correction prints. Master handover preserves the
  local global-time value and slews correction instead of resetting the clock.
- Normal turn signals use a common activation epoch20 ms in the future, then
  a global500 ms half-period. This preserves prompt start while aligning the
  two controllers. Hazards use the absolute global phase:2 Hz with cockpit
  present,4 Hz with cockpit absent. Local USER/X self-test remains supported.
- Pi EVENT display is keyed by committed index. Missing display records are
  repaired from the durable log at startup; replayed entries are labeled and
  shown in index order. This avoids treating repeated terminal redraws as
  duplicate events.

The consensus rules follow the [Raft paper](https://raft.github.io/raft.pdf),
with the handout's predefined-leader optimization and separate REF heartbeat.
This is lab code, not a production consensus library or a proof of all faults.

## Shared configuration

Edit `lab5/config.json`, run `python3 tools/lab5_config.py`, and rebuild all
three nodes. Both targets include the same generated configuration. Build
commands reject a stale generated header. Defaults match the handout: N=3,
cockpit leader,20 ms heartbeat,3 missed heartbeats,0..39 ms extra election
jitter,30000 log entries and Debug=true. Debug prints election and APPEND RPCs.

Use `"default_leader": "none"` (or `"random"`) for random startup; `random_seed`
controls pseudo-random election jitter. Real boot order and transport delays
also affect the winner. Fresh logs use the predefined leader; a returning node
retains its durable term and rejoins as a follower instead of resetting to term1.
`startup_grace_ms=500` bounds the initial wait for an absent predefined leader;
the usual election timeout follows. It does not change steady-state heartbeat
or failure deadlines.

`path_delay_us[source-1][receiver-1]` is the one-hop delay correction. The
included350 us values are **initial assumptions**, not measurements. Calibrate
each path on the actual Pi/STM32 setup. A relayed REF adds the measured local
residence plus the prior hop's correction; the final recipient adds its hop.
No PTP/NTP round-trip latency estimator is substituted for this mechanism.

### Clock acquisition and recovery

A clock mismatch is not proof that a node is dead. On missing freshness evidence,
nodes issue a request with a durably reserved32-bit nonce. A reply proves liveness
only for the matching outstanding request, within150 ms; the recorded proof time
is the request's send time. Outstanding liveness requests survive elections,
while role/leader/term changes revoke their authority to change time.

An epoch reply includes the full timestamp and a separate nonce-tagged sample.
The sample receives the same one-way hop/residence correction as REF; request
round-trip duration is used only to reject stale replies, not to estimate delay.
Full nonce matching and bounded low32 timestamp reconstruction prevent mixing
samples from previous boots. A verified leader/term authorization lasts500 ms
and is renewed during recovery. Ordinary REF acceptance still checks freshness.

Established time is never stepped. Normal phase correction is limited to2500 ppm;
large verified offsets use at most5% slew while SYNC is low. A finite recovery
correction can finish in holdover even after the previous master disappears.
Comparable but contradictory epoch samples leave `time_conflict=1` and block
master activation instead of choosing an arbitrary epoch.

Fresh state on a majority permits initial epoch creation after a200 ms discovery
window. Historical cold nodes first seek a surviving holdover. If all clocks
were lost, all three nodes must confirm no retained epoch before a new epoch
can start; two historical cold nodes alone remain `time_pending=1`. This is a
deliberate distinction from ordinary one-node restart and does not erase logs.

## Persistence and capacity

Pi uses the named `raft.log` file with an exclusive writer lock and fsync.
The containing directory is synced before opening completes. Copy-on-write
publication syncs copied data before the new bank marker; entry and metadata
validity markers likewise follow durable data. STM32 programming is synchronous.
STM32 uses the same logical log through a raw-Flash backend, not a RAM array
or a POSIX filesystem. Its index is derived from the logical record position;
term/event and commit/applied markers are persisted. Entries include a compact
checksum; bank headers and metadata have CRC checks.

The optional Lab5 linker layout reserves:

- `0x08000000..0x0801ffff`:128 KiB firmware.
- `0x08020000..0x0807ffff`:384 KiB log storage (two data banks plus one
  copy-on-write scratch bank).

This stores30000 entries without consuming30000 entries of MCU RAM. Persistent
client IDs are reserved in blocks to avoid a metadata write per generated
event. This implementation fails closed on capacity exhaustion; it does not
silently overwrite committed history. Embedded log terms are limited to65534
and a node's reserved client request IDs to8388607; these are explicit finite
storage-format limits.

**Do not flash this as if it used the previous full-Flash layout.** First use
formats the reserved log region. Keep all original hardware wiring, verify the
new signal pins are free, and back up any unrelated data in that region before
deployment. Do not erase logs for a normal power-off/rejoin test. A fresh demo
cluster needs fresh state on all three nodes; on Pi use a new state directory
(including a new `.events` directory), not a partially deleted old log.

Internal-Flash erase/compaction can stall execution and may interact with the
watchdog, election timing and PPS. Normal appends avoid erasing, but conflicting
suffix repair or metadata-bank recycling can require it. **Worst-case hardware
erase timing is not validated; this is a deployment/timing risk to measure,
not a claimed timing pass.**

## Run Lab 5

**Get the branch.** On the build computer and Pi, clone the repository below.
The Windows wheel sender also runs from this checkout. Use the existing
[dual-CAN wiring and Pi 4 setup](LAB3_TESTING_TUTORIAL.md), then connect the
[Lab 5 signals](#added-signal-pins). Both buses run at 500 kbit/s.

```sh
git clone --branch codex/lab5-time-consensus --single-branch https://github.com/ShariaFarhan/18649-T14-lab2.git
cd 18649-T14-lab2
```

**Build and flash both STM32s.** In a configured Zephyr 4.1 workspace, replace
`/absolute/path/to/repo` below with the checkout path.

```sh
west build -b nucleo_f401re /absolute/path/to/repo/stm32/lab3_app \
  -d build-lab5-front -- -DNODE_ID=2 -DLAB5_ENABLED=ON
west build -b nucleo_f401re /absolute/path/to/repo/stm32/lab3_app \
  -d build-lab5-rear -- -DNODE_ID=3 -DLAB5_ENABLED=ON
```

Disconnect actuator power and close serial monitors. Plug in one board at a
time through ST-LINK USB. Copy `build-lab5-front/zephyr/zephyr.bin` to the
Front board's Nucleo USB drive and `build-lab5-rear/zephyr/zephyr.bin` to the
Rear board's drive. Wait for each write to finish. The 115200-baud consoles
identify N2 and N3. Keep the existing flash log for restart/rejoin tests.

**Start the Pi.** From the repo root on the Pi, with GCC, Python 3 and GPIO
access, build the cockpit. Check the CAN interfaces after each Pi reboot.

```sh
sh tools/build_lab5_pi.sh
ip -br link show type can
readlink /sys/class/net/can0/device  # A: spi0.0
readlink /sys/class/net/can1/device  # B: spi0.1
```

For each interface that is DOWN, run its corresponding command:

```sh
sudo ip link set can0 up type can bitrate 500000 restart-ms 20
sudo ip link set can1 up type can bitrate 500000 restart-ms 20
```

Stop any previous cockpit before starting this one. Replace `YOUR_WINDOWS_IP`
with the sender's address as seen by the Pi (the PC's Tailscale address if
using Tailscale). Keep this terminal open and reuse the same state directory.

```sh
mkdir -p build-lab5/state
./build-lab5/cockpit --lab5-log "$PWD/build-lab5/state/raft.log" \
  --pc-ip YOUR_WINDOWS_IP --steer 0,33151,65535 \
  --throttle 65535,0 --brake 65535,0
```

**Start the wheel sender.** Connect the G920 to Windows. From the repo root
in PowerShell, replace `YOUR_PI_IP` with the reachable Pi address. Keep this
terminal open and gently move the wheel to obtain its first real sample.

```powershell
New-Item -ItemType Directory -Force build-logs | Out-Null
python tools/windows_wheel_udp.py --pi YOUR_PI_IP --status build-logs/wheel-status.json
```

**Verify.** The Pi should show `NORMAL` after valid input and a healthy cluster.
If it shows `UDP_LOST`, check that `wheel-status.json` has `ready=true`, an
increasing `sent` count and a source IP matching `--pc-ip`. In a second Pi
terminal at the repo root:

```sh
python3 tools/lab5_capture.py --seconds 60 --output build-lab5/trace.json | grep NODE_STATUS
python3 tools/lab5_display.py build-lab5/state/raft.log --once
```

`LAB5 NETWORK` prints leader, term, commit index, committed-log hash, node
bitmap and the three link masks (A=1, B=2, both=3). Compare the same commit
index/hash/graph across nodes after convergence. `LAB5 LOG` prints entry
identity and live/replayed. `LAB5 PPS` prints global/local microseconds,
drift ratio, correction and SYNC. The display contains one row per EVENT index.
Both STM32 `NODE_STATUS` records and the Pi's `NETWORK` lines should agree on
one nonzero leader and term throughout the observation. With all branches
connected, expect `nodes=0xe links=3/3/3` after convergence. Compare hashes
at the same commit index (CAN status carries the low 16 bits). Confirm that
new EVENTs advance the commit index. For the physical PPS check, trigger on
the leader's rising edge and measure each follower relative to that edge.

## Added signal pins

All existing Lab3 GPIO assignments stay in place. These are additions only.
Check that no retained Lab2 sensor uses the selected pins before wiring.

| Signal | Both Nucleos | Pi BCM / physical |
|---|---|---|
| PPS | PC8, CN10 pin2 | GPIO12 / pin32 |
| LEADER | PC9, CN10 pin1 | GPIO16 / pin36 |
| SYNC | PC5, CN10 pin6 | GPIO20 / pin38 |
| REF_RX | PC4, CN10 pin34 | GPIO21 / pin40 |
| HZ | PC2, CN7 pin35 | GPIO26 / pin37 |
| Additional Pi relay marker | — | GPIO19 / pin35 |

Front D6 retains its existing RELAY_TX indication. Grounds are common; LED
indicators need their own series resistors. PPS pulse width is1 ms; synchronization
is evaluated from rising-edge alignment, not pulse width. GPIO scheduling,
kernel/driver latency and compensation calibration still require scope tests.

## New CAN format

Lab3 IDs and actuator payloads are retained. In Lab5 BLINK only, spare bytes3..6
carry the common activation epoch (low32 bits of global milliseconds) and
byte7 is0xA5. Receivers compute phase from global time rather than from arrival.

- REF ID=`0x040 + leader*4 + last_hop`; bytes: cycle8, term-low8, global-us48.
- REF_ACK ID=`0x080 + origin*16 + destination*4 + last_hop`; bytes: term16
  and echoed reference-time48. Fresh time is checked before
  refreshing liveness; delayed legacy heartbeats do not revive a failed node.
- Every heartbeat period the elected leader broadcasts a monotonic cookie
  (base 0x030); followers echo it (base 0x060). These use term16 and cookie48
  and IDs include source*4 + last_hop. The leader accepts a recent echo of
  one of its own last four cookies. This maintains quorum while acquiring
  time or catching up a log without asserting clock synchronization.
- PRE_VOTE, PRE_VOTE_REPLY, VOTE, VOTE_REPLY and APPEND_REPLY use the compact
  ID range 0x340..0x47f. Each fits one frame (term16, index16, previous-term
  or rejected previous-index16, status8, transaction8). APPEND, CLIENT and
  CLIENT_REPLY still use RPC ID `0x500 + (type-1)*16 + last_hop`.
- Front and Rear broadcast one diagnostic frame per second at 0x02a/0x02b.
  It contains their local leader, flags (SYNC bit0, fatal bit1, time pending
  bit2, degraded bus mask bits4..5), term16, commit16 and log-hash low16.
- TIME_QUERY/TIME_PROOF use base`0x280`/`0x2c0` plus
  `origin*16 + destination*4 + last_hop`: term16, nonce32, flags8, reserved8.
  Flags:1=usable epoch,2=pristine startup,4=retained epoch. The compact proof
  has no usable epoch; it keeps clock acquisition from flooding bulk RPC queues.
- TIME_REPLY (type11) uses six fragments at`0x310 + last_hop`, ahead of bulk
  log traffic. Fields: index=nonce, event/aux=global-us low/high32,
  commit=holdover age-us, ok=epoch flags, term=matched request term.
- TIME_SAMPLE uses base`0x240 + origin*16 + destination*4 + last_hop`:
  nonce32 and compensated global-us low32. It is accepted only with its matching
  TIME_REPLY. REF_ACK is also forwarded by a destination leader acting as bridge,
  so peers on opposite buses retain liveness evidence.
- RPCs are six8-byte CAN fragments. Byte0 carries destination2/source2/
  fragment4, byte1 is transaction8, and six remaining bytes carry the fragment.
  The reassembled message contains source/destination/type/status, term,
  index, previous term, commit, event and auxiliary value, with CRC32.

The last-hop ID changes when forwarding so a corrected REF timestamp does
not contend under the origin's identical CAN ID with different data.
Receive caches suppress duplicate fragments and loops. Per-bus nonblocking
queues preserve order within an ID, prioritize REF and expire stale traffic.
Heartbeats/control/REF/ACK software queues expire after20 ms; RPC fragments
can wait up to200 ms. Raft retries incomplete RPCs. Hardware-queued frames and
fault recovery still require physical verification.

## Tests completed and remaining checks

Software tests cover default leader, random seeds, current-term majority
commit, redirects/client graph updates, stale observations, cross-bus routes,
node loss/rejoin, term stability on link loss, fragmented CAN with relay,
STM32-master sync through Pi, whole-bus loss, duplicate/out-of-order/lost RPCs,
repeated crashes,30000-entry storage, interrupted writes, metadata recycling,
index-keyed display repair, drift/monotonic slew and common blink activation.
Additional regressions cover all three sole-bridge leaders, absent-default
startup, independent local-clock reset, a cold elected leader inheriting a
holdover, +/-65 ms isolation drift, stale boot nonces, historical cold-cluster
recovery, completion of a recovering holdover after master loss, and either
STM32 leader failing without declaring a communicating cockpit dead. A reordered
page-write model reproduces43 failures without barriers and zero committed-prefix
losses across180 crash cut points with the publication barriers.
The original Lab3 host regressions also pass. These are **not hardware timing
measurements or a formal Raft proof**.

The CAN arbitration regression additionally models simultaneous transmitters,
retained controller TX slots, and the need for a non-transmitting ACK receiver.
It checks 18 single-link and 36 ordered hop faults with each possible leader,
including Front B followed by Pi A, plus restoration and continued CTRL
delivery. This model does not validate electrical behavior or hardware timing.

Still measure on hardware: PPS spread, FL/RL<=1 ms over ten edges,
leader replacement<=1 s, Pi-loss fail-safe<=100 ms, handover edge disturbance
<=100 us, REF under relayed topology, Flash erase pauses and watchdog behavior.
Record actual values; do not replace them with the350 us delay assumption.

Two handout details need TA clarification:

1. Checkoff says losing a non-cockpit STM32 leader causes no failure mode,
   but also describes isolation as "as in Lab3." Without extra information,
   power-off and complete link isolation can look identical to the other nodes.
   This implementation follows the explicit Lab5 cockpit-failure policy:
   cockpit loss or explicit USER/X fault causes vehicle error; an isolated
   node itself fails safe, while a remaining majority with a live cockpit may
   continue. It does not claim to satisfy contradictory interpretations.
2. Body events NODE/CANLINK and checkoff MEMBER are represented by the same
   committed topology entries here. EVENT is the N2 demonstration counter.
   Confirm if the instructor expects a different EVENT payload/API.

Keep at least two communicating voters for election/commit tests. Restore a
failed node before removing a second one; one survivor cannot elect itself
by changing N. Cloud Redis/GUI is optional extra credit and is not included.
