/* Dual-CAN dispatch, health, split lights and zone-specific steering/drive outputs. */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include "lab3_can_rx.h"
#include "lab3_fault_policy.h"
#include "blinkers.h"
#include "system_state.h"
#include "runtime.h"
#include "transport_gate.h"
#if NODE_ID == 2
#include "steer_task.h"
#include "relay_task.h"
#include "force_task.h"
#else
#include "drive_task.h"
#include "test_points.h"
#endif

struct bus {
    const struct device *dev;
    unsigned index;
    bool ready;
    struct k_mutex tx_gate;
    atomic_t tx, tx_errors, dropped;
    uint32_t previous_tx, recoveries;
    int64_t last_progress, last_recovery;
    enum can_state controller_state;
    struct can_bus_err_cnt errors;
    bool tx_stalled;
};
static struct bus buses[] = {
    {.dev = DEVICE_DT_GET(DT_ALIAS(can_a)), .index = 0},
    {.dev = DEVICE_DT_GET(DT_ALIAS(can_b)), .index = 1},
};
struct received { struct can_frame frame; unsigned bus; };
K_MSGQ_DEFINE(received_q, sizeof(struct received), 64, 4);
static struct lab3_can_rx rx;
static const struct gpio_dt_spec leds[] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(degraded_a), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(degraded_b), gpios),
};
static struct lab3_health health;
static bool leds_ready;
static uint8_t led_bits = 255;
static struct system_state system_status;
static bool ctrl_seen, ctrl_stale = true, blink_seen, use_master_phase;
static uint64_t ctrl_at, blink_at, remote_fault_until;
static int16_t steering;
static uint8_t throttle, brake, ctrl_buttons, ctrl_flags;
static uint8_t blink_mode, blink_phase, peer_state[3];
static uint32_t ctrl_count, blink_count;
static uint8_t pending_reason, fault_seq, fault_pending;
static uint64_t fault_at;
static struct can_frame fault_frame, hb_frame;
static uint8_t hb_pending;
#define CTRL_TIMEOUT_MS 100u
#define BLINK_TIMEOUT_MS 750u
static void abort_local_tx(void);
static void local_button_action(bool failed)
{
    if (!failed) return; /* Main requires a fresh CTRL before resuming. */
#if NODE_ID == 2
    relay_task_pause();steer_task_submit(0,false,0);
#else
    drive_task_submit(0,0,false,0);
#endif
    abort_local_tx();
}

static void dispatch_frame(const struct received *item, uint64_t now)
{
    const struct can_frame *f = &item->frame;
    lab3_rx_mark(item->bus); /* Every dispatch, including duplicate copies. */
    if (f->flags & (CAN_FRAME_IDE | CAN_FRAME_RTR)) return;
    enum lab3_rx_result result = lab3_receive(&rx,f->id,item->bus,f->data,f->dlc,now);
#if NODE_ID == 2
    relay_task_observe(f,item->bus,result,now);
#endif
    if (result == LAB3_RX_INVALID && f->id == CAN_ID_CTRL) {
        system_state_bad_input(&system_status);
        if (!lab3_local_failed()) pending_reason=FAULT_R_BAD_INPUT;
        return;
    }
    if (result != LAB3_RX_NEW) return;
    int peer=lab3_hb_peer(f->id);
    if (peer>=0) { peer_state[peer]=f->data[HB_STATE]; return; }
    if (f->id==CAN_ID_CTRL) {
#if NODE_ID == 3
        if (test_points_command_received()) {
            system_state_output_failed(&system_status);pending_reason=FAULT_R_OUTPUT;
        }
#endif
        steering=(int16_t)((uint16_t)f->data[1] | ((uint16_t)f->data[2]<<8));
        throttle=f->data[CTRL_THROTTLE]; brake=f->data[CTRL_BRAKE];
        ctrl_buttons=f->data[CTRL_BUTTONS]; ctrl_flags=can_flags_of(f->data);
        ctrl_stale=!!(ctrl_flags & CTRL_FLAG_UDP_STALE);
        if (!ctrl_stale) {
            ctrl_seen=true; ctrl_at=now; system_state_valid(&system_status);
        }
        ++ctrl_count;
    } else if (f->id==CAN_ID_BLINK) {
        blink_mode=f->data[BLINK_MODE]; blink_phase=f->data[BLINK_PHASE];
        blink_seen=true; blink_at=now; ++blink_count;
    } else if (f->id==CAN_ID_FAULT && f->data[FAULT_ZONE]!=NODE_ID) {
        remote_fault_until=now+100;
    }
}

static void update_control_and_lights(uint64_t now)
{
    if (lab3_runtime_failed() && !system_status.output_fault) {
        system_state_output_failed(&system_status);pending_reason=FAULT_R_OUTPUT;
    }
#if NODE_ID == 2
    if ((steer_task_failed() || relay_task_failed()) && !system_status.output_fault) {
        system_state_output_failed(&system_status);
        pending_reason=FAULT_R_OUTPUT;
    }
#else
    if (drive_task_failed() && !system_status.output_fault) {
        system_state_output_failed(&system_status);
        pending_reason=FAULT_R_OUTPUT;
    }
#endif
    bool peer_fault=false;
    for (unsigned p=0;p<3;++p)
        if (p!=NODE_ID-1 && peer_state[p]==ZONE_STATE_LOCAL_FAULT &&
            (lab3_hb_fresh(&rx,p,0,now)||lab3_hb_fresh(&rx,p,1,now))) peer_fault=true;
    system_status.link_ok=ctrl_seen && !ctrl_stale && now-ctrl_at<CTRL_TIMEOUT_MS &&
                          !health.missing_peers && !peer_fault && now>=remote_fault_until;
    enum system_reason reason=system_state_reason(&system_status);
    lab3_err(reason!=SYS_OK);
#if NODE_ID == 2
    steer_task_submit(steering, reason==SYS_OK, ctrl_count);
#else
    drive_task_submit(throttle,brake,reason==SYS_OK,ctrl_count);
#endif
    bool fresh_blink=blink_seen && now-blink_at<BLINK_TIMEOUT_MS;
    bool master_live=lab3_hb_fresh(&rx,0,0,now)||lab3_hb_fresh(&rx,0,1,now);
    bool follow_master=lab3_follow_master(fresh_blink,master_live,reason==SYS_OK,blink_mode);
    int rc=0;
    if (follow_master) {
        static uint8_t applied_mode=255,applied_phase=255;
        if (!use_master_phase || applied_mode!=blink_mode || applied_phase!=blink_phase) {
            rc=blinkers_apply_phase((enum blinker_mode)blink_mode,blink_phase!=0);
            applied_mode=blink_mode;applied_phase=blink_phase;
        }
        use_master_phase=true;
    } else if (reason!=SYS_OK) {
        if (use_master_phase || blinkers_get_mode()!=BLINKER_MODE_HAZARD)
            rc=blinkers_set_error(true);
        use_master_phase=false;
    } else {
        if (use_master_phase || blinkers_get_mode()!=BLINKER_MODE_OFF)
            rc=blinkers_apply_phase(BLINKER_MODE_OFF,false);
        use_master_phase=false;
    }
    if (rc) { system_state_output_failed(&system_status); pending_reason=FAULT_R_OUTPUT; }
}

static void update_health(uint64_t now)
{
    health = lab3_health_get(&rx, NODE_ID - 1, now);
    for (unsigned b = 0; b < 2; ++b) {
        k_mutex_lock(&buses[b].tx_gate,K_FOREVER);
        bool degraded=!buses[b].ready || buses[b].tx_stalled ||
                      buses[b].controller_state==CAN_STATE_BUS_OFF;
        k_mutex_unlock(&buses[b].tx_gate);
        if (degraded) health.degraded|=1u<<b;
    }
    uint8_t display = health.degraded;
    if (leds_ready && led_bits != display) {
        int a = gpio_pin_set_dt(&leds[0], !!(display & 1));
        int b = gpio_pin_set_dt(&leds[1], !!(display & 2));
        if (!a && !b) led_bits = display;
        else printk("DEG output failed %d/%d\n", a, b);
        printk("N%d DEG_A=%d DEG_B=%d missing_peers=0x%x isolated=%d\n", NODE_ID,
            !!(health.degraded & 1), !!(health.degraded & 2), health.missing_peers, health.isolated);
    }
}

static void tx_done(const struct device *dev, int error, void *arg)
{
    ARG_UNUSED(dev);
    struct bus *b = arg;
    if (error) atomic_inc(&b->tx_errors); else atomic_inc(&b->tx);
}
static bool frame_allowed(const struct can_frame *frame)
{ return lab3_tx_allowed(lab3_local_failed(),NODE_ID,frame->id,frame->data); }
static void queue_marked(void *arg)
{ lab3_tx_mark(((struct bus *)arg)->index); }
static int node_send(unsigned bus,const struct can_frame *frame)
{
    struct bus *b=&buses[bus];
    return lab3_transport_send(&b->tx_gate,b->dev,&b->ready,frame,frame_allowed,tx_done,b,queue_marked);
}
static void abort_local_tx(void)
{
    /* TEST_BTN thread, never ISR. Drain old hardware TX under the same
     * per-controller gate used by every send and recovery operation. */
    for (unsigned i=0;i<ARRAY_SIZE(buses);i++) {
        struct bus *b=&buses[i];
        k_mutex_lock(&b->tx_gate,K_FOREVER);
        if (b->ready) (void)lab3_transport_restart_locked(b->dev,&b->ready);
        b->last_progress=b->last_recovery=k_uptime_get();
        b->previous_tx=(uint32_t)atomic_get(&b->tx);b->tx_stalled=false;
        k_mutex_unlock(&b->tx_gate);
    }
}
#if NODE_ID == 2
static int relay_send(unsigned bus,const struct can_frame *frame)
{
    /* Include relay completions in bus progress so recovery does not mistake
     * a busy forwarding controller for a stalled controller. */
    return node_send(bus,frame);
}
#endif
static void receive(const struct device *dev, struct can_frame *frame, void *arg)
{
    ARG_UNUSED(dev);
    struct bus *b = arg;
    struct received item = {.frame = *frame, .bus = b->index};
    if (k_msgq_put(&received_q, &item, K_NO_WAIT)) atomic_inc(&b->dropped);
}
static void recover_controllers(int64_t now)
{
    for (unsigned i = 0; i < ARRAY_SIZE(buses); ++i) {
        struct bus *b = &buses[i];
        if (!device_is_ready(b->dev)) continue;
        if (k_mutex_lock(&b->tx_gate,K_NO_WAIT)) continue;
        uint32_t done = (uint32_t)atomic_get(&b->tx);
        if (done != b->previous_tx) { b->previous_tx = done;b->last_progress = now;b->tx_stalled=false; }
        int rc = can_get_state(b->dev, &b->controller_state, &b->errors);
        if (lab3_local_failed()) { b->last_progress=now;b->tx_stalled=false; }
        else if (now-b->last_progress>=LAB3_HEARTBEAT_TIMEOUT_MS) b->tx_stalled=true;
        bool recover = !b->ready || b->tx_stalled ||
                       (!rc && b->controller_state == CAN_STATE_BUS_OFF);
        /* A missing ACK can leave MCP2515 error-passive with all TX slots held;
         * it need not reach bus-off. Stop aborts TX and releases driver slots.
         * Retry only the affected controller, with staggered bounded backoff. */
        int64_t retry_ms = 60 + NODE_ID * 2 + (int)i * 3;
        if (!recover || now - b->last_recovery < retry_ms) {
            k_mutex_unlock(&b->tx_gate);continue;
        }
        b->last_recovery = now;
        int restart=lab3_transport_restart_locked(b->dev,&b->ready);
        /* A restarted controller stays degraded until actual TX progress.
         * The staggered >=60 ms retry gives it an observation window. */
        ++b->recoveries;
        k_mutex_unlock(&b->tx_gate);
        if (b->recoveries == 1 || b->recoveries % 25 == 0)
            printk("N%d CAN_%c recover=%u state=%d TEC=%u REC=%u restart=%d\n",
                NODE_ID, 'A'+i, b->recoveries, b->controller_state,
                b->errors.tx_err_cnt, b->errors.rx_err_cnt, restart);
    }
}
static void heartbeat(uint8_t seq, uint64_t now)
{
    ARG_UNUSED(now);
    struct can_frame frame = {.id = CAN_ID_HB_N1 + NODE_ID - 1, .dlc = CAN_FRAME_DLC};
    enum system_reason reason=system_state_reason(&system_status);
    frame.data[0] = can_byte0(seq, reason==SYS_OK ? 0 : CAN_FLAG_SYS_ERROR);
    frame.data[HB_STATE] = reason==SYS_OK ? ZONE_STATE_NORMAL :
        (reason==SYS_BAD_INPUT || reason==SYS_OUTPUT_FAULT) ? ZONE_STATE_LOCAL_FAULT : ZONE_STATE_SYS_ERROR;
    frame.data[HB_REASON] = reason;
    frame.data[HB_BUS] = health.degraded;
#if NODE_ID == 3
    struct lab3_drive_status drive=drive_task_status();
    frame.data[HB_DUTY]=drive.duty;
    /* Scope-only output: do not advertise the Lab2 physical-drive armed bit. */
    frame.data[HB_DRIVE_FLAGS]=(drive.braking ? 2u : 0u) | (drive.fault ? 4u : 0u);
#endif
    /* Previously unused bytes expose logical HB deliveries for stage-1 verification. */
    uint16_t applied = lab3_hb_applied_total(&rx);
    frame.data[6] = (uint8_t)applied; frame.data[7] = (uint8_t)(applied >> 8);
    hb_frame=frame;hb_pending=3;
}
static void send_pending_heartbeat(void)
{
    if (lab3_local_failed()) { hb_pending=0;return; }
    for (unsigned b=0;b<2;++b) {
        if (!(hb_pending&(1u<<b))) continue;
        int rc=node_send(b,&hb_frame);
        if (!rc) hb_pending &= ~(1u<<b);
        else if (rc != -EAGAIN) atomic_inc(&buses[b].tx_errors);
    }
}
static void send_pending_fault(uint64_t now)
{
    if (fault_pending && now-fault_at>=100) fault_pending=0;
    if (!fault_pending && pending_reason && now-fault_at>=20) {
        memset(&fault_frame,0,sizeof(fault_frame));
        fault_frame.id=CAN_ID_FAULT;fault_frame.dlc=8;
        fault_frame.data[0]=can_byte0(fault_seq++,CAN_FLAG_SYS_ERROR);
        fault_frame.data[FAULT_ZONE]=NODE_ID;fault_frame.data[FAULT_REASON]=pending_reason;
        pending_reason=0;fault_pending=3;fault_at=now;
    }
    for (unsigned b=0;b<2;++b)
        if ((fault_pending&(1u<<b)) &&
            !node_send(b,&fault_frame)) fault_pending &= ~(1u<<b);
}
int main(void)
{
    printk("\nLAB3 CTRL + BLINK N%d; left D14/right D15\n", NODE_ID);
    for (unsigned b=0;b<ARRAY_SIZE(buses);b++) k_mutex_init(&buses[b].tx_gate);
    int runtime_rc=lab3_runtime_init(local_button_action);
    leds_ready = true;
    for (unsigned b = 0; b < 2; ++b) {
        if (!gpio_is_ready_dt(&leds[b]) || gpio_pin_configure_dt(&leds[b], GPIO_OUTPUT_INACTIVE))
            leds_ready = false;
    }
    int lights_rc=blinkers_init();
    int actuator_rc=0;
#if NODE_ID == 2
    actuator_rc=steer_task_init();
    printk("STEER: D7/PA8, 50 Hz, 1000/1500/2000 us; error centers; init=%d\n",actuator_rc);
#else
    actuator_rc=drive_task_init();
    printk("DRIVE scope outputs: PWM D6/D7 20kHz; DIR A0/A1,A2/A3; CTRL_RX A4; PWM_SET D8; init=%d\n",actuator_rc);
#endif
    system_state_init(&system_status,!runtime_rc && leds_ready && lights_rc==0 && actuator_rc==0);
    if (!lights_rc) (void)blinkers_set_error(true);
    for (unsigned b = 0; b < 2; ++b) {
        struct bus *bus = &buses[b];
        k_mutex_lock(&bus->tx_gate,K_FOREVER);
        if (!device_is_ready(bus->dev)) {
            k_mutex_unlock(&bus->tx_gate);printk("CAN_%c NOT READY\n", 'A' + b);continue;
        }
        int rc = can_set_mode(bus->dev, CAN_MODE_NORMAL);
        if (!rc) rc = can_set_bitrate(bus->dev, 500000);
        for (unsigned p = 0; !rc && p < 3; ++p) {
            struct can_filter f = {.id = CAN_ID_HB_N1 + p, .mask = CAN_STD_ID_MASK};
            int filter = can_add_rx_filter(bus->dev, receive, bus, &f);
            if (filter < 0) rc = filter;
        }
        const uint16_t app_ids[]={CAN_ID_CTRL,CAN_ID_BLINK,CAN_ID_FAULT,CAN_ID_FORCE};
        for (unsigned p=0;!rc && p<ARRAY_SIZE(app_ids);++p) {
            struct can_filter f={.id=app_ids[p],.mask=CAN_STD_ID_MASK};
            int filter=can_add_rx_filter(bus->dev,receive,bus,&f);
            if (filter<0) rc=filter;
        }
        if (!rc) rc = can_start(bus->dev);
        bus->ready = rc == 0;
        bus->controller_state = bus->ready ? CAN_STATE_ERROR_ACTIVE : CAN_STATE_STOPPED;
        bus->last_progress = k_uptime_get();
        k_mutex_unlock(&bus->tx_gate);
        printk("CAN_%c init=%d\n", 'A' + b, rc);
    }
#if NODE_ID == 2
    int relay_rc=relay_task_init(relay_send);
    if (relay_rc) { system_state_output_failed(&system_status);pending_reason=FAULT_R_OUTPUT; }
    printk("RELAY: N2 only, original payload, priority 4, D6 marker; init=%d\n",relay_rc);
    force_task_init(node_send);
#endif
    uint32_t workers=BIT(WORK_MAIN)|BIT(WORK_CONTROL)|BIT(WORK_BUTTON);
#if NODE_ID == 2
    workers|=BIT(WORK_RELAY)|BIT(WORK_FORCE);
#else
    workers|=BIT(WORK_PID);
#endif
    int watchdog_rc=lab3_watchdog_start(workers);
    if (watchdog_rc) { system_state_output_failed(&system_status);pending_reason=FAULT_R_OUTPUT; }
    printk("USER button single=fail/double=restore; ERR=A5; independent watchdog init=%d\n",watchdog_rc);
    int64_t next_hb = k_uptime_get(), next_status = next_hb, next_recovery = next_hb;
    uint8_t seq = 0;
    for (;;) {
        struct received item;
        int64_t now = k_uptime_get();
        int64_t wait = next_hb > now ? next_hb - now : 0;
        if (wait > 1) wait = 1; /* Refresh LED health even when both receive queues are quiet. */
        if (!k_msgq_get(&received_q, &item, K_MSEC(wait))) {
            lab3_profile_begin(PROFILE_DISPATCH);
            dispatch_frame(&item,(uint64_t)k_uptime_get());
            lab3_profile_end(PROFILE_DISPATCH);
        }
        now = k_uptime_get();
        bool local_failed=lab3_local_failed();
        if (system_status.self_test!=local_failed) {
            system_status.self_test=local_failed;hb_pending=0;fault_pending=0;
            if (local_failed) pending_reason=FAULT_R_SELF_TEST;
            else { ctrl_seen=false;system_status.seen_valid=false; }
        }
        if (now >= next_recovery) {
            recover_controllers(now);
            next_recovery = k_uptime_get() + 10;
        }
        lab3_profile_begin(PROFILE_HEALTH);
        update_health(now);
        update_control_and_lights(now);
        lab3_profile_end(PROFILE_HEALTH);
        if (now >= next_hb) {
            if (!local_failed) {
                heartbeat(seq, now);
                seq = (seq + 1) & CAN_SEQ_MASK;
            }
            next_hb += ((now - next_hb) / 20 + 1) * 20;
        }
        send_pending_fault(now);
        lab3_profile_begin(PROFILE_HB);
        send_pending_heartbeat();
        lab3_profile_end(PROFILE_HB);
        lab3_watchdog_poll();
        if (now >= next_status) {
            printk("N%d new=%u dup=%u repeat=%u old=%u conflict=%u invalid=%u TX=%ld/%ld TXERR=%ld/%ld DROP=%ld/%ld\n",
                NODE_ID, rx.counts[LAB3_RX_NEW], rx.counts[LAB3_RX_DUPLICATE],
                rx.counts[LAB3_RX_REPEAT], rx.counts[LAB3_RX_OLD],
                rx.counts[LAB3_RX_CONFLICT], rx.counts[LAB3_RX_INVALID],
                (long)atomic_get(&buses[0].tx), (long)atomic_get(&buses[1].tx),
                (long)atomic_get(&buses[0].tx_errors), (long)atomic_get(&buses[1].tx_errors),
                (long)atomic_get(&buses[0].dropped), (long)atomic_get(&buses[1].dropped));
            for (unsigned p = 0; p < 3; ++p) if (p != NODE_ID - 1)
                printk("  peer N%u applied=%u A=%u B=%u fresh=%d/%d\n", p+1,
                    rx.hb_applied[p], rx.hb_bus_count[p][0], rx.hb_bus_count[p][1],
                    lab3_hb_fresh(&rx,p,0,now), lab3_hb_fresh(&rx,p,1,now));
            printk("  CAN states=%d/%d TEC=%u/%u recoveries=%u/%u\n",
                buses[0].controller_state,buses[1].controller_state,
                buses[0].errors.tx_err_cnt,buses[1].errors.tx_err_cnt,
                buses[0].recoveries,buses[1].recoveries);
            printk("  CTRL applied=%u steer=%d throttle=%u brake=%u buttons=%u flags=%u state=%s BLINK applied=%u mode=%u phase=%u\n",
                ctrl_count,steering,throttle,brake,ctrl_buttons,ctrl_flags,
                system_reason_name(system_state_reason(&system_status)),blink_count,blink_mode,blink_phase);
#if NODE_ID == 2
            printk("  SRV pulse_us=%u output_fault=%d\n",steer_task_pulse_us(),steer_task_failed());
            struct relay_stats relay=relay_task_stats();
            printk("  RELAY queued_to_A/B=%u/%u failed=%u/%u offered=%u suppressed=%u superseded=%u expired=%u\n",
                relay.queued[0],relay.queued[1],relay.failed[0],relay.failed[1],
                relay.offered,relay.suppressed,relay.superseded,relay.expired);
#else
            struct lab3_drive_status drive=drive_task_status();
            struct test_point_counts points=test_points_counts();
            printk("  DRIVE duty=%u brake=%d output_fault=%d CTRL_RX=%u PWM_SET=%u\n",
                   drive.duty,drive.braking,drive.fault,points.command_rx,points.pwm_set);
#endif
            next_status = now + 1000;
        }
    }
}
