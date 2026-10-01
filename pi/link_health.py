"""Separate source-data and UART-return deadlines; no actuator side effects."""
UDP_TIMEOUT_S=.150
FEEDBACK_TIMEOUT_S=.150
COMMAND_PERIOD_S=.020

def health_snapshot(now,last_rx,last_hb,last_tx):
    udp_stale=not last_rx or now-last_rx>UDP_TIMEOUT_S
    feedback_stale=not last_hb or now-last_hb>FEEDBACK_TIMEOUT_S
    return dict(udp_stale=udp_stale,feedback_stale=feedback_stale,
                udp_age_ms=round((now-last_rx)*1000,2) if last_rx else None,
                feedback_age_ms=round((now-last_hb)*1000,2) if last_hb else None,
                uart_tx_gap_ms=round((now-last_tx)*1000,2) if last_tx else None)

def advance_deadline(deadline,now,period=COMMAND_PERIOD_S):
    if now<deadline:return deadline
    return deadline+(int((now-deadline)/period)+1)*period
