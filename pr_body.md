## Buddy 1 — WiFi, command & telemetry: finish the TODOs

Completes the remaining Buddy 1 deliverables. **All real-radio code is gated behind a default-off `RC_NET_ENABLE`**, so the graded mission image builds, links and behaves exactly as before and pulls in no lwIP/pico network headers.

### What's implemented
- **UDP sink** (`subsystems/sub_telemetry_udp.c`) — one datagram per message; the "prove the WiFi path, no broker" transport.
- **MQTT sink + command subscribe** (`subsystems/sub_telemetry_mqtt.c`) — publishes over the lwIP MQTT app client and subscribes to `car/01/cmd`. QoS 0 publish, QoS 1 command.
- **Shared WiFi bring-up** (`subsystems/net_wifi.{c,h}`) — CYW43 association used by both sinks.
- **Connection recovery** (`sub_telemetry.c` → `telemetry_link_ok()`) — per-tick `is_up()` check; on a drop: close, doubling `tk_dly_tsk` backoff (`RC_NET_BACKOFF_MIN..MAX_MS`), single reopen. No busy loop; only the lowest-priority telemetry task ever waits.
- **Command path** — MQTT decodes an inbound command → `sub_telemetry_deliver_command()` → publishes `RC_EVT_COMMAND_RX`. `sub_nav` handles it and gains `sub_nav_inject_command()`. Remote **STOP** works from any state; other commands run only while line-following. Nav state stays owned by one task.
- **Docs** — `docs/buddy1-telemetry/MQTT.md` (topics, schemas, enable steps, test procedure) and `BUILD.md` §5.11 "Turning on WiFi telemetry".

### Design notes
- Uses the core's pre-reserved `RC_EVT_COMMAND_RX` / `RC_EVT_LINK_STATE` events and `rc_pl_command_t` — no shared-type churn.
- `RC_TOPIC_BASE` shared by publish topics and the command topic so they can't drift.
- Framing/topics/scheduling in `sub_telemetry.c` unchanged — new transports only fill the existing `sub_telemetry_sink_t`.

### Reviewer notes / caveats
- **Not ARM-compiled** in this branch: verified by inspection + the Barr-C 80-col gate. The default `RC_NET_ENABLE=0` build touches only already-present APIs.
- `RC_NET_ENABLE=1` needs the port's Pico W WiFi/lwIP/MQTT dev profile (link `pico_cyw43_arch_lwip_threadsafe_background` + lwIP MQTT app). Steps in `MQTT.md`.
- The MQTT inbound-data callback runs in `cyw43_arch`/lwIP context and calls `rc_event_publish()`; if this port maps lwIP callbacks to hard-IRQ context, switch that one call to `rc_event_publish_i()` (flagged at the call site).

🤖 Generated with [Claude Code](https://claude.com/claude-code)
