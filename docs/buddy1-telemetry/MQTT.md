# Buddy 1 — MQTT & Telemetry Communication

The communication contract for the car: what it publishes, what it
listens for, how the transport is wired, and how to stand a broker up and
test it. This is the "MQTT documentation" deliverable.

## Architecture — one framing, swappable transport

`sub_telemetry.c` owns the framing (JSON), the topic design and the
publish schedule. It never talks to a socket directly; it calls through a
`sub_telemetry_sink_t` — five function pointers (`open`, `publish`,
`close`, `is_up`, plus a name). Three sinks implement that interface:

| Sink | File | Use |
|---|---|---|
| console | `sub_telemetry.c` | Always available, prints to the USB console. The default and the fallback. |
| udp | `sub_telemetry_udp.c` | One datagram per message to a fixed host:port. Proves the WiFi path with no broker. |
| mqtt | `sub_telemetry_mqtt.c` | Full publish + command subscribe over a broker. The real deliverable. |

Swapping transport is one call — `sub_telemetry_set_sink(...)` — and
nothing in the framing or scheduling changes. `app_main.c` makes that call
at boot under `RC_NET_ENABLE`.

## Enabling the radio path

Everything network lives behind `RC_NET_ENABLE` (`core/rc_config.h`),
**off by default** so the graded mission image builds and links exactly as
before. Turning it on requires a port build that provides the Pico W WiFi
profile — it is a development profile in `mtk3smp-rp2040`, not part of the
qualified release:

1. Link `pico_cyw43_arch_lwip_threadsafe_background` (brings in the
   CYW43439 driver + lwIP).
2. Enable the lwIP **MQTT app** (`LWIP_MQTT`/`MQTT_APP`) so
   `lwip/apps/mqtt.h` is available.
3. Provide an `lwipopts.h` with `LWIP_UDP=1`, `LWIP_TCP=1`, `MEM_SIZE`
   large enough for a couple of pbufs, and `NO_SYS=1`.
4. Set the credentials/addresses in `rc_config.h` (`RC_WIFI_SSID`,
   `RC_WIFI_PASS`, `RC_WIFI_COUNTRY`, `RC_MQTT_BROKER_IP`,
   `RC_UDP_DEST_IP`, ...).
5. Build with `-DRC_NET_ENABLE=1`.

With it off, `sub_telemetry_udp_sink()` and `sub_telemetry_mqtt_sink()`
return `NULL`, `sub_telemetry_set_sink(NULL)` keeps the console sink, and
no lwIP header is pulled into the build.

## Topics

Base is `RC_TOPIC_BASE` = `car/01`. Several cars can share a broker:
`car/+/state` addresses every car, `car/01/#` just this one.

### Published by the car

| Topic | When | Payload |
|---|---|---|
| `car/01/state` | every 250 ms | live status (below) |
| `car/01/status` | every 2 s | heartbeat / link health |
| `car/01/barcode` | on decode | `{"sym","cmd","rev"}` |
| `car/01/hump` | on hump end | `{"peak_mm","ms","pitch"}` |
| `car/01/obstacle` | on scan profile | `{"n","near_mm","at_deg","w_mm","cl_l","cl_r"}` |
| `car/01/impact` | on IMU jolt | `{"impact":1}` |

### Subscribed by the car

| Topic | Payload | Effect |
|---|---|---|
| `car/01/cmd` | one letter (below) | injected as a navigation command |

## Message schemas

`state` (per tick):

```json
{"seq":42,"t":10345,"spd_l":250,"spd_r":248,"dist":1820,
 "m_l":80,"m_r":80,"line":"10","cls":2,"peak":31,"bc":"A"}
```

| Field | Meaning |
|---|---|
| `seq` | message counter (spot gaps/reordering) |
| `t` | ms since boot |
| `spd_l`/`spd_r` | wheel speed mm/s, signed |
| `dist` | mean distance travelled, mm |
| `m_l`/`m_r` | commanded motor duty per side |
| `line` | left/right line-sensor flags, `"01"` etc. |
| `cls` | motion class (`rc_motion_class_t`) |
| `peak` | tallest hump seen, mm |
| `bc` | last barcode char, `-` if none |

`status` (heartbeat):

```json
{"up":10345,"tx":210,"fail":3,"drop_fast":0,"drop_slow":0,"sink":"mqtt"}
```

`tx`/`fail` are cumulative send outcomes; `drop_*` are event-bus drops per
lane (any `drop_fast` above zero is a real warning); `sink` is the active
transport.

## Command payloads (`car/01/cmd`)

The first character of the payload is the command. Both mnemonic and
barcode letters are accepted:

| Payload | Command |
|---|---|
| `L` or `A` | turn left |
| `R` or `B` | turn right |
| `S` or `C` | go straight |
| `U` or `D` | U-turn |
| `X` or `P` | stop |

A command is delivered by `sub_telemetry_deliver_command()`, which
publishes `RC_EVT_COMMAND_RX` on the event bus. `sub_nav` handles it on
the fast lane: **STOP** is honoured from any state (remote kill switch);
every other command runs only while line following, and executes exactly
like the same barcode command. Nav state is never touched from the network
callback directly, so the mission state machine stays owned by one task.

## QoS and retain

- Telemetry publishes are **QoS 0, retain 0**. Telemetry is a stream of
  the latest truth; a dropped sample is stale a tick later, so a
  retransmit buys nothing and costs airtime.
- The command subscription is **QoS 1** so a one-shot command is not
  silently lost.

## Connection recovery

The telemetry task checks `sink->is_up()` every tick. On a drop it closes
the transport, waits a backoff that doubles from `RC_NET_BACKOFF_MIN_MS`
to `RC_NET_BACKOFF_MAX_MS`, then makes a single reopen attempt — using
`tk_dly_tsk`, never a busy retry loop, so a dead link only ever stalls the
lowest-priority telemetry task and never the control path. The backoff
resets the moment the link is healthy again. WiFi association and the MQTT
handshake are both re-driven through this one path.

## Standing up a broker and testing

Broker (Mosquitto on the demo laptop):

```bash
mosquitto -v -p 1883          # foreground, verbose
```

Watch everything the car sends:

```bash
mosquitto_sub -h 192.168.4.1 -t 'car/01/#' -v
```

Send a command:

```bash
mosquitto_pub -h 192.168.4.1 -t car/01/cmd -m L    # turn left
mosquitto_pub -h 192.168.4.1 -t car/01/cmd -m X    # stop
```

UDP sink (no broker needed — build with the udp sink selected):

```bash
nc -u -l 5005                 # or a small Python udp listener
```

Set `RC_UDP_DEST_IP`/`RC_MQTT_BROKER_IP` to the laptop's address on the
shared hotspot, and `RC_WIFI_SSID`/`RC_WIFI_PASS` to that hotspot.

## Build order (why UDP before MQTT)

`console (works today) → udp → mqtt`. The console sink is the baseline
demo. UDP proves association + the lwIP send path with almost no moving
parts. MQTT adds the broker handshake, keep-alive and the inbound
subscription. Bringing them up in that order means each step fails in only
one new place.
