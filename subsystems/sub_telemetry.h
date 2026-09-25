/*
 *  sub_telemetry.h  -  Buddy 1
 *
 *  Telemetry framing and transport, with the transport behind an
 *  interface.
 *
 *  Why the indirection: the mtk3 RP2040 port lists WiFi and lwIP as a
 *  development profile outside the qualified release, and it ships no
 *  MQTT client at all. The docs are explicit that anything layered on
 *  lwIP, MQTT included, is left as an exercise. If the telemetry
 *  subsystem is written directly against an MQTT client that does not
 *  exist yet, Buddy 1 has nothing to demonstrate until the day it does.
 *
 *  So: the framing, the topic design, the message structures and the
 *  publish scheduling all live here and work today over the console
 *  sink. Swapping in UDP and then MQTT is a matter of registering a
 *  different sink, and the rest of the subsystem does not change.
 *
 *  That ordering is also the safe one for the demo. Console first gets
 *  you a working telemetry framework; UDP proves the network path; MQTT
 *  is the last mile.
 */
/* Include guard: without this, if two .c files both #include this header
 * (directly or indirectly), the compiler would see everything below twice
 * and error out. #ifndef/#define/#endif is the standard C way to make a
 * header "idempotent" (safe to include more than once). */
#ifndef SUB_TELEMETRY_H
#define SUB_TELEMETRY_H

/* brings in rc_result_t, rc_event_t, rc_nav_cmd_t, etc. */
#include "rc_types.h"

/*
 *  A transport. Return RC_OK if the message was handed off. Called from
 *  the telemetry task, so it may block on a bounded wait, but must not
 *  wait forever.
 *
 *  This struct is a "struct of function pointers" — a very common C
 *  pattern for building something like an interface/plugin in a language
 *  that has no classes or interfaces. Each field below is not a value,
 *  it's a *pointer to a function* with a given shape (return type +
 *  argument types). Any code that has filled in all five fields with
 *  real functions can be handed around wherever a "transport" is needed,
 *  and the caller (this module) never needs to know or care *which*
 *  transport it's actually talking to (console, UDP, MQTT, ...). This is
 *  how sub_telemetry.c stays unchanged no matter which sink is plugged
 *  in later — see the big comment at the top of this file.
 */
typedef struct {
    /* short human-readable label, e.g. "console" or "udp" — used only for
     * logging/diagnostics */
    const char *name;

    /* Called once to start the transport (e.g. open a socket, connect to
     * a broker). Must return RC_OK on success. */
    rc_result_t (*open)(void);

    /* Called once per outgoing message. `topic` is the destination string
     * (e.g. "car/01/state"), `payload` is the message body (JSON text
     * here), `len` is the payload's length in bytes so the transport
     * doesn't have to recompute it. Must not block forever — see note
     * above the struct. */
    rc_result_t (*publish)(const char *topic, const char *payload,
                           uint16_t len);

    /* Called to shut the transport down cleanly (e.g. close a socket). */
    rc_result_t (*close)(void);

    /* Called to ask "is this transport currently usable?" without trying
     * to send anything. Returns a plain bool (true/false), not an
     * rc_result_t, because it's just a status check, not an action that
     * can itself fail. */
    bool        (*is_up)(void);
} sub_telemetry_sink_t;

/* This defines a *function pointer type*, not a function. Read it as:
 * "sub_telemetry_cmd_cb_t is the name for any function that takes an
 * rc_nav_cmd_t, an int32_t, and a void* context, and returns nothing
 * (void)." Once a variable has this type, it can be pointed at any
 * function matching that shape and then called through the variable —
 * this is how C does "callbacks" (a function you hand to someone else,
 * for them to call back into your code later when something happens).
 * `void *ctx` is a generic "remember whatever you like" pointer: the
 * caller of sub_telemetry_on_command supplies it once, and it comes back
 * unchanged on every call, so the callback can find its own state
 * without needing global variables.
 * Incoming command callback, for the command subscription path. */
typedef void (*sub_telemetry_cmd_cb_t)(rc_nav_cmd_t cmd, int32_t arg,
                                       void *ctx);

/* Starts this subsystem: subscribes to the events it reports on and
 * launches the background telemetry task. Call once at boot. */
rc_result_t sub_telemetry_init(void);

/* Install a transport. Pass NULL to fall back to the console sink.
 * `sink` here is a *pointer* to a sub_telemetry_sink_t — i.e. the address
 * of the struct in memory, not a copy of it. Passing a pointer instead of
 * the whole struct is both cheaper (one address vs. five fields) and
 * lets this module keep using the *same* struct the caller built,
 * without copying it around. `const` on the parameter is a promise to
 * the compiler (and to readers) that this function will not modify what
 * the pointer points to — it only reads the sink's fields. */
rc_result_t sub_telemetry_set_sink(const sub_telemetry_sink_t *sink);

/* Registers the function to call when a command arrives from outside the
 * robot (once a receiving transport actually delivers one — nothing
 * does yet, see the TODOs near the bottom of this file). */
rc_result_t sub_telemetry_on_command(sub_telemetry_cmd_cb_t cb, void *ctx);

/* Push a message out of band, outside the periodic schedule. Used for
 * events that should not wait for the next tick, such as a decoded
 * barcode or a completed hump measurement.
 * `const rc_event_t *evt` — a pointer to the event data, marked const
 * because this function only reads it, never modifies the caller's
 * event. Passing a pointer avoids copying the whole event struct. */
rc_result_t sub_telemetry_publish_event(const rc_event_t *evt);

/* Built-in sinks.
 * Returns a pointer to the one, shared, built-in console sink instance
 * (see console_sink in sub_telemetry.c) — call this to get a sink you
 * can hand to sub_telemetry_set_sink(), or to explicitly go back to the
 * safe console fallback. */
const sub_telemetry_sink_t *sub_telemetry_console_sink(void);

/*
 *  Network sinks. Implemented in sub_telemetry_udp.c and
 *  sub_telemetry_mqtt.c, both behind RC_NET_ENABLE (rc_config.h). When
 *  the radio path is compiled out — the default — each of these returns
 *  NULL, and sub_telemetry_set_sink(NULL) then keeps the console sink, so
 *  callers can wire them unconditionally without breaking a console-only
 *  build.
 *
 *      sub_telemetry_udp_sink()    telemetry as UDP datagrams (prove the
 *                                  network path; no broker needed)
 *      sub_telemetry_mqtt_sink()   full MQTT publish + command subscribe
 */
const sub_telemetry_sink_t *sub_telemetry_udp_sink(void);
const sub_telemetry_sink_t *sub_telemetry_mqtt_sink(void);

/*
 *  The receive path. A transport that has decoded an inbound command
 *  calls this from its own task/callback context. If a callback was
 *  registered with sub_telemetry_on_command() it is invoked; otherwise
 *  the command is published as RC_EVT_COMMAND_RX so sub_nav can act on it
 *  in the fast dispatcher. Either way the caller does not touch nav state
 *  directly, which keeps the mission state machine single-threaded.
 */
rc_result_t sub_telemetry_deliver_command(rc_nav_cmd_t cmd, int32_t arg);

/* SUB_TELEMETRY_H -- matches the #ifndef/#define include guard at the top of
 * the file */
#endif
