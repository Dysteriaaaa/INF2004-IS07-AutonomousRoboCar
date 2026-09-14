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
#ifndef SUB_TELEMETRY_H
#define SUB_TELEMETRY_H

#include "rc_types.h"

/*
 *  A transport. Return RC_OK if the message was handed off. Called from
 *  the telemetry task, so it may block on a bounded wait, but must not
 *  wait forever.
 */
typedef struct {
    const char *name;
    rc_result_t (*open)(void);
    rc_result_t (*publish)(const char *topic, const char *payload,
                           uint16_t len);
    rc_result_t (*close)(void);
    bool        (*is_up)(void);
} sub_telemetry_sink_t;

/* Incoming command callback, for the command subscription path. */
typedef void (*sub_telemetry_cmd_cb_t)(rc_nav_cmd_t cmd, int32_t arg,
                                       void *ctx);

rc_result_t sub_telemetry_init(void);

/* Install a transport. Pass NULL to fall back to the console sink. */
rc_result_t sub_telemetry_set_sink(const sub_telemetry_sink_t *sink);

rc_result_t sub_telemetry_on_command(sub_telemetry_cmd_cb_t cb, void *ctx);

/* Push a message out of band, outside the periodic schedule. Used for
 * events that should not wait for the next tick, such as a decoded
 * barcode or a completed hump measurement. */
rc_result_t sub_telemetry_publish_event(const rc_event_t *evt);

/* Built-in sinks. */
const sub_telemetry_sink_t *sub_telemetry_console_sink(void);

/*
 *  TODO Buddy 1: implement these two in their own files, matching the
 *  sink interface above. Neither is written yet.
 *
 *      sub_telemetry_udp_sink()    over the port's lwIP UDP profile
 *      sub_telemetry_mqtt_sink()   over an MQTT client you bring in
 */

#endif /* SUB_TELEMETRY_H */
