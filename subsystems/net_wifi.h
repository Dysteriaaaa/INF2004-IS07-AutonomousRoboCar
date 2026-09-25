/*
 *  net_wifi.h  -  Buddy 1
 *
 *  The one place the CYW43439 radio is brought up. Both network sinks
 *  (UDP and MQTT) sit on top of an associated station, so association
 *  lives here rather than being duplicated in each sink.
 *
 *  Everything is behind RC_NET_ENABLE (rc_config.h). When the radio path
 *  is compiled out, these are trivial stubs: net_wifi_up() reports
 *  RC_ERR_STATE and net_wifi_is_up() is always false, so a sink built on
 *  top simply reports itself down and the console sink keeps running.
 */
#ifndef NET_WIFI_H
#define NET_WIFI_H

#include "rc_types.h"

/* Bring the station up, idempotently. First call initialises the radio
 * and joins RC_WIFI_SSID; later calls just re-check the link and rejoin
 * if it has dropped. Blocks up to RC_WIFI_CONNECT_TMO_MS on a join, so
 * call it only from a task that is allowed to wait (the telemetry task),
 * never from a callback or ISR. Returns RC_OK once associated. */
rc_result_t net_wifi_up(void);

/* True only while the station is associated and has an IP link. Cheap;
 * safe to poll every telemetry tick. */
bool net_wifi_is_up(void);

/* Drop the association but keep the radio initialised, so a later
 * net_wifi_up() rejoins quickly. Used by a sink's close() during
 * backoff. */
void net_wifi_down(void);

#endif /* NET_WIFI_H */
