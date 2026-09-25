/*
 *  sub_telemetry_mqtt.c  -  Buddy 1
 *
 *  The real deliverable: a telemetry sink over MQTT, plus the reverse
 *  path - a subscription to RC_TOPIC_CMD that turns an inbound message
 *  into a navigation command. It sits on the lwIP MQTT app client
 *  (lwip/apps/mqtt.h), which is the "MQTT client you bring in" the header
 *  refers to; the port must enable it (see docs/buddy1-telemetry/MQTT.md).
 *
 *  Publish is fire-and-forget QoS 0: telemetry is a stream of the latest
 *  truth, so a dropped sample is stale a tick later anyway and not worth a
 *  retransmit. The command topic is where reliability would matter, and it
 *  is only ever inbound here.
 *
 *  All of this is behind RC_NET_ENABLE; with the radio compiled out,
 *  sub_telemetry_mqtt_sink() returns NULL and no lwIP is pulled in.
 */
/* rc_prelude.h first, per the tree rule: PROHIBIT_DEF_SIZE_T before any
 * kernel/libc/lwIP header, and NULL. */
#include "rc_prelude.h"
#include "sub_telemetry.h"
#include "rc_config.h"

#if RC_NET_ENABLE

#include <tm/tmonitor.h>
#include "rc_event.h"
#include "net_wifi.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"

static mqtt_client_t *client;
static ip_addr_t      broker;
static volatile bool  connected;    /* set by the connection callback */

/* Static because the lwIP client keeps a pointer to it for the life of the
 * connection rather than copying it. */
static const struct mqtt_connect_client_info_t client_info = {
    RC_MQTT_CLIENT_ID,      /* client_id                          */
    NULL, NULL,             /* username, password (open broker)   */
    RC_MQTT_KEEPALIVE_S,    /* keep-alive seconds                 */
    NULL, NULL, 0, 0        /* no last-will topic/message/qos/ret */
};

/* Small assembly buffer for one inbound command payload. */
static char    rx_buf[16];
static uint16_t rx_len;

/* Announce broker reachability so the heartbeat and others can see it.
 * connected==true implies WiFi is up too. */
static void announce_broker(bool up)
{
    rc_event_t evt;

    evt.id                = RC_EVT_LINK_STATE;
    evt.u.link.connected  = net_wifi_is_up();
    evt.u.link.broker_up  = up;
    (void)rc_event_publish(&evt);
}

/* --- inbound command path --------------------------------------------- */

/* Map a payload's leading character to a command. Accepts both the
 * mnemonic letters (L/R/S/U/X) and the barcode letters (A/B/C/D) so the
 * same publisher tooling works for either. */
static rc_nav_cmd_t parse_cmd(char c)
{
    switch (c) {
    case 'L': case 'l': case 'A': case 'a': return RC_CMD_TURN_LEFT;
    case 'R': case 'r': case 'B': case 'b': return RC_CMD_TURN_RIGHT;
    case 'S': case 's': case 'C': case 'c': return RC_CMD_GO_STRAIGHT;
    case 'U': case 'u': case 'D': case 'd': return RC_CMD_U_TURN;
    case 'X': case 'x': case 'P': case 'p': return RC_CMD_STOP;
    default:                                return RC_CMD_NONE;
    }
}

static void incoming_pub_cb(void *arg, const char *topic, u32_t tot_len)
{
    (void)arg;
    (void)topic;    /* we subscribe to exactly one topic */
    (void)tot_len;
    rx_len = 0U;    /* start a fresh payload */
}

/* NOTE: this runs in lwIP/cyw43_arch context, not the telemetry task.
 * sub_telemetry_deliver_command() forwards to the event bus via
 * rc_event_publish(); if this port maps lwIP callbacks to hard-IRQ
 * context, switch that one call to rc_event_publish_i(). */
static void incoming_data_cb(void *arg, const u8_t *data, u16_t len,
                             u8_t flags)
{
    rc_nav_cmd_t cmd;
    uint16_t     i;

    (void)arg;

    if ((rx_len + len) < sizeof(rx_buf)) {
        for (i = 0U; i < len; i++) {
            rx_buf[rx_len++] = (char)data[i];
        }
    }

    if ((flags & MQTT_DATA_FLAG_LAST) != 0U) {
        rx_buf[rx_len] = '\0';
        cmd = parse_cmd(rx_buf[0]);
        if (cmd != RC_CMD_NONE) {
            (void)sub_telemetry_deliver_command(cmd, 0);
        }
        rx_len = 0U;
    }
}

/* --- connection lifecycle --------------------------------------------- */

static void sub_req_cb(void *arg, err_t err)
{
    (void)arg;
    if (err != ERR_OK) {
        tm_printf((UB *)"[mqtt] subscribe failed (%d)\n", (int)err);
    }
}

static void conn_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t s)
{
    (void)arg;

    if (s == MQTT_CONNECT_ACCEPTED) {
        connected = true;
        tm_printf((UB *)"[mqtt] connected\n");
        mqtt_set_inpub_callback(c, incoming_pub_cb, incoming_data_cb, NULL);
        (void)mqtt_subscribe(c, RC_TOPIC_CMD, 1, sub_req_cb, NULL);
        announce_broker(true);
    } else {
        connected = false;
        tm_printf((UB *)"[mqtt] disconnected (%d)\n", (int)s);
        announce_broker(false);
    }
}

/* --- sink interface --------------------------------------------------- */

static bool mqtt_up(void)
{
    return net_wifi_is_up() && connected && (client != NULL)
           && (mqtt_client_is_connected(client) != 0U);
}

static rc_result_t mqtt_open(void)
{
    err_t err;

    if (net_wifi_up() != RC_OK) {
        return RC_ERR_STATE;
    }
    if (client == NULL) {
        client = mqtt_client_new();
        if (client == NULL) {
            return RC_ERR_NOSPACE;
        }
    }
    if (mqtt_client_is_connected(client) != 0U) {
        return RC_OK;
    }
    if (ip4addr_aton(RC_MQTT_BROKER_IP, &broker) == 0) {
        tm_printf((UB *)"[mqtt] bad broker ip\n");
        return RC_ERR_PARAM;
    }

    /* Connect is asynchronous: conn_cb flips `connected` on accept, and
     * is_up() reflects that. Returning RC_OK here means "attempt
     * started", and the telemetry task's recovery loop will keep waiting
     * on is_up() until the handshake completes or times out. */
    cyw43_arch_lwip_begin();
    err = mqtt_client_connect(client, &broker, RC_MQTT_BROKER_PORT,
                              conn_cb, NULL, &client_info);
    cyw43_arch_lwip_end();

    return (err == ERR_OK) ? RC_OK : RC_ERR_HARDWARE;
}

static rc_result_t mqtt_publish_fn(const char *topic, const char *payload,
                                   uint16_t len)
{
    err_t err;

    if (!mqtt_up()) {
        return RC_ERR_STATE;
    }

    cyw43_arch_lwip_begin();
    err = mqtt_publish(client, topic, payload, len, 0 /*qos*/, 0 /*retain*/,
                       NULL, NULL);
    cyw43_arch_lwip_end();

    return (err == ERR_OK) ? RC_OK : RC_ERR_HARDWARE;
}

static rc_result_t mqtt_close(void)
{
    if (client != NULL) {
        cyw43_arch_lwip_begin();
        mqtt_disconnect(client);
        cyw43_arch_lwip_end();
    }
    connected = false;
    net_wifi_down();
    return RC_OK;
}

static const sub_telemetry_sink_t mqtt_sink = {
    "mqtt", mqtt_open, mqtt_publish_fn, mqtt_close, mqtt_up
};

const sub_telemetry_sink_t *sub_telemetry_mqtt_sink(void)
{
    return &mqtt_sink;
}

#else  /* RC_NET_ENABLE == 0 */

const sub_telemetry_sink_t *sub_telemetry_mqtt_sink(void)
{
    return NULL;    /* set_sink(NULL) keeps the console sink */
}

#endif /* RC_NET_ENABLE */
