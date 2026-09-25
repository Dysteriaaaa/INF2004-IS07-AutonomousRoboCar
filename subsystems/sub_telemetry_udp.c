/*
 *  sub_telemetry_udp.c  -  Buddy 1
 *
 *  A telemetry sink that sends each message as one UDP datagram to a
 *  fixed host:port (RC_UDP_DEST_IP/PORT). It carries no broker and no
 *  topics of its own, so it is the cheapest way to prove the WiFi path
 *  end to end before taking on MQTT: point `nc -u -l 5005` (or a tiny
 *  Python listener) at it and watch the JSON arrive. The datagram body is
 *  "<topic> <payload>\n" so a listener can still tell the message classes
 *  apart.
 *
 *  Fills the sub_telemetry_sink_t interface, so sub_telemetry.c uses it
 *  through exactly the same code path as the console sink. Everything is
 *  behind RC_NET_ENABLE; with the radio compiled out,
 *  sub_telemetry_udp_sink() returns NULL and this file pulls in no lwIP.
 */
/* rc_prelude.h first, per the tree rule: PROHIBIT_DEF_SIZE_T before any
 * kernel/libc/lwIP header, and NULL. */
#include "rc_prelude.h"
#include "sub_telemetry.h"
#include "rc_config.h"

#if RC_NET_ENABLE

#include <tm/tmonitor.h>
#include "rc_fmt.h"
#include "net_wifi.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"

/* One datagram's worth of "<topic> <payload>". TOPIC_MAX + PAYLOAD_MAX in
 * sub_telemetry.c are 48 + 192; 256 leaves room for the space and NUL. */
#define UDP_DGRAM_MAX   (256)

static struct udp_pcb *pcb;
static ip_addr_t       dest;

static rc_result_t udp_open(void)
{
    if (net_wifi_up() != RC_OK) {
        return RC_ERR_STATE;
    }

    if (pcb == NULL) {
        if (ip4addr_aton(RC_UDP_DEST_IP, &dest) == 0) {
            tm_printf((UB *)"[udp] bad dest ip\n");
            return RC_ERR_PARAM;
        }
        cyw43_arch_lwip_begin();
        pcb = udp_new();
        if (pcb != NULL) {
            (void)udp_connect(pcb, &dest, RC_UDP_DEST_PORT);
        }
        cyw43_arch_lwip_end();
        if (pcb == NULL) {
            return RC_ERR_NOSPACE;
        }
    }
    return RC_OK;
}

static rc_result_t udp_publish(const char *topic, const char *payload,
                               uint16_t len)
{
    char        line[UDP_DGRAM_MAX];
    int         n;
    struct pbuf *p;
    err_t       err;

    (void)len;

    if ((pcb == NULL) || !net_wifi_is_up()) {
        return RC_ERR_STATE;
    }

    n = rc_snprintf(line, sizeof(line), "%s %s\n", topic, payload);
    if (n <= 0) {
        return RC_ERR_PARAM;
    }
    /* rc_snprintf never overflows `line`, but like snprintf it returns the
     * length it *would* have written. Clamp so a future oversized payload
     * cannot make pbuf_take read past the buffer. */
    if (n >= (int)sizeof(line)) {
        n = (int)sizeof(line) - 1;
    }

    cyw43_arch_lwip_begin();
    p = pbuf_alloc(PBUF_TRANSPORT, (uint16_t)n, PBUF_RAM);
    if (p != NULL) {
        (void)pbuf_take(p, line, (uint16_t)n);
        err = udp_send(pcb, p);
        pbuf_free(p);
    } else {
        err = ERR_MEM;
    }
    cyw43_arch_lwip_end();

    return (err == ERR_OK) ? RC_OK : RC_ERR_HARDWARE;
}

static rc_result_t udp_close(void)
{
    if (pcb != NULL) {
        cyw43_arch_lwip_begin();
        udp_remove(pcb);
        cyw43_arch_lwip_end();
        pcb = NULL;
    }
    net_wifi_down();
    return RC_OK;
}

static bool udp_is_up(void)
{
    return (pcb != NULL) && net_wifi_is_up();
}

static const sub_telemetry_sink_t udp_sink = {
    "udp", udp_open, udp_publish, udp_close, udp_is_up
};

const sub_telemetry_sink_t *sub_telemetry_udp_sink(void)
{
    return &udp_sink;
}

#else  /* RC_NET_ENABLE == 0 */

const sub_telemetry_sink_t *sub_telemetry_udp_sink(void)
{
    return NULL;    /* set_sink(NULL) keeps the console sink */
}

#endif /* RC_NET_ENABLE */
