/*
 *  net_wifi.c  -  Buddy 1
 *
 *  CYW43439 station bring-up for the Pico W. See net_wifi.h for the
 *  contract and rc_config.h (RC_NET_ENABLE and the RC_WIFI_* knobs) for
 *  the switch and credentials.
 */
/* rc_prelude.h first, per the tree rule: it defines PROHIBIT_DEF_SIZE_T
 * before any kernel/libc header and supplies NULL. */
#include "rc_prelude.h"
#include "net_wifi.h"
#include "rc_config.h"

#if RC_NET_ENABLE

#include <tm/tmonitor.h>
#include "rc_event.h"
#include "pico/cyw43_arch.h"

/* Whether cyw43_arch_init() has run. The radio is initialised once and
 * then left up; only the association is dropped and remade on recovery. */
static bool  arch_ready;
/* Last link state we announced, so RC_EVT_LINK_STATE is published only on
 * a change rather than every poll. */
static bool  announced_up;

/* Tell the rest of the car the link changed. Buddy 1 owns
 * RC_EVT_LINK_STATE; the telemetry heartbeat and anyone else can react. */
static void announce(bool up)
{
    rc_event_t evt;

    if (up == announced_up) {
        return;
    }
    announced_up = up;

    evt.id                = RC_EVT_LINK_STATE;
    evt.u.link.connected  = up;
    evt.u.link.broker_up  = false;      /* the MQTT sink owns broker state */
    (void)rc_event_publish(&evt);
}

bool net_wifi_is_up(void)
{
    if (!arch_ready) {
        return false;
    }
    return cyw43_tcpip_link_status(CYW43_ITF_STA) == CYW43_LINK_UP;
}

rc_result_t net_wifi_up(void)
{
    int rc;

    if (!arch_ready) {
        /* Country code sets the regulatory domain (legal TX power and
         * channels); keep RC_WIFI_COUNTRY correct for where you demo. */
        if (cyw43_arch_init_with_country(
                CYW43_COUNTRY(RC_WIFI_COUNTRY[0],
                              RC_WIFI_COUNTRY[1], 0)) != 0) {
            tm_printf((UB *)"[wifi] arch init failed\n");
            return RC_ERR_HARDWARE;
        }
        cyw43_arch_enable_sta_mode();
        arch_ready = true;
    }

    if (net_wifi_is_up()) {
        announce(true);
        return RC_OK;
    }

    tm_printf((UB *)"[wifi] joining %s ...\n", (UB *)RC_WIFI_SSID);
    rc = cyw43_arch_wifi_connect_timeout_ms(RC_WIFI_SSID, RC_WIFI_PASS,
                                            CYW43_AUTH_WPA2_AES_PSK,
                                            RC_WIFI_CONNECT_TMO_MS);
    if (rc != 0) {
        tm_printf((UB *)"[wifi] join failed (%d)\n", rc);
        announce(false);
        return RC_ERR_TIMEOUT;
    }

    tm_printf((UB *)"[wifi] up\n");
    announce(true);
    return RC_OK;
}

void net_wifi_down(void)
{
    if (arch_ready) {
        /* Leave the BSS but keep the radio initialised for a fast rejoin. */
        cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
    }
    announce(false);
}

#else  /* RC_NET_ENABLE == 0: no radio in this build */

rc_result_t net_wifi_up(void)   { return RC_ERR_STATE; }
bool        net_wifi_is_up(void) { return false; }
void        net_wifi_down(void)  { }

#endif /* RC_NET_ENABLE */
