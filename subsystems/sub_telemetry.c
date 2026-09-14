/*
 *  sub_telemetry.c
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <string.h>
#include <stdio.h>

#include "sub_telemetry.h"
#include "sub_terrain.h"
#include "sub_barcode.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "rc_config.h"
#include "rc_event.h"
#include "rc_time.h"

#define TOPIC_MAX       (48)
#define PAYLOAD_MAX     (192)

/*
 *  Topic design. One base, one leaf per message class, which keeps a
 *  broker subscription simple: car/+/state subscribes to every car's
 *  state, car/01/# to everything from this one.
 */
#define TOPIC_BASE      "car/01"

static const sub_telemetry_sink_t *sink;
static ID       telem_tskid;
static uint32_t seq;
static uint32_t tx_count;
static uint32_t tx_fail;
static sub_telemetry_cmd_cb_t cmd_cb;
static void    *cmd_ctx;

/* Cached state, updated by callbacks, serialised by the telemetry task.
 * Each field is written by exactly one callback and read by one task, so
 * no lock is needed for a word-sized value on this architecture. */
static volatile int32_t  st_speed_l;
static volatile int32_t  st_speed_r;
static volatile uint32_t st_dist_mm;
static volatile bool     st_line_l;
static volatile bool     st_line_r;

/* ------------------------------------------------------------------ *
 *  Console sink. Always available, needs no radio, works from the first
 *  day of integration.
 * ------------------------------------------------------------------ */

static rc_result_t console_open(void)
{
    return RC_OK;
}

static rc_result_t console_publish(const char *topic, const char *payload,
                                   uint16_t len)
{
    (void)len;
    tm_printf((UB *)"[telem] %s %s\n", topic, payload);
    return RC_OK;
}

static rc_result_t console_close(void)
{
    return RC_OK;
}

static bool console_is_up(void)
{
    return true;
}

static const sub_telemetry_sink_t console_sink = {
    "console", console_open, console_publish, console_close, console_is_up
};

const sub_telemetry_sink_t *sub_telemetry_console_sink(void)
{
    return &console_sink;
}

/* ------------------------------------------------------------------ *
 *  Framing
 *
 *  JSON, because it is readable during debugging and every broker tool
 *  understands it. It is not the most compact choice. If the resource
 *  efficiency part of the assessment bites, swap the body of these
 *  functions for a packed binary struct and keep the topics; nothing
 *  else in the tree cares.
 * ------------------------------------------------------------------ */

static rc_result_t send(const char *leaf, const char *payload)
{
    char topic[TOPIC_MAX];
    rc_result_t res;

    if ((sink == NULL) || !sink->is_up()) {
        tx_fail++;
        return RC_ERR_STATE;
    }

    (void)snprintf(topic, sizeof(topic), "%s/%s", TOPIC_BASE, leaf);

    res = sink->publish(topic, payload, (uint16_t)strlen(payload));
    if (res == RC_OK) {
        tx_count++;
    } else {
        tx_fail++;
    }
    return res;
}

static void publish_state(void)
{
    char buf[PAYLOAD_MAX];

    (void)snprintf(buf, sizeof(buf),
        "{\"seq\":%lu,\"t\":%lu,\"spd_l\":%ld,\"spd_r\":%ld,"
        "\"dist\":%lu,\"m_l\":%d,\"m_r\":%d,\"line\":\"%c%c\","
        "\"cls\":%d,\"peak\":%u,\"bc\":\"%c\"}",
        (unsigned long)seq,
        (unsigned long)rc_time_ms(),
        (long)st_speed_l,
        (long)st_speed_r,
        (unsigned long)st_dist_mm,
        (int)drv_motor_get(RC_SIDE_LEFT),
        (int)drv_motor_get(RC_SIDE_RIGHT),
        st_line_l ? '1' : '0',
        st_line_r ? '1' : '0',
        (int)sub_terrain_motion_class(),
        (unsigned int)sub_terrain_max_peak_mm(),
        (sub_barcode_last() != '\0') ? sub_barcode_last() : '-');

    seq++;
    (void)send("state", buf);
}

static void publish_heartbeat(void)
{
    char buf[PAYLOAD_MAX];

    (void)snprintf(buf, sizeof(buf),
        "{\"up\":%lu,\"tx\":%lu,\"fail\":%lu,"
        "\"drop_fast\":%lu,\"drop_slow\":%lu,\"sink\":\"%s\"}",
        (unsigned long)rc_time_ms(),
        (unsigned long)tx_count,
        (unsigned long)tx_fail,
        (unsigned long)rc_event_dropped(RC_LANE_FAST),
        (unsigned long)rc_event_dropped(RC_LANE_SLOW),
        (sink != NULL) ? sink->name : "none");

    (void)send("status", buf);
}

/* ------------------------------------------------------------------ *
 *  Event subscriptions. Slow lane throughout: telemetry must never get
 *  in front of the control path.
 * ------------------------------------------------------------------ */

static void on_odometry(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    st_speed_l = evt->u.odometry.speed_l_mm_s;
    st_speed_r = evt->u.odometry.speed_r_mm_s;
    st_dist_mm = (evt->u.odometry.dist_l_mm + evt->u.odometry.dist_r_mm) / 2U;
}

static void on_line(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    st_line_l = evt->u.line.on_line_l;
    st_line_r = evt->u.line.on_line_r;
}

static void on_notable(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    (void)sub_telemetry_publish_event(evt);
}

rc_result_t sub_telemetry_publish_event(const rc_event_t *evt)
{
    char buf[PAYLOAD_MAX];

    switch (evt->id) {
    case RC_EVT_BARCODE_DECODED:
        (void)snprintf(buf, sizeof(buf),
            "{\"sym\":\"%c\",\"cmd\":%d,\"rev\":%d}",
            evt->u.barcode.symbol,
            (int)evt->u.barcode.command,
            evt->u.barcode.reversed ? 1 : 0);
        return send("barcode", buf);

    case RC_EVT_HUMP_END:
        (void)snprintf(buf, sizeof(buf),
            "{\"peak_mm\":%u,\"ms\":%lu,\"pitch\":%d}",
            (unsigned int)evt->u.hump.peak_mm,
            (unsigned long)evt->u.hump.duration_ms,
            (int)evt->u.hump.max_pitch_deg);
        return send("hump", buf);

    case RC_EVT_OBSTACLE_PROFILE:
        (void)snprintf(buf, sizeof(buf),
            "{\"n\":%u,\"near_mm\":%u,\"at_deg\":%d,\"w_mm\":%u,"
            "\"cl_l\":%u,\"cl_r\":%u}",
            (unsigned int)evt->u.profile.n_points,
            (unsigned int)evt->u.profile.closest_mm,
            (int)evt->u.profile.closest_angle_deg,
            (unsigned int)evt->u.profile.width_mm,
            (unsigned int)evt->u.profile.clearance_left_mm,
            (unsigned int)evt->u.profile.clearance_right_mm);
        return send("obstacle", buf);

    case RC_EVT_IMPACT:
        return send("impact", "{\"impact\":1}");

    default:
        return RC_ERR_PARAM;
    }
}

/* ------------------------------------------------------------------ *
 *  Task
 * ------------------------------------------------------------------ */

static void telemetry_task(INT stacd, void *exinf)
{
    uint32_t ticks = 0U;

    (void)stacd;
    (void)exinf;

    if (sink != NULL) {
        (void)sink->open();
    }

    for (;;) {
        tk_dly_tsk(RC_PERIOD_TELEM_MS);

        /*
         *  TODO Buddy 1: connection recovery belongs here. Check
         *  sink->is_up(), and on a drop, close, wait with a backoff, and
         *  reopen. Do the backoff with tk_dly_tsk, never a retry loop.
         */

        publish_state();

        ticks++;
        if ((ticks % 8U) == 0U) {       /* every 2 s at the default rate */
            publish_heartbeat();
        }
    }
}

/* ------------------------------------------------------------------ */

rc_result_t sub_telemetry_init(void)
{
    T_CTSK ctsk;

    sink = &console_sink;

    (void)rc_event_subscribe(RC_EVT_ODOMETRY, RC_LANE_SLOW, on_odometry, NULL);
    (void)rc_event_subscribe(RC_EVT_LINE_SAMPLE, RC_LANE_SLOW, on_line, NULL);
    (void)rc_event_subscribe(RC_EVT_BARCODE_DECODED, RC_LANE_SLOW,
                             on_notable, NULL);
    (void)rc_event_subscribe(RC_EVT_HUMP_END, RC_LANE_SLOW, on_notable, NULL);
    (void)rc_event_subscribe(RC_EVT_OBSTACLE_PROFILE, RC_LANE_SLOW,
                             on_notable, NULL);
    (void)rc_event_subscribe(RC_EVT_IMPACT, RC_LANE_SLOW, on_notable, NULL);

    (void)memset(&ctsk, 0, sizeof(ctsk));
    ctsk.itskpri = RC_PRI_TELEMETRY;
    ctsk.stksz   = RC_STACK_SZ;
    ctsk.task    = telemetry_task;
    ctsk.tskatr  = TA_HLNG | TA_RNG3;

    telem_tskid = tk_cre_tsk(&ctsk);
    if (telem_tskid <= E_OK) {
        return RC_ERR_HARDWARE;
    }
    (void)tk_sta_tsk(telem_tskid, 0);

    return RC_OK;
}

rc_result_t sub_telemetry_set_sink(const sub_telemetry_sink_t *new_sink)
{
    if (sink != NULL) {
        (void)sink->close();
    }
    sink = (new_sink != NULL) ? new_sink : &console_sink;

    return sink->open();
}

rc_result_t sub_telemetry_on_command(sub_telemetry_cmd_cb_t cb, void *ctx)
{
    cmd_cb  = cb;
    cmd_ctx = ctx;
    return RC_OK;
}
