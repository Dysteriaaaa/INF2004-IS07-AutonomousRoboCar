/*
 *  sub_nav.c
 */
#include "rc_prelude.h"
#include <tm/tmonitor.h>
#include "sub_nav.h"
#include "sub_line.h"
#include "sub_barcode.h"
#include "sub_motion.h"
#include "sub_scan.h"
#include "sub_terrain.h"
#include "rc_config.h"
#include "rc_event.h"

/* Start a full scan when something gets this close while watching. */
#define WATCH_TRIGGER_MM    (300U)

/* Turn angles for barcode commands. Measured, not geometric; see the
 * TODO in sub_motion about slip. */
#define TURN_DEG            (90)
#define UTURN_DEG           (180)

static rc_nav_state_t state;
static rc_pl_plan_t   pending_plan;

static void enter(rc_nav_state_t next);

/* ------------------------------------------------------------------ *
 *  Obstacle bypass
 *
 *  Three queued moves chained by their completion callbacks. No waiting
 *  anywhere: each callback fires when its move finishes and starts the
 *  next one.
 *
 *      step aside  ->  run past  ->  turn back  ->  search for the line
 *
 *  TODO Buddy 5 and Buddy 2 together: the third leg should not be a
 *  fixed turn. The car should turn back toward the line and then let
 *  sub_line's search find it, which is what the code does, but the
 *  angles here are open loop. Tighten them once the turn calibration in
 *  sub_motion is done.
 * ------------------------------------------------------------------ */

static void bypass_leg3(uint32_t id, bool ok, uint32_t mm, void *ctx);

static void bypass_leg2(uint32_t id, bool ok, uint32_t mm, void *ctx)
{
    int16_t back;

    (void)id;
    (void)mm;
    (void)ctx;

    if (!ok) {
        enter(RC_NAV_STOPPED);
        return;
    }

    back = (pending_plan.action == RC_CMD_TURN_LEFT)
           ? (int16_t)TURN_DEG : (int16_t)(-TURN_DEG);

    (void)sub_motion_turn_deg(back, bypass_leg3, NULL);
}

static void bypass_leg3(uint32_t id, bool ok, uint32_t mm, void *ctx)
{
    (void)id;
    (void)mm;
    (void)ctx;

    if (!ok) {
        enter(RC_NAV_STOPPED);
        return;
    }

    /* Hand back to the line follower and let it hunt. sub_line publishes
     * RC_EVT_LINE_REACQUIRED when it finds the track again. */
    enter(RC_NAV_RECOVERING);
    (void)sub_line_begin_search();
    (void)sub_line_enable(true);
    (void)sub_motion_drive(250, 0);
}

static void bypass_leg1(uint32_t id, bool ok, uint32_t mm, void *ctx)
{
    (void)id;
    (void)mm;
    (void)ctx;

    if (!ok) {
        enter(RC_NAV_STOPPED);
        return;
    }
    (void)sub_motion_forward_mm(pending_plan.forward_mm, bypass_leg2, NULL);
}

/* ------------------------------------------------------------------ *
 *  State entry
 * ------------------------------------------------------------------ */

static void enter(rc_nav_state_t next)
{
    if (next == state) {
        return;
    }
    state = next;

    switch (next) {
    case RC_NAV_FOLLOWING:
        (void)sub_line_enable(true);
        (void)sub_barcode_arm(false);
        (void)sub_scan_set_watch(true, WATCH_TRIGGER_MM);
        break;

    case RC_NAV_READING_BARCODE:
        /* Keep following. The barcode sits on the track and the car has
         * to drive over it to read it. */
        (void)sub_barcode_arm(true);
        break;

    case RC_NAV_EXECUTING_CMD:
        (void)sub_line_enable(false);
        (void)sub_barcode_arm(false);
        break;

    case RC_NAV_AVOIDING:
        (void)sub_line_enable(false);
        (void)sub_scan_set_watch(false, 0U);
        break;

    case RC_NAV_RECOVERING:
        (void)sub_barcode_arm(false);
        break;

    case RC_NAV_STOPPED:
    case RC_NAV_IDLE:
    default:
        (void)sub_line_enable(false);
        (void)sub_barcode_arm(false);
        (void)sub_scan_set_watch(false, 0U);
        (void)sub_motion_stop(true);
        break;
    }
}

/* ------------------------------------------------------------------ *
 *  Event handlers, all slow lane except the ones that must be prompt
 * ------------------------------------------------------------------ */

static void cmd_done(uint32_t id, bool ok, uint32_t mm, void *ctx)
{
    (void)id;
    (void)mm;
    (void)ctx;

    if (ok) {
        enter(RC_NAV_FOLLOWING);
    } else {
        enter(RC_NAV_STOPPED);
    }
}

static void on_barcode(const rc_event_t *evt, void *ctx)
{
    (void)ctx;

    if (state != RC_NAV_READING_BARCODE) {
        return;
    }

    enter(RC_NAV_EXECUTING_CMD);

    switch (evt->u.barcode.command) {
    case RC_CMD_TURN_LEFT:
        (void)sub_motion_turn_deg(-TURN_DEG, cmd_done, NULL);
        break;
    case RC_CMD_TURN_RIGHT:
        (void)sub_motion_turn_deg(TURN_DEG, cmd_done, NULL);
        break;
    case RC_CMD_U_TURN:
        (void)sub_motion_turn_deg(UTURN_DEG, cmd_done, NULL);
        break;
    case RC_CMD_GO_STRAIGHT:
    default:
        enter(RC_NAV_FOLLOWING);
        break;
    }
}

/* A command that arrived from outside the sensor path: Buddy 1's remote
 * channel (WiFi/MQTT), delivered as RC_EVT_COMMAND_RX. Runs on the fast
 * dispatcher like every other handler here, so it is free to drive the
 * state machine directly.
 *
 * STOP is honoured from any state - it is the remote kill switch. Every
 * other command is a manoeuvre that only makes sense while line
 * following; ignoring it mid-avoidance keeps a stray message from
 * yanking the car off a bypass. The execution mirrors on_barcode() so a
 * remote "turn left" and a barcode "turn left" behave identically. */
static void on_command_rx(const rc_event_t *evt, void *ctx)
{
    (void)ctx;

    if (evt->u.command.command == RC_CMD_STOP) {
        tm_printf((UB *)"[nav] remote STOP\n");
        enter(RC_NAV_STOPPED);
        return;
    }

    if (state != RC_NAV_FOLLOWING) {
        tm_printf((UB *)"[nav] remote cmd ignored (busy)\n");
        return;
    }

    enter(RC_NAV_EXECUTING_CMD);

    switch (evt->u.command.command) {
    case RC_CMD_TURN_LEFT:
        (void)sub_motion_turn_deg(-TURN_DEG, cmd_done, NULL);
        break;
    case RC_CMD_TURN_RIGHT:
        (void)sub_motion_turn_deg(TURN_DEG, cmd_done, NULL);
        break;
    case RC_CMD_U_TURN:
        (void)sub_motion_turn_deg(UTURN_DEG, cmd_done, NULL);
        break;
    case RC_CMD_GO_STRAIGHT:
    default:
        enter(RC_NAV_FOLLOWING);
        break;
    }
}

static void on_ultra(const rc_event_t *evt, void *ctx)
{
    (void)ctx;

    if (state != RC_NAV_FOLLOWING) {
        return;
    }
    if (!evt->u.ultra.valid) {
        return;
    }
    if (evt->u.ultra.range_mm > WATCH_TRIGGER_MM) {
        return;
    }

    /* Something ahead. Slow down first, then look properly. Stopping
     * dead is not required and costs completion time. */
    enter(RC_NAV_AVOIDING);
    (void)sub_motion_drive(150, 0);
    (void)sub_scan_start();
}

static void on_plan(const rc_event_t *evt, void *ctx)
{
    int16_t first;

    (void)ctx;

    if (state != RC_NAV_AVOIDING) {
        return;
    }

    pending_plan = evt->u.plan;

    if (pending_plan.action == RC_CMD_STOP) {
        tm_printf((UB *)"[nav] boxed in, stopping\n");
        enter(RC_NAV_STOPPED);
        return;
    }
    if (pending_plan.action == RC_CMD_GO_STRAIGHT) {
        enter(RC_NAV_FOLLOWING);
        return;
    }

    (void)sub_line_begin_search();

    first = (pending_plan.action == RC_CMD_TURN_LEFT)
            ? (int16_t)(-TURN_DEG) : (int16_t)TURN_DEG;

    (void)sub_motion_turn_deg(first, bypass_leg1, NULL);
}

static void on_line_reacquired(const rc_event_t *evt, void *ctx)
{
    (void)evt;
    (void)ctx;

    if (state == RC_NAV_RECOVERING) {
        tm_printf((UB *)"[nav] line reacquired\n");
        enter(RC_NAV_FOLLOWING);
    }
}

static void on_line_lost(const rc_event_t *evt, void *ctx)
{
    (void)evt;
    (void)ctx;

    if (state == RC_NAV_FOLLOWING) {
        /* A genuine loss while following usually means a junction with a
         * barcode just ahead, or the track ended. Arm the decoder and
         * let sub_line hunt; if nothing turns up it will keep arcing. */
        enter(RC_NAV_READING_BARCODE);
    }
}

static void on_junction(const rc_event_t *evt, void *ctx)
{
    (void)evt;
    (void)ctx;

    if ((state == RC_NAV_FOLLOWING)
        && (sub_line_state() == RC_LINE_JUNCTION)) {
        enter(RC_NAV_READING_BARCODE);
    }
}

/* ------------------------------------------------------------------ */

rc_result_t sub_nav_init(void)
{
    (void)rc_event_subscribe(RC_EVT_BARCODE_DECODED, RC_LANE_FAST,
                             on_barcode, NULL);
    (void)rc_event_subscribe(RC_EVT_ULTRA_RESULT, RC_LANE_FAST,
                             on_ultra, NULL);
    (void)rc_event_subscribe(RC_EVT_AVOIDANCE_PLAN, RC_LANE_FAST,
                             on_plan, NULL);
    (void)rc_event_subscribe(RC_EVT_LINE_REACQUIRED, RC_LANE_FAST,
                             on_line_reacquired, NULL);
    (void)rc_event_subscribe(RC_EVT_LINE_LOST, RC_LANE_FAST,
                             on_line_lost, NULL);
    (void)rc_event_subscribe(RC_EVT_LINE_SAMPLE, RC_LANE_SLOW,
                             on_junction, NULL);
    /* Buddy 1's remote command channel. Fast lane: a remote STOP must not
     * queue behind telemetry. */
    (void)rc_event_subscribe(RC_EVT_COMMAND_RX, RC_LANE_FAST,
                             on_command_rx, NULL);

    state = RC_NAV_IDLE;
    return RC_OK;
}

rc_result_t sub_nav_start(void)
{
    sub_terrain_reset();
    enter(RC_NAV_FOLLOWING);
    return RC_OK;
}

rc_result_t sub_nav_stop(void)
{
    enter(RC_NAV_STOPPED);
    return RC_OK;
}

rc_nav_state_t sub_nav_state(void)
{
    return state;
}

rc_result_t sub_nav_inject_command(rc_nav_cmd_t cmd, int32_t arg)
{
    rc_event_t evt;

    evt.id                = RC_EVT_COMMAND_RX;
    evt.u.command.command = cmd;
    evt.u.command.arg     = arg;
    return rc_event_publish(&evt);
}
