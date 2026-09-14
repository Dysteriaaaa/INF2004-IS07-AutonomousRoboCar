/*
 *  sub_scan.c
 */
#include <tk/tkernel.h>
#include <string.h>

#include "sub_scan.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "rc_config.h"
#include "rc_event.h"

/* Anything further than this is background, not an obstacle. */
#define OBSTACLE_MM         (400U)

/* Minimum gap the car needs to fit through, plus margin. Measure your
 * car's widest point and add 40 mm. */
#define CAR_WIDTH_MM        (150U)

typedef enum {
    S_IDLE = 0,
    S_WATCH,
    S_COARSE_MOVE,
    S_COARSE_PING,
    S_FINE_MOVE,
    S_FINE_PING,
    S_DONE
} scan_state_t;

typedef struct {
    int16_t  angle;
    uint16_t range_mm;
    bool     valid;
} point_t;

static scan_state_t state;
static point_t      points[RC_SCAN_MAX_POINTS];
static uint32_t     n_points;
static int16_t      cur_angle;
static int16_t      fine_from;
static int16_t      fine_to;
static uint16_t     watch_trigger_mm = 300U;
static bool         watch_on;
static ID           scan_tskid;
static ID           scan_flgid;
static sub_scan_done_cb_t user_cb;
static void        *user_ctx;

#define FLG_RESULT      (1U << 0)
#define FLG_START       (1U << 1)
#define FLG_ABORT       (1U << 2)

/* ------------------------------------------------------------------ *
 *  Profiling
 *
 *  Turns the collected (angle, range) points into the five numbers the
 *  brief asks for. All integer, all cheap.
 * ------------------------------------------------------------------ */

static void build_profile(rc_pl_profile_t *p)
{
    uint32_t i;
    uint16_t closest = 0xFFFFU;
    int16_t  closest_angle = 90;
    int16_t  first_hit = -1;
    int16_t  last_hit = -1;
    uint32_t clear_l = 0U;
    uint32_t clear_r = 0U;

    (void)memset(p, 0, sizeof(*p));
    p->n_points = (uint8_t)n_points;

    for (i = 0U; i < n_points; i++) {
        if (!points[i].valid) {
            continue;
        }
        if (points[i].range_mm < closest) {
            closest       = points[i].range_mm;
            closest_angle = points[i].angle;
        }
        if (points[i].range_mm < OBSTACLE_MM) {
            if (first_hit < 0) {
                first_hit = points[i].angle;
            }
            last_hit = points[i].angle;
        } else {
            /* Free bearings on each side of straight ahead become the
             * clearance figures. Counting bearings rather than measuring
             * a gap is crude but it is what a single ranging beam can
             * honestly support. */
            if (points[i].angle < 90) {
                clear_r++;
            } else if (points[i].angle > 90) {
                clear_l++;
            } else {
                /* dead ahead, neither side */
            }
        }
    }

    if (closest == 0xFFFFU) {
        return;                 /* nothing seen at all */
    }

    p->closest_mm        = closest;
    p->closest_angle_deg = closest_angle;

    /*
     *  Width from the angular span at the measured distance:
     *      width = 2 * range * tan(span/2)
     *  and for spans under about 60 degrees, tan(x) is close enough to x
     *  in radians that the error is smaller than the HC-SR04's 15 degree
     *  beam width, which dominates everything here anyway.
     */
    if ((first_hit >= 0) && (last_hit >= first_hit)) {
        int32_t span_ddeg = ((int32_t)last_hit - (int32_t)first_hit) * 10;
        int32_t span_mrad = (span_ddeg * 1745) / 1000;
        p->width_mm = (uint16_t)(((int32_t)closest * span_mrad) / 1000);
    }

    p->clearance_left_mm  = (uint16_t)(clear_l * RC_SCAN_COARSE_STEP);
    p->clearance_right_mm = (uint16_t)(clear_r * RC_SCAN_COARSE_STEP);
}

/* ------------------------------------------------------------------ *
 *  Avoidance planning
 *
 *  TODO Buddy 5: this picks a side and a step distance from the
 *  clearance counts. It is a starting point, not a finished planner.
 *  Two things to add once it drives:
 *
 *    1. Remember which side you went last time. On a narrow track,
 *       alternating sides on successive obstacles wastes distance.
 *    2. The brief lists "reverse and reattempt". That case belongs here,
 *       when both clearances are below CAR_WIDTH_MM.
 * ------------------------------------------------------------------ */

static void build_plan(const rc_pl_profile_t *p, rc_pl_plan_t *plan)
{
    (void)memset(plan, 0, sizeof(*plan));

    if (p->closest_mm == 0U) {
        plan->action = RC_CMD_GO_STRAIGHT;
        return;
    }

    if (p->closest_mm > OBSTACLE_MM) {
        plan->action = RC_CMD_GO_STRAIGHT;
        return;
    }

    if ((p->clearance_left_mm < CAR_WIDTH_MM)
        && (p->clearance_right_mm < CAR_WIDTH_MM)) {
        plan->action = RC_CMD_STOP;
        return;
    }

    if (p->clearance_left_mm >= p->clearance_right_mm) {
        plan->action = RC_CMD_TURN_LEFT;
    } else {
        plan->action = RC_CMD_TURN_RIGHT;
    }

    /* Step aside by half the car width plus half the obstacle width, and
     * run past it by its width plus a margin. */
    plan->lateral_mm = (uint16_t)((CAR_WIDTH_MM / 2U) + (p->width_mm / 2U));
    plan->forward_mm = (uint16_t)(p->width_mm + 100U);
}

/* ------------------------------------------------------------------ *
 *  Result callback from the ultrasonic driver. INTERRUPT context, so it
 *  does the absolute minimum and hands off to the scan task.
 * ------------------------------------------------------------------ */

static void on_range(uint16_t range_mm, bool valid, void *ctx)
{
    (void)ctx;

    if (n_points < RC_SCAN_MAX_POINTS) {
        points[n_points].angle    = cur_angle;
        points[n_points].range_mm = range_mm;
        points[n_points].valid    = valid;
        n_points++;
    }
    (void)tk_set_flg(scan_flgid, FLG_RESULT);
}

/* ------------------------------------------------------------------ *
 *  Scan task
 *
 *  Every wait in here is a kernel wait with a timeout, so the task is
 *  off the run queue for the whole servo travel and the whole 30 ms of
 *  ultrasonic flight time.
 * ------------------------------------------------------------------ */

static void publish_and_finish(void)
{
    rc_pl_profile_t profile;
    rc_pl_plan_t    plan;
    rc_event_t      evt;

    build_profile(&profile);
    build_plan(&profile, &plan);

    evt.id         = RC_EVT_OBSTACLE_PROFILE;
    evt.u.profile  = profile;
    (void)rc_event_publish(&evt);

    evt.id    = RC_EVT_AVOIDANCE_PLAN;
    evt.u.plan = plan;
    (void)rc_event_publish(&evt);

    if (user_cb != NULL) {
        user_cb(&profile, &plan, user_ctx);
    }

    state = S_IDLE;
}

static bool step_to(int16_t angle)
{
    uint32_t settle = drv_servo_settle_ms(drv_servo_get_angle(), angle);

    cur_angle = angle;
    (void)drv_servo_set_angle(angle);
    (void)tk_dly_tsk((INT)settle);

    return (drv_ultrasonic_ping(angle) == RC_OK);
}

static bool wait_result(void)
{
    UINT ptn;
    ER   er;

    /* Timeout is the driver's own echo timeout plus slack. If the driver
     * is working this never expires; it is here so a wedged measurement
     * cannot stall the scan forever. */
    er = tk_wai_flg(scan_flgid, FLG_RESULT | FLG_ABORT,
                    TWF_ORW | TWF_CLR, &ptn, 100);

    return (er == E_OK) && ((ptn & FLG_ABORT) == 0U);
}

static void run_coarse(void)
{
    int16_t a;

    n_points = 0U;

    for (a = RC_SCAN_COARSE_START; a <= RC_SCAN_COARSE_END;
         a = (int16_t)(a + RC_SCAN_COARSE_STEP)) {
        state = S_COARSE_MOVE;
        if (!step_to(a)) {
            continue;
        }
        state = S_COARSE_PING;
        if (!wait_result()) {
            return;
        }
    }
}

static bool find_fine_window(void)
{
    uint32_t i;
    uint16_t closest = 0xFFFFU;
    int16_t  best = -1;

    for (i = 0U; i < n_points; i++) {
        if (points[i].valid && (points[i].range_mm < closest)
            && (points[i].range_mm < OBSTACLE_MM)) {
            closest = points[i].range_mm;
            best    = points[i].angle;
        }
    }

    if (best < 0) {
        return false;
    }

    fine_from = (int16_t)(best - RC_SCAN_COARSE_STEP);
    fine_to   = (int16_t)(best + RC_SCAN_COARSE_STEP);

    if (fine_from < RC_SERVO_ANGLE_MIN) {
        fine_from = RC_SERVO_ANGLE_MIN;
    }
    if (fine_to > RC_SERVO_ANGLE_MAX) {
        fine_to = RC_SERVO_ANGLE_MAX;
    }
    return true;
}

static void run_fine(void)
{
    int16_t a;

    for (a = fine_from; a <= fine_to; a = (int16_t)(a + RC_SCAN_FINE_STEP)) {
        state = S_FINE_MOVE;
        if (!step_to(a)) {
            continue;
        }
        state = S_FINE_PING;
        if (!wait_result()) {
            return;
        }
    }
}

static void scan_task(INT stacd, void *exinf)
{
    UINT ptn;
    ER   er;

    (void)stacd;
    (void)exinf;

    for (;;) {
        if (state == S_WATCH) {
            /* Forward watch: one ping straight ahead per period. */
            cur_angle = 90;
            (void)drv_servo_set_angle(90);
            if (drv_ultrasonic_ping(90) == RC_OK) {
                (void)wait_result();
            }
            (void)tk_dly_tsk(RC_PERIOD_SCAN_MS);
            continue;
        }

        er = tk_wai_flg(scan_flgid, FLG_START, TWF_ORW | TWF_CLR, &ptn, 100);
        if (er != E_OK) {
            continue;
        }

        run_coarse();
        if (find_fine_window()) {
            run_fine();
        }
        publish_and_finish();

        if (watch_on) {
            state = S_WATCH;
        }
    }
}

/* ------------------------------------------------------------------ *
 *  Watch mode result hook: publishes nothing extra, sub_nav subscribes
 *  to RC_EVT_ULTRA_RESULT directly and decides when to escalate.
 * ------------------------------------------------------------------ */

rc_result_t sub_scan_init(void)
{
    T_CFLG cflg;
    T_CTSK ctsk;

    (void)memset(&cflg, 0, sizeof(cflg));
    cflg.flgatr = TA_TFIFO | TA_WMUL;
    scan_flgid  = tk_cre_flg(&cflg);
    if (scan_flgid <= E_OK) {
        return RC_ERR_HARDWARE;
    }

    (void)drv_ultrasonic_on_result(on_range, NULL);

    (void)memset(&ctsk, 0, sizeof(ctsk));
    ctsk.itskpri = RC_PRI_SENSE;
    ctsk.stksz   = RC_STACK_SZ;
    ctsk.task    = scan_task;
    ctsk.tskatr  = TA_HLNG | TA_RNG3;

    scan_tskid = tk_cre_tsk(&ctsk);
    if (scan_tskid <= E_OK) {
        return RC_ERR_HARDWARE;
    }
    (void)tk_sta_tsk(scan_tskid, 0);

    state = S_IDLE;
    return RC_OK;
}

rc_result_t sub_scan_start(void)
{
    if ((state != S_IDLE) && (state != S_WATCH)) {
        return RC_ERR_BUSY;
    }
    state = S_IDLE;
    return (tk_set_flg(scan_flgid, FLG_START) == E_OK) ? RC_OK : RC_ERR_STATE;
}

rc_result_t sub_scan_abort(void)
{
    (void)tk_set_flg(scan_flgid, FLG_ABORT);
    state = S_IDLE;
    return RC_OK;
}

bool sub_scan_busy(void)
{
    return (state != S_IDLE) && (state != S_WATCH);
}

rc_result_t sub_scan_on_complete(sub_scan_done_cb_t cb, void *ctx)
{
    user_cb  = cb;
    user_ctx = ctx;
    return RC_OK;
}

rc_result_t sub_scan_set_watch(bool on, uint16_t trigger_mm)
{
    watch_on         = on;
    watch_trigger_mm = trigger_mm;

    if (on && (state == S_IDLE)) {
        state = S_WATCH;
    } else if (!on && (state == S_WATCH)) {
        state = S_IDLE;
    } else {
        /* leave a running scan alone */
    }
    return RC_OK;
}
