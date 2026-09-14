/*
 *  sub_line.c
 */
#include "rc_prelude.h"
#include "sub_line.h"
#include "sub_motion.h"
#include "rc_config.h"
#include "rc_event.h"

/* How many consecutive all-dark samples before we call the line lost.
 * At 5 ms per sample, 20 samples is 100 ms, which at 250 mm/s is 25 mm
 * of travel. Short enough to react, long enough to ride over a gap. */
#define LOST_THRESHOLD      (20U)

/* Both sensors on the line for this long means a junction or a barcode
 * lead-in, not a centred car. */
#define JUNCTION_THRESHOLD  (6U)

static rc_line_state_t state = RC_LINE_TRACKING;
static bool     enabled;
static int16_t  base_permille = 350;
static uint32_t lost_count;
static uint32_t both_count;
static int16_t  last_position;

/* ------------------------------------------------------------------ *
 *  Steering
 *
 *  TODO Buddy 3: this is a proportional-only controller on a two-state
 *  position estimate, which is the simplest thing that can work. It will
 *  weave. Two upgrades worth making, in order:
 *
 *    1. Add a derivative term on position so the car stops overshooting
 *       the centre. Cheap, big improvement.
 *    2. Move to the analogue outputs and interpolate a real position
 *       between the two sensors. That turns the four-state estimate into
 *       a continuous one and is what "robust against speed variations"
 *       in the brief is really asking for.
 * ------------------------------------------------------------------ */

#define STEER_KP        (2)

static void steer_from_position(int16_t position)
{
    int16_t steer = (int16_t)((int32_t)position * STEER_KP / 2);

    (void)sub_motion_drive(base_permille, steer);
}

/* ------------------------------------------------------------------ *
 *  Sample callback. Dispatcher task context, fast lane.
 * ------------------------------------------------------------------ */

static void on_sample(const rc_event_t *evt, void *ctx)
{
    const rc_pl_line_t *s = &evt->u.line;
    rc_event_t          out;

    (void)ctx;

    if (!enabled) {
        return;
    }

    if (s->on_line_l && s->on_line_r) {
        both_count++;
        lost_count = 0U;

        if (both_count >= JUNCTION_THRESHOLD) {
            state = RC_LINE_JUNCTION;
        }
        /* Drive straight through. A junction is where the barcode
         * usually starts, so sub_barcode arms itself off this state. */
        (void)sub_motion_drive(base_permille, 0);

    } else if (s->on_line_l || s->on_line_r) {
        both_count = 0U;
        lost_count = 0U;

        if (state != RC_LINE_TRACKING) {
            if (state == RC_LINE_SEARCHING) {
                out.id = RC_EVT_LINE_REACQUIRED;
                (void)rc_event_publish(&out);
            }
            state = RC_LINE_TRACKING;
        }
        last_position = s->position;
        steer_from_position(s->position);

    } else {
        both_count = 0U;
        lost_count++;

        if (state == RC_LINE_SEARCHING) {
            /* Deliberately off-line. Keep whatever sub_scan commanded,
             * do not steer and do not declare it lost. */
            return;
        }

        if (lost_count >= LOST_THRESHOLD) {
            if (state != RC_LINE_LOST) {
                state = RC_LINE_LOST;
                out.id = RC_EVT_LINE_LOST;
                (void)rc_event_publish(&out);
            }
            /* Arc toward the side the line was last seen on. A car that
             * drives straight when it loses the line never finds it
             * again; one that arcs back usually does. */
            (void)sub_motion_drive(base_permille / 2,
                                   (last_position < 0) ? -400 : 400);
        } else {
            /* Brief dropout, hold course. */
            steer_from_position(last_position);
        }
    }
}

/* ------------------------------------------------------------------ */

rc_result_t sub_line_init(void)
{
    if (rc_event_subscribe(RC_EVT_LINE_SAMPLE, RC_LANE_FAST,
                           on_sample, NULL) < 0) {
        return RC_ERR_NOSPACE;
    }
    state   = RC_LINE_TRACKING;
    enabled = false;
    return RC_OK;
}

rc_result_t sub_line_enable(bool on)
{
    enabled    = on;
    lost_count = 0U;
    both_count = 0U;
    return RC_OK;
}

rc_result_t sub_line_begin_search(void)
{
    state      = RC_LINE_SEARCHING;
    lost_count = 0U;
    return RC_OK;
}

rc_line_state_t sub_line_state(void)
{
    return state;
}

rc_result_t sub_line_set_base(int16_t permille)
{
    base_permille = permille;
    return RC_OK;
}
