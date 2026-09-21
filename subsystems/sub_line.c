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
/* "U" suffix on a number literal means "unsigned" — it tells the compiler
 * this is an unsigned integer constant, matching the uint32_t counters
 * below (mixing signed and unsigned in comparisons is a classic bug
 * source, so the codebase is careful to match them). */
#define LOST_THRESHOLD      (20U)

/* Both sensors on the line for this long means a junction or a barcode
 * lead-in, not a centred car. */
#define JUNCTION_THRESHOLD  (6U)

/* `static` at file scope means "only visible inside this .c file" — these
 * globals are this module's private memory, invisible to (and unable to
 * clash with) globals of the same name in other subsystems. This is how
 * C fakes "private" state without classes. */
/* current line-follow mode, see sub_line.h enum */
static rc_line_state_t state = RC_LINE_TRACKING;
/* is steering actually allowed right now? (see sub_line_enable) */
static bool     enabled;
/* forward speed while tracking, in permille (parts per 1000 of full speed) */
static int16_t  base_permille = 350;
/* consecutive samples with neither sensor on the line */
static uint32_t lost_count;
/* consecutive samples with both sensors on the line (junction candidate) */
static uint32_t both_count;
/* most recent position reading, kept so we know which way to arc if the line
 * vanishes */
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

/*
 * STEER_KP - the steering "proportional gain" (the "P" in a PID
 * controller; "KP" = "K, proportional", the conventional name for this
 * number in control theory). See sub_motion.c for the fuller PID
 * explanation (proportional / integral / derivative) — this module only
 * uses the proportional term.
 *
 * What it is multiplied by: the raw sideways "position" error reported by
 * drv_ir_sample_line() — how far off-centre the car currently is. With
 * only two digital (on/off) sensors, position can only be one of four
 * values: -500 (drifted right, steer left), 0 (centred), +500 (drifted
 * left, steer right), or "unknown/lost" (handled separately in
 * on_sample()). See steer_from_position() below for the exact formula.
 *
 * Why the value is 2: it is a small whole number chosen so the resulting
 * steer command is a gentle correction rather than a violent one, while
 * keeping the arithmetic in plain integers (no floating point — the
 * RP2040 has no hardware FPU, see TEAM_GUIDE.md §5). With position at its
 * maximum of 500 and KP=2, steer_from_position() produces a steer value
 * of 500 (out of the same -1000..1000 permille range base_permille uses),
 * i.e. a firm but not maximal correction.
 *
 * What happens if you raise/lower it:
 *   - Raise it (e.g. to 4): the car reacts more aggressively to being
 *     off-centre. It snaps back toward the line faster, but overshoots
 *     the centre more and can start oscillating/weaving side to side
 *     (this is the classic "gain too high" problem in any P controller).
 *   - Lower it (e.g. to 1): the car reacts more gently, so it drifts
 *     further before correcting and may run wide on sharp curves — but
 *     it will feel smoother in a straight line.
 *   - This module has no derivative term (see the TODO comment above) to
 *     damp overshoot, which is exactly why weaving shows up at higher KP
 *     values — there's nothing here to counteract it yet.
 *
 * How it connects to the rest of the module: steer_from_position() is
 * called from on_sample() every time a new sensor reading arrives (and
 * also during brief dropouts, to hold the last known course). Its output
 * becomes the `steer_permille` argument to sub_motion_drive() (Buddy 2's
 * module), which biases the left/right wheel speeds to actually turn the
 * car — this function only decides *how hard* to steer, it does not touch
 * the motors directly.
 */
#define STEER_KP        (2)

/* Turns a raw sideways position error into a motor steering command.
 * `position` is a signed number: negative means "drifted right of the
 * line, so steer left"; positive means "drifted left, so steer right"
 * (see drv_ir_sample_line() in drv_ir.c for exactly how that number is
 * produced from the two on/off sensors). */
static void steer_from_position(int16_t position)
{
    /* Proportional control: steer_output = position_error * KP.
     * The intermediate multiplication is done in int32_t (a wider,
     * 32-bit signed integer) instead of int16_t so that
     * position * STEER_KP cannot silently overflow a 16-bit value before
     * we divide it back down — a common C gotcha when multiplying two
     * small integers together. The final "/ 2" scales the result back
     * into the same -1000..1000 permille range sub_motion_drive()
     * expects for a steering bias. The result is cast back down to
     * int16_t only at the very end, once it's safely back in range. */
    int16_t steer = (int16_t)((int32_t)position * STEER_KP / 2);

    /* Hand the steering bias to Buddy 2's motion module: drive forward at
     * our current base speed while biasing left/right wheel power by
     * `steer`. (void) in front just tells the compiler "yes, I know this
     * function returns a status code, and I'm deliberately ignoring it
     * here" — it silences an "unused return value" warning. */
    (void)sub_motion_drive(base_permille, steer);
}

/* ------------------------------------------------------------------ *
 *  Sample callback. Dispatcher task context, fast lane.
 * ------------------------------------------------------------------ */

/* Called automatically (via the event bus, see TEAM_GUIDE.md §1.2) every
 * time drv_ir_sample_line() publishes a new reading, roughly every 5ms.
 * This is the whole "brain" of line following: it looks at which
 * sensor(s) currently see the line and decides whether to steer, arc
 * back to search, or declare the line lost. */
static void on_sample(const rc_event_t *evt, void *ctx)
{
    /* `evt` is a pointer to the event data — a pointer just holds the
     * memory address of the actual struct rather than copying the whole
     * struct, which is cheaper. `&evt->u.line` takes the address of the
     * "line" field inside the event's union (a union is a block of
     * memory that different event types share/reinterpret depending on
     * evt->id) and `s` is a pointer to that same data so we can read its
     * fields via `s->...`. */
    const rc_pl_line_t *s = &evt->u.line;
    rc_event_t          out;

    /* This callback's `ctx` (user context pointer) isn't needed here; the
     * (void) cast tells the compiler that's deliberate, not an oversight,
     * so it won't warn about an unused parameter. */
    (void)ctx;

    if (!enabled) {
        return;
    }

    if (s->on_line_l && s->on_line_r) {
        /* Both sensors see black at once -> either a wide junction/
         * barcode lead-in, or briefly, a centred car crossing a gap.
         * We don't jump to "junction" on a single sample; we require
         * JUNCTION_THRESHOLD consecutive ones first (debouncing). */
        both_count++;
        lost_count = 0U;

        if (both_count >= JUNCTION_THRESHOLD) {
            state = RC_LINE_JUNCTION;
        }
        /* Drive straight through. A junction is where the barcode
         * usually starts, so sub_barcode arms itself off this state. */
        (void)sub_motion_drive(base_permille, 0);

    } else if (s->on_line_l || s->on_line_r) {
        /* Exactly one sensor sees black: normal tracking case. Reset the
         * junction/lost counters since we're clearly not in either
         * situation, remember which side is currently reporting the line
         * (used later if we lose it), and steer proportionally. */
        both_count = 0U;
        lost_count = 0U;

        if (state != RC_LINE_TRACKING) {
            if (state == RC_LINE_SEARCHING) {
                /* We were deliberately off-line (e.g. driving around an
                 * obstacle) and just found it again — tell anyone
                 * listening (e.g. sub_scan) that the search succeeded. */
                out.id = RC_EVT_LINE_REACQUIRED;
                (void)rc_event_publish(&out);
            }
            state = RC_LINE_TRACKING;
        }
        last_position = s->position;
        steer_from_position(s->position);

    } else {
        /* Neither sensor sees black. Count consecutive misses rather than
         * reacting instantly, so a single sensor glitch or a small gap in
         * the line doesn't immediately trigger "lost" behaviour. */
        both_count = 0U;
        lost_count++;

        if (state == RC_LINE_SEARCHING) {
            /* Deliberately off-line. Keep whatever sub_scan commanded,
             * do not steer and do not declare it lost. */
            return;
        }

        if (lost_count >= LOST_THRESHOLD) {
            /* Officially lost now. Only publish the "line lost" event on
             * the transition into this state (not every sample), so
             * listeners see one clean notification rather than a flood. */
            if (state != RC_LINE_LOST) {
                state = RC_LINE_LOST;
                out.id = RC_EVT_LINE_LOST;
                (void)rc_event_publish(&out);
            }
            /* Arc toward the side the line was last seen on. A car that
             * drives straight when it loses the line never finds it
             * again; one that arcs back usually does. The ternary
             * `condition ? a : b` below picks -400 (steer left) if the
             * last known position was negative, otherwise +400 (steer
             * right) — same sign convention as steer_from_position()'s
             * `position` argument above. base_permille / 2 slows down
             * while searching so the arc is tighter and safer. */
            (void)sub_motion_drive(base_permille / 2,
                                   (last_position < 0) ? -400 : 400);
        } else {
            /* Brief dropout (fewer than LOST_THRESHOLD misses so far) —
             * not worth reacting to yet. Just keep steering as if the
             * last known position still holds, riding over small gaps in
             * the line without panicking. */
            steer_from_position(last_position);
        }
    }
}

/* ------------------------------------------------------------------ */

/* Call once at boot. Registers on_sample() to be called automatically
 * whenever a new RC_EVT_LINE_SAMPLE event is published (i.e. every time
 * drv_ir_sample_line() runs) — this is the publish/subscribe pattern from
 * TEAM_GUIDE.md §1.2. RC_LANE_FAST means this reacts quickly rather than
 * queuing behind slower background work like telemetry. */
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

/* Turns steering on/off. While disabled, on_sample() still updates state
 * (so we know what's happening) but never calls sub_motion_drive() —
 * used when another subsystem (e.g. obstacle avoidance) needs full
 * control of the wheels. */
rc_result_t sub_line_enable(bool on)
{
    enabled    = on;
    lost_count = 0U;
    both_count = 0U;
    return RC_OK;
}

/* Tell this module "we just left the line on purpose" so it looks for the
 * line again instead of immediately declaring it lost. */
rc_result_t sub_line_begin_search(void)
{
    state      = RC_LINE_SEARCHING;
    lost_count = 0U;
    return RC_OK;
}

/* Reports the current line-follow mode (tracking / lost / searching /
 * junction) so other modules (e.g. sub_barcode, sub_nav) can react. */
rc_line_state_t sub_line_state(void)
{
    return state;
}

/* Sets the forward cruising speed used while tracking, in permille (parts
 * per thousand of full speed — e.g. 350 means 35.0%). Permille is used
 * everywhere in this codebase instead of a fraction/percent-with-decimals
 * because the RP2040 has no hardware floating point (see TEAM_GUIDE.md
 * §5), so everything is kept as whole numbers. */
rc_result_t sub_line_set_base(int16_t permille)
{
    base_permille = permille;
    return RC_OK;
}
