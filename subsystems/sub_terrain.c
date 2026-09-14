/*
 *  sub_terrain.c
 */
#include "rc_prelude.h"
#include "sub_terrain.h"
#include "drv_imu.h"
#include "drv_encoder.h"
#include "rc_config.h"
#include "rc_event.h"
#include "rc_time.h"

/*
 *  Hump detection thresholds, in tenths of a degree of pitch.
 *
 *  A hump shows up as a clean pitch-up, a plateau, then a pitch-down.
 *  Cornering and acceleration also tilt the car, so the entry threshold
 *  has to sit above normal driving noise. Measure yours: drive the car
 *  hard on flat ground, log the pitch, and set ENTER above the peak you
 *  see there.
 */
/* Pitch angle (nose-up tilt) that must be crossed before we believe the
 * car has started climbing a hump, in tenths of a degree (40 = 4.0
 * degrees). Used in on_sample() below. Needs to sit comfortably above
 * the pitch noise seen during normal acceleration/braking/cornering on
 * flat ground - see "Tune hump thresholds empirically" in TEAM_GUIDE.md. */
#define PITCH_ENTER_DDEG    (40)     /* 4.0 degrees */
/* Pitch angle the car must settle back within before we consider the
 * hump finished (smaller than PITCH_ENTER_DDEG on purpose - this gap
 * between "enter" and "exit" thresholds, called hysteresis, stops small
 * pitch wobble right at the threshold from flickering hump-start/hump-end
 * on and off rapidly). */
#define PITCH_EXIT_DDEG     (20)     /* 2.0 degrees */
/* A climb/descend shorter than this (milliseconds) is too fast to be a
 * real hump - probably sensor noise - and gets discarded in on_sample(). */
#define HUMP_MIN_MS         (150UL)
/* A climb longer than this without cresting is treated as a long slope,
 * not a hump, and the state machine resets to flat rather than waiting
 * forever - see the H_CLIMBING case in on_sample(). */
#define HUMP_MAX_MS         (5000UL)

/* Anything above this on any axis is a collision, not driving.
 * Units: milli-g (thousandths of Earth's gravity) - see drv_imu.h. A
 * sudden jolt this large (2.2 g) is far beyond normal acceleration or
 * braking, so it's treated as an impact in classify() below. */
#define IMPACT_MG           (2200)

/* The three phases of the hump state machine (see on_sample() below):
 * an `enum` just gives readable names to a small set of whole-number
 * states, e.g. H_FLAT is really the number 0 under the hood, H_CLIMBING
 * is 1, H_DESCENDING is 2 - but writing H_CLIMBING in code is far less
 * error-prone than remembering "1 means climbing." */
typedef enum {
    H_FLAT = 0,       /* not currently on a hump - the normal/default state */
    H_CLIMBING,       /* pitch has crossed PITCH_ENTER_DDEG nose-up; front wheels are going up the ramp */
    H_DESCENDING       /* pitch has swung past crest and gone nose-down; car is coming back down */
} hump_state_t;

/* `static` module-level variables: private state remembered between
 * calls, visible only inside this file (see the same pattern explained
 * in drv_imu.c). Together these track "where are we in a hump right
 * now" and "what's the car doing overall." */
static hump_state_t      h_state;             /* current phase of the hump state machine */
static uint32_t          h_start_ms;           /* rc_time_ms() timestamp when the current hump/climb began */
static int16_t           h_peak_pitch;         /* steepest pitch (tenths of a degree) seen so far this hump */
static uint16_t          max_peak_mm;           /* tallest hump height (mm) seen across the whole run so far */
static rc_motion_class_t m_class;               /* current motion category, e.g. cruising/turning/climbing */
static uint32_t          dist_at_hump_start;    /* left-wheel odometry distance (mm) recorded when the hump began */
static sub_terrain_hump_cb_t user_cb;           /* optional caller-registered "hump finished" callback, or NULL */
static void             *user_ctx;              /* opaque pointer handed back unchanged to user_cb, see sub_terrain.h */

/* ------------------------------------------------------------------ *
 *  Peak height estimate
 *
 *  Double-integrating vertical acceleration does not work here. Over a
 *  two second climb the bias drift in a 12 bit accelerometer produces an
 *  error larger than the hump itself.
 *
 *  What does work: the car is a rigid body on a known wheelbase. While
 *  the front wheels are on the ramp and the rear are not, the pitch
 *  angle and the wheelbase give the height difference directly:
 *
 *      rise = wheelbase * sin(pitch)
 *
 *  and for the small angles involved, sin(pitch) is pitch in radians to
 *  within a couple of percent.
 *
 *  This measures the height of the ramp under the car, which is the
 *  hump's peak once the car has climbed the whole thing. It is an
 *  estimate, not a measurement, and you should say so in the terrain
 *  analysis report. Validate it against a ruler on three known humps and
 *  quote the error.
 *
 *  TODO Buddy 4: an independent cross-check worth adding. The distance
 *  travelled between hump entry and the pitch peak, times sin(pitch),
 *  gives a second estimate of the same height from encoder data. If the
 *  two disagree badly, one of your thresholds is wrong.
 */
/* Converts a peak pitch angle into an estimated hump height in
 * millimetres. See the big comment block above for the geometry
 * (rise = wheelbase * sin(pitch), sin(pitch) ~= pitch in radians for
 * small angles). Walking through the actual numbers: */
static uint16_t pitch_to_height_mm(int16_t peak_ddeg)
{
    int32_t rad_milli;

    /* Height doesn't care whether the car was pitching up or down at its
     * peak (a negative peak_ddeg can happen mid-transition) - only the
     * *size* of the tilt matters for how tall the ramp was, so take the
     * absolute value first. */
    if (peak_ddeg < 0) {
        peak_ddeg = (int16_t)(-peak_ddeg);
    }

    /* tenths of a degree to milliradians: ddeg * pi / 1800
     *
     * `peak_ddeg` is in tenths of a degree (e.g. 40 = 4.0 degrees). To
     * use it in the small-angle formula we first need it in radians (the
     * angle unit where 2*pi radians = 360 degrees, the unit sin/rise-run
     * geometry naturally uses), scaled up by 1000 to stay a whole number
     * ("milliradians" = thousandths of a radian - same fixed-point
     * trick as everywhere else in this codebase, since there's no FPU).
     * The conversion factor pi/1800 (degrees-in-tenths to radians)
     * rounds to 1745/1,000,000, which is why the code multiplies by 1745
     * and divides by 1000 - that combination gives milliradians with
     * good integer precision without ever needing a fractional pi. */
    rad_milli = ((int32_t)peak_ddeg * 1745) / 1000;

    /* rise = wheelbase * sin(pitch) ~= wheelbase * pitch_in_radians.
     * RC_WHEEL_BASE_MM is the front-to-back wheel distance in mm (see
     * rc_config.h, set from a real measurement of the car - Buddy 2's
     * territory). Multiplying it by the milliradian angle and dividing
     * by 1000 (to undo the earlier x1000 scaling) gives the estimated
     * rise in millimetres - i.e. how tall the ramp under the car is at
     * the peak of the climb, which is the hump's height. */
    return (uint16_t)(((int32_t)RC_WHEEL_BASE_MM * rad_milli) / 1000);
}

/* ------------------------------------------------------------------ *
 *  Motion classification
 *
 *  TODO Buddy 4: this is deliberately simple. The brief lists six motion
 *  events and this covers five of them. "Turning" is the one that needs
 *  work, because without a gyro the honest source is the encoder
 *  difference, not the IMU. Wire drv_encoder_speed_mm_s for both sides
 *  in here and classify a turn when they diverge past a threshold.
 * ------------------------------------------------------------------ */

/* Decides what single word best describes what the car is doing right
 * now, from the current IMU sample plus wheel-encoder speeds. Checked in
 * priority order (impact beats everything, hump state beats plain
 * driving, etc) - only the first matching case wins. `s` is a pointer to
 * the just-published IMU sample (the `const` means this function
 * promises not to modify it - it only reads). */
static void classify(const rc_pl_imu_t *s, int16_t pitch)
{
    rc_motion_class_t next = m_class;
    int32_t           ax = s->acc_x;   /* `->` reads a field through a pointer - same as (*s).acc_x */
    /* No gyroscope means no direct way to sense "the car is turning."
     * The honest substitute: if the left wheel is spinning faster than
     * the right (or vice versa), the car must be turning - the same way
     * a tank or wheelchair turns by driving its two sides at different
     * speeds. This borrows Buddy 2's encoder-speed driver rather than
     * inventing a second, less reliable, IMU-based estimate. */
    int32_t           sp_l = drv_encoder_speed_mm_s(RC_SIDE_LEFT);
    int32_t           sp_r = drv_encoder_speed_mm_s(RC_SIDE_RIGHT);
    int32_t           diff = sp_l - sp_r;
    rc_event_t        evt;

    if ((ax > IMPACT_MG) || (ax < -IMPACT_MG)) {
        /* A jolt this big on the forward/backward axis, in either
         * direction, means the car hit something - report it immediately
         * and don't bother checking anything else this sample. */
        next = RC_MOTION_IMPACT;
        evt.id = RC_EVT_IMPACT;
        (void)rc_event_publish(&evt);
    } else if (h_state == H_CLIMBING) {
        next = RC_MOTION_CLIMBING;
    } else if (h_state == H_DESCENDING) {
        next = RC_MOTION_DESCENDING;
    } else if ((sp_l == 0) && (sp_r == 0)) {
        next = RC_MOTION_STATIONARY;
    } else if ((diff > 150) || (diff < -150)) {
        /* TODO Buddy 4: 150 (mm/s difference between wheels) is a
         * placeholder guessed threshold, not yet validated against real
         * turning tests - see TEAM_GUIDE.md TODOs. */
        next = RC_MOTION_TURNING;
    } else if (ax > 200) {
        next = RC_MOTION_ACCELERATING;
    } else if (ax < -200) {
        next = RC_MOTION_DECELERATING;
    } else {
        next = RC_MOTION_CRUISING;
    }

    (void)pitch;  /* not used for classification (yet) - silences an unused-parameter warning */

    /* Only publish an event when the class actually changed, so
     * subscribers aren't spammed with "still cruising" every single
     * sample - just the moments the answer changes. */
    if (next != m_class) {
        m_class = next;
        evt.id  = RC_EVT_MOTION_CLASS;
        evt.u.motion_class.cls = next;
        (void)rc_event_publish(&evt);
    }
}

/* ------------------------------------------------------------------ *
 *  Sample callback. Dispatcher task context, fast lane.
 * ------------------------------------------------------------------ */

/* The heart of this module. Runs once every time drv_imu.c publishes a
 * new RC_EVT_IMU_SAMPLE (see rc_event_subscribe in sub_terrain_init()
 * below) - i.e. about 100 times a second. Advances the hump-detection
 * state machine (H_FLAT -> H_CLIMBING -> H_DESCENDING -> H_FLAT) and then
 * runs general motion classification every sample regardless of hump
 * state. `evt` is a pointer to the just-published event data; `ctx` is
 * the free-form context pointer that was passed to rc_event_subscribe
 * (unused here, hence the `(void)ctx;` below to silence an "unused
 * parameter" compiler warning). */
static void on_sample(const rc_event_t *evt, void *ctx)
{
    int16_t    pitch = drv_imu_pitch_ddeg();
    uint32_t   now   = rc_time_ms();
    rc_event_t out;
    uint32_t   duration;
    uint16_t   height;

    (void)ctx;

    /* `switch` picks one of the branches below based on the current
     * value of h_state - equivalent to a chain of if/else-if, but
     * clearer to read when there are exactly a handful of named states. */
    switch (h_state) {
    case H_FLAT:
        /* Not on a hump yet. Pitch crossing the "enter" threshold
         * nose-up means the front wheels have just started climbing a
         * ramp - start tracking a new hump: remember when it began, seed
         * the peak-so-far with this first reading, and snapshot the
         * current odometry distance (used by the TODO cross-check
         * mentioned in the big comment above pitch_to_height_mm). */
        if (pitch > PITCH_ENTER_DDEG) {
            h_state            = H_CLIMBING;
            h_start_ms         = now;
            h_peak_pitch       = pitch;
            dist_at_hump_start = drv_encoder_distance_mm(RC_SIDE_LEFT);

            out.id = RC_EVT_HUMP_BEGIN;
            (void)rc_event_publish(&out);
        }
        break;

    case H_CLIMBING:
        /* Keep tracking the steepest pitch seen - that peak is what
         * pitch_to_height_mm() will later convert into a height. */
        if (pitch > h_peak_pitch) {
            h_peak_pitch = pitch;
        }
        if (pitch < -PITCH_ENTER_DDEG) {
            /* Pitch has swung from nose-up to clearly nose-down: the car
             * has crested the hump and is now coming down the other
             * side. */
            h_state = H_DESCENDING;
        } else if ((now - h_start_ms) > HUMP_MAX_MS) {
            h_state = H_FLAT;       /* a long slope, not a hump */
        } else {
            /* still climbing */
        }
        break;

    case H_DESCENDING:
    default:
        /* Pitch has settled back close to level (within the narrower
         * "exit" band - see PITCH_EXIT_DDEG's comment on why it's
         * smaller than the enter threshold) - the hump is over. */
        if ((pitch > -PITCH_EXIT_DDEG) && (pitch < PITCH_EXIT_DDEG)) {
            duration = now - h_start_ms;
            h_state  = H_FLAT;

            /* Discard anything shorter than HUMP_MIN_MS - too quick to
             * be a real physical hump, more likely sensor jitter. */
            if (duration >= HUMP_MIN_MS) {
                height = pitch_to_height_mm(h_peak_pitch);
                if (height > max_peak_mm) {
                    max_peak_mm = height;
                }

                out.id = RC_EVT_HUMP_END;
                out.u.hump.peak_mm       = height;
                out.u.hump.duration_ms   = duration;
                out.u.hump.max_pitch_deg = (int16_t)(h_peak_pitch / 10);
                (void)rc_event_publish(&out);

                /* In addition to the event above, also call the
                 * caller-registered callback directly, if one was set
                 * via sub_terrain_on_hump() - this lets a caller get a
                 * plain function call instead of having to subscribe to
                 * the event bus themselves. */
                if (user_cb != NULL) {
                    user_cb(height, duration, user_ctx);
                }
            }
        }
        break;
    }

    /* Run general motion classification every sample, independent of
     * which hump-state branch ran above. `&evt->u.imu` takes the address
     * of the imu-specific part of the event payload (a `union` inside
     * rc_event_t that's interpreted differently depending on evt->id) to
     * pass to classify() as a pointer, avoiding a copy of the whole
     * struct. */
    classify(&evt->u.imu, pitch);
}

/* ------------------------------------------------------------------ */

/* Call once at boot. Resets internal state, then subscribes on_sample()
 * to RC_EVT_IMU_SAMPLE on the FAST lane (see TEAM_GUIDE.md section 0.2 on
 * the event bus/lanes) - fast because hump/impact detection should react
 * without delay, the same reasoning steering does. From this point on,
 * on_sample() runs automatically every time drv_imu_sample() publishes a
 * new reading; nothing else needs to call into this file directly. */
rc_result_t sub_terrain_init(void)
{
    sub_terrain_reset();

    if (rc_event_subscribe(RC_EVT_IMU_SAMPLE, RC_LANE_FAST,
                           on_sample, NULL) < 0) {
        return RC_ERR_NOSPACE;
    }
    return RC_OK;
}

/* Stores the caller's callback and context pointer for later - see the
 * sub_terrain_hump_cb_t comment in sub_terrain.h for what this shape
 * means and dist_at_hump_start/on_sample() above for where it gets
 * invoked. */
rc_result_t sub_terrain_on_hump(sub_terrain_hump_cb_t cb, void *ctx)
{
    user_cb  = cb;
    user_ctx = ctx;
    return RC_OK;
}

/* Simple getter: hands back the module-level max_peak_mm variable. */
uint16_t sub_terrain_max_peak_mm(void)
{
    return max_peak_mm;
}

/* Simple getter: hands back the module-level m_class variable. */
rc_motion_class_t sub_terrain_motion_class(void)
{
    return m_class;
}

/* Resets all tracked state back to "just booted, car is still" - used
 * both by sub_terrain_init() and by anyone wanting to start a fresh run
 * without rebooting. */
void sub_terrain_reset(void)
{
    h_state      = H_FLAT;
    h_peak_pitch = 0;
    max_peak_mm  = 0U;
    m_class      = RC_MOTION_STATIONARY;
}
