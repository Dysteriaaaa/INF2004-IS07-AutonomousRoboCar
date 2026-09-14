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
#define PITCH_ENTER_DDEG    (40)     /* 4.0 degrees */
#define PITCH_EXIT_DDEG     (20)     /* 2.0 degrees */
#define HUMP_MIN_MS         (150UL)
#define HUMP_MAX_MS         (5000UL)

/* Anything above this on any axis is a collision, not driving. */
#define IMPACT_MG           (2200)

typedef enum {
    H_FLAT = 0,
    H_CLIMBING,
    H_DESCENDING
} hump_state_t;

static hump_state_t      h_state;
static uint32_t          h_start_ms;
static int16_t           h_peak_pitch;
static uint16_t          max_peak_mm;
static rc_motion_class_t m_class;
static uint32_t          dist_at_hump_start;
static sub_terrain_hump_cb_t user_cb;
static void             *user_ctx;

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
static uint16_t pitch_to_height_mm(int16_t peak_ddeg)
{
    int32_t rad_milli;

    if (peak_ddeg < 0) {
        peak_ddeg = (int16_t)(-peak_ddeg);
    }

    /* tenths of a degree to milliradians: ddeg * pi / 1800 */
    rad_milli = ((int32_t)peak_ddeg * 1745) / 1000;

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

static void classify(const rc_pl_imu_t *s, int16_t pitch)
{
    rc_motion_class_t next = m_class;
    int32_t           ax = s->acc_x;
    int32_t           sp_l = drv_encoder_speed_mm_s(RC_SIDE_LEFT);
    int32_t           sp_r = drv_encoder_speed_mm_s(RC_SIDE_RIGHT);
    int32_t           diff = sp_l - sp_r;
    rc_event_t        evt;

    if ((ax > IMPACT_MG) || (ax < -IMPACT_MG)) {
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
        next = RC_MOTION_TURNING;
    } else if (ax > 200) {
        next = RC_MOTION_ACCELERATING;
    } else if (ax < -200) {
        next = RC_MOTION_DECELERATING;
    } else {
        next = RC_MOTION_CRUISING;
    }

    (void)pitch;

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

static void on_sample(const rc_event_t *evt, void *ctx)
{
    int16_t    pitch = drv_imu_pitch_ddeg();
    uint32_t   now   = rc_time_ms();
    rc_event_t out;
    uint32_t   duration;
    uint16_t   height;

    (void)ctx;

    switch (h_state) {
    case H_FLAT:
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
        if (pitch > h_peak_pitch) {
            h_peak_pitch = pitch;
        }
        if (pitch < -PITCH_ENTER_DDEG) {
            h_state = H_DESCENDING;
        } else if ((now - h_start_ms) > HUMP_MAX_MS) {
            h_state = H_FLAT;       /* a long slope, not a hump */
        } else {
            /* still climbing */
        }
        break;

    case H_DESCENDING:
    default:
        if ((pitch > -PITCH_EXIT_DDEG) && (pitch < PITCH_EXIT_DDEG)) {
            duration = now - h_start_ms;
            h_state  = H_FLAT;

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

                if (user_cb != NULL) {
                    user_cb(height, duration, user_ctx);
                }
            }
        }
        break;
    }

    classify(&evt->u.imu, pitch);
}

/* ------------------------------------------------------------------ */

rc_result_t sub_terrain_init(void)
{
    sub_terrain_reset();

    if (rc_event_subscribe(RC_EVT_IMU_SAMPLE, RC_LANE_FAST,
                           on_sample, NULL) < 0) {
        return RC_ERR_NOSPACE;
    }
    return RC_OK;
}

rc_result_t sub_terrain_on_hump(sub_terrain_hump_cb_t cb, void *ctx)
{
    user_cb  = cb;
    user_ctx = ctx;
    return RC_OK;
}

uint16_t sub_terrain_max_peak_mm(void)
{
    return max_peak_mm;
}

rc_motion_class_t sub_terrain_motion_class(void)
{
    return m_class;
}

void sub_terrain_reset(void)
{
    h_state      = H_FLAT;
    h_peak_pitch = 0;
    max_peak_mm  = 0U;
    m_class      = RC_MOTION_STATIONARY;
}
