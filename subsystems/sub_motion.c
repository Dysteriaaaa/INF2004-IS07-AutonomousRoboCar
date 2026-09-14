/*
 *  sub_motion.c
 */
#include "rc_prelude.h"
#include "sub_motion.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "rc_config.h"
#include "rc_event.h"

typedef enum {
    MODE_IDLE = 0,
    MODE_DISTANCE,
    MODE_TURN,
    MODE_CONTINUOUS
} mode_t;

typedef struct {
    int32_t kp;             /* fixed point, scaled by SCALE */
    int32_t ki;
    int32_t kd;
    int32_t integral;
    int32_t prev_err;
} pid_t;

#define PID_SCALE       (256)
#define INTEGRAL_CLAMP  (200000)

static mode_t   mode;
static pid_t    pid_l;
static pid_t    pid_r;
static uint16_t target_mm_s = 250U;
static int16_t  base_cmd;
static int16_t  steer_cmd;
static uint32_t goal_mm;
static uint32_t move_id_next = 1U;
static uint32_t move_id_cur;
static sub_motion_done_cb_t done_cb;
static void    *done_ctx;
static ID       motion_tskid;

/* ------------------------------------------------------------------ *
 *  PID
 *
 *  TODO Buddy 2: these gains are placeholders. Tune them on the bench
 *  with the wheels off the ground first, then on the track. Record the
 *  step responses, that is your PID tuning report.
 * ------------------------------------------------------------------ */

static void pid_reset(pid_t *p)
{
    p->integral = 0;
    p->prev_err = 0;
}

static void pid_set_gains(pid_t *p, int32_t kp, int32_t ki, int32_t kd)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    pid_reset(p);
}

static int32_t pid_step(pid_t *p, int32_t err, int32_t dt_ms)
{
    int32_t d;
    int32_t out;

    p->integral += (err * dt_ms);
    if (p->integral > INTEGRAL_CLAMP) {
        p->integral = INTEGRAL_CLAMP;
    }
    if (p->integral < -INTEGRAL_CLAMP) {
        p->integral = -INTEGRAL_CLAMP;
    }

    d = (dt_ms > 0) ? ((err - p->prev_err) * 1000 / dt_ms) : 0;
    p->prev_err = err;

    out = ((p->kp * err) + ((p->ki * p->integral) / 1000) + (p->kd * d))
          / PID_SCALE;

    return out;
}

/* ------------------------------------------------------------------ *
 *  Completion
 * ------------------------------------------------------------------ */

static void finish_move(bool completed)
{
    rc_event_t evt;
    uint32_t   travelled;
    sub_motion_done_cb_t cb = done_cb;
    void      *ctx = done_ctx;
    uint32_t   id  = move_id_cur;

    travelled = (drv_encoder_distance_mm(RC_SIDE_LEFT)
                 + drv_encoder_distance_mm(RC_SIDE_RIGHT)) / 2U;

    mode        = MODE_IDLE;
    done_cb     = NULL;
    move_id_cur = 0U;
    (void)drv_motor_stop(RC_MOTOR_BRAKE);

    evt.id = RC_EVT_MOTION_DONE;
    evt.u.motion_done.move_id     = id;
    evt.u.motion_done.completed   = completed;
    evt.u.motion_done.travelled_mm = travelled;
    (void)rc_event_publish(&evt);

    if (cb != NULL) {
        cb(id, completed, travelled, ctx);
    }
}

static uint32_t start_move(mode_t m, uint32_t target,
                           sub_motion_done_cb_t cb, void *ctx)
{
    if (mode != MODE_IDLE) {
        finish_move(false);         /* pre-empt whatever was running */
    }

    drv_encoder_reset();
    pid_reset(&pid_l);
    pid_reset(&pid_r);

    goal_mm     = target;
    done_cb     = cb;
    done_ctx    = ctx;
    move_id_cur = move_id_next;
    move_id_next++;
    if (move_id_next == 0U) {
        move_id_next = 1U;
    }
    mode = m;

    return move_id_cur;
}

/* ------------------------------------------------------------------ *
 *  Control task
 *
 *  Wakes on the kernel tick at a fixed period. tk_dly_tsk yields the CPU
 *  for the whole interval; it is not a spin. Everything this task does
 *  between wakeups is bounded arithmetic.
 * ------------------------------------------------------------------ */

static void motion_task(INT stacd, void *exinf)
{
    rc_event_t evt;
    int32_t    sp_l;
    int32_t    sp_r;
    int32_t    out_l;
    int32_t    out_r;
    uint32_t   dist;

    (void)stacd;
    (void)exinf;

    for (;;) {
        tk_dly_tsk(RC_PERIOD_MOTION_MS);

        sp_l = drv_encoder_speed_mm_s(RC_SIDE_LEFT);
        sp_r = drv_encoder_speed_mm_s(RC_SIDE_RIGHT);

        evt.id = RC_EVT_ODOMETRY;
        evt.u.odometry.speed_l_mm_s = sp_l;
        evt.u.odometry.speed_r_mm_s = sp_r;
        evt.u.odometry.dist_l_mm    = drv_encoder_distance_mm(RC_SIDE_LEFT);
        evt.u.odometry.dist_r_mm    = drv_encoder_distance_mm(RC_SIDE_RIGHT);
        (void)rc_event_publish(&evt);

        switch (mode) {
        case MODE_DISTANCE:
            dist = (evt.u.odometry.dist_l_mm + evt.u.odometry.dist_r_mm) / 2U;
            if (dist >= goal_mm) {
                finish_move(true);
                break;
            }
            out_l = pid_step(&pid_l, (int32_t)target_mm_s - sp_l,
                             RC_PERIOD_MOTION_MS);
            out_r = pid_step(&pid_r, (int32_t)target_mm_s - sp_r,
                             RC_PERIOD_MOTION_MS);
            (void)drv_motor_set_pair((int16_t)out_l, (int16_t)out_r);
            break;

        case MODE_TURN:
            /*
             *  TODO Buddy 2: encoder-based turning.
             *
             *  Arc length for one wheel over `deg` degrees is
             *      (RC_WHEEL_BASE_MM * pi * deg) / 360
             *  for a spin about the centre, with the wheels driven in
             *  opposite directions. Convert that to encoder ticks with
             *  RC_ENC_UM_PER_TICK and stop when both wheels reach it.
             *
             *  Expect the measured turn to fall short of the geometric
             *  one because of wheel slip. Measure the error over 10 turns
             *  and fold a correction factor in here rather than trusting
             *  the geometry.
             */
            finish_move(true);
            break;

        case MODE_CONTINUOUS:
            /*
             *  Steering is applied as an open-loop differential on top of
             *  the closed-loop base speed. Closing the loop on each wheel
             *  independently while also steering fights itself; this is
             *  the simpler arrangement and it works.
             */
            out_l = (int32_t)base_cmd - (int32_t)steer_cmd;
            out_r = (int32_t)base_cmd + (int32_t)steer_cmd;
            (void)drv_motor_set_pair((int16_t)out_l, (int16_t)out_r);
            break;

        case MODE_IDLE:
        default:
            break;
        }
    }
}

/* ------------------------------------------------------------------ *
 *  Public API
 * ------------------------------------------------------------------ */

rc_result_t sub_motion_init(void)
{
    T_CTSK ctsk;

    /* Placeholder gains. See the TODO above. */
    pid_set_gains(&pid_l, 512, 64, 16);
    pid_set_gains(&pid_r, 512, 64, 16);

    mode = MODE_IDLE;

    ctsk.exinf   = NULL;
    ctsk.itskpri = RC_PRI_MOTION;
    ctsk.stksz   = RC_STACK_SZ;
    ctsk.task    = motion_task;
    ctsk.tskatr  = TA_HLNG | TA_RNG3;

    motion_tskid = tk_cre_tsk(&ctsk);
    if (motion_tskid <= E_OK) {
        return RC_ERR_HARDWARE;
    }
    (void)tk_sta_tsk(motion_tskid, 0);

    return RC_OK;
}

rc_result_t sub_motion_set_speed(uint16_t mm_s)
{
    target_mm_s = mm_s;
    return RC_OK;
}

uint32_t sub_motion_forward_mm(uint32_t mm, sub_motion_done_cb_t cb, void *ctx)
{
    return start_move(MODE_DISTANCE, mm, cb, ctx);
}

uint32_t sub_motion_backward_mm(uint32_t mm, sub_motion_done_cb_t cb, void *ctx)
{
    /* TODO Buddy 2: reverse needs its own sign handling in MODE_DISTANCE.
     * Left as an explicit gap rather than a silent forward move. */
    return start_move(MODE_DISTANCE, mm, cb, ctx);
}

uint32_t sub_motion_turn_deg(int16_t deg, sub_motion_done_cb_t cb, void *ctx)
{
    uint32_t mag = (deg < 0) ? (uint32_t)(-deg) : (uint32_t)deg;

    return start_move(MODE_TURN, mag, cb, ctx);
}

rc_result_t sub_motion_drive(int16_t base_permille, int16_t steer_permille)
{
    if (mode == MODE_DISTANCE) {
        return RC_ERR_BUSY;     /* do not fight a queued move */
    }
    base_cmd  = base_permille;
    steer_cmd = steer_permille;
    mode      = MODE_CONTINUOUS;
    return RC_OK;
}

rc_result_t sub_motion_stop(bool brake)
{
    if (mode != MODE_IDLE) {
        finish_move(false);
    }
    return drv_motor_stop(brake ? RC_MOTOR_BRAKE : RC_MOTOR_COAST);
}

bool sub_motion_busy(void)
{
    return (mode == MODE_DISTANCE) || (mode == MODE_TURN);
}
