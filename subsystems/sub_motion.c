/*
 *  sub_motion.c
 *
 *  The "brain" of Buddy 2's module: decides what duty to send the motors
 *  every control cycle so the wheels track a target speed (PID control)
 *  or a target distance/turn, and runs the non-blocking move state
 *  machine described in sub_motion.h. See TEAM_GUIDE.md's Buddy 2 section
 *  for the full plain-English walkthrough.
 */
#include "rc_prelude.h"
#include "sub_motion.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "rc_config.h"
#include "rc_event.h"

/* The car's current "what am I doing right now" state. An "enum"
 * (enumeration) is just a named list of integer constants - using
 * MODE_IDLE etc. instead of raw 0/1/2/3 makes the switch statement below
 * self-explanatory instead of a mess of magic numbers. */
typedef enum {
    MODE_IDLE = 0,     /* motors off / no move in progress */
    MODE_DISTANCE,      /* driving straight toward a distance goal (PID-controlled speed) */
    MODE_TURN,          /* turning toward a target angle (currently a stub, see TODO below) */
    MODE_CONTINUOUS     /* open-ended drive+steer, used by the line follower */
} mode_t;

/* One PID controller's tuning gains and running state. There are two
 * instances of this (pid_l, pid_r below) - one per wheel - because each
 * wheel can need a slightly different push to hit the same target speed
 * (friction, motor variance, floor grip differ side to side).
 *   kp, ki, kd - the three tuning gains (Proportional / Integral /
 *                Derivative), "fixed point, scaled by PID_SCALE" means
 *                these are whole numbers that stand in for a fraction -
 *                e.g. a "gain of 0.5" is stored as 128 when PID_SCALE is
 *                256, and divided back out at the end of pid_step(). This
 *                trick lets the math stay in integers since the RP2040
 *                has no FPU (floating point hardware).
 *   integral   - running total of past error over time (the "I" term's
 *                memory - see pid_step() below for what it's for).
 *   prev_err   - the error measured last cycle, kept so pid_step() can
 *                work out how fast the error is changing (the "D" term).
 */
typedef struct {
    int32_t kp;             /* fixed point, scaled by SCALE */
    int32_t ki;
    int32_t kd;
    int32_t integral;
    int32_t prev_err;
} pid_t;

/* Fixed-point scale factor. Gains above are stored multiplied by this
 * number so they can be whole integers instead of fractions (e.g. a real
 * gain of 2.5 is stored as 640 when PID_SCALE is 256); pid_step() divides
 * the final result by this same number to undo the scaling. Necessary
 * because the RP2040 has no hardware floating point. */
#define PID_SCALE       (256)

/* Upper/lower bound on the integral term's running total (see pid_step()
 * below). Without a clamp, the integral term can grow without bound while
 * the car is stalled or blocked (a problem called "integral windup"),
 * causing a huge, sudden burst of power the moment it's freed. */
#define INTEGRAL_CLAMP  (200000)

/* All of sub_motion's live state. "static" at file scope means these
 * variables are private to this .c file - nothing outside can see or
 * change them directly, only through the public functions below. There's
 * exactly one car, so this is simpler than passing a struct pointer
 * around everywhere. */
static mode_t   mode;                 /* current state-machine mode, see mode_t above */
static pid_t    pid_l;                /* left wheel's PID controller */
static pid_t    pid_r;                /* right wheel's PID controller */
static uint16_t target_mm_s = 250U;   /* cruise speed target for distance moves */
static int16_t  base_cmd;             /* base duty for MODE_CONTINUOUS */
static int16_t  steer_cmd;            /* steering bias for MODE_CONTINUOUS */
static uint32_t goal_mm;              /* distance target for the current MODE_DISTANCE move */
static uint32_t move_id_next = 1U;    /* next move id to hand out (0 is reserved for "no move") */
static uint32_t move_id_cur;          /* id of whichever move is running now */
static sub_motion_done_cb_t done_cb;  /* callback to fire when the current move finishes */
static void    *done_ctx;             /* caller's context pointer, passed back unchanged to done_cb */
static ID       motion_tskid;         /* RTOS task id of the background control task, motion_task() */

/* ------------------------------------------------------------------ *
 *  PID
 *
 *  TODO Buddy 2: these gains are placeholders. Tune them on the bench
 *  with the wheels off the ground first, then on the track. Record the
 *  step responses, that is your PID tuning report.
 * ------------------------------------------------------------------ */

/* Clear a controller's memory of past error - called whenever a fresh
 * move starts, so leftover integral/derivative state from a previous
 * move can't bleed into the new one. */
static void pid_reset(pid_t *p)
{
    p->integral = 0;
    p->prev_err = 0;
}

/* Set one controller's tuning gains and reset its running state. */
static void pid_set_gains(pid_t *p, int32_t kp, int32_t ki, int32_t kd)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    pid_reset(p);
}

/* The actual PID control-loop math, run once per wheel per control
 * cycle. Think of a car's cruise control: it doesn't know in advance
 * exactly how much throttle produces exactly 60 km/h (that changes with
 * hills, wind, load) - instead it constantly measures your actual speed,
 * compares it to the speed you asked for, and nudges the throttle based
 * on three things:
 *   - err = target speed - actual speed (positive means "going too
 *     slow, push harder"; negative means "going too fast, ease off").
 *   - "Proportional" (P, uses kp) = react to how wrong we are *right
 *     now*. Bigger error -> bigger correction, immediately.
 *   - "Integral" (I, uses ki) = react to how long/consistently we've
 *     been wrong. If the wheel is *always* a little slow even with a
 *     steady P correction (e.g. because of constant friction the P term
 *     alone can't fully cancel), the integral term keeps accumulating
 *     that leftover error over time and adds extra push until the steady
 *     gap disappears.
 *   - "Derivative" (D, uses kd) = react to how fast the error is
 *     changing. If the error is closing very quickly, the derivative
 *     term leans against a sudden overshoot (like easing off the gas
 *     just before you actually hit your target speed, instead of
 *     slamming past it).
 *
 *  Parameters: p is which wheel's controller state to use; err is target
 *  speed minus current speed for this cycle (mm/s); dt_ms is how many
 *  milliseconds elapsed since the last call (used to scale the integral
 *  and derivative terms correctly regardless of the control loop's exact
 *  timing).
 */
static int32_t pid_step(pid_t *p, int32_t err, int32_t dt_ms)
{
    int32_t d;
    int32_t out;

    /* Integral term: accumulate error over time (error * how long it's
     * been true for), so a small but persistent error slowly builds up a
     * bigger and bigger correction. */
    p->integral += (err * dt_ms);
    /* Clamp so a long stall or blockage can't make this grow forever
     * ("integral windup") and then unleash a huge jolt once freed. */
    if (p->integral > INTEGRAL_CLAMP) {
        p->integral = INTEGRAL_CLAMP;
    }
    if (p->integral < -INTEGRAL_CLAMP) {
        p->integral = -INTEGRAL_CLAMP;
    }

    /* Derivative term: (how much the error changed) / (how much time
     * passed) = rate of change of the error, i.e. "is the error growing
     * or shrinking, and how fast." Multiplying by 1000 and dividing by
     * dt_ms converts "per millisecond" into "per second" while staying in
     * integer math (no FPU on this chip). Guarded against dt_ms == 0 to
     * avoid dividing by zero. */
    d = (dt_ms > 0) ? ((err - p->prev_err) * 1000 / dt_ms) : 0;
    p->prev_err = err;

    /* Combine all three terms, each scaled by its own gain, then divide
     * by PID_SCALE once at the end to undo the fixed-point scaling
     * described where pid_t/PID_SCALE are defined above. The integral
     * term is additionally divided by 1000 because it accumulates
     * err*dt_ms (a much larger-scale quantity) rather than raw err. */
    out = ((p->kp * err) + ((p->ki * p->integral) / 1000) + (p->kd * d))
          / PID_SCALE;

    return out;
}

/* ------------------------------------------------------------------ *
 *  Completion
 * ------------------------------------------------------------------ */

/* Wrap up whatever move is currently running: work out how far the car
 * actually travelled, reset state back to idle, stop the motors, publish
 * the RC_EVT_MOTION_DONE event for anyone listening on the bus, and call
 * the caller's own callback (if they gave one). Called both when a move
 * finishes normally and when it's cut short (stopped or pre-empted). */
static void finish_move(bool completed)
{
    rc_event_t evt;
    uint32_t   travelled;
    /* Copy these out to locals before clearing them below, since the
     * callback might (indirectly) trigger a new move that overwrites the
     * globals while we're still using the old values. */
    sub_motion_done_cb_t cb = done_cb;
    void      *ctx = done_ctx;
    uint32_t   id  = move_id_cur;

    /* Average the two wheels' distances for one "how far did the car go"
     * number - if one wheel slipped slightly more than the other, this
     * smooths that out rather than reporting one wheel's figure only. */
    travelled = (drv_encoder_distance_mm(RC_SIDE_LEFT)
                 + drv_encoder_distance_mm(RC_SIDE_RIGHT)) / 2U;

    mode        = MODE_IDLE;
    done_cb     = NULL;
    move_id_cur = 0U;
    (void)drv_motor_stop(RC_MOTOR_BRAKE);

    /* Publish on the shared event bus (see TEAM_GUIDE.md §0.2) so any
     * other subsystem - not just the caller who started this move - can
     * find out a move finished (e.g. telemetry logging it). */
    evt.id = RC_EVT_MOTION_DONE;
    evt.u.motion_done.move_id     = id;
    evt.u.motion_done.completed   = completed;
    evt.u.motion_done.travelled_mm = travelled;
    (void)rc_event_publish(&evt);

    if (cb != NULL) {
        cb(id, completed, travelled, ctx);
    }
}

/* Common setup shared by every "queued move" entry point
 * (forward/backward/turn): pre-empts any move already running, resets
 * the encoders and PID state so the new move starts clean, records the
 * caller's target/callback, hands out a fresh move id, and switches the
 * state machine into the requested mode. Returns the new move's id. */
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
        move_id_next = 1U;   /* skip 0: that value is reserved to mean "no move" */
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

/* The background RTOS task that drives the whole module. Runs forever on
 * a fixed timer (RC_PERIOD_MOTION_MS, 20ms / 50Hz - see rc_config.h),
 * reading the encoders, publishing odometry, and - depending on the
 * current mode - running the PID loop, the (stub) turn logic, or
 * open-loop steering. This is the only place motor duty gets decided;
 * every public API call above just sets state that this loop reads.
 *
 * stacd/exinf are standard T-Kernel task-entry parameters (start code /
 * extended information) that this task doesn't use - cast to (void) to
 * tell the compiler that's intentional, not an oversight. */
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
        /* Sleep (yield the CPU) for one control period. This is the
         * non-blocking pattern from TEAM_GUIDE.md §0.3 applied at the
         * task level: the task isn't spinning/busy-waiting, it's parked
         * by the RTOS and other tasks run in the meantime. */
        tk_dly_tsk(RC_PERIOD_MOTION_MS);

        /* Read both wheels' current speed from the encoder driver - see
         * drv_encoder_speed_mm_s() in drv_encoder.c. This is the "actual
         * speed" half of the PID loop's error calculation below. */
        sp_l = drv_encoder_speed_mm_s(RC_SIDE_LEFT);
        sp_r = drv_encoder_speed_mm_s(RC_SIDE_RIGHT);

        /* Publish a fresh odometry snapshot every single cycle,
         * regardless of mode, so anything listening (telemetry, Buddy 4's
         * terrain code) always has current numbers even if the car is
         * just idling. */
        evt.id = RC_EVT_ODOMETRY;
        evt.u.odometry.speed_l_mm_s = sp_l;
        evt.u.odometry.speed_r_mm_s = sp_r;
        evt.u.odometry.dist_l_mm    = drv_encoder_distance_mm(RC_SIDE_LEFT);
        evt.u.odometry.dist_r_mm    = drv_encoder_distance_mm(RC_SIDE_RIGHT);
        (void)rc_event_publish(&evt);

        /* The move state machine: what to do this cycle depends entirely
         * on `mode`, set by whichever public API call is currently
         * "in charge" of the car. */
        switch (mode) {
        case MODE_DISTANCE:
            /* Average both wheels' distance travelled so far this move;
             * once it reaches the goal, the move is done. */
            dist = (evt.u.odometry.dist_l_mm + evt.u.odometry.dist_r_mm) / 2U;
            if (dist >= goal_mm) {
                finish_move(true);
                break;
            }
            /* Not there yet: run one PID step per wheel. err = target
             * speed - actual speed; pid_step() (above) turns that into a
             * new duty command. Doing this per-wheel (not just once for
             * the average) is what keeps the two wheels matched even if
             * one has more friction than the other. */
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
            finish_move(true);   /* stub: currently ends the "turn" instantly, does not actually turn the car */
            break;

        case MODE_CONTINUOUS:
            /*
             *  Steering is applied as an open-loop differential on top of
             *  the closed-loop base speed. Closing the loop on each wheel
             *  independently while also steering fights itself; this is
             *  the simpler arrangement and it works.
             *
             *  "Open-loop" means: no feedback is used to correct the
             *  steering bias itself (unlike the PID speed control above,
             *  which *is* closed-loop) - we just directly compute
             *  left = base - steer, right = base + steer and send it.
             *  E.g. steering right (positive steer_cmd) speeds up the
             *  left wheel and slows the right one, pivoting the car
             *  rightward.
             */
            out_l = (int32_t)base_cmd - (int32_t)steer_cmd;
            out_r = (int32_t)base_cmd + (int32_t)steer_cmd;
            (void)drv_motor_set_pair((int16_t)out_l, (int16_t)out_r);
            break;

        case MODE_IDLE:
        default:
            break;   /* nothing to drive, motors already stopped */
        }
    }
}

/* ------------------------------------------------------------------ *
 *  Public API
 * ------------------------------------------------------------------ */

/* Boot-time setup: seeds placeholder PID gains for both wheels, then
 * creates and starts the background motion_task RTOS task that does all
 * the real work above. Call once at startup, before any other
 * sub_motion_* function. */
rc_result_t sub_motion_init(void)
{
    T_CTSK ctsk;   /* "create task" parameter struct required by the RTOS */

    /* Placeholder gains. See the TODO above. */
    pid_set_gains(&pid_l, 512, 64, 16);
    pid_set_gains(&pid_r, 512, 64, 16);

    mode = MODE_IDLE;

    /* Fill in the RTOS task descriptor: exinf is extra data passed to
     * the task entry function (unused here); itskpri is the task's
     * scheduling priority (lower number = higher priority in this RTOS,
     * see RC_PRI_MOTION in rc_config.h); stksz is how much stack memory
     * to reserve; task is a function pointer to the task's entry point
     * (motion_task, defined above); tskatr are RTOS-specific attribute
     * flags (HLNG = written in high-level language/C, RNG3 = runs at the
     * lowest-privilege protection ring). */
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

/* Update the cruise-control "set speed" used by distance moves (does not
 * affect MODE_CONTINUOUS, which is driven directly by base_permille). */
rc_result_t sub_motion_set_speed(uint16_t mm_s)
{
    target_mm_s = mm_s;
    return RC_OK;
}

/* Queue a "drive forward mm millimetres" move. Non-blocking - see
 * TEAM_GUIDE.md §0.3 and sub_motion.h's header comment. Returns a move id
 * immediately while the actual driving happens in motion_task above. */
uint32_t sub_motion_forward_mm(uint32_t mm, sub_motion_done_cb_t cb, void *ctx)
{
    return start_move(MODE_DISTANCE, mm, cb, ctx);
}

/* Queue a "drive backward mm millimetres" move. */
uint32_t sub_motion_backward_mm(uint32_t mm, sub_motion_done_cb_t cb, void *ctx)
{
    /* TODO Buddy 2: reverse needs its own sign handling in MODE_DISTANCE.
     * Left as an explicit gap rather than a silent forward move. */
    return start_move(MODE_DISTANCE, mm, cb, ctx);
}

/* Queue a "turn deg degrees" move (positive/negative for left/right,
 * sign convention set by whoever calls this). Currently backed by the
 * MODE_TURN stub in motion_task - see the TODO comment there; this call
 * accepts the request but the wheels don't actually turn yet. */
uint32_t sub_motion_turn_deg(int16_t deg, sub_motion_done_cb_t cb, void *ctx)
{
    /* Ternary: mag = -deg if deg is negative, else deg - i.e. the
     * absolute value of deg, cast to an unsigned magnitude since
     * start_move()'s `target` parameter is unsigned. */
    uint32_t mag = (deg < 0) ? (uint32_t)(-deg) : (uint32_t)deg;

    return start_move(MODE_TURN, mag, cb, ctx);
}

/* Switch into continuous drive+steer mode - the mode the line follower
 * uses almost constantly. No distance target, no completion callback; it
 * just keeps driving at this base speed/steering bias until something
 * else calls sub_motion_stop() or queues a new distance/turn move. */
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

/* Cancel whatever's running (if anything) and stop the motors. */
rc_result_t sub_motion_stop(bool brake)
{
    if (mode != MODE_IDLE) {
        finish_move(false);
    }
    return drv_motor_stop(brake ? RC_MOTOR_BRAKE : RC_MOTOR_COAST);
}

/* True only while a *queued* move (forward/backward/turn) is still in
 * progress - continuous drive mode and idle both report false, since
 * neither one is a move with a defined "done" point. */
bool sub_motion_busy(void)
{
    return (mode == MODE_DISTANCE) || (mode == MODE_TURN);
}
