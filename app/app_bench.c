/*
 *  app_bench.c
 *
 *  One self-contained test per buddy. Each one runs in the initial task
 *  (usermain's task), sleeps with tk_dly_tsk between prints, and picks up
 *  live readings either straight from the driver or by subscribing to the
 *  same events the mission code uses. Nothing here touches another buddy's
 *  module, so a bench that misbehaves points at exactly one place.
 *
 *  Prints use tm_printf, which supports %d %u %x %s (and the 'l' length
 *  modifier). Values are cast to int for printing.
 */
#include "rc_prelude.h"
#include <tm/tmonitor.h>

#include "app_bench.h"
#include "rc_config.h"
#include "rc_event.h"
#include "rc_time.h"

#include "drv_encoder.h"
#include "drv_motor.h"
#include "drv_ir.h"
#include "drv_imu.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"

#include "sub_motion.h"
#include "sub_line.h"
#include "sub_barcode.h"
#include "sub_terrain.h"
#include "sub_scan.h"

#define P(...)  tm_printf((UB *)__VA_ARGS__)

/* ------------------------------------------------------------------ *
 *  Buddy 2 - motion: motors, encoders, closed-loop moves
 *
 *  Phase 1  wheels OFF the ground: open-loop duty on both motors while
 *           printing encoder count and signed speed. Both speeds must be
 *           positive; a negative one means that encoder's A/B are swapped.
 *  Phase 2  car ON THE FLOOR: every move the brief asks for. Forward
 *           500 mm logs each wheel's target speed, measured speed and
 *           duty every 40 ms - the PID step response for the tuning
 *           report. Then backward 500 mm, turn right 90, turn left 90 and
 *           a U-turn, each printing its result, with a pause after each
 *           so the car can be measured (tape, protractor).
 *  Phase 3  motors off; keep printing so you can turn a wheel by hand -
 *           this is where RC_ENC_TICKS_PER_REV is measured.
 * ------------------------------------------------------------------ */

/* Speed for phase 2 moves, and the pause after each one for measuring. */
#define BENCH_SPEED_MM_S    (250U)
#define BENCH_PAUSE_MS      (4000)
/* Give up on a move after this long even if no callback came. Moves end
 * on their own (at the goal, or stopped by the stall/timeout guards), so
 * this only keeps a lost callback from hanging the bench. */
#define BENCH_MOVE_MAX_MS   (30000U)

static volatile bool     move_done;
static volatile bool     move_ok;
static volatile uint32_t move_mm;

static void motion_cb(uint32_t id, bool completed, uint32_t mm, void *ctx)
{
    (void)id;
    (void)ctx;
    move_ok   = completed;
    move_mm   = mm;
    move_done = true;
}

/* One step-response sample: time since the move started, then per wheel
 * the target speed, the measured speed (both mm/s) and the duty
 * (permille). Comma separated, to paste into a spreadsheet. */
static void print_step(uint32_t t_ms)
{
    P("  %u,%d,%d,%d,%d,%d,%d\n", (unsigned)t_ms,
      (int)sub_motion_target_mm_s(RC_SIDE_LEFT),
      (int)drv_encoder_speed_mm_s(RC_SIDE_LEFT),
      (int)drv_motor_get(RC_SIDE_LEFT),
      (int)sub_motion_target_mm_s(RC_SIDE_RIGHT),
      (int)drv_encoder_speed_mm_s(RC_SIDE_RIGHT),
      (int)drv_motor_get(RC_SIDE_RIGHT));
}

/* Wait for the move `id` (already started, move_done cleared before it
 * was requested) to end, logging the step response if `log`, then print
 * how it went. `asked` and `unit` are only for the printout. */
static void bench_wait_move(const char *what, uint32_t id, uint32_t asked,
                            const char *unit, bool log)
{
    uint32_t t0     = rc_time_ms();
    uint32_t period = log ? 40U : 250U;

    if (id == 0U) {
        P("[bench] %s: request rejected\n", what);
        return;
    }
    if (log) {
        P("  t_ms,tgt_l,spd_l,duty_l,tgt_r,spd_r,duty_r\n");
    }
    while (!move_done && ((rc_time_ms() - t0) < BENCH_MOVE_MAX_MS)) {
        tk_dly_tsk((TMO)period);
        if (log) {
            print_step(rc_time_ms() - t0);
        }
    }
    P("[bench] %s: %s, wheels travelled %u mm (asked %u %s) in %u ms\n",
      what,
      !move_done ? "NO CALLBACK" : (move_ok ? "completed" : "ABORTED"),
      (unsigned)move_mm, (unsigned)asked, unit,
      (unsigned)(rc_time_ms() - t0));
}

static void print_wheels(void)
{
    P("  L cnt=%u spd=%d mm/s dir=%d | R cnt=%u spd=%d mm/s dir=%d\n",
      (unsigned)drv_encoder_count(RC_SIDE_LEFT),
      (int)drv_encoder_speed_mm_s(RC_SIDE_LEFT),
      (int)drv_encoder_dir(RC_SIDE_LEFT),
      (unsigned)drv_encoder_count(RC_SIDE_RIGHT),
      (int)drv_encoder_speed_mm_s(RC_SIDE_RIGHT),
      (int)drv_encoder_dir(RC_SIDE_RIGHT));
}

static void bench_motion(void)
{
    uint32_t i;

    P("\n[bench] MOTION. Phase 1: open loop, 30%% duty, 4 s."
      " WHEELS OFF THE GROUND.\n");
    P("        Expect both counts rising and both speeds POSITIVE.\n");
    (void)sub_motion_drive(300, 0);
    for (i = 0U; i < 16U; i++) {
        tk_dly_tsk(250);
        print_wheels();
    }
    (void)sub_motion_drive(0, 0);
    (void)sub_motion_stop(false);

    P("[bench] Phase 2: closed-loop moves at %u mm/s. PUT THE CAR ON THE"
      " FLOOR, 1 m clear in front. Starting in 5 s.\n",
      (unsigned)BENCH_SPEED_MM_S);
    (void)sub_motion_set_speed(BENCH_SPEED_MM_S);
    tk_dly_tsk(5000);

    move_done = false;
    bench_wait_move("forward 500 mm",
                    sub_motion_forward_mm(500U, motion_cb, NULL),
                    500U, "mm", true);
    tk_dly_tsk(BENCH_PAUSE_MS);

    move_done = false;
    bench_wait_move("backward 500 mm",
                    sub_motion_backward_mm(500U, motion_cb, NULL),
                    500U, "mm", false);
    tk_dly_tsk(BENCH_PAUSE_MS);

    move_done = false;
    bench_wait_move("turn right 90", sub_motion_turn_deg(90, motion_cb, NULL),
                    90U, "deg", false);
    tk_dly_tsk(BENCH_PAUSE_MS);

    move_done = false;
    bench_wait_move("turn left 90", sub_motion_turn_deg(-90, motion_cb, NULL),
                    90U, "deg", false);
    tk_dly_tsk(BENCH_PAUSE_MS);

    move_done = false;
    bench_wait_move("U-turn 180", sub_motion_turn_deg(180, motion_cb, NULL),
                    180U, "deg", false);

    P("[bench] Phase 3: motors off. Turn a wheel by hand and watch its"
      " count.\n");
    for (;;) {
        tk_dly_tsk(500);
        print_wheels();
    }
}

/* ------------------------------------------------------------------ *
 *  Buddy 3 - line sensors (motors stay off) and line following
 * ------------------------------------------------------------------ */

static volatile int16_t  line_pos;
static volatile uint16_t line_raw;

static void on_line_sample(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    line_pos = evt->u.line.position;
    line_raw = evt->u.line.barcode_raw;
}

static const char *line_state_name(rc_line_state_t s)
{
    switch (s) {
    case RC_LINE_TRACKING:  return "TRACKING";
    case RC_LINE_LOST:      return "LOST";
    case RC_LINE_SEARCHING: return "SEARCHING";
    case RC_LINE_JUNCTION:  return "JUNCTION";
    default:                return "?";
    }
}

static void print_line(void)
{
    P("  L=%d R=%d pos=%d barcode_raw=%u state=%s\n",
      drv_ir_on_line(RC_IR_LINE_L) ? 1 : 0,
      drv_ir_on_line(RC_IR_LINE_R) ? 1 : 0,
      (int)line_pos, (unsigned)line_raw,
      line_state_name(sub_line_state()));
}

static void bench_line(bool drive)
{
    (void)rc_event_subscribe(RC_EVT_LINE_SAMPLE, RC_LANE_SLOW,
                             on_line_sample, NULL);

    if (drive) {
        P("\n[bench] FOLLOW. The car WILL DRIVE. Put it on the line first.\n");
        (void)sub_line_enable(true);
    } else {
        P("\n[bench] LINE. Motors off. Slide the sensors over the line"
          " by hand.\n");
        P("        1 = sees black. If it reads 1 over white, flip"
          " IR_ACTIVE_HIGH in drv_ir.c.\n");
    }
    for (;;) {
        tk_dly_tsk(drive ? 250 : 100);
        print_line();
    }
}

/* ------------------------------------------------------------------ *
 *  Buddy 3 - barcode: every edge width, every decode
 * ------------------------------------------------------------------ */

static void on_bar_edge(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    P("  edge -> %s after %u us\n",
      evt->u.bar_edge.level_high ? "black" : "white",
      (unsigned)evt->u.bar_edge.width_us);
}

static void on_barcode(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    P("[bench] DECODED '%c' cmd=%d%s\n",
      evt->u.barcode.symbol, (int)evt->u.barcode.command,
      evt->u.barcode.reversed ? " (reversed)" : "");
}

static void bench_barcode(void)
{
    P("\n[bench] BARCODE. Motors off. Armed permanently; pull a barcode"
      " under the sensor.\n");
    P("        Watch the widths: wide bars should be ~2-3x narrow ones"
      " at any speed.\n");
    (void)rc_event_subscribe(RC_EVT_BARCODE_EDGE, RC_LANE_SLOW,
                             on_bar_edge, NULL);
    (void)rc_event_subscribe(RC_EVT_BARCODE_DECODED, RC_LANE_SLOW,
                             on_barcode, NULL);
    (void)sub_barcode_arm(true);
    for (;;) {
        tk_dly_tsk(5000);
        P("  raw=%u overruns=%u last='%c'\n",
          (unsigned)drv_ir_read_raw(RC_IR_BARCODE),
          (unsigned)drv_ir_barcode_overruns(),
          sub_barcode_last() ? sub_barcode_last() : '-');
    }
}

/* ------------------------------------------------------------------ *
 *  Buddy 4 - IMU: accelerometer, pitch, terrain class, humps
 * ------------------------------------------------------------------ */

static volatile int16_t acc[3];

static void on_imu(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    acc[0] = evt->u.imu.acc_x;
    acc[1] = evt->u.imu.acc_y;
    acc[2] = evt->u.imu.acc_z;
}

static void on_hump(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    if (evt->id == RC_EVT_HUMP_BEGIN) {
        P("[bench] hump BEGIN\n");
    } else {
        P("[bench] hump END peak=%u mm over %u ms, max pitch %d deg\n",
          (unsigned)evt->u.hump.peak_mm, (unsigned)evt->u.hump.duration_ms,
          (int)evt->u.hump.max_pitch_deg);
    }
}

static const char *class_name(rc_motion_class_t c)
{
    switch (c) {
    case RC_MOTION_STATIONARY:   return "STATIONARY";
    case RC_MOTION_ACCELERATING: return "ACCEL";
    case RC_MOTION_CRUISING:     return "CRUISE";
    case RC_MOTION_DECELERATING: return "DECEL";
    case RC_MOTION_TURNING:      return "TURNING";
    case RC_MOTION_CLIMBING:     return "CLIMBING";
    case RC_MOTION_DESCENDING:   return "DESCENDING";
    case RC_MOTION_IMPACT:       return "IMPACT";
    default:                     return "?";
    }
}

static void bench_imu(void)
{
    P("\n[bench] IMU. Level: pitch ~0, z ~1000 mg. Tip the nose up: pitch"
      " goes positive.\n");
    (void)rc_event_subscribe(RC_EVT_IMU_SAMPLE, RC_LANE_SLOW, on_imu,  NULL);
    (void)rc_event_subscribe(RC_EVT_HUMP_BEGIN, RC_LANE_SLOW, on_hump, NULL);
    (void)rc_event_subscribe(RC_EVT_HUMP_END,   RC_LANE_SLOW, on_hump, NULL);
    for (;;) {
        tk_dly_tsk(200);
        int16_t  pitch = drv_imu_pitch_ddeg();
        uint16_t mag   = (uint16_t)((pitch < 0) ? -pitch : pitch);
        /* Print the sign separately: -5 ddeg / 10 is 0 in C, so the "-"
         * would vanish for small nose-down angles. */
        P("  acc x=%d y=%d z=%d mg  pitch=%s%u.%u deg  class=%s"
          "  max peak=%u mm\n",
          (int)acc[0], (int)acc[1], (int)acc[2],
          (pitch < 0) ? "-" : "",
          (unsigned)(mag / 10U), (unsigned)(mag % 10U),
          class_name(sub_terrain_motion_class()),
          (unsigned)sub_terrain_max_peak_mm());
    }
}

/* ------------------------------------------------------------------ *
 *  Buddy 5 - ultrasonic (straight ahead) and the full scan
 * ------------------------------------------------------------------ */

static void on_ultra(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    if (evt->u.ultra.valid) {
        P("  %d deg: %u mm\n", (int)evt->u.ultra.angle_deg,
          (unsigned)evt->u.ultra.range_mm);
    } else {
        P("  %d deg: no echo\n", (int)evt->u.ultra.angle_deg);
    }
}

static void bench_ultra(void)
{
    P("\n[bench] ULTRA. Servo parked at 90, one ping every 100 ms."
      " Check against a tape at 100/300/1000 mm.\n");
    (void)rc_event_subscribe(RC_EVT_ULTRA_RESULT, RC_LANE_SLOW, on_ultra, NULL);
    (void)drv_servo_set_angle(90);
    tk_dly_tsk(500);
    for (;;) {
        if (!drv_ultrasonic_busy()) {
            (void)drv_ultrasonic_ping(90);
        }
        tk_dly_tsk(100);
    }
}

static const char *cmd_name(rc_nav_cmd_t c)
{
    switch (c) {
    case RC_CMD_TURN_LEFT:   return "TURN_LEFT";
    case RC_CMD_TURN_RIGHT:  return "TURN_RIGHT";
    case RC_CMD_GO_STRAIGHT: return "GO_STRAIGHT";
    case RC_CMD_U_TURN:      return "U_TURN";
    case RC_CMD_STOP:        return "STOP";
    case RC_CMD_NONE:
    default:                 return "NONE";
    }
}

static void on_scan_done(const rc_pl_profile_t *profile,
                         const rc_pl_plan_t *plan, void *ctx)
{
    (void)ctx;
    P("[bench] profile: %u points, closest %u mm at %d deg, width %u mm,"
      " clearance L=%u R=%u\n",
      (unsigned)profile->n_points, (unsigned)profile->closest_mm,
      (int)profile->closest_angle_deg, (unsigned)profile->width_mm,
      (unsigned)profile->clearance_left_mm,
      (unsigned)profile->clearance_right_mm);
    P("[bench] plan: %s lateral=%u mm forward=%u mm\n",
      cmd_name(plan->action), (unsigned)plan->lateral_mm,
      (unsigned)plan->forward_mm);
}

static void bench_scan(void)
{
    P("\n[bench] SCAN. Coarse 30..150 then fine sweep, repeated every few"
      " seconds. Motors off.\n");
    (void)rc_event_subscribe(RC_EVT_ULTRA_RESULT, RC_LANE_SLOW, on_ultra, NULL);
    (void)sub_scan_on_complete(on_scan_done, NULL);
    for (;;) {
        P("[bench] --- scan start\n");
        (void)sub_scan_start();
        tk_dly_tsk(100);            /* let the scan task pick the flag up */
        while (sub_scan_busy()) {
            tk_dly_tsk(50);
        }
        tk_dly_tsk(3000);
    }
}

/* ------------------------------------------------------------------ *
 *  Buddy 1 - telemetry: everything sensing, car stationary
 * ------------------------------------------------------------------ */

static void bench_telemetry(void)
{
    P("\n[bench] TELEMETRY. All sensors live, motors off. Watch the [telem]"
      " lines at 4 Hz.\n");
    P("        Both dropped counts must stay at 0.\n");
    for (;;) {
        tk_dly_tsk(5000);
        P("[bench] dropped fast=%u slow=%u\n",
          (unsigned)rc_event_dropped(RC_LANE_FAST),
          (unsigned)rc_event_dropped(RC_LANE_SLOW));
    }
}

/* ------------------------------------------------------------------ */

const char *rc_bench_name(void)
{
    switch (RC_BENCH) {
    case RC_BENCH_MOTION:    return "motion";
    case RC_BENCH_LINE:      return "line";
    case RC_BENCH_FOLLOW:    return "follow";
    case RC_BENCH_BARCODE:   return "barcode";
    case RC_BENCH_IMU:       return "imu";
    case RC_BENCH_ULTRA:     return "ultra";
    case RC_BENCH_SCAN:      return "scan";
    case RC_BENCH_TELEMETRY: return "telemetry";
    case RC_BENCH_NONE:
    default:                 return "none";
    }
}

void rc_bench_run(void)
{
    switch (RC_BENCH) {
    case RC_BENCH_MOTION:    bench_motion();      break;
    case RC_BENCH_LINE:      bench_line(false);   break;
    case RC_BENCH_FOLLOW:    bench_line(true);    break;
    case RC_BENCH_BARCODE:   bench_barcode();     break;
    case RC_BENCH_IMU:       bench_imu();         break;
    case RC_BENCH_ULTRA:     bench_ultra();       break;
    case RC_BENCH_SCAN:      bench_scan();        break;
    case RC_BENCH_TELEMETRY: bench_telemetry();   break;
    case RC_BENCH_NONE:
    default:
        for (;;) {
            tk_slp_tsk(TMO_FEVR);
        }
    }
}
