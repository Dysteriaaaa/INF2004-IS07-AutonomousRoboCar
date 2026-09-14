/*
 *  sub_scan.h  -  Buddy 5
 *
 *  The five stages from the brief, run as one non-blocking state
 *  machine: coarse scan, fine scan, profile, plan, recover.
 *
 *  The whole thing is driven by two events, the servo settle timeout and
 *  the ultrasonic result. Nothing polls and nothing waits. A full coarse
 *  scan at five angles takes roughly 5 x (servo travel + 30 ms of flight
 *  time), around 700 ms, during which the CPU is essentially idle and
 *  the motion PID keeps running at its normal rate.
 */
#ifndef SUB_SCAN_H
#define SUB_SCAN_H

#include "rc_types.h"

typedef void (*sub_scan_done_cb_t)(const rc_pl_profile_t *profile,
                                   const rc_pl_plan_t *plan,
                                   void *ctx);

rc_result_t sub_scan_init(void);

/*
 *  Start a coarse scan. Returns immediately. The result arrives through
 *  the callback and through RC_EVT_OBSTACLE_PROFILE and
 *  RC_EVT_AVOIDANCE_PLAN.
 */
rc_result_t sub_scan_start(void);

rc_result_t sub_scan_abort(void);
bool sub_scan_busy(void);

rc_result_t sub_scan_on_complete(sub_scan_done_cb_t cb, void *ctx);

/*
 *  Continuous forward watch. Pings straight ahead at a slow rate while
 *  the car is line following, and publishes RC_EVT_ULTRA_RESULT. sub_nav
 *  watches those and calls sub_scan_start when something gets close.
 */
rc_result_t sub_scan_set_watch(bool on, uint16_t trigger_mm);

#endif /* SUB_SCAN_H */
