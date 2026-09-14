/*
 *  sub_motion.h  -  Buddy 2
 *
 *  The motion API the rest of the team calls. The brief asks for
 *  moveForward(distance), turnLeft(angle) and so on. The obvious
 *  implementation of those blocks until the move finishes, and that
 *  cannot work here: the line follower needs to keep steering while the
 *  car drives, and the scanner needs to keep pinging.
 *
 *  So every command here is a request that returns immediately and
 *  reports completion through a callback. One move is active at a time;
 *  a new command pre-empts the old one and the old one's callback fires
 *  with completed = false.
 *
 *      uint32_t id = sub_motion_forward_mm(300, on_done, NULL);
 *      // returns now, car is still moving
 *
 *  Underneath, a PID task runs at RC_PERIOD_MOTION_MS closing the loop
 *  on encoder speed, and a separate straight-line correction term keeps
 *  the two wheels matched.
 */
#ifndef SUB_MOTION_H
#define SUB_MOTION_H

#include "rc_types.h"

/* Runs in dispatcher task context. Must not block. */
typedef void (*sub_motion_done_cb_t)(uint32_t move_id,
                                     bool completed,
                                     uint32_t travelled_mm,
                                     void *ctx);

rc_result_t sub_motion_init(void);

/* Target cruise speed for distance moves, mm/s. */
rc_result_t sub_motion_set_speed(uint16_t mm_s);

/*
 *  Queued moves. Each returns a move id, or 0 if the request was
 *  rejected. Passing cb = NULL is fine if you only want the
 *  RC_EVT_MOTION_DONE event.
 */
uint32_t sub_motion_forward_mm(uint32_t mm, sub_motion_done_cb_t cb, void *ctx);
uint32_t sub_motion_backward_mm(uint32_t mm, sub_motion_done_cb_t cb, void *ctx);
uint32_t sub_motion_turn_deg(int16_t deg, sub_motion_done_cb_t cb, void *ctx);

/*
 *  Continuous mode, for the line follower. Sets a base speed and a
 *  steering bias in permille; no distance target, no completion. This is
 *  the mode the car spends most of its run in.
 */
rc_result_t sub_motion_drive(int16_t base_permille, int16_t steer_permille);

rc_result_t sub_motion_stop(bool brake);

/* True while a queued move is still running. */
bool sub_motion_busy(void);

#endif /* SUB_MOTION_H */
