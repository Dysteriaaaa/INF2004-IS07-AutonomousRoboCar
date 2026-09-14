/*
 *  sub_motion.h  -  Buddy 2
 *
 *  The motion API the rest of the team calls. The brief asks for
 *  moveForward(distance), turnLeft(angle) and so on. The obvious
 *  implementation of those blocks until the move finishes, and that
 *  cannot work here: the line follower needs to keep steering while the
 *  car drives, and the scanner needs to keep pinging. See TEAM_GUIDE.md
 *  §0.3 ("Non-blocking - the one rule that shapes everything") for why
 *  this pattern is used everywhere in this project, not just here.
 *
 *  So every command here is a request that returns immediately and
 *  reports completion through a callback (a "callback" is just a
 *  function pointer you hand over in advance, which the callee invokes
 *  later when the thing you asked for is actually done - like leaving
 *  your phone number so a restaurant can text you when your table is
 *  ready, instead of you standing at the counter waiting). One move is
 *  active at a time; a new command pre-empts the old one and the old
 *  one's callback fires with completed = false.
 *
 *      uint32_t id = sub_motion_forward_mm(300, on_done, NULL);
 *      // returns now, car is still moving
 *
 *  Underneath, a PID task runs at RC_PERIOD_MOTION_MS closing the loop
 *  on encoder speed, and a separate straight-line correction term keeps
 *  the two wheels matched. "PID" = Proportional-Integral-Derivative
 *  control, the same idea as a car's cruise control - see sub_motion.c
 *  for the full explanation next to the actual math, and TEAM_GUIDE.md's
 *  Buddy 2 section for the plain-English overview.
 */
#ifndef SUB_MOTION_H
#define SUB_MOTION_H

#include "rc_types.h"

/* The shape of a "move finished" callback function. A "function pointer"
 * (the `(*sub_motion_done_cb_t)` syntax) is a variable that holds the
 * address of a function instead of holding data - it lets you pass
 * "a function to call later" around like any other value. Whoever calls
 * sub_motion_forward_mm() etc. supplies one of these; sub_motion.c
 * invokes it once the move ends.
 *   move_id      - which move this refers to (matches the id returned
 *                  when the move was started, useful if you started more
 *                  than one and want to know which finished).
 *   completed    - true if the move reached its goal normally, false if
 *                  it was cut short (stopped, or pre-empted by a newer
 *                  move request).
 *   travelled_mm - how far the car actually moved during this attempt.
 *   ctx          - your own pointer, handed back unchanged; a way to pass
 *                  "which object/struct is this callback for" without
 *                  global variables.
 * Runs in dispatcher task context (not inside an interrupt). Must not
 * block - see TEAM_GUIDE.md §0.2/§0.3. */
typedef void (*sub_motion_done_cb_t)(uint32_t move_id,
                                     bool completed,
                                     uint32_t travelled_mm,
                                     void *ctx);

/* Boot-time setup: sets placeholder PID gains and starts the background
 * control task. Call once at startup, before any other sub_motion_* call. */
rc_result_t sub_motion_init(void);

/* Target cruise speed for distance moves, mm/s. This is the "set speed"
 * dial on the cruise-control analogy - the PID loop tries to hold the
 * wheels at this speed while a distance move is in progress. */
rc_result_t sub_motion_set_speed(uint16_t mm_s);

/*
 *  Queued moves. Each returns a move id (used to match it up with the
 *  completion callback/event later), or 0 if the request was rejected.
 *  Passing cb = NULL is fine if you only want the RC_EVT_MOTION_DONE
 *  event (published on the event bus - see TEAM_GUIDE.md §0.2) instead
 *  of a direct callback.
 */
uint32_t sub_motion_forward_mm(uint32_t mm, sub_motion_done_cb_t cb, void *ctx);
uint32_t sub_motion_backward_mm(uint32_t mm, sub_motion_done_cb_t cb, void *ctx);
uint32_t sub_motion_turn_deg(int16_t deg, sub_motion_done_cb_t cb, void *ctx);

/*
 *  Continuous mode, for the line follower. Sets a base speed and a
 *  steering bias in permille (parts per thousand - this codebase's
 *  integer-only stand-in for a percentage, see TEAM_GUIDE.md §2); no
 *  distance target, no completion callback - it just keeps driving until
 *  told otherwise. This is the mode the car spends most of its run in.
 */
rc_result_t sub_motion_drive(int16_t base_permille, int16_t steer_permille);

/* Cancel whatever move is running and stop the motors. brake=true stops
 * fast (electrical braking); brake=false coasts to a stop. */
rc_result_t sub_motion_stop(bool brake);

/* True while a queued move (forward/backward/turn) is still running.
 * False during continuous drive mode or when idle. */
bool sub_motion_busy(void);

#endif /* SUB_MOTION_H */
