/*
 *  sub_nav.h  -  mission logic, shared
 *
 *  This is the one module that is nobody's buddy subsystem and
 *  everybody's problem. It owns the top-level state of the run and
 *  decides who is driving at any moment.
 *
 *  It holds no hardware and runs no control loop. It is purely a state
 *  machine fed by callbacks from the other five subsystems, which is why
 *  it reads as a list of event handlers rather than a loop.
 */
#ifndef SUB_NAV_H
#define SUB_NAV_H

#include "rc_types.h"

typedef enum {
    RC_NAV_IDLE = 0,
    RC_NAV_FOLLOWING,
    RC_NAV_READING_BARCODE,
    RC_NAV_EXECUTING_CMD,
    RC_NAV_AVOIDING,
    RC_NAV_RECOVERING,
    RC_NAV_STOPPED
} rc_nav_state_t;

rc_result_t sub_nav_init(void);

/* Begin the run. Everything before this is start-up and calibration. */
rc_result_t sub_nav_start(void);
rc_result_t sub_nav_stop(void);

rc_nav_state_t sub_nav_state(void);

#endif /* SUB_NAV_H */
