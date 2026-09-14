/*
 *  drv_motor.h  -  Buddy 2 hardware layer
 *
 *  Robo Pico drives each motor with two PWM pins rather than a PWM pin
 *  plus a direction bit. Forward is PWM on MxA with MxB held low, reverse
 *  swaps them, brake is both high, coast is both low.
 *
 *  This layer is pure output. It has no idea how fast the car is actually
 *  going. Closing that loop is sub_motion's job.
 */
#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "rc_types.h"

typedef enum {
    RC_MOTOR_COAST = 0,
    RC_MOTOR_BRAKE
} rc_motor_stop_t;

rc_result_t drv_motor_init(void);

/* Signed duty, -1000..+1000 permille. Positive is forward. Returns
 * immediately, no ramping. Add ramping in sub_motion if you want it. */
rc_result_t drv_motor_set(rc_side_t side, int16_t duty_permille);
rc_result_t drv_motor_set_pair(int16_t left_permille, int16_t right_permille);
rc_result_t drv_motor_stop(rc_motor_stop_t mode);

/* Last commanded value, for telemetry. */
int16_t drv_motor_get(rc_side_t side);

#endif /* DRV_MOTOR_H */
