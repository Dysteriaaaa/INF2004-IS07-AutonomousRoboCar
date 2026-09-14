/*
 *  drv_servo.h  -  Buddy 5 hardware layer
 *
 *  Standard hobby servo on a Robo Pico servo port: 50 Hz frame, pulse
 *  width from about 500 us to 2400 us.
 *
 *  There is no position feedback on an RC servo, so "has it arrived" is
 *  a timer, not a measurement. drv_servo_settle_ms tells the caller how
 *  long to wait before trusting a range reading taken at the new angle.
 *  The caller waits by arming its own state machine, never by spinning.
 */
#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include "rc_types.h"

#define RC_SERVO_ANGLE_MIN  (0)
#define RC_SERVO_ANGLE_MAX  (180)

rc_result_t drv_servo_init(void);

/* Command an angle in degrees. Returns immediately. */
rc_result_t drv_servo_set_angle(int16_t deg);

/* Last commanded angle. */
int16_t drv_servo_get_angle(void);

/* Milliseconds the servo needs to travel from `from` to `to` and stop
 * ringing. Used by sub_scan to schedule the ping after the move. */
uint32_t drv_servo_settle_ms(int16_t from, int16_t to);

/* Stop sending pulses. The servo goes limp and stops drawing current,
 * which matters on a shared battery when the motors are also loaded. */
rc_result_t drv_servo_release(void);

#endif /* DRV_SERVO_H */
