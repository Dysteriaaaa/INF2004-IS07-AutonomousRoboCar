/*
 *  drv_servo.c
 */
#include "drv_servo.h"
#include "rc_config.h"
#include "rc_pwm.h"

#define SERVO_HZ            (50UL)
#define PULSE_MIN_US        (500UL)
#define PULSE_MAX_US        (2400UL)
#define SWEEP_DEG           (RC_SERVO_ANGLE_MAX - RC_SERVO_ANGLE_MIN)

/* Typical 9 g servo is around 0.12 s per 60 degrees unloaded. Add a
 * fixed allowance for the mechanism to stop ringing. Measure yours and
 * shrink these, they dominate total scan time. */
#define US_PER_DEG          (2200UL)
#define SETTLE_FIXED_MS     (25UL)

static int16_t last_angle = 90;

rc_result_t drv_servo_init(void)
{
    rc_result_t res = rc_pwm_init_pin(RC_PIN_SERVO_SCAN, SERVO_HZ);

    if (res != RC_OK) {
        return res;
    }
    return drv_servo_set_angle(90);
}

rc_result_t drv_servo_set_angle(int16_t deg)
{
    uint32_t pulse_us;

    if (deg < RC_SERVO_ANGLE_MIN) {
        deg = RC_SERVO_ANGLE_MIN;
    }
    if (deg > RC_SERVO_ANGLE_MAX) {
        deg = RC_SERVO_ANGLE_MAX;
    }

    pulse_us = PULSE_MIN_US
             + (((PULSE_MAX_US - PULSE_MIN_US) * (uint32_t)deg) / SWEEP_DEG);

    last_angle = deg;

    return rc_pwm_set_pulse_us(RC_PIN_SERVO_SCAN, pulse_us);
}

int16_t drv_servo_get_angle(void)
{
    return last_angle;
}

uint32_t drv_servo_settle_ms(int16_t from, int16_t to)
{
    int32_t  travel = (int32_t)to - (int32_t)from;
    uint32_t mag;

    if (travel < 0) {
        travel = -travel;
    }
    mag = ((uint32_t)travel * US_PER_DEG) / 1000UL;

    return mag + SETTLE_FIXED_MS;
}

rc_result_t drv_servo_release(void)
{
    return rc_pwm_enable(RC_PIN_SERVO_SCAN, false);
}
