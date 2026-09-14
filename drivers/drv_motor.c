/*
 *  drv_motor.c
 */
#include "rc_prelude.h"
#include "drv_motor.h"
#include "rc_config.h"
#include "rc_pwm.h"

/* 20 kHz is above hearing and well inside what the Robo Pico output stage
 * handles. Drop to 10 kHz if your motors whine at low duty. */
#define MOTOR_PWM_HZ    (20000UL)
#define DUTY_MAX        (1000)

typedef struct {
    uint32_t pin_a;
    uint32_t pin_b;
    int16_t  last;
} motor_t;

static motor_t motors[2] = {
    { RC_PIN_MOTOR_L_A, RC_PIN_MOTOR_L_B, 0 },
    { RC_PIN_MOTOR_R_A, RC_PIN_MOTOR_R_B, 0 }
};

rc_result_t drv_motor_init(void)
{
    uint32_t    i;
    rc_result_t res;

    for (i = 0U; i < 2U; i++) {
        res = rc_pwm_init_pin(motors[i].pin_a, MOTOR_PWM_HZ);
        if (res != RC_OK) {
            return res;
        }
        res = rc_pwm_init_pin(motors[i].pin_b, MOTOR_PWM_HZ);
        if (res != RC_OK) {
            return res;
        }
        (void)rc_pwm_set_duty(motors[i].pin_a, 0U);
        (void)rc_pwm_set_duty(motors[i].pin_b, 0U);
        motors[i].last = 0;
    }
    return RC_OK;
}

rc_result_t drv_motor_set(rc_side_t side, int16_t duty_permille)
{
    motor_t *m;

    if (side > RC_SIDE_RIGHT) {
        return RC_ERR_PARAM;
    }
    if (duty_permille > DUTY_MAX) {
        duty_permille = DUTY_MAX;
    }
    if (duty_permille < -DUTY_MAX) {
        duty_permille = -DUTY_MAX;
    }

    m = &motors[side];
    m->last = duty_permille;

    /* Drop the opposite pin before raising the driving one, so the H
     * bridge is never told to do both at once. */
    if (duty_permille >= 0) {
        (void)rc_pwm_set_duty(m->pin_b, 0U);
        (void)rc_pwm_set_duty(m->pin_a, (uint16_t)duty_permille);
    } else {
        (void)rc_pwm_set_duty(m->pin_a, 0U);
        (void)rc_pwm_set_duty(m->pin_b, (uint16_t)(-duty_permille));
    }
    return RC_OK;
}

rc_result_t drv_motor_set_pair(int16_t left_permille, int16_t right_permille)
{
    (void)drv_motor_set(RC_SIDE_LEFT, left_permille);
    (void)drv_motor_set(RC_SIDE_RIGHT, right_permille);
    return RC_OK;
}

rc_result_t drv_motor_stop(rc_motor_stop_t mode)
{
    uint32_t i;
    uint16_t level = (mode == RC_MOTOR_BRAKE) ? (uint16_t)DUTY_MAX : 0U;

    for (i = 0U; i < 2U; i++) {
        (void)rc_pwm_set_duty(motors[i].pin_a, level);
        (void)rc_pwm_set_duty(motors[i].pin_b, level);
        motors[i].last = 0;
    }
    return RC_OK;
}

int16_t drv_motor_get(rc_side_t side)
{
    return (side > RC_SIDE_RIGHT) ? 0 : motors[side].last;
}
