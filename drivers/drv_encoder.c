/*
 *  drv_encoder.c
 */
#include <tk/tkernel.h>
#include <tk/syslib.h>

#include "drv_encoder.h"
#include "drv_motor.h"
#include "rc_config.h"
#include "rc_gpioirq.h"
#include "rc_event.h"
#include "rc_time.h"

/* Below this the wheel is treated as stopped rather than as very slow.
 * One second is generous, tighten it once you know your slowest useful
 * speed. */
#define STALL_TIMEOUT_US    (1000000UL)

/* An ST188 or slotted opto output can ring on a slow edge. Anything
 * closer together than this is the same physical slot. Set it from the
 * fastest edge rate you expect: at 20 slots and 600 RPM that is 5 ms
 * between slots, so 500 us of rejection is safe. */
#define DEBOUNCE_US         (500UL)

typedef struct {
    volatile uint32_t count;
    volatile uint32_t last_us;
    volatile uint32_t period_us;
    volatile uint32_t base_count;   /* snapshot at reset */
    uint32_t          pin;
} enc_t;

static enc_t encs[2] = {
    { 0U, 0U, 0U, 0U, RC_PIN_ENC_L },
    { 0U, 0U, 0U, 0U, RC_PIN_ENC_R }
};

/* ------------------------------------------------------------------ *
 *  ISR. Runs on processor 1 only, per the port's one-owner-core rule
 *  for shared peripheral interrupts.
 *
 *  Everything here is O(1): a subtraction, two stores, one ring push.
 * ------------------------------------------------------------------ */

static void encoder_isr(uint32_t pin, bool level, uint32_t t_us, void *ctx)
{
    enc_t     *e = (enc_t *)ctx;
    uint32_t   delta;
    rc_event_t evt;

    (void)pin;
    (void)level;

    delta = t_us - e->last_us;          /* wrap safe */
    if (delta < DEBOUNCE_US) {
        return;                          /* contact bounce, ignore */
    }

    e->last_us   = t_us;
    e->period_us = delta;
    e->count++;

    evt.id = RC_EVT_ENCODER_EDGE;
    evt.u.encoder.side     = (e == &encs[RC_SIDE_LEFT])
                             ? RC_SIDE_LEFT : RC_SIDE_RIGHT;
    evt.u.encoder.count    = e->count;
    evt.u.encoder.delta_us = delta;

    (void)rc_event_publish_i(&evt);
}

/* ------------------------------------------------------------------ */

rc_result_t drv_encoder_init(void)
{
    rc_result_t res;
    uint32_t    i;

    for (i = 0U; i < 2U; i++) {
        encs[i].count      = 0U;
        encs[i].last_us    = rc_time_us();
        encs[i].period_us  = 0U;
        encs[i].base_count = 0U;

        /* Rising edges only. Counting both edges would double the
         * resolution but a slotted disc has asymmetric mark/space, so
         * the widths would alternate and ruin the period estimate. */
        res = rc_gpioirq_attach(encs[i].pin, RC_EDGE_RISE, 1,
                                encoder_isr, &encs[i]);
        if (res != RC_OK) {
            return res;
        }
    }
    return RC_OK;
}

uint32_t drv_encoder_count(rc_side_t side)
{
    return (side > RC_SIDE_RIGHT) ? 0U : encs[side].count;
}

uint32_t drv_encoder_period_us(rc_side_t side)
{
    uint32_t period;
    uint32_t last;
    uint32_t sts;

    if (side > RC_SIDE_RIGHT) {
        return 0U;
    }

    /* Read both under one critical section: an edge landing between the
     * two reads would pair a fresh timestamp with a stale period. */
    DI(sts);
    period = encs[side].period_us;
    last   = encs[side].last_us;
    EI(sts);

    if ((rc_time_us() - last) > STALL_TIMEOUT_US) {
        return 0U;
    }
    return period;
}

int32_t drv_encoder_speed_mm_s(rc_side_t side)
{
    uint32_t period = drv_encoder_period_us(side);
    int32_t  speed;

    if (period == 0U) {
        return 0;
    }

    /* um per tick / us per tick = m/s * 1e6 / 1e6, so the division below
     * lands directly in mm/s once the numerator is scaled by 1000. */
    speed = (int32_t)((RC_ENC_UM_PER_TICK * 1000UL) / period);

    if (drv_motor_get(side) < 0) {
        speed = -speed;
    }
    return speed;
}

uint32_t drv_encoder_distance_mm(rc_side_t side)
{
    uint32_t ticks;

    if (side > RC_SIDE_RIGHT) {
        return 0U;
    }
    ticks = encs[side].count - encs[side].base_count;

    return (uint32_t)((ticks * RC_ENC_UM_PER_TICK) / 1000UL);
}

void drv_encoder_reset(void)
{
    uint32_t sts;

    DI(sts);
    encs[RC_SIDE_LEFT].base_count  = encs[RC_SIDE_LEFT].count;
    encs[RC_SIDE_RIGHT].base_count = encs[RC_SIDE_RIGHT].count;
    EI(sts);
}
