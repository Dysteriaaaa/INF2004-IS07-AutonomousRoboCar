/*
 *  drv_ultrasonic.c
 */
#include "rc_prelude.h"
#include "drv_ultrasonic.h"
#include "rc_config.h"
#include "rc_gpioirq.h"
#include "rc_event.h"
#include "rc_defer.h"
#include "rc_time.h"

/* RP2040 TIMER alarm interrupts are IRQ 0..3. */
#define INTNO_TIMER_1       (1U)
#define INTNO_TIMER_2       (2U)
#define INTPRI_TIMER        (2)

#define ALARM_TRIG          (1U)    /* fires when the trigger pulse ends */
#define ALARM_TMO           (2U)    /* fires if no echo comes back       */

#define TIMER_ALARM_REG(n)  (TIMER_BASE + 0x10U + ((n) * 4U))

/*
 *  Speed of sound 343 m/s at 20 C, out and back, so
 *      mm = us * 343 / 2000
 *  Integer only. The error from truncation is well under a millimetre,
 *  far below the module's own 3 mm spec.
 */
#define US_TO_MM(us)        (((us) * 343UL) / 2000UL)

typedef enum {
    ST_IDLE = 0,
    ST_TRIG,
    ST_WAIT_RISE,
    ST_WAIT_FALL
} ultra_state_t;

static volatile ultra_state_t state = ST_IDLE;
static volatile uint32_t      t_rise;
static volatile int16_t       tag_angle;
static drv_ultra_cb_t         user_cb;
static void                  *user_ctx;
static int32_t                defer_h = -1;

/* Set by an ISR, consumed by the bottom half. */
static volatile uint32_t      result_width_us;
static volatile bool          result_valid;
static volatile bool          result_ready;

/* ------------------------------------------------------------------ *
 *  TIMER alarm helpers
 * ------------------------------------------------------------------ */

static void alarm_arm(uint32_t n, uint32_t delay_us)
{
    out_w(TIMER_INTR, (1U << n));           /* clear any stale flag */
    set_w(TIMER_INTE, (1U << n));
    /* Writing the alarm register is what arms it. */
    out_w(TIMER_ALARM_REG(n), in_w(TIMER_TIMERAWL) + delay_us);
}

static void alarm_cancel(uint32_t n)
{
    clr_w(TIMER_INTE, (1U << n));
    out_w(TIMER_ARMED, (1U << n));          /* write 1 to disarm */
    out_w(TIMER_INTR, (1U << n));
}

/* ------------------------------------------------------------------ *
 *  Completion, shared by the success and timeout paths
 * ------------------------------------------------------------------ */

/*
 *  ISR side. Records the outcome and asks for a bottom half. The
 *  division that turns microseconds into millimetres, the event publish
 *  and the user callback all happen in ultra_drain, not here.
 *
 *  The two register operations that CANNOT be deferred stay: masking the
 *  echo pin and disarming the timeout. Leaving the echo unmasked for the
 *  length of a task hop would let a ringing 5 V divider generate spurious
 *  edges that the state machine would then have to filter.
 */
static void finish_i(uint32_t width_us, bool valid)
{
    (void)rc_gpioirq_enable(RC_PIN_ULTRA_ECHO, false);
    alarm_cancel(ALARM_TMO);

    result_width_us = width_us;
    result_valid    = valid;
    result_ready    = true;
    state           = ST_IDLE;

    rc_defer_signal_i(defer_h);
}

/* ------------------------------------------------------------------ *
 *  Bottom half. Task context.
 * ------------------------------------------------------------------ */

static void ultra_drain(void *ctx)
{
    rc_event_t evt;
    uint32_t   mm = 0U;
    uint32_t   width;
    bool       valid;

    (void)ctx;

    if (!result_ready) {
        return;
    }
    result_ready = false;

    width = result_width_us;
    valid = result_valid;

    if (valid) {
        mm = US_TO_MM(width);
        if ((mm > RC_ULTRA_MAX_MM) || (mm < RC_ULTRA_MIN_MM)) {
            valid = false;              /* out of the module's range */
        }
    }

    evt.id = RC_EVT_ULTRA_RESULT;
    evt.u.ultra.angle_deg = tag_angle;
    evt.u.ultra.range_mm  = (uint16_t)mm;
    evt.u.ultra.valid     = valid;
    (void)rc_event_publish(&evt);

    if (user_cb != NULL) {
        user_cb((uint16_t)mm, valid, user_ctx);
    }
}

/* ------------------------------------------------------------------ *
 *  Interrupt handlers
 * ------------------------------------------------------------------ */

/* Trigger pulse has run its 12 us. Drop it and start listening. */
static void trig_done_handler(UINT intno)
{
    (void)intno;
    out_w(TIMER_INTR, (1U << ALARM_TRIG));
    clr_w(TIMER_INTE, (1U << ALARM_TRIG));

    (void)gpio_set_val(RC_PIN_ULTRA_TRIG, 0U);

    state = ST_WAIT_RISE;
    (void)rc_gpioirq_enable(RC_PIN_ULTRA_ECHO, true);
    alarm_arm(ALARM_TMO, RC_ULTRA_TIMEOUT_US);

    EndOfInt(INTNO_TIMER_1);
}

/* No echo inside the timeout. Give up and report an invalid reading
 * rather than leaving the scan stalled waiting for a result. */
static void timeout_handler(UINT intno)
{
    (void)intno;
    out_w(TIMER_INTR, (1U << ALARM_TMO));

    if (state != ST_IDLE) {
        finish_i(0U, false);
    }
    EndOfInt(INTNO_TIMER_2);
}

static void echo_isr(uint32_t pin, bool level, uint32_t t_us, void *ctx)
{
    (void)pin;
    (void)ctx;

    if ((state == ST_WAIT_RISE) && level) {
        t_rise = t_us;
        state  = ST_WAIT_FALL;
    } else if ((state == ST_WAIT_FALL) && !level) {
        finish_i(t_us - t_rise, true);
    } else {
        /* Spurious edge, ignore. Happens on a noisy 5 V divider. */
    }
}

/* ------------------------------------------------------------------ *
 *  Public API
 * ------------------------------------------------------------------ */

rc_result_t drv_ultrasonic_init(void)
{
    T_DINT      dint;
    rc_result_t res;

    defer_h = rc_defer_register(ultra_drain, NULL);
    if (defer_h < 0) {
        return RC_ERR_NOSPACE;
    }

    (void)gpio_set_pin(RC_PIN_ULTRA_TRIG, GPIO_MODE_OUT);
    (void)gpio_set_val(RC_PIN_ULTRA_TRIG, 0U);

    /* Echo is registered now but left masked. It is only unmasked for
     * the window between the trigger ending and the result arriving, so
     * a floating or noisy echo line cannot generate interrupts while the
     * car is doing something else. */
    res = rc_gpioirq_attach(RC_PIN_ULTRA_ECHO, RC_EDGE_BOTH, -1,
                            echo_isr, NULL);
    if (res != RC_OK) {
        return res;
    }
    (void)rc_gpioirq_enable(RC_PIN_ULTRA_ECHO, false);

    dint.intatr = TA_HLNG;
    dint.inthdr = trig_done_handler;
    if (tk_def_int(INTNO_TIMER_1, &dint) != E_OK) {
        return RC_ERR_HARDWARE;
    }
    ClearInt(INTNO_TIMER_1);
    EnableInt(INTNO_TIMER_1, INTPRI_TIMER);

    dint.inthdr = timeout_handler;
    if (tk_def_int(INTNO_TIMER_2, &dint) != E_OK) {
        return RC_ERR_HARDWARE;
    }
    ClearInt(INTNO_TIMER_2);
    EnableInt(INTNO_TIMER_2, INTPRI_TIMER);

    state = ST_IDLE;
    return RC_OK;
}

rc_result_t drv_ultrasonic_on_result(drv_ultra_cb_t cb, void *ctx)
{
    user_cb  = cb;
    user_ctx = ctx;
    return RC_OK;
}

rc_result_t drv_ultrasonic_ping(int16_t tag_angle_deg)
{
    uint32_t sts;

    DI(sts);
    if (state != ST_IDLE) {
        EI(sts);
        return RC_ERR_BUSY;
    }
    state     = ST_TRIG;
    tag_angle = tag_angle_deg;
    EI(sts);

    (void)gpio_set_val(RC_PIN_ULTRA_TRIG, 1U);
    alarm_arm(ALARM_TRIG, RC_ULTRA_TRIG_US);

    return RC_OK;
}

bool drv_ultrasonic_busy(void)
{
    return (state != ST_IDLE);
}
