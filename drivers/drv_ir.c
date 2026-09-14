/*
 *  drv_ir.c
 */
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <bsp/libbsp.h>

#include "drv_ir.h"
#include "rc_config.h"
#include "rc_gpioirq.h"
#include "rc_event.h"
#include "rc_time.h"

/*
 *  Sensor polarity. On the Waveshare ST188 board DOUT goes LOW over a
 *  reflective (white) surface and HIGH over a non-reflective (black)
 *  one, so HIGH means on-line. Invert this if your modules differ, which
 *  is the single most common cause of a car that drives off the table.
 */
#define IR_ACTIVE_HIGH      (1)

#define ADC_DEVNAME         ((UB *)"adca")

static ID       adc_dd = -1;
static drv_ir_edge_cb_t bar_cb;
static void            *bar_ctx;
static volatile uint32_t bar_last_us;
static volatile bool     bar_enabled;

/* ------------------------------------------------------------------ *
 *  Barcode edge ISR
 * ------------------------------------------------------------------ */

static void barcode_isr(uint32_t pin, bool level, uint32_t t_us, void *ctx)
{
    uint32_t   width;
    rc_event_t evt;

    (void)pin;
    (void)ctx;

    if (!bar_enabled) {
        return;
    }

    width       = t_us - bar_last_us;
    bar_last_us = t_us;

    evt.id = RC_EVT_BARCODE_EDGE;
    evt.u.bar_edge.level_high = level;
    evt.u.bar_edge.width_us   = width;
    (void)rc_event_publish_i(&evt);

    if (bar_cb != NULL) {
        bar_cb(level, width, bar_ctx);
    }
}

/* ------------------------------------------------------------------ *
 *  Init
 * ------------------------------------------------------------------ */

rc_result_t drv_ir_init(void)
{
    rc_result_t res;

    /* Line sensors: plain inputs, polled. The comparator output is a
     * push-pull drive on both modules, so no pull is needed. */
    (void)gpio_set_pin(RC_PIN_IR_LINE_L, GPIO_MODE_IN);
    (void)gpio_set_pin(RC_PIN_IR_LINE_R, GPIO_MODE_IN);

    /* Barcode digital pin, interrupt driven, masked until armed. */
    res = rc_gpioirq_attach(RC_PIN_IR_BARCODE_D, RC_EDGE_BOTH, 0,
                            barcode_isr, NULL);
    if (res != RC_OK) {
        return res;
    }
    (void)rc_gpioirq_enable(RC_PIN_IR_BARCODE_D, false);
    bar_enabled = false;

    /* Barcode analogue pin, for threshold calibration. Opening the ADC
     * is allowed to fail: the digital path still works without it. */
    adc_dd = tk_opn_dev(ADC_DEVNAME, TD_READ);

    return RC_OK;
}

/* ------------------------------------------------------------------ *
 *  Reading
 * ------------------------------------------------------------------ */

bool drv_ir_on_line(rc_ir_ch_t ch)
{
    uint32_t pin;
    uint32_t val;

    switch (ch) {
    case RC_IR_LINE_L:
        pin = RC_PIN_IR_LINE_L;
        break;
    case RC_IR_LINE_R:
        pin = RC_PIN_IR_LINE_R;
        break;
    case RC_IR_BARCODE:
        pin = RC_PIN_IR_BARCODE_D;
        break;
    default:
        return false;
    }

    val = gpio_get_val(pin);

#if IR_ACTIVE_HIGH
    return (val != 0U);
#else
    return (val == 0U);
#endif
}

uint16_t drv_ir_read_raw(rc_ir_ch_t ch)
{
    UW  buf = 0U;
    SZ  asz = 0;
    ER  er;

    if ((ch != RC_IR_BARCODE) || (adc_dd <= E_OK)) {
        return 0U;
    }

    /* The mtk3 ADC driver takes the channel number as the start
     * position of the read. A single-sample read is quick enough for the
     * sense task, but note that it does take the driver's own lock, so
     * do not call it from an interrupt handler. */
    er = tk_srea_dev(adc_dd, (W)RC_ADC_CH_IR_BARCODE, &buf, 1, &asz);
    if (er != E_OK) {
        return 0U;
    }
    return (uint16_t)(buf & 0x0FFFU);
}

void drv_ir_sample_line(void)
{
    rc_event_t evt;
    bool       l = drv_ir_on_line(RC_IR_LINE_L);
    bool       r = drv_ir_on_line(RC_IR_LINE_R);
    int16_t    pos;

    /*
     *  Two digital sensors give four states, so position is coarse:
     *
     *      L  R   meaning              position
     *      0  0   line lost            keep the last sign
     *      1  0   drifting right       -500
     *      0  1   drifting left        +500
     *      1  1   centred, or junction    0
     *
     *  This is enough for a working follower. If you want smooth
     *  proportional steering, move to the analogue outputs and
     *  interpolate here instead. That is a sub_line decision; the driver
     *  only reports what it sees.
     */
    if (l && r) {
        pos = 0;
    } else if (l) {
        pos = -500;
    } else if (r) {
        pos = 500;
    } else {
        pos = 0;                        /* sub_line handles the lost case */
    }

    evt.id = RC_EVT_LINE_SAMPLE;
    evt.u.line.on_line_l  = l;
    evt.u.line.on_line_r  = r;
    evt.u.line.position   = pos;
    evt.u.line.barcode_raw = drv_ir_read_raw(RC_IR_BARCODE);

    (void)rc_event_publish(&evt);
}

rc_result_t drv_ir_on_barcode_edge(drv_ir_edge_cb_t cb, void *ctx)
{
    bar_cb  = cb;
    bar_ctx = ctx;
    return RC_OK;
}

rc_result_t drv_ir_barcode_enable(bool on)
{
    if (on) {
        bar_last_us = rc_time_us();
    }
    bar_enabled = on;
    return rc_gpioirq_enable(RC_PIN_IR_BARCODE_D, on);
}
