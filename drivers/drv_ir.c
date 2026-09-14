/*
 *  drv_ir.c
 */
#include "rc_prelude.h"
#include "drv_ir.h"
#include "rc_config.h"
#include "rc_gpioirq.h"
#include "rc_event.h"
#include "rc_defer.h"
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
static int32_t           defer_h = -1;

/*
 *  Edge ring. Single producer (the ISR), single consumer (the bottom
 *  half), power of two, so head and tail need no lock as long as each
 *  side only writes its own index.
 */
#define BAR_RING_SZ     (32U)
#define BAR_RING_MASK   (BAR_RING_SZ - 1U)

typedef struct {
    uint32_t width_us;
    bool     level;
} bar_edge_rec_t;

static bar_edge_rec_t    bar_ring[BAR_RING_SZ];
static volatile uint32_t bar_head;
static volatile uint32_t bar_tail;
static volatile uint32_t bar_overrun;

/* ------------------------------------------------------------------ *
 *  Barcode edge ISR
 * ------------------------------------------------------------------ */

static void barcode_isr(uint32_t pin, bool level, uint32_t t_us, void *ctx)
{
    uint32_t width;
    uint32_t next;

    (void)pin;
    (void)ctx;

    if (!bar_enabled) {
        return;
    }

    /*
     *  One subtraction, one store into the ring, one index bump, one
     *  tk_set_flg. The decoder does not run here.
     *
     *  Bar width IS the data for Code 39, so the timestamp this handler
     *  was given must not be delayed by work that could happen later.
     */
    width       = t_us - bar_last_us;
    bar_last_us = t_us;

    next = (bar_head + 1U) & BAR_RING_MASK;
    if (next == bar_tail) {
        bar_overrun++;      /* decoder is not keeping up, count it */
        return;
    }

    bar_ring[bar_head].width_us = width;
    bar_ring[bar_head].level    = level;
    bar_head = next;

    rc_defer_signal_i(defer_h);
}

/* ------------------------------------------------------------------ *
 *  Bottom half. Publishes one event per buffered edge, in order.
 * ------------------------------------------------------------------ */

static void barcode_drain(void *ctx)
{
    rc_event_t     evt;
    bar_edge_rec_t rec;

    (void)ctx;

    while (bar_tail != bar_head) {
        rec      = bar_ring[bar_tail];
        bar_tail = (bar_tail + 1U) & BAR_RING_MASK;

        evt.id = RC_EVT_BARCODE_EDGE;
        evt.u.bar_edge.level_high = rec.level;
        evt.u.bar_edge.width_us   = rec.width_us;
        (void)rc_event_publish(&evt);

        if (bar_cb != NULL) {
            bar_cb(rec.level, rec.width_us, bar_ctx);
        }
    }
}

uint32_t drv_ir_barcode_overruns(void)
{
    return bar_overrun;
}

/* ------------------------------------------------------------------ *
 *  Init
 * ------------------------------------------------------------------ */

rc_result_t drv_ir_init(void)
{
    rc_result_t res;

    defer_h = rc_defer_register(barcode_drain, NULL);
    if (defer_h < 0) {
        return RC_ERR_NOSPACE;
    }

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
        bar_head    = 0U;
        bar_tail    = 0U;
    }
    bar_enabled = on;
    return rc_gpioirq_enable(RC_PIN_IR_BARCODE_D, on);
}
