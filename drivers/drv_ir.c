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
/* This is a preprocessor macro used as a compile-time on/off switch, not
 * a runtime bool — see its use in drv_ir_on_line() below, wrapped in
 * `#if IR_ACTIVE_HIGH` / `#else`. The preprocessor deletes whichever
 * branch doesn't apply before the compiler even sees it, so there's zero
 * runtime cost to supporting both sensor wirings. */
#define IR_ACTIVE_HIGH      (1)

/* Device name string for the ADC (analogue-to-digital converter — the
 * hardware block that turns a sensor's analogue voltage into a number
 * the CPU can read) driver. `(UB *)` is a cast — it tells the compiler
 * "treat this string literal as a pointer to UB (unsigned byte)", which
 * is the type this particular RTOS's device-open call expects instead of
 * the normal `char *`. */
#define ADC_DEVNAME         ((UB *)"adca")

static ID       adc_dd = -1;              /* ADC device descriptor once opened; -1/invalid until drv_ir_init() succeeds */
static drv_ir_edge_cb_t bar_cb;           /* optional extra callback for raw barcode edges, set via drv_ir_on_barcode_edge() */
static void            *bar_ctx;          /* context pointer passed back to bar_cb untouched */
/* `volatile` tells the compiler "this variable can change at any moment,
 * outside of normal program flow (here: from inside an interrupt), so
 * never cache it in a register or optimize away a re-read of it." Every
 * variable touched by both the ISR and normal task code in this file is
 * marked volatile for that reason. */
static volatile uint32_t bar_last_us;     /* timestamp (microseconds) of the previous barcode edge, to compute the next width */
static volatile bool     bar_enabled;     /* is the barcode ISR currently allowed to record edges? */
static int32_t           defer_h = -1;    /* handle for the "bottom half" deferred-work registration, see rc_defer_register() */

/*
 *  Edge ring. Single producer (the ISR), single consumer (the bottom
 *  half), power of two, so head and tail need no lock as long as each
 *  side only writes its own index.
 */
/*
 * A "ring buffer" (a.k.a. circular buffer) is a fixed-size array used as
 * a queue: a "head" index is where the next item gets written, a "tail"
 * index is where the next item gets read from, and both indices wrap
 * back to 0 after reaching the end — so the array is reused forever
 * without ever needing to shift elements around. It's used here because
 * the ISR (producer) and barcode_drain() (consumer) run at different
 * times, and the ring lets the ISR hand off work instantly without
 * waiting for the consumer to catch up.
 *
 * BAR_RING_SZ is a power of two (32) specifically so BAR_RING_MASK
 * (31, i.e. binary 0b11111) can wrap an index with a fast bitwise AND
 * (`index & BAR_RING_MASK`) instead of a slower division/modulo — AND-ing
 * with "all 1 bits up to the size" has the exact same effect as
 * `% BAR_RING_SZ` when the size is a power of two, but is cheaper on a
 * small CPU like the RP2040.
 */
#define BAR_RING_SZ     (32U)
#define BAR_RING_MASK   (BAR_RING_SZ - 1U)

typedef struct {
    uint32_t width_us;   /* how long (microseconds) the level held before this edge */
    bool     level;      /* the new level after this edge: true = just went high, false = just went low */
} bar_edge_rec_t;

static bar_edge_rec_t    bar_ring[BAR_RING_SZ];  /* the ring buffer's backing storage */
static volatile uint32_t bar_head;   /* next slot the ISR will write to */
static volatile uint32_t bar_tail;   /* next slot the bottom half will read from */
static volatile uint32_t bar_overrun; /* count of edges dropped because the ring filled up before being drained */

/* ------------------------------------------------------------------ *
 *  Barcode edge ISR
 * ------------------------------------------------------------------ */

/* This is an ISR (Interrupt Service Routine): a function the RP2040's
 * hardware jumps to automatically, pausing whatever else was running,
 * the instant the barcode pin's voltage changes. Because it interrupts
 * everything else, the golden rule (see TEAM_GUIDE.md §0.2) is that it
 * must do the absolute minimum — record a timestamp, stash the data,
 * signal that there's work to do — and never loop, publish an event
 * itself, or do anything slow like division. `pin`, `level` (the pin's
 * new state), and `t_us` (the exact microsecond timestamp of the change)
 * are supplied by the interrupt framework that calls this. */
static void barcode_isr(uint32_t pin, bool level, uint32_t t_us, void *ctx)
{
    uint32_t width;
    uint32_t next;

    /* These two parameters aren't used by this handler; the (void) casts
     * tell the compiler that's intentional so it doesn't warn. */
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

    /* Compute where the NEXT write would land (wrapping via the bitmask
     * trick explained above `BAR_RING_SZ`), without committing to it yet.
     * If that would land on `bar_tail` (the slot the consumer hasn't
     * read yet), the ring is completely full — we must drop this edge
     * rather than overwrite unread data. */
    next = (bar_head + 1U) & BAR_RING_MASK;
    if (next == bar_tail) {
        bar_overrun++;      /* decoder is not keeping up, count it */
        return;
    }

    bar_ring[bar_head].width_us = width;
    bar_ring[bar_head].level    = level;
    bar_head = next;

    /* Hand off to the "bottom half" (barcode_drain, below), which runs
     * later in normal task context rather than inside this interrupt.
     * `_i` suffix is this RTOS's convention for "safe to call from
     * inside an interrupt" — a normal kernel call here could corrupt
     * scheduler state. */
    rc_defer_signal_i(defer_h);
}

/* ------------------------------------------------------------------ *
 *  Bottom half. Publishes one event per buffered edge, in order.
 * ------------------------------------------------------------------ */

/* The "bottom half" — the slower, safer counterpart to barcode_isr()
 * above. This runs in normal task context (not inside an interrupt), so
 * it's allowed to do the "expensive" work of building and publishing an
 * event for each buffered edge, draining the ring buffer until it's
 * caught up with the ISR. */
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

/* Call once at boot. Sets up the two line-sensor GPIO pins, attaches the
 * barcode ISR (masked off until armed), and opens the ADC device used
 * for calibration readings. See drv_ir.h for the full picture. */
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

/* Reads one sensor's digital state and returns whether it currently sees
 * the black line, correcting for board polarity (see IR_ACTIVE_HIGH). */
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

    /* `#if` / `#else` / `#endif` here is a PREPROCESSOR conditional: the
     * compiler decides which branch to actually compile based on the
     * IR_ACTIVE_HIGH macro's value, and throws the other branch away
     * entirely before compiling — unlike a runtime `if`, there is no
     * choice left at all once the program is built. This is how the
     * whole sensor-polarity question (HIGH-means-line vs LOW-means-line)
     * gets resolved with zero runtime cost. */
#if IR_ACTIVE_HIGH
    return (val != 0U);
#else
    return (val == 0U);
#endif
}

/* Reads the raw ADC brightness value (0..4095) on the barcode sensor's
 * analogue pin. Only the barcode channel is wired to an ADC input in
 * this design (see drv_ir.h) — the two line sensors are digital-only, so
 * this returns 0 for them. Used for trim-pot calibration, not for normal
 * line-following or barcode decoding. */
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

/* Called every 5ms (RC_PERIOD_LINE_MS) from the sensing task. Polls both
 * line sensors, turns the two on/off readings into a coarse sideways
 * position number, and publishes it as RC_EVT_LINE_SAMPLE for
 * sub_line.c's on_sample() to react to. */
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

/* Registers an extra callback invoked from INTERRUPT context (see
 * barcode_drain()) for every raw barcode edge, in addition to the
 * RC_EVT_BARCODE_EDGE event that's always published. Most code should
 * just subscribe to the event instead; this exists for callers that need
 * the lower latency of a direct call. */
rc_result_t drv_ir_on_barcode_edge(drv_ir_edge_cb_t cb, void *ctx)
{
    bar_cb  = cb;
    bar_ctx = ctx;
    return RC_OK;
}

/* Arms or disarms the barcode interrupt. On arming, resets the timing
 * baseline and ring buffer indices so stale data from before this arm
 * can't leak into the first read. Disarming masks the interrupt in
 * hardware (via rc_gpioirq_enable) so ordinary track noise can't
 * generate phantom edges while no barcode is expected. */
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
