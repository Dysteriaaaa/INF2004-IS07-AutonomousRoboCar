/*
 *  drv_ir.h  -  Buddy 3 hardware layer
 *
 *  Three reflective IR sensors. Two watch the line, one reads barcodes.
 *
 *  The two line sensors are read digitally and polled, because at the
 *  5 ms line period a poll is cheaper than an interrupt and the timing
 *  does not need to be tight.
 *
 *  The barcode sensor is the opposite case. Bar widths at speed are tens
 *  of milliseconds and the ratio between wide and narrow bars is what
 *  carries the data, so each transition needs a microsecond stamp. That
 *  one is interrupt driven and publishes a width with every edge.
 *
 *  Both the ST188 and TCRT5000 modules present a comparator output on
 *  DOUT with a trim pot for the threshold, and the raw divider on AOUT.
 *  The analogue path is exposed for calibration: read AOUT over the white
 *  track and over the black line, and set the pot midway between them.
 */
#ifndef DRV_IR_H
#define DRV_IR_H

#include "rc_types.h"

typedef enum {
    RC_IR_LINE_L = 0,
    RC_IR_LINE_R,
    RC_IR_BARCODE
} rc_ir_ch_t;

/*
 *  Barcode edge callback. INTERRUPT context. `width_us` is how long the
 *  level that just ended had been held, which is the bar or space width.
 */
typedef void (*drv_ir_edge_cb_t)(bool new_level, uint32_t width_us, void *ctx);

rc_result_t drv_ir_init(void);

/* Digital state. True means "over the line", after polarity correction. */
bool drv_ir_on_line(rc_ir_ch_t ch);

/* Raw ADC counts, 0..4095. Only RC_IR_BARCODE is wired to an ADC pin in
 * the default map. Returns 0 for the others. */
uint16_t drv_ir_read_raw(rc_ir_ch_t ch);

/* Poll both line sensors and publish RC_EVT_LINE_SAMPLE. Called from the
 * sense task at RC_PERIOD_LINE_MS. */
void drv_ir_sample_line(void);

rc_result_t drv_ir_on_barcode_edge(drv_ir_edge_cb_t cb, void *ctx);

/* Barcode edges are only interesting while the car expects a barcode.
 * Masking when idle keeps a shiny patch of track from flooding the bus. */
rc_result_t drv_ir_barcode_enable(bool on);

/* Edges dropped because the decoder could not keep up. Should stay 0. */
uint32_t drv_ir_barcode_overruns(void);

#endif /* DRV_IR_H */
