/*
 *  sub_barcode.h  -  Buddy 3, barcode decoding
 *
 *  Consumes RC_EVT_BARCODE_EDGE, which carries the width in microseconds
 *  of each bar and space as the car drives over them, and produces a
 *  decoded character plus the navigation command it maps to.
 *
 *  Why widths and not a clocked sample: the car's speed varies, so the
 *  absolute width of a bar is meaningless. What is stable is the RATIO
 *  between wide and narrow bars, which is 2:1 or 3:1 in Code 39. The
 *  decoder works entirely in ratios and so is speed independent, which
 *  is what "robust operation at varying speeds" in the brief needs.
 */
#ifndef SUB_BARCODE_H
#define SUB_BARCODE_H

#include "rc_types.h"

typedef void (*sub_barcode_cb_t)(char symbol, rc_nav_cmd_t cmd, void *ctx);

rc_result_t sub_barcode_init(void);

/* Arm or disarm decoding. Leave it disarmed while the car is not
 * expecting a barcode, so track noise does not produce phantom commands.
 * sub_nav arms this when sub_line reports a junction. */
rc_result_t sub_barcode_arm(bool on);

rc_result_t sub_barcode_on_decode(sub_barcode_cb_t cb, void *ctx);

/* Last successful decode, for telemetry. */
char sub_barcode_last(void);

#endif /* SUB_BARCODE_H */
