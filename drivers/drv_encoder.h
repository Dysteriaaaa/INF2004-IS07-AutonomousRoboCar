/*
 *  drv_encoder.h  -  Buddy 2 hardware layer
 *
 *  Single-channel slotted encoders, one per wheel. There is no quadrature
 *  here, so direction is NOT measurable from the disc. The driver takes
 *  direction from whatever drv_motor was last commanded to do, which is
 *  correct except during the fraction of a second the wheel is still
 *  coasting after a reversal. Keep that in mind when tuning.
 *
 *  Timing comes from the free-running microsecond counter inside the
 *  edge ISR, not from the kernel tick, so speed is usable down to very
 *  low RPM without a long averaging window.
 */
#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include "rc_types.h"

rc_result_t drv_encoder_init(void);

/* Total edges seen since boot, monotonic. */
uint32_t drv_encoder_count(rc_side_t side);

/* Microseconds between the two most recent edges. Returns 0 when the
 * wheel has been still longer than the stall timeout. */
uint32_t drv_encoder_period_us(rc_side_t side);

/* Signed speed in mm/s, derived from the period and the commanded
 * direction. Integer maths throughout, there is no FPU. */
int32_t drv_encoder_speed_mm_s(rc_side_t side);

/* Distance in mm since the last drv_encoder_reset. */
uint32_t drv_encoder_distance_mm(rc_side_t side);

void drv_encoder_reset(void);

#endif /* DRV_ENCODER_H */
