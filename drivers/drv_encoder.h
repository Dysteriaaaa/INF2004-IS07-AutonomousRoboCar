/*
 *  drv_encoder.h  -  Buddy 2 hardware layer
 *
 *  Single-channel slotted encoders, one per wheel. There is no quadrature
 *  here (a second sensor offset in phase, which is what would let you
 *  tell direction directly from the disc), so direction is NOT
 *  measurable from the disc itself. The driver takes direction from
 *  whatever drv_motor was last commanded to do, which is correct except
 *  during the fraction of a second the wheel is still coasting after a
 *  reversal. Keep that in mind when tuning.
 *
 *  Timing comes from the free-running microsecond counter inside the
 *  edge ISR (Interrupt Service Routine - code the chip jumps to the
 *  instant the sensor pin changes state, see TEAM_GUIDE.md §0.2), not
 *  from the kernel tick, so speed is usable down to very low RPM without
 *  a long averaging window.
 *
 *  "Odometry" (used throughout this project) just means: turning a count
 *  of wheel clicks into a distance and a speed - the same idea as
 *  counting clicks on a bicycle's spoke card.
 */
#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include "rc_types.h"

/* Call once at boot. Attaches the interrupt handlers to both encoder
 * pins and registers the deferred ("bottom half") worker that turns raw
 * edges into published events - see drv_encoder.c for why the real work
 * can't happen inside the interrupt itself. */
rc_result_t drv_encoder_init(void);

/* Total edges (wheel-clicks) seen since boot, monotonic (only ever goes
 * up, never resets on its own). */
uint32_t drv_encoder_count(rc_side_t side);

/* Microseconds between the two most recent edges. Returns 0 when the
 * wheel has been still longer than the stall timeout (so a stopped wheel
 * reads as "0", not as "infinitely slow"). */
uint32_t drv_encoder_period_us(rc_side_t side);

/* Signed speed in mm/s, derived from the period and the commanded
 * direction (borrowed from drv_motor_get(), see drv_motor.h). Integer
 * maths throughout, there is no FPU (floating-point hardware unit) on
 * this chip, so no fractional numbers are used anywhere in this path. */
int32_t drv_encoder_speed_mm_s(rc_side_t side);

/* Distance in mm since the last drv_encoder_reset() call - this is the
 * actual odometry distance count used by sub_motion.c to know when a
 * "drive forward 300mm" move is done. */
uint32_t drv_encoder_distance_mm(rc_side_t side);

/* Zero the distance baseline (not the raw click count) so a new move can
 * measure "distance travelled during just this move" starting from zero. */
void drv_encoder_reset(void);

#endif /* DRV_ENCODER_H */
