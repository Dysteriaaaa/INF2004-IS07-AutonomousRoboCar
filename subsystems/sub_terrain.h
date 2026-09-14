/*
 *  sub_terrain.h  -  Buddy 4
 *
 *  Hump detection, peak measurement, motion classification and impact
 *  detection, all from the LSM303DLHC accelerometer.
 *
 *  Read the warning at the top of drv_imu.h first. There is no
 *  gyroscope, which rules out the textbook approach to most of this.
 */
#ifndef SUB_TERRAIN_H
#define SUB_TERRAIN_H

#include "rc_types.h"

typedef void (*sub_terrain_hump_cb_t)(uint16_t peak_mm,
                                      uint32_t duration_ms,
                                      void *ctx);

rc_result_t sub_terrain_init(void);
rc_result_t sub_terrain_on_hump(sub_terrain_hump_cb_t cb, void *ctx);

/* Highest hump peak seen this run, in mm. This is the number the brief
 * asks to be reported. */
uint16_t sub_terrain_max_peak_mm(void);

rc_motion_class_t sub_terrain_motion_class(void);

void sub_terrain_reset(void);

#endif /* SUB_TERRAIN_H */
