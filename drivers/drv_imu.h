/*
 *  drv_imu.h  -  Buddy 4 hardware layer
 *
 *  GY-511 / LSM303DLHC over I2C.
 *
 *  Read this before writing any Buddy 4 code: THIS PART HAS NO
 *  GYROSCOPE. It is a 3-axis accelerometer and a 3-axis magnetometer on
 *  two separate dice with two separate I2C addresses. That changes three
 *  of the tasks in the project brief:
 *
 *    - Tilt must come from the gravity vector in the accelerometer, not
 *      from integrating a rate.
 *    - Turn rate has no direct source. Use differential encoder counts
 *      from Buddy 2, which is more accurate anyway. The magnetometer
 *      will read heading but the motors sit centimetres away and will
 *      swamp it, so treat it as a slow correction at best.
 *    - Hump height cannot come from double-integrating acceleration; the
 *      drift over a two second climb is larger than the hump. Estimate
 *      peak pitch instead and convert with the wheelbase. See
 *      sub_terrain.c for that.
 *
 *  The driver itself just produces calibrated samples. Interpretation
 *  lives in sub_terrain.
 */
#ifndef DRV_IMU_H
#define DRV_IMU_H

#include "rc_types.h"

rc_result_t drv_imu_init(void);

/*
 *  Read one accelerometer sample. Blocks on the I2C driver's own wait,
 *  which is bounded by DEVCNF_I2C1_TMO, so it must be called from a task
 *  and never from an interrupt handler. The sense task owns this.
 */
rc_result_t drv_imu_read_accel(int16_t *x, int16_t *y, int16_t *z);
rc_result_t drv_imu_read_mag(int16_t *x, int16_t *y, int16_t *z);

/*
 *  Sample both, apply the stored bias, publish RC_EVT_IMU_SAMPLE.
 *  Called from the sense task at RC_PERIOD_IMU_MS.
 */
void drv_imu_sample(void);

/*
 *  Zero the accelerometer with the car sitting level and still. Averages
 *  over `n` samples. Call once at start-up, before the run.
 */
rc_result_t drv_imu_calibrate(uint16_t n);

/* Pitch in tenths of a degree, from the gravity vector. Positive is nose
 * up. Only meaningful when the car is not accelerating hard. */
int16_t drv_imu_pitch_ddeg(void);

#endif /* DRV_IMU_H */
