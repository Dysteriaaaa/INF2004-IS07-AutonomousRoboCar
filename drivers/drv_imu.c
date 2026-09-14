/*
 *  drv_imu.c
 */
#include "rc_prelude.h"
#include <dev_i2c.h>
#include "drv_imu.h"
#include "rc_config.h"
#include "rc_event.h"

/* LSM303DLHC registers. The accelerometer and magnetometer are separate
 * dice with separate address spaces; do not mix them up. */
#define A_CTRL_REG1     (0x20U)
#define A_CTRL_REG4     (0x23U)
#define A_OUT_X_L       (0x28U)

#define M_CRA_REG       (0x00U)
#define M_CRB_REG       (0x01U)
#define M_MR_REG        (0x02U)
#define M_OUT_X_H       (0x03U)

/* 100 Hz, all three axes enabled, normal power mode. */
#define A_ODR_100HZ     (0x57U)
/* +/-2 g, high resolution. At 2 g one LSB is about 1 mg, which is the
 * resolution we want for hump detection. */
#define A_FS_2G_HR      (0x08U)

#define M_ODR_75HZ      (0x18U)
#define M_GAIN_1_3      (0x20U)
#define M_CONTINUOUS    (0x00U)

/* Auto-increment on a multi-byte read is the MSB of the sub-address on
 * the accelerometer die. The magnetometer auto-increments by default. */
#define AUTO_INC        (0x80U)

#define I2C_DEVNAME     ((UB *)"iicb")   /* unit 1 */

static ID      i2c_dd = -1;
static int16_t bias_x;
static int16_t bias_y;
static int16_t bias_z;
static int16_t last_ax;
static int16_t last_ay;
static int16_t last_az;

/* ------------------------------------------------------------------ *
 *  I2C helpers
 * ------------------------------------------------------------------ */

static ER reg_write(UW sadr, UB reg, UB val)
{
    T_I2C_EXEC exec;
    UB         snd[2];
    SZ         asz;

    snd[0] = reg;
    snd[1] = val;

    exec.sadr     = sadr;
    exec.snd_size = 2;
    exec.snd_data = snd;
    exec.rcv_size = 0;
    exec.rcv_data = NULL;

    return tk_swri_dev(i2c_dd, TDN_I2C_EXEC, &exec, sizeof(exec), &asz);
}

static ER reg_read_burst(UW sadr, UB reg, UB *buf, INT len)
{
    T_I2C_EXEC exec;
    SZ         asz;

    exec.sadr     = sadr;
    exec.snd_size = 1;
    exec.snd_data = &reg;
    exec.rcv_size = len;
    exec.rcv_data = buf;

    return tk_swri_dev(i2c_dd, TDN_I2C_EXEC, &exec, sizeof(exec), &asz);
}

/* ------------------------------------------------------------------ *
 *  Init
 * ------------------------------------------------------------------ */

rc_result_t drv_imu_init(void)
{
    i2c_dd = tk_opn_dev(I2C_DEVNAME, TD_UPDATE);
    if (i2c_dd <= E_OK) {
        return RC_ERR_HARDWARE;
    }

    if (reg_write(RC_I2C_ADDR_ACCEL, A_CTRL_REG1, A_ODR_100HZ) != E_OK) {
        return RC_ERR_HARDWARE;
    }
    if (reg_write(RC_I2C_ADDR_ACCEL, A_CTRL_REG4, A_FS_2G_HR) != E_OK) {
        return RC_ERR_HARDWARE;
    }

    (void)reg_write(RC_I2C_ADDR_MAG, M_CRA_REG, M_ODR_75HZ);
    (void)reg_write(RC_I2C_ADDR_MAG, M_CRB_REG, M_GAIN_1_3);
    (void)reg_write(RC_I2C_ADDR_MAG, M_MR_REG,  M_CONTINUOUS);

    return RC_OK;
}

/* ------------------------------------------------------------------ *
 *  Reads
 * ------------------------------------------------------------------ */

rc_result_t drv_imu_read_accel(int16_t *x, int16_t *y, int16_t *z)
{
    UB b[6];

    if ((x == NULL) || (y == NULL) || (z == NULL)) {
        return RC_ERR_PARAM;
    }
    if (reg_read_burst(RC_I2C_ADDR_ACCEL, A_OUT_X_L | AUTO_INC, b, 6)
        != E_OK) {
        return RC_ERR_HARDWARE;
    }

    /* Accelerometer is little endian and left justified: the 12 bit
     * result sits in the top bits, so shift down by 4. */
    *x = (int16_t)(((int16_t)((uint16_t)b[1] << 8) | b[0]) >> 4);
    *y = (int16_t)(((int16_t)((uint16_t)b[3] << 8) | b[2]) >> 4);
    *z = (int16_t)(((int16_t)((uint16_t)b[5] << 8) | b[4]) >> 4);

    return RC_OK;
}

rc_result_t drv_imu_read_mag(int16_t *x, int16_t *y, int16_t *z)
{
    UB b[6];

    if ((x == NULL) || (y == NULL) || (z == NULL)) {
        return RC_ERR_PARAM;
    }
    if (reg_read_burst(RC_I2C_ADDR_MAG, M_OUT_X_H, b, 6) != E_OK) {
        return RC_ERR_HARDWARE;
    }

    /* Magnetometer is BIG endian, and the axis order is X, Z, Y. Both of
     * those catch people out. */
    *x = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
    *z = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
    *y = (int16_t)(((uint16_t)b[4] << 8) | b[5]);

    return RC_OK;
}

void drv_imu_sample(void)
{
    rc_event_t evt;
    int16_t    ax = 0;
    int16_t    ay = 0;
    int16_t    az = 0;
    int16_t    mx = 0;
    int16_t    my = 0;
    int16_t    mz = 0;

    if (drv_imu_read_accel(&ax, &ay, &az) != RC_OK) {
        return;
    }
    (void)drv_imu_read_mag(&mx, &my, &mz);

    ax -= bias_x;
    ay -= bias_y;
    /* Z keeps its 1 g offset on purpose: it is the reference the pitch
     * calculation needs. Only the level-sitting error is removed. */
    az -= bias_z;

    last_ax = ax;
    last_ay = ay;
    last_az = az;

    evt.id = RC_EVT_IMU_SAMPLE;
    evt.u.imu.acc_x = ax;
    evt.u.imu.acc_y = ay;
    evt.u.imu.acc_z = az;
    evt.u.imu.mag_x = mx;
    evt.u.imu.mag_y = my;
    evt.u.imu.mag_z = mz;

    (void)rc_event_publish(&evt);
}

rc_result_t drv_imu_calibrate(uint16_t n)
{
    int32_t sx = 0;
    int32_t sy = 0;
    int32_t sz = 0;
    int16_t x;
    int16_t y;
    int16_t z;
    uint16_t i;
    uint16_t got = 0U;

    if (n == 0U) {
        return RC_ERR_PARAM;
    }

    for (i = 0U; i < n; i++) {
        if (drv_imu_read_accel(&x, &y, &z) == RC_OK) {
            sx += x;
            sy += y;
            sz += z;
            got++;
        }
        /* Yielding between samples, not spinning. This runs once at
         * start-up before the run begins, so the delay is free. */
        (void)tk_dly_tsk(5);
    }

    if (got == 0U) {
        return RC_ERR_HARDWARE;
    }

    bias_x = (int16_t)(sx / (int32_t)got);
    bias_y = (int16_t)(sy / (int32_t)got);

    /* Sitting level, Z reads 1 g. Store only the deviation from that so
     * the sign of pitch stays meaningful. */
    bias_z = (int16_t)((sz / (int32_t)got) - 1000);

    return RC_OK;
}

int16_t drv_imu_pitch_ddeg(void)
{
    /*
     *  Small-angle approximation: for pitch under about 25 degrees,
     *  asin(ax/g) is within a couple of percent of ax/g in radians, and
     *  a speed hump never exceeds that. Avoiding atan2 keeps this
     *  integer-only, which matters because the Cortex-M0+ has no FPU and
     *  a soft-float atan2 would cost thousands of cycles at 100 Hz.
     *
     *  ax is in mg, 1 g = 1000. radians = ax/1000.
     *  tenths of a degree = radians * 573.
     */
    return (int16_t)(((int32_t)last_ax * 573) / 1000);
}
