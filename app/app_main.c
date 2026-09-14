/*
 *  app_main.c
 *
 *  Start-up. Builds everything in dependency order, runs calibration,
 *  then hands the run to sub_nav and goes to sleep.
 *
 *  Order matters here:
 *    1. rc_event first, because every other init subscribes or publishes.
 *    2. rc_gpioirq before any driver that attaches a pin.
 *    3. drivers before the subsystems that sit on them.
 *    4. sub_nav last, because it subscribes to all the others' events.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "rc_config.h"
#include "rc_event.h"
#include "rc_gpioirq.h"
#include "rc_time.h"

#include "drv_motor.h"
#include "drv_encoder.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "drv_ir.h"
#include "drv_imu.h"

#include "sub_motion.h"
#include "sub_line.h"
#include "sub_barcode.h"
#include "sub_terrain.h"
#include "sub_scan.h"
#include "sub_telemetry.h"
#include "sub_nav.h"

#define IMU_CAL_SAMPLES     (100U)

static ID sense_tskid;
static ID blink_tskid;

/* ------------------------------------------------------------------ *
 *  Sense task
 *
 *  Owns every peripheral that has to be polled rather than interrupted:
 *  the two line sensors and the IMU. Both go through the I2C or GPIO
 *  paths that the port's ownership rules say belong to one core, so
 *  keeping them in a single task is deliberate, not lazy.
 * ------------------------------------------------------------------ */

static void sense_task(INT stacd, void *exinf)
{
    uint32_t ms = 0U;

    (void)stacd;
    (void)exinf;

    for (;;) {
        tk_dly_tsk(RC_PERIOD_LINE_MS);
        ms += RC_PERIOD_LINE_MS;

        drv_ir_sample_line();

        if ((ms % RC_PERIOD_IMU_MS) == 0U) {
            drv_imu_sample();
        }
    }
}

/* ------------------------------------------------------------------ *
 *  Liveness. An external LED, because the Pico W's on-board LED hangs
 *  off the CYW43439 radio rather than an RP2040 pin.
 * ------------------------------------------------------------------ */

static void blink_task(INT stacd, void *exinf)
{
    UINT on = 0U;

    (void)stacd;
    (void)exinf;

    (void)gpio_set_pin(RC_PIN_STATUS_LED, GPIO_MODE_OUT);

    for (;;) {
        on = (on != 0U) ? 0U : 1U;
        (void)gpio_set_val(RC_PIN_STATUS_LED, on);
        tk_dly_tsk(500);
    }
}

/* ------------------------------------------------------------------ *
 *  Init helper. A silent failure here surfaces much later as a hang.
 * ------------------------------------------------------------------ */

static BOOL step(const char *what, rc_result_t res)
{
    if (res != RC_OK) {
        tm_printf((UB *)"[init] %s FAILED (%d)\n", what, (int)res);
        return FALSE;
    }
    tm_printf((UB *)"[init] %s ok\n", what);
    return TRUE;
}

/* ------------------------------------------------------------------ */

EXPORT INT usermain(void)
{
    T_CTSK ctsk;

    tm_printf((UB *)"\n=== robotic car: bring-up ===\n");

    if (!step("event bus",  rc_event_init()))      { return 1; }
    if (!step("gpio irq",   rc_gpioirq_init()))    { return 1; }

    if (!step("motor",      drv_motor_init()))     { return 1; }
    if (!step("encoder",    drv_encoder_init()))   { return 1; }
    if (!step("servo",      drv_servo_init()))     { return 1; }
    if (!step("ultrasonic", drv_ultrasonic_init())){ return 1; }
    if (!step("ir",         drv_ir_init()))        { return 1; }

    /* The IMU is allowed to fail without stopping the run. A car that
     * cannot measure humps is still a car that can finish the course. */
    (void)step("imu", drv_imu_init());

    if (!step("motion",    sub_motion_init()))    { return 1; }
    if (!step("line",      sub_line_init()))      { return 1; }
    if (!step("barcode",   sub_barcode_init()))   { return 1; }
    if (!step("terrain",   sub_terrain_init()))   { return 1; }
    if (!step("scan",      sub_scan_init()))      { return 1; }
    if (!step("telemetry", sub_telemetry_init())) { return 1; }
    if (!step("nav",       sub_nav_init()))       { return 1; }

    /* Housekeeping tasks. */
    ctsk.itskpri = RC_PRI_SENSE;
    ctsk.stksz   = RC_STACK_SZ;
    ctsk.task    = sense_task;
    ctsk.exinf   = NULL;
    ctsk.tskatr  = TA_HLNG | TA_RNG3;
    sense_tskid  = tk_cre_tsk(&ctsk);
    if (sense_tskid > E_OK) {
        (void)tk_sta_tsk(sense_tskid, 0);
    }

    ctsk.itskpri = RC_PRI_TELEMETRY;
    ctsk.task    = blink_task;
    blink_tskid  = tk_cre_tsk(&ctsk);
    if (blink_tskid > E_OK) {
        (void)tk_sta_tsk(blink_tskid, 0);
    }

    /* Calibration. Car must be level and still. */
    tm_printf((UB *)"[init] hold still, calibrating IMU...\n");
    (void)drv_imu_calibrate(IMU_CAL_SAMPLES);

    (void)sub_motion_set_speed(250U);
    (void)sub_line_set_base(350);

    tm_printf((UB *)"[init] ready, starting run\n");
    (void)sub_nav_start();

    /* The initial task must not return: the kernel shuts down if it
     * does. Sleep forever and let the other tasks run. */
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}
