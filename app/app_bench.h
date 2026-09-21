/*
 *  app_bench.h
 *
 *  Bench modes: run ONE buddy's subsystem on its own instead of the full
 *  mission, with its readings printed to the console. Picked at build time:
 *
 *      ./build/build.sh bench=motion       (Buddy 2)
 *      ./build/build.sh bench=line         (Buddy 3, sensors only, motors off)
 *      ./build/build.sh bench=follow       (Buddy 3, the car drives)
 *      ./build/build.sh bench=barcode      (Buddy 3)
 *      ./build/build.sh bench=imu          (Buddy 4)
 *      ./build/build.sh bench=ultra        (Buddy 5, one ping per 100 ms)
 *      ./build/build.sh bench=scan         (Buddy 5, full sweep + plan)
 *      ./build/build.sh bench=telemetry    (Buddy 1)
 *
 *  With no bench= argument RC_BENCH is RC_BENCH_NONE and usermain runs the
 *  mission as normal. Every driver and subsystem is still initialised in a
 *  bench build; only what happens after "[init] ready" changes.
 */
#ifndef APP_BENCH_H
#define APP_BENCH_H

#define RC_BENCH_NONE       (0)
#define RC_BENCH_MOTION     (1)
#define RC_BENCH_LINE       (2)
#define RC_BENCH_FOLLOW     (3)
#define RC_BENCH_BARCODE    (4)
#define RC_BENCH_IMU        (5)
#define RC_BENCH_ULTRA      (6)
#define RC_BENCH_SCAN       (7)
#define RC_BENCH_TELEMETRY  (8)

#ifndef RC_BENCH
#define RC_BENCH            RC_BENCH_NONE
#endif

/* Human-readable name of the selected bench, for the boot banner. */
const char *rc_bench_name(void);

/* Runs the selected bench forever. Called from usermain in place of
 * sub_nav_start(); never returns. Not called when RC_BENCH is NONE. */
void rc_bench_run(void);

#endif /* APP_BENCH_H */
