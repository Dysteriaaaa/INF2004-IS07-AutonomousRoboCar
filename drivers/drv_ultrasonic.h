/*
 *  drv_ultrasonic.h  -  Buddy 5 hardware layer
 *
 *  HC-SR04, fully interrupt driven. No part of a measurement spins.
 *
 *  The classic Arduino driver does this:
 *
 *      digitalWrite(TRIG, HIGH); delayMicroseconds(10);
 *      digitalWrite(TRIG, LOW);  pulseIn(ECHO, HIGH);
 *
 *  Both of those are busy-waits, and pulseIn can hold the CPU for 30 ms
 *  when nothing echoes back. On a car that is running a PID loop at 50 Hz
 *  that is a missed control update and a swerve. Here:
 *
 *    ping()        raises TRIG, arms TIMER alarm 1 for +12 us, returns
 *    alarm 1 ISR   drops TRIG, unmasks the ECHO edge, arms alarm 2
 *                  as the timeout, returns
 *    ECHO rise ISR stamps the start, returns
 *    ECHO fall ISR computes the width, publishes the result, returns
 *    alarm 2 ISR   publishes an invalid result if no echo came back
 *
 *  Total CPU time per measurement is a few microseconds spread over five
 *  interrupts. The 30 ms of flight time costs nothing.
 *
 *  The RP2040 TIMER alarms are unused by this mtk3 port (the kernel tick
 *  is SysTick and the physical timer is built on PWM), so alarms 1 and 2
 *  are ours. Alarms 0 and 3 are left free.
 */
#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include "rc_types.h"

/*
 *  Result callback. Runs in INTERRUPT context, so keep it to a store and
 *  a publish. If you want task context instead, subscribe to
 *  RC_EVT_ULTRA_RESULT and let the dispatcher hand it to you.
 */
typedef void (*drv_ultra_cb_t)(uint16_t range_mm, bool valid, void *ctx);

rc_result_t drv_ultrasonic_init(void);

/* Optional direct hook, in addition to the event published on every
 * completed measurement. Pass NULL to clear. */
rc_result_t drv_ultrasonic_on_result(drv_ultra_cb_t cb, void *ctx);

/*
 *  Start one measurement. Returns RC_ERR_BUSY if one is already in
 *  flight. Never blocks. The result arrives via the callback and via
 *  RC_EVT_ULTRA_RESULT, tagged with the servo angle at the time of the
 *  ping so a scan can pair range to bearing without extra bookkeeping.
 */
rc_result_t drv_ultrasonic_ping(int16_t tag_angle_deg);

bool drv_ultrasonic_busy(void);

#endif /* DRV_ULTRASONIC_H */
