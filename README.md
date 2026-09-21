# Robotic Car Framework

Callback-driven, non-blocking framework for the autonomous robotic car
project. Raspberry Pi Pico W on a Cytron Robo Pico, µT-Kernel 3.0 from the
`mtk3smp-rp2040` port.

Start with **`docs/HARDWARE.md`**. Section 1 lists four pin and hardware
conflicts that will each cost a day if nobody warns you.

## Layout

```
core/         event bus, timebase, GPIO interrupt mux, PWM helper
drivers/      one module per physical device
subsystems/   one module per buddy, plus the mission state machine
app/          usermain and the housekeeping tasks
docs/         hardware and setup guide
```

| File | Buddy | What it owns |
|---|---|---|
| `sub_telemetry.*` | 1 | Framing, topics, publish schedule, pluggable transport |
| `sub_motion.*` | 2 | PID, odometry, the queued-move API |
| `sub_line.*`, `sub_barcode.*` | 3 | Line following, Code 39 decoding |
| `sub_terrain.*` | 4 | Hump detection, peak estimate, motion classification |
| `sub_scan.*` | 5 | Coarse/fine scan, profiling, avoidance planning |
| `sub_nav.*` | shared | Mission state machine |

## What "non-blocking" means here

It does **not** mean tasks never block. A task waiting in `tk_wai_flg` or
`tk_dly_tsk` has yielded the CPU, which is exactly what you want in an RTOS.

It means:

1. **No busy-waits.** No `while (gpio_get_val(ECHO));`, no
   `delayMicroseconds` inside a driver.
2. **No long operation holds a task.** A 30 ms ultrasonic flight time costs
   five interrupts and a few microseconds of CPU, not 30 ms of a task.
3. **Every command returns immediately.** `sub_motion_forward_mm(300)` starts
   a move and returns; completion arrives on a callback.
4. **ISRs do the minimum.** Timestamp, update a counter, publish. All real
   work happens in a dispatcher task.

The one thing the framework deliberately does *not* do is generate the
HC-SR04's 10 µs trigger with a delay loop. It uses an RP2040 TIMER alarm, so
even that is interrupt-driven.

## The event bus

Everything is wired through `core/rc_event.h`. Producers publish an
`rc_event_t`; consumers register a callback against an event id. Subsystems
never call each other directly, which is what lets five people develop
against stubs and integrate late.

```c
rc_event_subscribe(RC_EVT_HUMP_END, RC_LANE_SLOW, on_hump, NULL);
```

**Two lanes.** `RC_LANE_FAST` is a high-priority dispatcher for the control
path; `RC_LANE_SLOW` is a low-priority one for telemetry, logging and
planning. A slow telemetry consumer therefore cannot delay the PID loop.

**Publishing is ISR-safe.** It is a bounded copy into a power-of-two ring
followed by `tk_set_flg`, which is legal from interrupt context. A message
buffer would not be — `tk_snd_mbf` can wait, and the encoder and echo ISRs
both need to publish.

**Callback rules:**

- Callbacks run in the dispatcher **task**, so kernel calls are fine.
- Callbacks must not block. No `TMO_FEVR`.
- Publishing from inside a callback is fine.
- The exceptions are documented per-driver: `drv_ultrasonic_on_result` and
  `drv_ir_on_barcode_edge` fire in **interrupt** context. Subscribe to the
  matching event instead if you want task context.

`rc_event_dropped(lane)` returns the count of events lost to a full ring.
Non-zero means a consumer is too slow or a ring is too small. Telemetry
reports it in the heartbeat — watch it.

## What is finished and what is not

Complete and ready to build against:

- The whole of `core/` — event bus, timebase, GPIO interrupt mux, PWM
- `drv_motor`, `drv_encoder`, `drv_servo`, `drv_ultrasonic`, `drv_ir`,
  `drv_imu`
- Every subsystem's interface, event flow and state machine
- The console telemetry sink and the full framing layer

Deliberately left as marked `TODO`, because it is the graded work:

| Where | What |
|---|---|
| `sub_motion.c` | PID gains (placeholders), encoder-based turning, reverse distance |
| `sub_line.c` | Derivative term; analogue position interpolation |
| `sub_barcode.c` | Code 39 patterns for B, C, D are placeholders |
| `sub_terrain.c` | Encoder cross-check on hump height; turn classification |
| `sub_scan.c` | Side memory, "reverse and reattempt" case |
| `sub_telemetry.c` | UDP and MQTT sinks, connection recovery |

Each is a comment at the point it matters, explaining what to do and why the
obvious approach does not work.

## Barr-C notes

- Fixed-width types throughout. Kernel types (`UW`, `ID`, `ER`) appear only at
  the kernel API boundary.
- Braces on every `if`/`else`, including single statements.
- Every `if`/`else if` chain ends in an `else`; every `switch` has a
  `default`.
- No magic numbers: constants are named, and every pin lives in
  `rc_config.h`.
- File-scope objects are `static`.
- No floating point anywhere. The Cortex-M0+ has no FPU.

## First steps

1. `git clone --recurse-submodules`, then `./build/setup.sh`. That fetches the
   kernel port and applies the `docs/HARDWARE.md` §1 patches for you.
2. Measure your car and update the mechanical constants in `rc_config.h`.
3. Check `IR_ACTIVE_HIGH` in `drv_ir.c` against your sensor modules.
4. `./build/build.sh` (single core first; the console is USB, GP0/GP1 are the
   left encoder). Flash `build/out/*.uf2`. Work through the bring-up table in
   `docs/HARDWARE.md` §7.
