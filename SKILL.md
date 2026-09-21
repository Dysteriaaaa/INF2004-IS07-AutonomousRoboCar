---
name: inf2004-robocar
description: Context for the INF2004-IS07 autonomous robotic car - a Raspberry Pi Pico W line-following, barcode-decoding, obstacle-avoiding car on micro T-Kernel 3.0 (mtk3smp-rp2040). Use when working on any file in this repo, when asked about the robo car, RoboCar, line following, barcode decoding, hump detection, ultrasonic scanning, telemetry, the event bus, or any rc_/drv_/sub_ prefixed module. Also use when asked to build, debug, extend, or review this codebase, or when adding a new sensor, subsystem, or driver to it.
---

# INF2004-IS07 Autonomous Robotic Car

Read this before changing anything in this repo. Several of the decisions here
look wrong until you know the constraint that forced them, and the most
expensive mistakes on this project are the ones that look like cleanups.

## What the project is

A five-person team build of an autonomous robotic car for a course module. The
car must:

1. Follow a black line using IR sensors
2. Detect and decode barcodes on the track, then execute the encoded command
   (turn left, turn right, go straight, U-turn)
3. Detect speed humps and report the highest peak encountered
4. Detect obstacles, profile them with a servo-mounted ultrasonic sensor,
   navigate around them, and reacquire the line
5. Report telemetry over WiFi

Assessment covers navigation accuracy, completion time, reliability, code
quality, resource efficiency and system robustness.

## Platform

| Thing | Value |
|---|---|
| MCU board | Raspberry Pi Pico W (RP2040 + CYW43439) |
| Carrier | Cytron Robo Pico |
| RTOS | micro T-Kernel 3.0, `sirfonzie/mtk3smp-rp2040` |
| Language | C, Barr-C coding standard |
| Toolchain | `arm-none-eabi-gcc` 13.2.1 |
| Build | The port's own `make` tree, **not** CMake and not the Pico SDK build |
| FPU | **None.** Cortex-M0+. Everything is integer or fixed point |

The project brief says "Pico C SDK". The port only pulls the SDK in for the
USB-CDC console, and uses TinyUSB from it rather than its CMake build. On this
car that console is **mandatory** (`CONSOLE=usb_cdc`): GP0/GP1, the port's
default UART0, are the left wheel encoder. If that matters for marking,
confirm with the module lead.

## Repo layout

```
core/         event bus, timebase, GPIO interrupt mux, PWM, bottom halves
drivers/      one module per physical device
subsystems/   one module per team member, plus the mission state machine
app/          usermain and housekeeping tasks
docs/         HARDWARE.md - read this for wiring and setup
              img/hw/ - pin map diagram + the script that generates it
documents/    datasheets, the Week 6 design review, and its diagrams
build/        setup.sh / build.sh / flash.sh, robocar.mk (the makefile hook),
              patch_port.py (board patches); build/out/ holds the .uf2 images
external/     mtk3smp-rp2040 kernel port as a pinned git submodule
TEAM_GUIDE.md - beginner-friendly, per-buddy build/flash steps and a
               line-by-line code walkthrough of every subsystem and driver
```

Two generated pin diagrams live in `docs/img/hw/`. Edit the scripts, never
the PNGs, and re-run them:

| Script | Output | Review copy | What it shows |
|---|---|---|---|
| `gen_pin_map.py` | `robopico_pin_map.png` | `d2_pin_layout.png` | The Robo Pico drawn to scale (terminals/servo top, Grove 1 left, Grove 7 right, Grove 2-6 bottom, headers + socket centre) with poster-style chips fanning out from each connector: GPIO, function in use, device |
| `gen_board_view.py` | `robopico_board_view.png` | `d2b_board_view.png` | The same wiring drawn on the physical board, for finding the right socket |

Board geometry and port assignments came from the Cytron Robo Pico datasheet
in `documents/Robo_Pico.pdf` (its text is glyph-encoded, so the board images
have to be extracted and read); signal assignments come from
`core/rc_config.h` and section 2 of the Week 6 design review. Facts worth
keeping: the Robo Pico has **seven** Grove ports (1 on the left edge, 7 on
the right edge, 2-6 along the bottom); the silkscreen carries **GP26 on both
Grove 5 and Grove 6**, so line sensor 2 on Grove 5 must have only its DO wire
connected; the MAKER port shares GP2/GP3 with Grove 2 (the ultrasonic) and
must stay empty; and the two blue parts at the bottom corners are WS2812 RGB
LEDs on GP18, not buttons (the buttons are GP20/GP21).

Current wiring, one Grove cable per device: Grove 1 = left encoder A/B
(GP0/GP1), Grove 2 = HC-SR04 TRIG/ECHO (GP2/GP3), Grove 3 = IMU on I²C0
(SDA GP4 / SCL GP5), Grove 4 = line sensor 1 DO (GP16), Grove 5 = line
sensor 2 DO (GP6), Grove 6 = barcode AO/DO (GP26/GP27), Grove 7 = right
encoder A/B (GP7/GP28). Motors on M1 (GP8/GP9) and M2 (GP10/GP11), scan
servo on servo port 4 (GP15), status LED on GP19. Buzzer, buttons and UART
are unused.

`TEAM_GUIDE.md` is the onboarding document for team members with no prior
C or embedded background: it explains the event bus, the non-blocking
pattern and the mission state machine in plain language, then gives each
"Buddy" a getting-started checklist, a function-by-function code
walkthrough, and their module's TODO list. Every subsystem and driver
source file also carries matching inline comments aimed at the same
audience. Point someone here before re-explaining project basics from
scratch.

| Module | Owner | Responsibility |
|---|---|---|
| `sub_telemetry.*` | Buddy 1 | Framing, topics, publish schedule, pluggable transport |
| `sub_motion.*` | Buddy 2 | PID, odometry, the queued-move API |
| `sub_line.*`, `sub_barcode.*` | Buddy 3 | Line following, Code 39 decoding |
| `sub_terrain.*` | Buddy 4 | Hump detection, peak estimate, motion classification |
| `sub_scan.*` | Buddy 5 | Coarse/fine scan, obstacle profiling, avoidance planning |
| `sub_nav.*` | shared | Mission state machine |

## Build status

Both configurations build clean and link to a flashable UF2:

```
SMP=0 usb_cdc   62,784 B text   20,892 B bss    0 warnings
SMP=1 usb_cdc   73,188 B text   25,000 B bss    0 warnings
```

(arm-none-eabi-gcc 15.2.1, Pico SDK 2.3.1 for TinyUSB, via
`build/build.sh`.) Start on `SMP=0`. Nothing has been run on hardware yet.

## Architecture: the event bus

Everything is wired through `core/rc_event.h`. Producers publish an
`rc_event_t`; consumers register a callback against an event id. Subsystems
never call each other directly, which is what lets five people develop against
stubs and integrate late.

```c
rc_event_subscribe(RC_EVT_HUMP_END, RC_LANE_SLOW, on_hump, NULL);
```

**Two priority lanes.** `RC_LANE_FAST` is a high-priority dispatcher task for
the control path. `RC_LANE_SLOW` is a low-priority one for telemetry, logging
and planning. A slow telemetry consumer therefore cannot delay the PID loop.
When adding a subscription, ask whether the car steers differently because of
it. If not, it goes on the slow lane.

`rc_event_dropped(lane)` counts events lost to a full ring. It should stay at
zero; telemetry reports it in the heartbeat.

## Architecture: the interrupt rule

This is the constraint that shapes the drivers. **Two encoder pins, the
ultrasonic echo pin and the barcode pin all share one interrupt vector,
IO_IRQ_BANK0.** Time spent in any one handler is time the other three are
blocked, and a delayed barcode edge timestamp is corrupted data, because bar
*width* is what Code 39 encodes.

So an interrupt handler in this tree may do **only**:

- read the microsecond counter
- a fixed number of register reads and writes (ack, mask, arm)
- stores into its own driver's state, with no loops over data
- one subtraction, if it needs an elapsed time
- `rc_defer_signal_i()`, which is a single `tk_set_flg`

It may **not** publish events, run a decoder, divide, or call a user callback.
All of that goes in a bottom half registered with `rc_defer_register()`, which
runs in a task above both dispatchers.

Measured on the SMP=0 image:

| Handler | Instructions | ~Time @125 MHz |
|---|---|---|
| `encoder_isr` | 18 (+1 GPIO read for channel B since the A/B change; re-measure) | 0.2 µs |
| `timeout_handler` | 34 | 0.4 µs |
| `barcode_isr` | 36 | 0.4 µs |
| `trig_done_handler` | 36 | 0.4 µs |
| `echo_isr` | 49 | 0.5 µs |
| `gpio_bank0_handler` | 119 | 1.3 µs |

`gpio_bank0_handler` is the shared demultiplexer and includes the bit scan, so
it is the one to watch if more pins get added.

**If you add an interrupt, follow this pattern.** Record and flag in the ISR,
do the work in a drain. Re-measure with `objdump -d` afterwards.

## Architecture: what "non-blocking" means here

It does **not** mean tasks never block. A task waiting in `tk_wai_flg` or
`tk_dly_tsk` has yielded the CPU, which is correct on an RTOS.

It means:

1. **No busy-waits.** No `while (gpio_get_val(ECHO));`, no
   `delayMicroseconds` inside a driver.
2. **No long operation holds a task.** The 30 ms ultrasonic flight time costs
   five interrupts and a few microseconds of CPU, not 30 ms of a task.
3. **Every command returns immediately.** `sub_motion_forward_mm(300)` starts
   a move and returns; completion arrives on a callback.

Even the HC-SR04's 10 µs trigger pulse is generated with an RP2040 TIMER alarm
rather than a delay loop.

**If you are about to add a delay or a poll loop to a driver, stop.** The
pattern to copy is `drv_ultrasonic.c`.

## Five things that look like bugs but are not

### 1. `rc_prelude.h` exists and must be included first

`include/tk/syslib.h` does `typedef SZ size_t;` where `SZ` is a **signed**
int. newlib's `stddef.h` declares `size_t` as **unsigned** int. Include
`<string.h>` or `<stdio.h>` alongside any mtk3 header and you get:

```
error: conflicting types for 'size_t'; have 'unsigned int'
```

The port's own sources never hit this because none of them include libc string
or stdio headers. `rc_prelude.h` includes `<stddef.h>` first, then defines
mtk3's documented `PROHIBIT_DEF_SIZE_T` escape hatch, then pulls in the tk
headers. **Include it first in every `.c` file.** Do not "tidy it away".

### 2. There is no `snprintf`

`core/rc_fmt.c` provides `rc_snprintf` instead. newlib's printf is roughly
10-20 kB of flash and some configurations call `malloc` inside it, which is not
something to do from a periodic RTOS task. `rc_snprintf` supports `%d %i %u
%ld %lu %c %s %%` and nothing else. No floats, deliberately: there is no FPU.
The linked image contains **zero** newlib printf or malloc symbols.

### 3. `sts` is `UINT`, not `uint32_t`, wherever it reaches a kernel API

`ISpinLock` takes `UINT *`. On arm-none-eabi `uint32_t` is `long unsigned
int`. Same width, incompatible pointer type. This only shows up in the `SMP=1`
build, so an SMP=0-only check will miss it.

### 4. `memset` on structs was removed on purpose

Field-by-field initialisation avoids pulling libc in and keeps the `size_t`
problem from coming back. Do not "simplify" it.

### 5. The `finish_i` / `ultra_drain` split in `drv_ultrasonic.c`

Masking the echo pin and disarming the timeout stay in the ISR because they
cannot wait for a task hop. The division, the event and the callback are
deferred. That asymmetry is intentional.

## Hardware conflicts that are already worked around

These are in `docs/HARDWARE.md` in full. Summary:

| Conflict | Resolution |
|---|---|
| Stock I²C driver hardcodes **I²C0 to GP8/GP9**, which are the left motor pins | The IMU is on Grove 3 (GP4/GP5), which is I²C0 in silicon; patch the **unit-0** pin table from GP8/GP9 to **GP4/GP5** and open `"iica"`. SDA is GP4, SCL is GP5 |
| Port defaults `BOARD_LED_PIN` to **GP16**, which is line sensor 1 | Change it to GP19 in `sysdef.h`; the Pico W's on-board LED is on the radio, not an RP2040 pin |
| Port's default console is **UART0 on GP0/GP1**, which is the left encoder | Build with `CONSOLE=usb_cdc`; the console rides the Pico's own USB port |
| Kernel's physical timer is built on the **PWM block** | Slices 4, 5, 7 are taken by motors and the servo. This tree uses RP2040 TIMER alarms instead, which the kernel does not touch |
| **HC-SR04 ECHO is 5 V**; RP2040 is not 5 V tolerant | 1 kΩ / 2 kΩ divider. Skipping this destroys the Pico |
| Robo Pico uses **two PWM pins per motor**, not PWM + direction | Handled in `drv_motor.c` |

Every pin is defined in `core/rc_config.h` and nowhere else. If you find a bare
GPIO number in a `.c` file, that is a bug.

## The IMU has no gyroscope

The part is an **LSM303DLHC**: 3-axis accelerometer plus 3-axis magnetometer,
two dice, two I²C addresses (`0x19` accel, `0x1E` mag). There is no gyro.

This changes three of Buddy 4's tasks:

- **Tilt** comes from the gravity vector, not an integrated rate.
- **Turn rate** has no direct source. Use the encoder difference. The
  magnetometer reads heading but the motors sit centimetres away and swamp it.
- **Hump height** cannot come from double-integrating acceleration; drift over
  a two-second climb exceeds the hump. `sub_terrain.c` estimates peak pitch and
  converts with the wheelbase. It is an estimate, and the report should say so.

Register traps: the accelerometer is little-endian and **left-justified**
(shift down by 4), and needs `0x80` on the sub-address for auto-increment. The
magnetometer is **big-endian** with axis order **X, Z, Y**.

## Known incomplete work

These are deliberate `TODO` markers, not oversights. Each has a comment at the
point it matters explaining what to do and why the obvious approach fails.

| Where | What |
|---|---|
| `sub_motion.c` | PID gains are placeholders. Encoder-based turning is a stub. Reverse distance needs sign handling |
| `sub_barcode.c` | Code 39 patterns for **B, C and D are placeholders**. Only `*` and `A` are real. Fill from a Code 39 reference before testing decoding |
| `sub_line.c` | Proportional-only on a four-state position estimate. Will weave. Add a derivative term, then move to analogue interpolation |
| `sub_terrain.c` | Encoder cross-check on hump height; turn classification |
| `sub_scan.c` | Side memory across obstacles; the "reverse and reattempt" case |
| `sub_telemetry.c` | UDP and MQTT sinks; connection recovery |

All mechanical constants in `rc_config.h` (wheel diameter, slot count,
wheelbase, car width) are placeholders until measured on the actual car.

## Telemetry: why the transport is behind an interface

The port's own docs list WiFi and lwIP as a **development profile** outside the
qualified release, and **no MQTT client ships** — MQTT is under "Not ported",
with transports present and anything on top left as an exercise.

So `sub_telemetry.h` puts the transport behind `sub_telemetry_sink_t`. The
framing, topic design and publish scheduling all work today over the console
sink. Recommended order: console (shipped) → UDP (best-tested network path) →
MQTT (last mile). Do not rewrite this to call an MQTT client directly.

Topics are `car/01/{state,status,barcode,hump,obstacle,impact}`.

## Building

The kernel port is a git submodule at `external/mtk3smp-rp2040` (pinned,
`ignore = dirty`). Three scripts in `build/` wrap its `make` tree:

```sh
git clone --recurse-submodules <repo>    # or: git submodule update --init
./build/setup.sh        # patch the port + install the app makefile hook
./build/build.sh        # SMP=0  -> build/out/mtk3pico_smp0_usb_cdc.uf2
./build/build.sh smp    # SMP=1  -> build/out/mtk3pico_smp1_usb_cdc.uf2
./build/flash.sh        # picotool load, Pico in BOOTSEL mode
```

How it fits together, because it is the first thing that breaks if someone
"tidies" it:

- The port compiles `$(wildcard ../app_program/*.c)` and nothing else.
  `build/robocar.mk` replaces `build_make/mtkernel_3/app_program/subdir.mk`
  and points that rule at `core/ drivers/ subsystems/ app/` three levels up
  (`RC_ROOT := ../../..`), with objects under `build_make/mtkernel_3/robocar/`.
  It also adds `-I../device/include` for `dev_i2c.h`/`dev_adc.h`, which the
  port's `INCPATH` never lists. Nothing else in the port's build is touched.
- `build/patch_port.py` applies the board patches (docs/HARDWARE.md §1):
  `BOARD_LED_PIN` 16→19; `hw_setting.c` no longer muxes GP0/GP1 to UART,
  moves I²C0 from GP8/GP9 to GP4/GP5, and stops parking GP27/GP28 as ADC;
  `i2c_rp2040.c` unit 0 on GP4/GP5; `TM_CONSOLE_UART 0`; `DEVCNF_USE_SER 0`.
  Idempotent; asserts exactly one match per patch so a port update fails
  loudly instead of silently half-applying.
- `build.sh` always passes `CONSOLE=usb_cdc` and `E2U=`. The latter empties
  the port's elf2uf2 variable so no host `g++` is needed; `picotool uf2
  convert` produces the image. Toolchain, SDK and picotool are auto-detected
  under `~/.pico-sdk` (what the VS Code Pico extension installs); only GNU
  `make` has to be added (`winget install ezwinports.make`).
- The kernel's `usermain()` is `WEAK_FUNC`, so `app/app_main.c` overrides it
  with no link tricks. The port's `app_program/` demo is simply not compiled.
- `./build/build.sh bench=<motion|line|follow|barcode|imu|ultra|scan|telemetry>`
  passes `RC_CFLAGS=-DRC_BENCH=RC_BENCH_<NAME>` to our objects only. `usermain`
  then calls `rc_bench_run()` (`app/app_bench.c`, one plain function per
  buddy) instead of `sub_nav_start()`. Images are suffixed `_bench-<name>`;
  the mission image contains none of the bench code. build.sh deletes
  `app/*.o` before every build because the port's stale-object guard does
  not key on this define.

Console is the Pico's own USB port; no USB-serial adapter exists on this car.

## SMP rules, from the port's qualification notes

1. SIO hardware spinlocks 0-2 are kernel-reserved.
2. A shared peripheral IRQ must have one owner core. `rc_gpioirq.c` enables
   IO_IRQ_BANK0 in core 0's NVIC only, so all GPIO handlers run on processor 1.
3. `volatile` is not inter-core synchronisation. Use the port's atomics.
4. UART, I²C, ADC, DMA and PWM are single-owner resources.
5. The CYW43439 radio is owned by processor 1. Its service task must be pinned
   `TP_PRC1`.
6. Task-to-processor assignment is static.

## Style

Barr-C. Fixed-width types; kernel types (`UW`, `ID`, `ER`) only at the kernel
API boundary. Braces on every `if`/`else`. Every `if`/`else if` chain ends in
an `else`; every `switch` has a `default`. No magic numbers. File-scope objects
are `static`. No floating point.

## When helping on this project

- Check `docs/HARDWARE.md` before suggesting a pin.
- Before adding an ISR, read the interrupt rule above and follow the
  record-and-flag pattern.
- Before adding a libc call, check it does not reintroduce the `size_t`
  collision.
- Prefer integer and fixed-point maths. There is no FPU, and a soft-float
  `atan2` at 100 Hz costs thousands of cycles.
- Verify with `./build/build.sh && ./build/build.sh smp`. Both must stay at zero warnings.
- The `TODO` markers are the graded work. Help reason about them rather than
  filling them in silently; if you do implement one, say clearly that you have.
