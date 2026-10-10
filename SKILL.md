---
name: inf2004-robocar
description: Context for the INF2004-IS07 autonomous robotic car - a Raspberry Pi Pico W line-following, barcode-decoding, obstacle-avoiding car on micro T-Kernel 3.0 (mtk3smp-rp2040). Use when working on any file in this repo, when asked about the robo car, RoboCar, line following, barcode decoding, hump detection, ultrasonic scanning, WiFi/UDP/MQTT telemetry, the event bus, or any rc_/drv_/sub_/net_ prefixed module. Also use when asked to build, debug, extend, or review this codebase, or when adding a new sensor, subsystem, or driver to it.
---

# INF2004-IS07 Autonomous Robotic Car

Read this before changing anything in this repo. Several of the decisions here
look wrong until you know the constraint that forced them, and the most
expensive mistakes on this project are the ones that look like cleanups.

*Last verified against commit `2c04bc7` (both images rebuilt, sizes and ISR
costs re-measured). The architecture sections age well; the status tables do
not - run `git log` and re-check "Where the project is now" first.*

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
| Toolchain | `arm-none-eabi-gcc` 15.2.1 (Arm GNU Toolchain 15.2.Rel1) |
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
subsystems/   one module per team member, plus the mission state machine and
              the network transports (net_wifi.*, sub_telemetry_udp.c,
              sub_telemetry_mqtt.c - all behind RC_NET_ENABLE)
app/          usermain, housekeeping tasks, app_bench.c (per-buddy bench modes)
docs/         HARDWARE.md (wiring/setup reference) and Robo_Pico.pdf (shared board datasheet)
              buddyN-<name>/ - one folder per buddy: the guide .md plus that
                buddy's own datasheets (buddy3 = IR/LM393/TCRT5000 + barcode spec,
                buddy4 = LSM303DLHC/OKY3231-2, buddy5 = HC-SR04, buddy1 = Pico W
                + MQTT.md, the WiFi/broker runbook)
              img/hw/ - part photos, pin map diagrams + the scripts that generate them
              img/week6/ - the Week 6 report figures (d2 pin layout ... d6 sequences)
              report/ - the Week 6 template. The submitted design review .docx
                was removed from the repo in 2c04bc7; see "The Week 6 report"
              Project Write-up.pdf - the assignment brief, kept at the docs root
              PicoCar.pptx - module slides
build/        setup.sh / build.sh / flash.sh, robocar.mk (the makefile hook),
              patch_port.py (board patches); build/out/ holds the .uf2 images
external/     mtk3smp-rp2040 kernel port as a pinned git submodule
TEAM_GUIDE.md - shared guide: big picture, shared code (app_main, sub_nav,
               core/ toolbox, bench harness), shared hardware, pin map, ownership
BUILD.md      - install -> clone -> setup -> build -> flash -> test, every step,
               VS Code Pico extension first; §5 = one bench per buddy + mission
.vscode/      tasks.json (build/flash as VS Code tasks via Git Bash),
               settings.json (Git Bash as the workspace terminal),
               c_cpp_properties.json (IntelliSense include paths)
```

Three generated diagrams live in `docs/img/hw/`. Edit the scripts, never
the PNGs, and re-run them:

| Script | Output | Review copy | What it shows |
|---|---|---|---|
| `gen_pin_map.py` | `robopico_pin_map.png` | `d2_pin_layout.png` | The Robo Pico drawn to scale (terminals/servo top, Grove 1 left, Grove 7 right, Grove 2-6 bottom, headers + socket centre) with poster-style chips fanning out from each connector: GPIO, function in use, device |
| `gen_board_view.py` | `robopico_board_view.png` | `d2b_board_view.png` | The same wiring drawn on the physical board, for finding the right socket |
| `gen_hcsr04_divider.py` | `hcsr04_divider.png` | - | The HC-SR04 Echo 5 V->3.3 V divider: schematic (1 k / 2 k, junction to GP3) plus a mini-breadboard layout with the Grove cable colours |

Regenerate with `python docs/img/hw/<script>.py` - each writes its PNG and,
for the two pin maps, a second copy under `docs/img/week6/` for the report.
Two Windows traps: the save fails with `OSError: [Errno 22]` if the PNG is
open in a viewer (close it and re-run), and a Pillow script that prints
non-ASCII needs `PYTHONIOENCODING=utf-8`.

Board geometry and port assignments came from the Cytron Robo Pico datasheet
in `docs/Robo_Pico.pdf` (its text is glyph-encoded, so the board images
have to be extracted and read); signal assignments come from
`core/rc_config.h` and section 2 of the Week 6 design review (`docs/report/`). Facts worth
keeping: the Robo Pico has **seven** Grove ports (1 on the left edge, 7 on
the right edge, 2-6 along the bottom); the silkscreen carries **GP26 on both
Grove 5 and Grove 6**, so line sensor 2 on Grove 5 must have only its DO wire
connected; the MAKER port shares GP2/GP3 with Grove 2 (the ultrasonic) and
must stay empty (it is drawn on the board but deliberately has no mapping
chip in the diagram); and the two blue parts at the bottom corners are WS2812 RGB
LEDs on GP18, not buttons (the buttons are GP20/GP21).

Current wiring, one Grove cable per device: Grove 1 = left encoder A/B
(GP0/GP1), Grove 2 = HC-SR04 TRIG/ECHO (GP2/GP3), Grove 3 = IMU on I²C0
(SDA GP4 / SCL GP5), Grove 4 = line sensor 1 DO (GP16), Grove 5 = line
sensor 2 DO (GP6), Grove 6 = barcode AO/DO (GP26/GP27), Grove 7 = right
encoder A/B (GP7/GP28). Right motor on M1 (GP8/GP9), left motor on M2 (GP10/GP11), scan
servo on servo port 4 (GP15), status LED on GP19. Buzzer, buttons and UART
are unused.

Onboarding is split three ways for team members with no prior C or
embedded background, and the split is deliberate - keep it when editing:

- `TEAM_GUIDE.md` = what is *shared*: §1 big picture (folders, event bus,
  non-blocking rule), §2 shared code line by line (`app_main.c`, `sub_nav.c`,
  `core/` toolbox, bench harness), §3 shared hardware (kit, pin map, status
  LED, ownership table), §4 links to the buddy guides, §5 team rules.
- `docs/buddyN-*/buddyN-*.md` = what is *individual*: that buddy's code
  function by function, their hardware with a "Your hardware - wiring, pin
  by pin" section (which sensor pin goes to which Grove/terminal label), a
  pointer to their bench in `BUILD.md`, and TODOs. `§N.x` references in
  those files point at `TEAM_GUIDE.md`.
- `BUILD.md` = *how to run it*: install (VS Code + Pico extension + Git for
  Windows, no make/WSL), clone, setup task, build/flash tasks, Serial
  Monitor, then §5 one subsection per bench with good/bad output, and the
  full mission; §6 terminal equivalents; §7 troubleshooting.

Every subsystem and driver source file also carries matching inline
comments aimed at the same audience. Point someone at these before
re-explaining project basics from scratch.

Build commands are run from the repo root in **Git Bash**, never WSL (the
Pico tools live on the Windows side; `build.sh` detects and refuses WSL).
On Windows `build.sh` fetches a portable GNU make into `build/tools/` if
none is on PATH. `.vscode/tasks.json` exposes setup/build/bench/flash as
tasks that always run through Git Bash.

| Module | Owner | Responsibility |
|---|---|---|
| `sub_telemetry.*` | Buddy 1 | Framing, topics, publish schedule, pluggable transport |
| `sub_motion.*` | Buddy 2 | PID, odometry, the queued-move API |
| `sub_line.*`, `sub_barcode.*` | Buddy 3 | Line following, Code 39 decoding |
| `sub_terrain.*` | Buddy 4 | Hump detection, peak estimate, motion classification |
| `sub_scan.*` | Buddy 5 | Coarse/fine scan, obstacle profiling, avoidance planning |
| `sub_nav.*` | shared | Mission state machine |

## Where the project is now

**Check `git log` before trusting anything below.** The buddies work on their
own branches (`buddy1/wifi-telemetry`, `buddy2/motion-control-system`, ...)
and merge to `main` through pull requests, so a lot can land between sessions.
State as of **2c04bc7**:

| Area | State |
|---|---|
| Event bus, drivers, bench harness, build scripts | Done, stable since b51f449 |
| **Buddy 1** telemetry | **Done.** Console + UDP + MQTT sinks, reconnect backoff, inbound command path (`car/01/cmd` -> `sub_nav_inject_command`). The radio sits behind `RC_NET_ENABLE`, off by default |
| **Buddy 2** motion | **Done.** Real turning, reverse, feedforward + PID per wheel, trapezoid speed profile, straight-line sync, stall and timeout safety stops, a move queue. The MODE_TURN stub is gone |
| **Buddy 3** line + barcode | Decoder and follower work; proportional-only steering, Code 39 table covers `*`, A-D, Z |
| **Buddy 4** IMU + terrain | Hump FSM and classification in place; thresholds unvalidated |
| **Buddy 5** scan + avoidance | Sweep, profile and plan in place; bypass leg 3 is a fixed turn |
| Hardware | Buddy 1 exercised the `cyw43_arch` + lwIP MQTT sequence on a **real Pico W** with a standalone Pico-SDK harness (same calls as `net_wifi.c`). The **car firmware itself has still not been run on the car** |

## Build status

Both configurations build clean and link to a flashable UF2:

```
SMP=0 usb_cdc   66,780 B text   21,188 B bss    0 warnings
SMP=1 usb_cdc   77,184 B text   25,296 B bss    0 warnings
```

(arm-none-eabi-gcc 15.2.1, Pico SDK 2.3.1 for TinyUSB, via `build/build.sh`,
`RC_NET_ENABLE=0`.) Start on `SMP=0`. The ~4 kB growth over the b51f449
figures is buddy 1's transports and buddy 2's motion rewrite, not bloat:
`sub_motion.o` went 796 -> 3,057 B text when the turn/profile/stall logic
replaced the stub.

Under the strict-warning flags (see **Style**) two `-Wsign-conversion`
warnings remain, both the same shape - a `tk_dly_tsk()` argument cast to
`INT`/`TMO` where the API takes `RELTIM`:
`subsystems/sub_telemetry.c:494` and `app/app_bench.c:104`. Cast to
`(RELTIM)` when you touch those lines. Everything else is clean, and the
80-column check reports zero.

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

Re-measured on the SMP=0 image at 2c04bc7 (`arm-none-eabi-objdump -d`,
instructions between the symbol and the next blank line):

| Handler | Instructions | ~Time @125 MHz |
|---|---|---|
| `encoder_isr` | 25 | 0.3 µs |
| `timeout_handler` | 34 | 0.4 µs |
| `trig_done_handler` | 34 | 0.4 µs |
| `barcode_isr` | 38 | 0.4 µs |
| `echo_isr` | 49 | 0.5 µs |
| `gpio_bank0_handler` | 117 | 1.3 µs |

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
| Stock I²C driver hardcodes **I²C0 to GP8/GP9**, which are the right motor (MOTOR 1) pins | The IMU is on Grove 3 (GP4/GP5), which is I²C0 in silicon; patch the **unit-0** pin table from GP8/GP9 to **GP4/GP5** and open `"iica"`. SDA is GP4, SCL is GP5 |
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
| `core/rc_config.h` | `RC_ENC_TICKS_PER_REV` is still the placeholder 20. Every distance, speed and turn scales off it, so it is the **first** thing to measure on the car. `RC_WHEEL_DIAM_MM` is already measured (64 mm) |
| `sub_barcode.c` | Table carries the standard Code 39 patterns for `*`, A-D and Z (three bits set each, no entry is another's mirror). Add letters only if the track needs them |
| `sub_line.c` | Proportional-only on a four-state position estimate. Will weave. Add a derivative term, then move to analogue interpolation |
| `sub_terrain.c` | Encoder cross-check on hump height; turn classification |
| `sub_scan.c` | Side memory across obstacles; the "reverse and reattempt" case |
| `sub_scan.c` + `sub_nav.c` | Bypass leg 3 is a fixed turn, not a closed-loop return to the line |

`sub_motion.c` and `sub_telemetry.c` used to be on this list and are not any
more - do not reintroduce "the turn is a stub" or "there is no MQTT" into any
document. The remaining mechanical constants in `rc_config.h` (wheelbase, car
width, encoder ticks per rev) are placeholders until measured on the car.

## Motion: what the rewrite added (do not simplify it back)

`sub_motion.c` is now the largest module in the tree (~1,000 lines, 3 kB of
flash) and every part of it is load-bearing:

- **Five modes**, not four: `MODE_SPEED` (closed-loop mm/s + steer, via
  `sub_motion_drive_speed()`) joins IDLE / DISTANCE / TURN / CONTINUOUS.
  `MODE_CONTINUOUS` stays open-loop because the line follower wants a duty,
  not a speed.
- **Feedforward + PID per wheel.** The feedforward term is a straight-line
  guess at the duty a speed needs (`FF_START_PERMILLE`, `FF_TOP_SPEED_MM_S`);
  the PID only trims it. Deleting the feedforward makes the integrator do all
  the work and the car lurches on every start.
- **Trapezoid speed profile** (`profile_speed`): accelerate from
  `MIN_SPEED_MM_S`, cruise, decelerate inside `DECEL_ZONE_MM`. This is why a
  queued move stops on the mark instead of skidding past it.
- **Straight-line sync** (`sync_correction`): a tick-difference term that
  steers the slower wheel up. Distinct from the per-wheel PID, which only
  knows its own wheel.
- **Turns** are a spin in place to an encoder-tick goal derived from
  `RC_WHEEL_BASE_MM`, with `TURN_SLIP_PERMILLE` as the measured fudge factor
  for wheel slip (still 1000 = no correction until someone measures it).
- **Safety**: `stall_check()` aborts a move when a wheel is pushed at
  `STALL_DUTY_PERMILLE` or more and reports no ticks for `STALL_CYCLES`, and
  every queued move carries a timeout. Both end as `MOTION_DONE(aborted)`,
  which is the contract callers already handle.
- **64-bit intermediates** (`um_to_ticks` takes a `uint64_t`): distance in
  micrometres times a tick count overflows 32 bits on a long move. 64-bit
  integer maths is fine on an M0+ (libgcc); 64-bit *float* is not.

## Telemetry: three sinks behind one interface

The port's own docs list WiFi and lwIP as a **development profile** outside
the qualified release, and **no MQTT client ships**. So `sub_telemetry.h`
keeps the transport behind `sub_telemetry_sink_t` and buddy 1 implemented
three of them: console (`sub_telemetry.c`), UDP (`sub_telemetry_udp.c`) and
MQTT (`sub_telemetry_mqtt.c`). Framing, topics and scheduling are shared, so
swapping transport is one `sub_telemetry_set_sink()` call in `app_main.c`.

**`RC_NET_ENABLE` (rc_config.h) is the master switch and defaults to 0.**
With it off, `sub_telemetry_udp_sink()` and `sub_telemetry_mqtt_sink()`
return `NULL`, `set_sink(NULL)` keeps the console, and no lwIP header enters
the build. The graded mission image is the radio-off one; console telemetry
over USB is the reliable demo baseline.

**Turning it on needs a one-time change to the port, which is not committed.**
`-DRC_NET_ENABLE=1` on its own fails at `pico/cyw43_arch.h: No such file`
(verified). `docs/buddy1-telemetry/MQTT.md` has the procedure: link
`pico_cyw43_arch_lwip_threadsafe_background`, enable the lwIP MQTT app, and
supply an `lwipopts.h` with **`MEMP_NUM_SYS_TIMEOUT` >= 16** - DHCP, DNS, TCP
and MQTT each take a timer slot and lwIP panics right after the MQTT connect
with the default pool. Two hardware-found facts live in that file too:
`cyw43_tcpip_link_status` takes `(&cyw43_state, CYW43_ITF_STA)`, and the
Pico W is **2.4 GHz only** (a 5 GHz-only hotspot fails the join with `rc=-2`).

Topics are `car/01/{state,status,barcode,hump,obstacle,impact}` published, and
`car/01/cmd` subscribed. An inbound command reaches the state machine through
`sub_telemetry_deliver_command()` -> `sub_nav_inject_command()`, which just
publishes `RC_EVT_COMMAND_RX` - the radio never calls a subsystem directly.

The port's own docs list WiFi and lwIP as a **development profile** outside the
qualified release, and **no MQTT client ships** — MQTT is under "Not ported",
with transports present and anything on top left as an exercise.

So `sub_telemetry.h` puts the transport behind `sub_telemetry_sink_t`. The
framing, topic design and publish scheduling all work today over the console
sink. Recommended order: console (shipped) → UDP (best-tested network path) →
MQTT (last mile). Do not rewrite this to call an MQTT client directly.

Topics are `car/01/{state,status,barcode,hump,obstacle,impact}`.

## The Week 6 report

The submitted design review `.docx` was **removed from the repo** in 2c04bc7;
`docs/report/` now holds only the blank template. If a future report has to be
edited, the method that worked (recorded because none of it is obvious):

- Edit the XML, do not re-generate the document: `unzip` the `.docx`, run the
  docx skill's `merge_runs.py` (Word splits a visible sentence across many
  `<w:r>` runs, so the phrase you can see often is not a contiguous string),
  patch `word/document.xml`, rezip with Python's `zipfile` preserving entry
  order, then `validate.py --original <old>` - it reports the paragraph count
  either side, which catches a mangled edit immediately.
- Figures were `word/media/image9.png` (Figure 2a, the pin map,
  relationship `rId24`) and `image10.png` (Figure 2b, the board view,
  `rId25`). Re-embedding is a straight byte replacement of those entries.
- Word holds a lock while the file is open; a `~$`-prefixed file beside it
  means "close Word first" (`.gitignore` already excludes those).
- Run the validator with `PYTHONIOENCODING=utf-8` on Windows, or it dies
  printing its own arrow character.

The report's gap table (G1-G10) was written to track the same work as the
`TODO` markers in the source. If the report comes back, G1 (turning) and G2
(Code 39 table) are now done, and G8 (80-column) is enforced by the build.

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
  under `~/.pico-sdk` (what the VS Code Pico extension installs). GNU `make`
  is the only missing piece, and on Windows `build.sh` downloads a portable
  copy into `build/tools/` by itself the first time; `winget install
  ezwinports.make` is the fallback if that download is blocked.
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
are `static`. No floating point. **80 columns**: `build/build.sh` prints every
over-long line in `core/ drivers/ subsystems/ app/` as a warning; keep it at
zero (a long trailing comment goes on its own lines above the code).

Strict-warning audit (done once, keep it clean): run make directly from
`external/mtk3smp-rp2040/build_make` with
`RC_CFLAGS="-DRC_BENCH=RC_BENCH_NONE -Wall -Wextra -Wshadow -Wconversion
-Wundef -Wcast-qual -Wdouble-promotion"` - `build.sh` overwrites `RC_CFLAGS`,
so it cannot be passed through that script. Our objects compile with zero
warnings under those flags; the remaining noise is the port's own headers
(`TK_SUPPORT_SERCD`, `CNF_SMP` undefined, K&R prototypes in `typedef.h`).

Things fixed in the audit that are easy to reintroduce: `rc_pwm_set_pulse_us`
scales per millisecond (per-microsecond integer ticks truncate 3.205 to 3 and
shorten every servo pulse by 6 %); every `T_CFLG` needs `iflgptn = 0`; PID
output is clamped in 32-bit before the `int16_t` cast.

## How the team works, and what they expect

Standing agreements. These are preferences the team has stated, not guesses:

- **Keep this file current.** Whenever anything in the repo changes - code,
  pins, diagrams, document structure - update `SKILL.md` in the same batch of
  work. It is the project's context file and a stale one is worse than none.
- **Every document assumes Windows + VS Code with the Raspberry Pi Pico
  extension**, driving the `build/*.sh` scripts through the `.vscode` tasks
  or Git Bash. **Never route a reader through WSL**; if some step genuinely
  cannot be done any other way, say so explicitly at that step and explain
  why. (`build.sh` detects WSL and refuses, on purpose.)
- **Write for someone with no C, no embedded and no Pico background.** Spell
  out every step, explain what each piece of code is for and how it links to
  the rest, and prefer a worked example over a summary. That is why the
  guides are as long as they are - length is not the problem, a skipped step
  is.
- **Diagrams must look like the thing they describe.** The pin map is drawn
  as the actual Robo Pico board with poster-style chips fanning out from the
  real connectors; an earlier table-shaped version was rejected. Keep that
  style, and keep unused parts of the board out of the mapping (the MAKER
  port is drawn but deliberately carries no chip).
- **Finish by committing and pushing** to `main` on
  `Dysteriaaaa/INF2004-IS07-AutonomousRoboCar`, unless told otherwise.
  Buddies work on `buddyN/<topic>` branches and merge through pull requests;
  pull before starting so you are not editing three-week-old files.

Two mechanical traps that cost real time in past sessions:

- **Do not write multi-line patch scripts in a bash heredoc.** Backslash
  escapes and non-ASCII characters get mangled on the way through. Write the
  script to a file (in the scratchpad) with the file-writing tool and run it.
- **Never `git checkout -- .` inside `external/mtk3smp-rp2040`.** It reverts
  the makefile hook that `build/setup.sh` installed and the next build quietly
  compiles the port's demo instead of this car. Re-run `./build/setup.sh`.

## When helping on this project

- **Read `git log` first.** Several people commit to this repo between
  sessions; assume the tree has moved since whatever you last remember.
- Check `docs/HARDWARE.md` before suggesting a pin.
- Before adding an ISR, read the interrupt rule above and follow the
  record-and-flag pattern.
- Before adding a libc call, check it does not reintroduce the `size_t`
  collision.
- Prefer integer and fixed-point maths. There is no FPU, and a soft-float
  `atan2` at 100 Hz costs thousands of cycles.
- Verify with `./build/build.sh && ./build/build.sh smp`. Both must stay at
  zero warnings, and the 80-column check must stay silent.
- The `TODO` markers are the graded work. Help reason about them rather than
  filling them in silently; if you do implement one, say clearly that you have.
- Keep the three-document split (`TEAM_GUIDE.md` / `docs/buddyN-*/` /
  `BUILD.md`) intact, and update all three when a change touches shared
  behaviour.
