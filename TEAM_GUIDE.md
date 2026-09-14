# Team Guide — Autonomous RoboCar

Welcome. This guide exists so that **anyone on the team — even with zero
embedded-systems or C background — can open it and know exactly what to do
next.** It is organized by role ("Buddy 1" through "Buddy 5"), matching the
ownership table in the [README](README.md) and [SKILL.md](SKILL.md). Read
your own section fully before touching code. Skim the "Shared Foundations"
section too — it explains the plumbing every module sits on top of, and you
will bump into it no matter which piece you own.

If anything here disagrees with `docs/HARDWARE.md`, trust `docs/HARDWARE.md`
— it is the most detailed and most frequently updated source on wiring and
calibration.

---

## 0. The big picture (read this first, everyone)

The car is built from small, focused pieces of code that never call each
other directly. Instead, every piece **shouts into a shared announcement
system** (called the "event bus") when something happens — "I just read the
IR sensor," "I just decoded a barcode," "the wheel just finished this many
millimetres." Any other piece that cares can **subscribe** to that kind of
announcement and react. This is exactly like a group chat where people only
post updates relevant to their role, and everyone else mutes the channels
they don't need — nobody has to personally phone every other person on the
team every time something happens.

Why build it this way? Because it lets five people write and test their own
module against fake/stub data, without waiting for everyone else's code to
exist, and then plug all five together at the end and have it mostly just
work.

### 0.1 Folders — where does my code live?

```
core/         the plumbing everyone shares (event bus, timing, interrupts, PWM)
drivers/      one file per physical part (motor, encoder, servo, ultrasonic sensor, IR sensor, IMU)
subsystems/   one file per team member's "job", plus the shared mission logic
app/          the startup code that wires everything together and boots the car
docs/         HARDWARE.md — wiring, pin numbers, and calibration steps
```

A **driver** talks directly to one physical part (e.g. "read the raw motor
encoder"). A **subsystem** is the smart layer on top that makes decisions
(e.g. "drive forward exactly 300mm and tell me when you're done"). You will
mostly work in `subsystems/`, and read (but rarely edit) the matching
`drivers/` file underneath it.

### 0.2 The event bus, in plain terms

File: `core/rc_event.h`. Two ideas to remember:

- **Publish / subscribe.** A "producer" (an interrupt, a driver, a
  subsystem) calls `rc_event_publish(...)` with a small packet of data (an
  `rc_event_t`) tagged with an ID like `RC_EVT_ODOMETRY` or
  `RC_EVT_BARCODE_DECODED`. Any code that previously called
  `rc_event_subscribe(id, lane, my_function, ctx)` for that same ID gets
  `my_function` called automatically with the data. You never call another
  subsystem's function by name — you just publish or subscribe.
- **Two lanes, like two mail queues with different priority.**
  `RC_LANE_FAST` is for things that must react quickly — steering,
  obstacle response. `RC_LANE_SLOW` is for things that can wait a beat —
  telemetry, logging, planning. This exists so that, for example, a slow
  network send can never delay the wheels reacting to an obstacle. When you
  subscribe to something, ask yourself: "does the car steer differently
  because of this, right now?" If yes, fast lane. If no, slow lane.

**Golden rule for any code you write that runs inside an interrupt** (an
"ISR" — a tiny piece of code the chip automatically jumps to the instant a
sensor pin changes state, pausing everything else): it may only record a
timestamp, flip a couple of registers, and hand off to a task via
`rc_defer_signal_i()`. It may **not** loop, publish an event directly, call
a callback, or do division. If you're ever tempted to add a
`while(...)`-style wait anywhere in a driver, stop — see §0.3.

### 0.3 "Non-blocking" — the one rule that shapes everything

Nothing in this codebase is allowed to freeze the whole car while it waits
for something slow to finish (like a 30-millisecond ultrasonic echo, or a
motor move that takes two seconds). Instead:

1. You call a function like `sub_motion_forward_mm(300, my_callback, ctx)`.
2. It returns **immediately** — the car starts moving in the background.
3. Later, when the move actually finishes, `my_callback` gets called
   automatically (via the event bus) to tell you it's done.

This is the same pattern as clicking "download" in a browser: you don't
freeze the whole browser while the file downloads, you get a notification
when it's done. Every module in this project follows this pattern — get
used to writing "start it and register a callback" instead of "wait for
it."

### 0.4 The mission state machine (shared code, `sub_nav.c`)

This file belongs to everyone jointly — it is the "traffic controller" that
decides which mode the car is in (following the line, reading a barcode,
executing a turn command, avoiding an obstacle, recovering after avoidance,
or stopped) and turns the right subsystems on/off for each mode. You
probably won't need to edit it, but understanding it helps you see how your
own module fits into the whole mission:

- `RC_NAV_FOLLOWING` — driving normally, line follower active, obstacle
  watch active.
- `RC_NAV_READING_BARCODE` — the line follower lost the line or saw a
  junction; the barcode decoder is now armed and listening.
- `RC_NAV_EXECUTING_CMD` — a barcode was decoded; the car is turning
  (left/right/U-turn) or continuing straight.
- `RC_NAV_AVOIDING` — an obstacle was seen close up; a full ultrasonic
  scan is running and a bypass path is being planned.
- `RC_NAV_RECOVERING` — the car has driven around an obstacle and is
  hunting for the line again.
- `RC_NAV_STOPPED` / `RC_NAV_IDLE` — everything off.

Every buddy's module publishes events (line lost, barcode decoded,
ultrasonic result, scan plan ready) that `sub_nav.c` listens for to decide
when to change mode. If your module isn't reacting the way you expect,
check here first — it's often a case of the nav state machine not being in
the mode you assumed.

### 0.5 Getting the project running for the first time (everyone)

1. Read `docs/HARDWARE.md` §1 — it lists four pin/hardware conflicts that
   *will* cost you a day each if you don't know about them up front (I²C
   pin collision with the left motor, the status LED pin collision, the
   kernel timer/PWM collision, and the 5V ultrasonic echo pin needing a
   voltage divider or it will destroy the board).
2. Follow the "Building" steps in `README.md` / `docs/HARDWARE.md` §5 to
   clone the RTOS port, apply the two required patches, and get a first
   `make -j8` build (`SMP=0`, single-core, the simplest configuration) to
   succeed with zero warnings.
3. Flash it (hold BOOTSEL, copy the `.uf2` file to the Pico's drive) and
   connect a USB-serial adapter to see console output at 115200 baud.
4. Work through the **bring-up order** in `docs/HARDWARE.md` §7 — it is a
   numbered checklist (blink LED → console print → motors → encoders →
   closed-loop speed → line sensors → line following → servo → ultrasonic
   → full scan → IMU → hump → barcode → telemetry → full mission). Do not
   skip ahead; each step is designed to isolate one thing at a time so a
   failure tells you exactly where to look.

---

## 1. Roles at a glance

| Buddy | Files you own | What you're building |
|---|---|---|
| 1 | `subsystems/sub_telemetry.*` | Reporting the car's status over WiFi (and receiving commands, eventually) |
| 2 | `subsystems/sub_motion.*`, `drivers/drv_motor.*`, `drivers/drv_encoder.*` | Making the wheels move the right speed/distance/angle |
| 3 | `subsystems/sub_line.*`, `subsystems/sub_barcode.*`, `drivers/drv_ir.*` | Staying on the black line, reading barcodes |
| 4 | `subsystems/sub_terrain.*`, `drivers/drv_imu.*` | Detecting speed humps and classifying how the car is moving |
| 5 | `subsystems/sub_scan.*`, `drivers/drv_ultrasonic.*`, `drivers/drv_servo.*` | Scanning for obstacles and planning a way around them |

Now jump to your section.

---

### Buddy 1 — WiFi, Command & Telemetry

**What this module does**

This module is the robot's "reporting and remote-control desk." While other
parts of the code drive motors and read sensors, this one packages up
status updates (speed, distance, line-sensor readings, detected barcodes,
bumps in the road) into small text messages and sends them out so a human
or a laptop can watch what the robot is doing in real time. It also has a
slot ready for the reverse direction — receiving driving commands from
outside — though that path isn't wired up yet. Today the messages go to the
on-screen debug console; the design lets that be swapped for WiFi later
without rewriting anything else.

**How to get started**

1. Open `subsystems/sub_telemetry.c` and `subsystems/sub_telemetry.h` side
   by side — the header is the "public menu" of what this module offers,
   the `.c` file is the implementation.
2. Build and flash the project as-is first. `sub_telemetry_init()` is
   already wired into `usermain` — watch the debug console and you should
   see lines like `[telem] car/01/state {...}` appearing repeatedly. That's
   the console sink working; it's your baseline before touching anything.
3. Read `send()`, `publish_state()`, and `publish_heartbeat()` in the `.c`
   file to see how a message goes from "some numbers" to "a line of JSON on
   a topic."
4. Look at the `TODO Buddy 1` comments — one in `sub_telemetry.h` and one
   inside `telemetry_task()` in the `.c` file. Those are your two real
   jobs: build a transport that actually leaves the board (UDP, then MQTT)
   and add reconnect logic when the link drops.
5. When you build a transport, write a new `.c` file that fills in a
   `sub_telemetry_sink_t` struct (four function pointers, described below)
   and pass it to `sub_telemetry_set_sink()`. You do not need to touch the
   framing or scheduling code — that's the whole point of the sink
   interface.
6. Recommended build order (see `README.md` "Telemetry" section):
   **console (already works) → UDP → MQTT**. Don't start with MQTT — the
   RTOS port doesn't ship an MQTT client at all, so you'd have nothing to
   demo until you also built or imported one.

**Code walkthrough**

```c
typedef struct {
    const char *name;
    rc_result_t (*open)(void);
    rc_result_t (*publish)(const char *topic, const char *payload);
    void (*close)(void);
    bool (*is_up)(void);
} sub_telemetry_sink_t;
```
Think of a "sink" as a courier service. It doesn't matter to the rest of
the module whether the courier is "print it to the screen," "send it over
WiFi as UDP packets," or "hand it to an MQTT broker" (MQTT is a lightweight
publish/subscribe messaging protocol commonly used for IoT devices) — as
long as it can `open` (start the service), `publish` (hand over one
message), `close` (shut down), and report `is_up` (is the service currently
working). This is the pluggable-transport design.

```c
typedef void (*sub_telemetry_cmd_cb_t)(rc_nav_cmd_t cmd, int32_t arg, void *ctx);
```
The shape of a function that gets called when a command arrives from
outside the robot (e.g., "turn left," "stop"). It's currently just a
declared shape with a registration slot — nothing yet actually receives
commands and calls it.

```c
rc_result_t sub_telemetry_init(void);
```
The startup routine. Like hiring the reporting desk's staff: it registers
interest in five kinds of events (odometry, line-sensor readings, barcode
decodes, "hump" bumps, obstacle profiles, impacts), defaults the courier to
the console sink, and starts a background task (`telemetry_task`) that
runs forever, periodically sending updates. Called once at boot.

```c
rc_result_t sub_telemetry_set_sink(const sub_telemetry_sink_t *sink);
```
Swaps the active courier. Pass in your new UDP or MQTT sink struct and it
closes the old one, opens the new one, and from then on every message goes
through it. Pass `NULL` to fall back to the safe console sink.

```c
rc_result_t sub_telemetry_on_command(sub_telemetry_cmd_cb_t cb, void *ctx);
```
Registers the function to call when an incoming command shows up (once
someone builds the receiving side). Right now it just stores the pointer;
nothing invokes it yet.

```c
rc_result_t sub_telemetry_publish_event(const rc_event_t *evt);
```
Push a message immediately, skipping the regular schedule — used for
things that shouldn't wait, like "we just read a barcode" or "we just hit
something." It builds a small JSON label for the event and sends it on
its own topic (e.g. `car/01/barcode`).

```c
const sub_telemetry_sink_t *sub_telemetry_console_sink(void);
```
Hands back the built-in "read it aloud" courier — the one that just
prints to the debug console. Useful as the safe fallback or for testing.

Static helpers worth knowing: `send()` is the shared mailroom — it builds
the full topic string (like `car/01/state`), checks the courier is
actually up, and counts successes/failures. `publish_state()` builds the
main heartbeat-style JSON blob of live numbers (speed, distance, line
sensors, terrain class, last barcode) sent every tick. `publish_heartbeat()`
sends a less frequent "is everything healthy" summary (uptime, transmit
counts, failures, dropped events, which sink is active) roughly every 2
seconds.

**Missing / TODO for Buddy 1**

- **`sub_telemetry_udp_sink()`** — not implemented. Needs its own file
  implementing the `sub_telemetry_sink_t` interface over the RP2040 port's
  lwIP UDP support (lwIP is the lightweight TCP/IP networking stack this
  platform ships). Build this first, before MQTT.
- **`sub_telemetry_mqtt_sink()`** — not implemented. Needs an MQTT client
  to be brought in separately (the RTOS doesn't ship one), then wired into
  the same sink interface. Deliberately the last step.
- **Connection recovery** — flagged with a TODO inside `telemetry_task()`.
  Currently the task never checks whether the sink is still connected once
  opened. Needs: check `sink->is_up()` each tick, and on a drop, call
  `sink->close()`, wait with a backoff delay (using `tk_dly_tsk`, never a
  busy retry loop), then reopen.
- **Command receiving path** — `sub_telemetry_on_command()` only stores a
  callback; nothing actually listens for or decodes incoming commands over
  WiFi yet. Build this alongside the UDP/MQTT sinks.

---

### Buddy 2 — Motion Control (PID, Odometry, Movement)

**Files:** `subsystems/sub_motion.c` / `.h`, `drivers/drv_motor.c` / `.h`,
`drivers/drv_encoder.c` / `.h`

**What this module does**

This module is the car's legs and inner ear. It decides how much power to
send to the left and right wheel motors, and it listens to two little
slotted wheels (encoders) that click every time the wheel turns a fixed
amount — the same idea as counting clicks on a bike's spoke card to know
how fast and how far you've gone. That click-counting is called
**odometry**: turning "number of clicks" into "distance travelled" and
"current speed." Because motors never spin at exactly the speed you ask
for (battery sag, friction, floor grip all get in the way), this module
also runs **PID control** — think of a car's cruise control, which watches
your actual speed, compares it to the speed you want, and constantly
nudges the throttle up or down to close the gap. Every other module — line
following, obstacle avoidance — just says "go forward 300mm" or "steer
this much" and trusts this module to make the wheels do it.

**How to get started**

1. **Measure the hardware.** Measure your wheel diameter and count the
   slots on each encoder disc. Set `RC_WHEEL_DIAM_MM`,
   `RC_ENC_SLOTS_PER_REV`, and `RC_WHEEL_BASE_MM` (distance between the two
   wheels, needed for turning math) in `rc_config.h`.
2. **Sanity-check speed sensing.** Prop the car up with wheels off the
   ground, command a fixed motor duty (via `drv_motor_set`), and log
   `drv_encoder_speed_mm_s()`. Confirm the reading is stable and roughly
   proportional to duty before going further.
3. **Tune PID gains.** Open `sub_motion_init()` in `sub_motion.c` — the
   `pid_set_gains(&pid_l, 512, 64, 16)` calls are placeholder numbers, not
   tuned values. Adjust `kp`/`ki`/`kd`, run `sub_motion_forward_mm()`, and
   log the speed step response until it settles without overshooting or
   oscillating.
4. **Calibrate distance.** With wheels on the ground, run
   `sub_motion_forward_mm(1000)` ten times, measure the actual distance
   with a tape each time, and fold the average error into
   `RC_ENC_UM_PER_TICK` (micrometres of travel per encoder click).
5. **Calibrate turns.** Same repeated-trial process for
   `sub_motion_turn_deg()`. Expect the real turn to fall short of the math
   because wheels slip when spinning in place — apply a correction factor
   rather than trusting pure geometry.

**Code walkthrough**

**`drv_motor.h` / `drv_motor.c` — the "pure output" layer.** This layer
knows nothing about speed; it just turns electrical power on and off. The
motor driver board (Robo Pico) uses two PWM pins per motor instead of one
PWM pin plus a direction wire. PWM (**pulse-width modulation**) is how you
get a variable "speed" out of a digital on/off pin — it flips the pin on
and off very fast (20,000 times a second here) and varies the fraction of
time it's on, called the **duty cycle**. A higher duty cycle means more
average power, like flicking a light switch fast enough that the bulb
just looks dimmer instead of blinking.

- `drv_motor_init(void)` — sets up PWM on both motors' pin pairs and
  zeroes them.
- `drv_motor_set(rc_side_t side, int16_t duty_permille)` — commands one
  wheel's power, from -1000 to +1000 "permille" (parts per thousand, so
  ±100.0%). Positive drives forward, negative reverse. It always drops the
  "off" pin to zero before raising the "on" pin, so the motor driver chip
  is never told to do both directions at once.
- `drv_motor_set_pair(int16_t left_permille, int16_t right_permille)` —
  convenience call to set both wheels in one go.
- `drv_motor_stop(rc_motor_stop_t mode)` — either coasts (power off, wheel
  free-spins to a stop) or brakes (both pins high, which shorts the motor
  and stops it quickly).
- `drv_motor_get(rc_side_t side)` — reports the last commanded duty for a
  wheel; used to infer which way a wheel is currently being told to spin
  (see encoder direction, below).

**`drv_encoder.h` / `drv_encoder.c` — turning wheel clicks into speed and
distance.** Each wheel has a slotted disc and an optical sensor; every
time a slot passes, it fires an electrical pulse. Detecting that pulse the
instant it happens is done with an **ISR** (Interrupt Service Routine) — a
tiny piece of code the processor jumps to immediately when a pin changes
state, interrupting whatever else it was doing, so timing is precise. The
ISR here (`encoder_isr`) does the bare minimum: records a timestamp, counts
the edge, rejects nearby noise pulses ("debounce"), and hands off to a
"bottom half" (`encoder_drain`) that runs in normal task context to safely
publish the info as an event — keeping the interrupt short matters because
other things (ultrasonic sensor, barcode reader) share the same interrupt
hardware and can't be kept waiting.

- `drv_encoder_init(void)` — registers the interrupt handlers on both
  encoder pins and the deferred bottom-half worker.
- `drv_encoder_count(rc_side_t side)` — total wheel-clicks seen since
  boot, never resets.
- `drv_encoder_period_us(rc_side_t side)` — microseconds between the two
  most recent clicks; returns 0 if the wheel has been still too long (a
  "stall timeout"), so a stopped wheel doesn't look like an infinitely
  slow one.
- `drv_encoder_speed_mm_s(rc_side_t side)` — converts that period into a
  real speed in millimetres/second, using integer math only (the RP2040
  has no FPU — no hardware floating-point unit — so all math is done in
  whole numbers). It borrows direction (forward/backward) from what the
  motor was last told to do, since these single-channel encoders can't
  tell direction on their own.
- `drv_encoder_distance_mm(rc_side_t side)` — clicks since the last reset,
  converted to millimetres — the actual odometry distance count.
- `drv_encoder_reset(void)` — zeroes the distance baseline so a new move
  can measure "distance travelled during this move" from zero.

**`sub_motion.h` / `sub_motion.c` — the brain that ties it together.** The
public API is non-blocking (see §0.3): calling
`sub_motion_forward_mm(300, callback, ctx)` returns immediately with a
move ID while the car drives in the background; when the move finishes
your callback fires and an `RC_EVT_MOTION_DONE` event is published.

- `sub_motion_init(void)` — sets placeholder PID gains and starts a
  background task (`motion_task`) on a fixed timer.
- `sub_motion_set_speed(uint16_t mm_s)` — sets the target cruising speed
  used for distance moves.
- `sub_motion_forward_mm` / `sub_motion_backward_mm(uint32_t mm, cb, ctx)`
  — queue a "drive this far" move.
- `sub_motion_turn_deg(int16_t deg, cb, ctx)` — queue a "turn this many
  degrees" move.
- `sub_motion_drive(int16_t base_permille, int16_t steer_permille)` —
  continuous mode with no distance target and no completion callback; sets
  a base speed plus a steering bias (used constantly by the line follower).
- `sub_motion_stop(bool brake)` — cancels whatever's running and stops the
  motors.
- `sub_motion_busy(void)` — true while a queued move is still in progress.
- **The PID loop (`pid_step`)** — every control cycle, computes the error
  (target speed minus actual measured speed) and combines three terms:
  proportional (react to how big the error is right now), integral (react
  to how long/consistently the error has persisted, correcting steady
  drift), and derivative (react to how fast the error is changing, to
  avoid overshoot). The result becomes the new motor duty.
- **The move state machine (`motion_task` + `mode`)** — runs on a timer,
  and depending on the current mode (idle / driving a distance / turning /
  continuous drive) either runs the PID loop toward a distance goal,
  steers open-loop, or (for turns, currently) just stops immediately — see
  TODOs below. It also publishes an `RC_EVT_ODOMETRY` event every cycle
  with each wheel's speed and distance.

**Missing / TODO for Buddy 2**

- **PID gains are placeholders.** `sub_motion_init()` uses numbers never
  tuned on real hardware. Per `docs/HARDWARE.md` §4.2, tune on the bench
  (wheels off the ground first, then on the track), and record step-
  response logs as the tuning report.
- **Encoder-based turning is a stub.** In `motion_task`, `MODE_TURN`
  currently just finishes immediately — it does not actually drive the
  wheels or measure the turn. Needs: compute arc length per wheel
  (`RC_WHEEL_BASE_MM * pi * deg / 360`), convert to encoder ticks via
  `RC_ENC_UM_PER_TICK`, drive the wheels in opposite directions, stop when
  both sides hit that tick count, and apply a measured slip-correction
  factor (average error over ~10 test turns) rather than raw geometry.
- **Reverse/backward moves don't handle sign correctly.** There's no
  negative/reverse sign handling built into distance moves yet, so
  backward moves won't behave correctly until implemented.
- **Distance/turn calibration constants need real measurements.**
  `RC_WHEEL_DIAM_MM`, `RC_ENC_SLOTS_PER_REV`, `RC_WHEEL_BASE_MM`, and
  `RC_ENC_UM_PER_TICK` in `rc_config.h` all need to be set from actual
  measurements and refined against tape-measure trials.

---

### Buddy 3 — Line Following & Barcode Decoding

**Files:** `subsystems/sub_line.c/.h`, `subsystems/sub_barcode.c/.h`,
`drivers/drv_ir.c/.h`

**What this module does**

The car has two small infrared sensors pointed at the ground, one slightly
left of centre and one slightly right. Black line absorbs infrared light,
white track reflects it, so each sensor can tell the software whether it's
currently sitting over the black line or not. By comparing "is the left
one on the line?" against "is the right one on the line?", the code can
tell whether the car has drifted off-centre and steer it back — this is
line following. Separately, a third sensor reads a printed barcode on the
track: as the wheels roll over alternating black bars and white gaps, the
sensor sees the light flicker on and off, and the code measures how long
each flicker lasts. In the Code 39 barcode standard used here, it's not
the absolute timing that matters (that changes with speed) but the
*ratio* between "short" and "long" flickers, and that ratio pattern spells
out a letter, which the car then treats as a driving instruction (turn
left, turn right, U-turn, or go straight).

**How to get started**

1. **Wire it up per `docs/HARDWARE.md` §4.3**: line-left DOUT → GP6,
   line-right DOUT → GP7 (both digital, polled), barcode DOUT → GP27
   (digital, interrupt-driven), barcode AOUT → GP26/ADC0.
2. **Check sensor polarity before anything else.** `IR_ACTIVE_HIGH` at the
   top of `drv_ir.c` says HIGH-on-the-pin means "over the line." On the
   Waveshare ST188 board this is correct, but if you're using TCRT5000
   modules or wired something differently, it may be backwards — and the
   car will confidently drive itself off the table. Test with
   `drv_ir_on_line()` over white then black to confirm; flip the macro if
   it's wrong.
3. **Calibrate the trim pots** (HARDWARE.md §4.3): read raw AOUT values
   over white and over black with `drv_ir_read_raw(RC_IR_BARCODE)`, set
   the trim pot so DOUT flips roughly midway between them, and redo this
   under your actual demo lighting (fluorescent vs sunlight differ a lot
   in IR).
4. **Bring-up order** (HARDWARE.md §7, steps 6–7 and 13): confirm DOUT
   flips cleanly crossing the line and AOUT reads clearly different; test
   line following on a straight then a curve; barcode decoding comes
   later, since a barcode read is armed off the line follower's junction
   detection.
5. Call `drv_ir_init()`, `sub_line_init()`, `sub_barcode_init()` at
   startup. Line following starts disabled — call `sub_line_enable(true)`
   when you want it steering.
6. **Fix the barcode table before testing decode** — see TODOs below;
   decoding cannot be correct for 'B', 'C', 'D' until this is done.

**Code walkthrough**

**`drv_ir.h` — the hardware layer**

```c
rc_result_t drv_ir_init(void);
```
Sets up the two line-sensor pins as digital inputs, attaches an interrupt
handler to the barcode pin (an ISR — a small function the chip jumps to
automatically the instant a pin's voltage changes), and opens the ADC
device (analogue-to-digital converter — turns a sensor's raw voltage into
a number). The barcode interrupt starts disabled until armed.

```c
bool drv_ir_on_line(rc_ir_ch_t ch);
```
Returns true/false for whether a given sensor currently sees black, after
correcting for `IR_ACTIVE_HIGH` polarity.

```c
uint16_t drv_ir_read_raw(rc_ir_ch_t ch);
```
Returns the raw brightness reading (0–4095) from the ADC, but only for the
barcode channel — the two line sensors aren't wired to an ADC pin in this
design, so calling it for them returns 0. This is what you use for
trim-pot calibration.

```c
void drv_ir_sample_line(void);
```
Called every 5ms to read both line sensors and turn their two true/false
readings into a rough sideways "position" number, published as an event
for `sub_line.c`. With only two digital sensors there are only four
possible combinations: both on line = centred (also what a junction/
barcode start looks like); only left on = drifted right (steer left);
only right on = drifted left (steer right); neither = "lost," left for
`sub_line.c` to handle.

```c
rc_result_t drv_ir_on_barcode_edge(drv_ir_edge_cb_t cb, void *ctx);
rc_result_t drv_ir_barcode_enable(bool on);
uint32_t    drv_ir_barcode_overruns(void);
```
The barcode path is interrupt-driven rather than polled, because a bar's
*duration* is the actual data and any delay corrupts the measurement.
Every time the sensor's output flips, the ISR stamps the exact microsecond
and stores the elapsed width into a small ring buffer (a fixed-size queue
that wraps back to the start when full) so the actual decoding — too slow
for an interrupt — happens later in normal program flow ("deferred"
processing, here `barcode_drain()`). `drv_ir_barcode_enable(on)` keeps
this off unless a barcode is actually expected. `drv_ir_barcode_overruns()`
reports edges dropped because the ring filled before the decoder kept up
— should stay at zero.

**`sub_line.h` — steering decisions**

```c
rc_result_t sub_line_init(void);
rc_result_t sub_line_enable(bool on);
rc_result_t sub_line_begin_search(void);
rc_line_state_t sub_line_state(void);
rc_result_t sub_line_set_base(int16_t permille);
```
`init` registers for line-sample events. `enable` turns actual steering
on/off (state still tracked while disabled, but no motor commands issued
— used when another subsystem has taken over driving). `begin_search`
tells the follower "we left the line on purpose (e.g. going around an
obstacle), don't panic, just watch for it to reappear." `state` reports
tracking / lost / searching / junction (both sensors on black at once,
usually meaning a barcode is starting — this state arms the barcode
decoder). `set_base` sets the base driving speed in "permille" (parts per
thousand, i.e. 1000 = full speed) to keep the math in whole numbers since
there's no FPU.

**Internal logic worth understanding:** `steer_from_position()` implements
"proportional control" — the further off-centre the car is, the harder it
steers back, in direct proportion to the error. `on_sample()` runs on
every new reading: both sensors on line counts toward a junction; only one
steers proportionally toward centre; neither counts toward "lost," and
once lost the car arcs back toward whichever side the line was last seen
on (an arc finds the line again far more reliably than driving straight).

**`sub_barcode.h` — decoding**

```c
rc_result_t sub_barcode_init(void);
rc_result_t sub_barcode_arm(bool on);
rc_result_t sub_barcode_on_decode(sub_barcode_cb_t cb, void *ctx);
char sub_barcode_last(void);
```
`init` registers for raw bar/space width events. `arm` turns decoding
on/off (and the underlying interrupt) — left off except when a barcode is
expected, so track noise can't produce a phantom navigation command.
`on_decode` lets another subsystem register a callback for when a
character is decoded, handing back the character and its navigation
command. `last` returns the last decoded character, for logging.

**Internal logic — ratio-based decoding:** Code 39 encodes each character
as 9 elements (alternating bars/spaces), of which exactly 3 are "wide" and
6 "narrow." Because speed varies, absolute duration is meaningless, but
the *ratio* between wide and narrow stays roughly constant — so
`classify()` looks at the min/max width in the current window of 9
readings, takes the midpoint as a threshold, and marks each element wide
(1) or narrow (0), packing the result into a 9-bit pattern. It sanity-
checks that exactly 3 bits are "wide," since real Code 39 always has
exactly 3. `lookup()` matches the pattern against a known table; since the
car might cross the barcode in either direction, it also tries the
reversed pattern before giving up.

**Missing / TODO for Buddy 3**

- **Line following is proportional-only (no derivative term).**
  `steer_from_position()` only reacts to how far off-centre the car is,
  not how fast that's changing — the code will "weave." Add a derivative
  term to damp the overshoot. High value, small change.
- **Position estimate is coarse (4-state, not continuous).** Both line
  sensors are read as digital on/off, giving only four possible positions.
  To get smooth, continuous steering, switch to reading the *analogue*
  AOUT signal and interpolate a real numeric position. Note the two line
  sensors currently aren't wired to ADC pins — that would need to be
  added to the pin map alongside the code change.
- **Code 39 lookup table is incomplete — B, C, D are placeholder
  patterns.** In `sub_barcode.c`'s `table[]`, only `*` and `A` have
  correct bit patterns; B, C, D are marked `TODO: placeholder` and are
  simply wrong (some even duplicate `A`'s pattern) — decoding will
  silently fail or misfire for these until fixed. Look up the real,
  standard Code 39 encodings and replace the placeholders (9-bit
  patterns, most-significant-bit first, matching the existing format).
  **Do this before testing any barcode with B, C, or D symbols.**

---

### Buddy 4 — IMU & Terrain Detection

**Files:** `subsystems/sub_terrain.c/.h`, `drivers/drv_imu.c/.h`

**What this module does**

This module lets the car feel the ground it's driving on. It uses two
sensors on one small breakout board: an **accelerometer**, which works
like a tilt sensor — it feels which way is "down" and how hard the car is
being pushed or shoved — and a **magnetometer**, which works like a
compass, sensing the Earth's magnetic field to (weakly) tell direction.
Together they let the car notice when it's tipping nose-up over a speed
hump, estimate how tall that hump is, and tell "driving straight,"
"turning," "climbing a hump," "hitting something," and "stopped" apart.
**Important: this board has no gyroscope** (the part that would normally
measure spin rate directly), so the "obvious" textbook approach of
integrating rotation over time to get an angle is not available here —
every trick in this module works around that missing piece.

**How to get started**

1. **Read the warning comment at the top of `drv_imu.h` first.** It lists
   the three places the missing gyroscope changes the plan: tilt comes
   from gravity, not integration; turning comes from wheel encoders
   (Buddy 2's territory), not the IMU; hump height comes from pitch angle,
   not double-integrated acceleration.
2. **Wire and mount the board.** GY-511 (LSM303DLHC chip) on I²C. Per
   `docs/HARDWARE.md` §4.4: SDA→GP2, SCL→GP3, VCC→3V3, GND→GND. Mount it
   **rigidly** — if it's on a wobbly standoff, it registers its own wobble
   as fake terrain.
3. **Bring the driver up.** Call `drv_imu_init()` once at startup. It
   configures the accelerometer for 100 Hz sampling across all three axes
   at ±2g range (at that range each measurable step is about a thousandth
   of a g, the precision hump detection needs), and sets the magnetometer
   to continuous-read mode.
4. **Calibrate.** With the car level and completely still, call
   `drv_imu_calibrate(100)`. Skip this and every pitch reading afterward
   will be offset by the board's mounting tilt and manufacturing
   tolerance.
5. **Wire up sampling.** `drv_imu_sample()` should be called periodically
   (`RC_PERIOD_IMU_MS`) from the sensing task — it reads both sensors,
   subtracts the calibration bias, and broadcasts an `RC_EVT_IMU_SAMPLE`
   event that `sub_terrain.c` listens for.
6. **Call `sub_terrain_init()`** so the terrain module subscribes to those
   events and starts tracking hump state and motion class.
7. **Tune hump thresholds empirically** — don't guess. Drive hard on flat
   ground, log `drv_imu_pitch_ddeg()`, and set `PITCH_ENTER_DDEG` in
   `sub_terrain.c` comfortably above the peak noise you see from normal
   acceleration/braking/cornering. Current values (4.0° enter / 2.0° exit)
   are a starting point, not a validated setting.
8. Register a callback via `sub_terrain_on_hump()` if you want hump
   notifications directly, alongside the `RC_EVT_HUMP_END` event.

**Code walkthrough**

**`drv_imu.h` / `drv_imu.c` — the hardware layer.** Talks to the chip over
I²C (a two-wire protocol letting several sensor chips share the same pins,
each answering to its own address) and hands back numbers — it does not
decide what a "hump" is; that lives in `sub_terrain.c`.

- `drv_imu_init(void)` — opens the I²C bus device, writes configuration
  registers to turn on 100 Hz/±2g accelerometer sampling and continuous
  75 Hz magnetometer sampling. Returns an error if any write fails, so a
  dead or unwired sensor is caught at boot.
- `drv_imu_read_accel(int16_t *x, int16_t *y, int16_t *z)` — reads one raw
  accelerometer sample (milli-g, thousandths of Earth's gravity). Register
  trap #1: the chip returns the 12-bit measurement "left-justified" in a
  16-bit slot, so the code must shift right by 4 before it's usable —
  forgetting this silently multiplies every reading by 16. Blocks (waits),
  so only call from a normal task, never an ISR.
- `drv_imu_read_mag(int16_t *x, int16_t *y, int16_t *z)` — reads one raw
  magnetometer sample. Register trap #2: this chip is big-endian (opposite
  byte order from the accelerometer) and the registers come out in order
  X, Z, Y — not X, Y, Z. Get this wrong and axes get silently swapped.
- `drv_imu_sample(void)` — the routine the sensing task calls on a timer.
  Reads both sensors, subtracts stored calibration bias from X/Y/Z, and
  publishes `RC_EVT_IMU_SAMPLE`. Z keeps its natural 1g offset — only the
  *error* in Z is removed, because the pitch math needs that gravity
  reference to remain.
- `drv_imu_calibrate(uint16_t n)` — takes `n` accelerometer samples
  (yielding between them, never busy-waiting) and averages them into a
  bias to subtract from future readings. Must run once at startup, car
  level and motionless.
- `drv_imu_pitch_ddeg(void)` — turns the last accelerometer X reading into
  a pitch angle (nose tilt), in tenths of a degree. This is the core trick
  that replaces a gyroscope: when the car sits still or moves at constant
  speed, the only acceleration felt is gravity, which always points
  straight down. Tilt the car and part of "down" shows up on X instead of
  Z — more tilt, more X signal. It uses a small-angle shortcut (for
  pitches under ~25°, angle-in-radians ≈ X-acceleration ÷ gravity) instead
  of real trigonometry, because the RP2040 has no FPU (floating-point
  hardware unit) and real trig at 100 Hz would cost thousands of cycles.
  Only trustworthy when the car isn't also accelerating or braking hard,
  since that adds its own signal onto the same axis.

**`sub_terrain.h` / `sub_terrain.c` — the interpretation layer.** Turns
raw tilt numbers into "there's a hump" and "the car is turning."

- `sub_terrain_init(void)` — resets internal state and subscribes
  `on_sample()` to `RC_EVT_IMU_SAMPLE`, so terrain logic re-runs every
  time a new IMU reading arrives.
- `sub_terrain_on_hump(sub_terrain_hump_cb_t cb, void *ctx)` — registers a
  callback fired the moment a hump finishes, with its peak height and
  duration.
- `sub_terrain_max_peak_mm(void)` — returns the tallest hump peak seen so
  far this run — the number the project brief asks to be reported.
- `sub_terrain_motion_class(void)` — returns the current motion category
  (stationary, cruising, accelerating, decelerating, turning, climbing,
  descending, or impact).
- `sub_terrain_reset(void)` — clears hump-tracking and motion-class state,
  e.g. at the start of a fresh run.

Internally, `on_sample()` runs a small state machine
(`H_FLAT` → `H_CLIMBING` → `H_DESCENDING` → back to `H_FLAT`) driven by
pitch crossing threshold values: pitch rising past `PITCH_ENTER_DDEG`
means the front wheels started climbing; pitch swinging negative means
the car has crested; pitch settling back near zero means the hump is
over. Minimum/maximum duration filters reject events too quick to be real
or too slow (a gentle ramp, not a hump).

The clever part is **`pitch_to_height_mm()`** — hump height without a
gyroscope and without trusting double-integrated acceleration (which
drifts badly over the ~2 seconds a climb takes, ending up bigger than the
hump itself). Instead the car is treated as a rigid ruler of known length
(the wheelbase). While the front wheels are up on the ramp and the rear
ones aren't yet, the whole car tilts by the pitch angle, and geometry
says `rise = wheelbase * sin(pitch)`. For small angles, `sin(pitch)` ≈
pitch itself in radians, so the code multiplies wheelbase by
pitch-in-radians using only integer math.

`classify()` decides the motion class each sample: checks for a hard jolt
on X first (an "impact"), then defers to the hump state machine if
active, then falls back to wheel-encoder speeds (via
`drv_encoder_speed_mm_s()`) to detect stationary, turning, accelerating,
decelerating, or cruising.

**Missing / TODO for Buddy 4**

- **Turning classification is a placeholder, not real motion sensing.**
  The code comment above `classify()` admits it directly. "Turning"
  currently uses a crude threshold on left/right encoder speed difference
  (`diff > 150` or `< -150`), wired in quickly rather than tuned. Needs
  real tuning: verify the threshold against actual turning tests on the
  car, not a guessed constant.
- **Hump height has no independent cross-check yet.** Pitch-to-height is
  an *estimate*, not a measurement — validate it against a ruler on three
  known humps and quote the error in the terrain report. Beyond that,
  there's a concrete TODO: use distance travelled between hump entry and
  pitch peak (via `drv_encoder_distance_mm()`, already captured as
  `dist_at_hump_start`) multiplied by `sin(pitch)` for a *second*,
  independent height estimate from a different sensor (wheels, not tilt).
  Large disagreement between the two estimates is a signal a threshold
  needs retuning.

---

### Buddy 5 — Obstacle Scanning & Avoidance

**Files:** `subsystems/sub_scan.c/.h`, `drivers/drv_ultrasonic.c/.h`,
`drivers/drv_servo.c/.h`

**What this module does**

Imagine a bat, or a submarine's sonar operator: shout a short sound, then
listen for the echo bouncing off something in front of you — the longer
the echo takes to come back, the farther away the object is. That's
exactly what the HC-SR04 ultrasonic sensor does, except with an
ultrasonic "click" (40 kHz, above human hearing) instead of a shout.
Because one sensor only measures distance in a single straight line, it's
mounted on a small hobby servo motor — the same kind that swivels a
security camera — so it can be aimed left and right to build up a picture
of what's around the car, not just what's directly ahead. Buddy 5's job is
to sweep that sensor across an arc (a "coarse scan"), zoom in around
anything it finds (a "fine scan"), turn the raw distance readings into a
description of the obstacle (how close, how wide, how much room on each
side), and then decide whether the car should go straight, swerve left,
swerve right, or stop.

**How to get started**

1. **Wire it correctly first** (`docs/HARDWARE.md` §4.5): servo signal →
   GP12, HC-SR04 TRIG → GP16, HC-SR04 ECHO → GP17 **through a voltage
   divider** (1 kΩ ECHO→GP17, 2 kΩ GP17→GND). The RP2040 is not 5V
   tolerant and ECHO is a 5V signal — skipping the divider is the fastest
   way to destroy the board.
2. **Centre the servo mechanically at 90° before bolting on the bracket.**
   Power it, command 90° (`drv_servo_init()`'s default), and physically
   mount the bracket so "90°" really points straight ahead.
3. **Measure your actual servo's settle time and update the constants in
   `drv_servo.c`** (`US_PER_DEG`, `SETTLE_FIXED_MS`, used by
   `drv_servo_settle_ms()`). Defaults are rough guesses for a typical 9g
   servo. A full coarse scan takes roughly 5 × (servo travel time + 30ms
   of ultrasonic flight time) — getting this wrong makes scans too slow
   or makes them ping before the sensor has stopped swinging.
4. **Set `CAR_WIDTH_MM` in `sub_scan.c`** (currently a placeholder `150`)
   to your car's widest point plus about 40mm margin — this is what the
   avoidance planner compares gaps against.
5. Follow the HARDWARE.md §7 bring-up order: servo sweep test (step 8),
   ultrasonic vs. tape measure at 100/300/1000mm (step 9), full scan while
   confirming the motor PID loop keeps running smoothly throughout (step
   10).

**Code walkthrough**

Jargon up front: **ISR** (Interrupt Service Routine) — a tiny function
the chip jumps to automatically the instant something happens in
hardware. **PWM** (Pulse Width Modulation) — controlling servo
position/motor speed via rapid on/off pulses where what matters is how
long the pulse stays "on." **Duty cycle** — the fraction of each PWM
cycle spent "on." **TIMER alarm interrupt** — the RP2040's hardware
timers can be told "interrupt me in X microseconds," used here instead of
looping and counting.

**`drv_servo.h` / `drv_servo.c` — aiming the sensor**

- `drv_servo_init(void)` — sets up the PWM signal (50 Hz, standard for
  hobby servos) and centers it at 90°.
- `drv_servo_set_angle(int16_t deg)` — moves the servo to a given angle by
  converting it into a pulse width between 500–2400 microseconds (how
  hobby servos read "go to this position"). Clamps to 0–180°, returns
  immediately without waiting for arrival.
- `drv_servo_get_angle(void)` — returns whatever angle was last commanded
  (no feedback sensor exists — cheap hobby servos are "send and hope").
- `drv_servo_settle_ms(int16_t from, int16_t to)` — estimates how many
  milliseconds the arm needs to physically get there and stop wobbling.
  Since there's no feedback, this is the only way the code knows when a
  reading at the new angle can be trusted.
- `drv_servo_release(void)` — stops sending PWM pulses, letting the servo
  go limp and stop drawing current. Useful because the servo and drive
  motors share one battery, and a servo move during a motor current spike
  can brown out the Pico.

**`drv_ultrasonic.h` / `drv_ultrasonic.c` — the "shout and listen"**

- `drv_ultrasonic_init(void)` — sets up TRIG as output, registers (masked)
  an ECHO interrupt, and claims two RP2040 hardware TIMER alarms.
- `drv_ultrasonic_on_result(cb, ctx)` — registers a callback for when a
  measurement finishes. Note: this callback runs *inside an interrupt*,
  so it must be extremely short.
- `drv_ultrasonic_ping(int16_t tag_angle_deg)` — kicks off one
  measurement, tagged with the servo angle it was taken at (so later code
  matches "this distance" to "this direction" for free). Returns
  immediately, `RC_ERR_BUSY` if one is already running.
- `drv_ultrasonic_busy(void)` — reports whether a measurement is running.

The clever part — why this file doesn't look like the textbook Arduino
version — is that a naive implementation would busy-wait up to 30
milliseconds for the echo, stalling the motor-balancing loop and causing
a visible swerve. Instead the whole measurement is a **5-step interrupt
relay**, each step doing microseconds of work then handing off:

1. `ping()` raises TRIG, arms a timer alarm for +12μs, returns instantly.
2. Alarm 1 fires: drops TRIG low, un-masks ECHO's interrupt, arms a
   second alarm as a timeout safety net.
3. ECHO rises: stamps the current time.
4. ECHO falls: computes how long ECHO was high, converts to distance
   using the speed of sound (343 m/s, halved for the round trip), and
   hands the result to a "bottom half" task (`ultra_drain`) to publish,
   since ISRs must stay short.
5. If ECHO never toggles in time, the timeout alarm fires and reports an
   invalid reading, so a lost echo can never stall a scan forever.

**`sub_scan.h` / `sub_scan.c` — scanning, profiling, and planning**

- `sub_scan_init(void)` — creates the background task and event flag (a
  kernel object used to wake a task when something happens) driving the
  scan state machine, and registers for raw ultrasonic results.
- `sub_scan_start(void)` — kicks off a full scan (coarse then fine),
  returns immediately; results arrive later via callback and published
  events.
- `sub_scan_abort(void)` — cancels a scan in progress.
- `sub_scan_busy(void)` — reports whether a scan is running.
- `sub_scan_on_complete(cb, ctx)` — registers a callback fired once a scan
  finishes, with the obstacle "profile" and the avoidance "plan."
- `sub_scan_set_watch(bool on, uint16_t trigger_mm)` — turns on a slow,
  continuous forward ping while just line-following, so the car notices
  something getting close and triggers a full scan.

Internally this runs as a non-blocking state machine on its own task:
`S_COARSE_MOVE`/`S_COARSE_PING` sweeps from `RC_SCAN_COARSE_START` to
`RC_SCAN_COARSE_END` in `RC_SCAN_COARSE_STEP` increments, moving then
pinging at each stop. `find_fine_window()` picks the closest hit from the
coarse pass and re-scans a tighter window around it at finer angular
steps (`S_FINE_MOVE`/`S_FINE_PING`) for a sharper read. Every wait
(`step_to`, `wait_result`) is a real kernel wait-with-timeout, so the task
is genuinely off the CPU during servo travel and echo flight — the motor
control loop keeps running unaffected.

Once scanning finishes, `build_profile()` turns the (angle, distance)
points into: closest obstacle distance and angle, an estimated width (via
simple trigonometry — angular span of "close" readings times distance,
using the small-angle approximation `tan(x) ≈ x` since there's no FPU for
real trig), and left/right clearance (bearings counted as "clear" on each
side of straight-ahead). `build_plan()` makes the decision: go straight if
nothing's close; stop if both sides are narrower than `CAR_WIDTH_MM`;
otherwise turn toward whichever side has more clearance and compute how
far sideways (`lateral_mm`) and forward (`forward_mm`) to steer around it.

**Missing / TODO for Buddy 5**

- **No memory of which side was chosen last time.** On a track with
  several obstacles in a row, the planner picks left-or-right
  independently each time from current clearance counts only. If it
  alternates sides between obstacles, the car wastes distance zig-
  zagging. Needs a small piece of state remembering the last avoidance
  direction (with a bias toward repeating it when clearance is roughly
  tied).
- **No "reverse and reattempt" case.** The project brief calls for this,
  but `build_plan()` currently just returns `RC_CMD_STOP` when both
  `clearance_left_mm` and `clearance_right_mm` are below `CAR_WIDTH_MM` —
  it stops rather than backing up and trying a different approach angle.
  Needs: when both sides are too narrow, issue a reverse command, then
  re-trigger `sub_scan_start()` from the new position.
- **Report the real angular resolution honestly.** The HC-SR04's ~15°
  beam width means `RC_SCAN_FINE_STEP` (6°) is oversampling for smoothing
  only — the sensor physically cannot distinguish bearings closer than
  ~15° apart. Say so in writeups rather than claiming 6° accuracy.

---

## 2. Cross-cutting notes (everyone should skim)

- **Every pin lives in `core/rc_config.h`, nowhere else.** If you find a
  bare GPIO number in a `.c` file, that's a bug — fix it by adding a named
  constant there instead.
- **No floating point anywhere.** The RP2040 (Cortex-M0+) has no hardware
  floating-point unit, so every calculation in this tree uses whole
  numbers or fixed-point tricks (like the small-angle approximations in
  Buddy 4 and Buddy 5's code). Don't introduce `float`/`double`.
- **The `TODO` markers listed in each section above are placeholder
  values you're expected to fill in as the graded work of this course**
  — that's intentional, not a bug in the framework. When you finish one,
  say so clearly (e.g. in your commit message or report) so the team and
  graders know it's now real.
- **Before adding any interrupt-handling code**, read §0.2 above and copy
  the pattern in `drv_ultrasonic.c` — record-and-flag in the ISR, do the
  real work in a deferred task.
- **Build and check for warnings after every change**: `make -j8` (single
  core) and `make SMP=1 -j8` (dual core). Both must stay at zero warnings.
