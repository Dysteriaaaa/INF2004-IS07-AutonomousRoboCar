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

### 0.5 Building the project — a beginner's step-by-step (everyone)

This section assumes you have **never used a terminal to build C code, never
flashed a microcontroller, and don't know what any of these tools are.**
That's fine — every step below explains what you're doing and why, not just
the command to type. If a step feels scary, it isn't: you can't damage
anything just by typing a command wrong (the one genuine "can destroy
hardware" risk in this whole project is skipping the ultrasonic sensor's
voltage divider — covered in §0.6 — everything below is safe to retry).

#### 0.5.0 Words you'll see everywhere, defined once

- **Terminal / command line** — a text-only window where you type commands
  instead of clicking icons. On Windows, use **Git Bash** (installed
  alongside Git, see below) rather than the default Command Prompt — this
  project's build scripts assume a Unix-style shell.
- **Repository ("repo")** — a folder of code tracked by Git (a version-
  control tool, i.e. a tool that remembers every change ever made to the
  files). "Clone a repo" means "download a copy of it."
  This project actually needs **two** repos: this one (the car's own
  code — subsystems, drivers, etc.) and the RTOS "port" repo
  (`mtk3smp-rp2040`, the operating system this project runs on top of,
  including the actual build system / `make` files). This repo is not
  buildable by itself — it has to be dropped inside the port repo first.
- **Toolchain** — the programs that turn human-readable `.c` files into a
  binary the microcontroller can actually run. Here it's
  `arm-none-eabi-gcc`, a version of the C compiler `gcc` that outputs code
  for ARM chips (like the one inside the Pico) instead of your own laptop's
  chip.
- **`make`** — a tool that runs the right compiler commands for you, in
  the right order, based on a recipe file (`Makefile`). You will mostly
  just type `make` and let it work.
- **Microcontroller (MCU) / the Pico** — a tiny, self-contained computer on
  a chip, with no operating system of its own until you give it one (that's
  what the RTOS in this project is). It has no screen, no keyboard — the
  only way to "talk" to it while it runs is the serial console (§0.5.5) or
  by watching its LEDs/motors/sensors do something.
- **Flashing** — copying your compiled program onto the Pico's own memory
  so it runs automatically every time it's powered on, replacing whatever
  program was on it before.
- **`.uf2` file** — the specific file format the Pico accepts when
  flashing. `make` produces this for you; you don't build it by hand.
- **BOOTSEL** — a small physical button on the Pico board. Holding it while
  plugging in the USB cable tells the chip "don't run your program, just
  wait for me to copy a new one onto you." This makes the Pico appear to
  your laptop as if it were a USB flash drive.
- **Baud rate (115200)** — the speed the serial console talks at. Both
  sides (the Pico and your terminal program) must be set to the same
  number or you'll see garbled text instead of readable output.

#### 0.5.1 Install the tools (once per laptop)

You need four things. Exact install commands vary by OS — search
"install X on Windows/Mac/Linux" if the short version below isn't enough,
or ask a teammate who's already done it rather than guessing blindly.

1. **Git** — lets you download (clone) the two repositories.
   Windows: install "Git for Windows" from git-scm.com — this also gives
   you **Git Bash**, the terminal you'll use for every command in this
   guide. Mac: `brew install git` (or it's often preinstalled). Linux:
   `sudo apt install git` (Debian/Ubuntu) or your distro's equivalent.
2. **`arm-none-eabi-gcc`, baseline version 13.2.1** — the ARM compiler.
   This usually comes as part of the "GNU Arm Embedded Toolchain" or
   "arm-none-eabi-gcc" package. Install it, then confirm it's on your
   `PATH` (i.e. your terminal can find it without you typing the full
   folder path) by running:
   ```sh
   arm-none-eabi-gcc --version
   ```
   If that prints a version number, you're set. If it says "command not
   found," the install either failed or didn't add itself to your `PATH`
   — reinstall, or manually add its `bin` folder to your `PATH`
   environment variable.
3. **A host C++ compiler (`g++`)** — part of the build for a couple of
   host-side tools the port uses. On Windows this usually comes with the
   arm-none-eabi-gcc bundle or MinGW; on Mac it's part of Xcode Command
   Line Tools (`xcode-select --install`); on Linux, `sudo apt install
   g++`.
4. **A USB-serial adapter + driver**, and **a serial terminal program**
   to read the Pico's console output (details in §0.5.5). You don't need
   this to build — only to actually see what the car is doing once
   flashed.

#### 0.5.2 Get the two repositories onto your laptop

Open your terminal (Git Bash on Windows) and pick a folder to work in,
e.g. your Desktop. Then:

```sh
# 1. Download the RTOS port. This is the operating system + the actual
#    build system (Makefiles) that knows how to compile everything.
git clone https://github.com/sirfonzie/mtk3smp-rp2040.git
cd mtk3smp-rp2040

# 2. This project's own code should already be on your laptop (you're
#    reading this file from inside it). Copy or clone it into place at
#    app_program/robocar/ inside the port repo you just downloaded:
#    e.g. on Windows/Git Bash:
#    cp -r "/path/to/your/INF2004-IS07-AutonomousRoboCar" app_program/robocar

# 3. The port ships its own sample program in app_program/*.c — move
#    those files out of the way so the build uses this project's
#    usermain (app/app_main.c) instead of the port's own demo.
```

If you're not sure of the exact folder layout the port expects, check
`mtk3smp-rp2040`'s own `README`/`PORT_RP2040.md` — the important point is
just that this repo's contents end up as a sub-folder the port's Makefile
is told to build, and the port's own sample `app_program/*.c` files are
moved aside so they don't conflict.

#### 0.5.3 Apply the two required patches — do this before building

These fix real hardware conflicts on this specific carrier board (Robo
Pico). Skipping them doesn't crash the build — it silently produces a
robot that can't talk to its IMU, or has no status LED. Full detail and
exact code is in `docs/HARDWARE.md` §1.2 and §1.3; the short version:

1. **I²C pin patch.** The port's stock I²C driver
   (`device/i2c/sysdepend/rp2040/i2c_rp2040.c`) wires I²C channel 1 to
   pins GP6/GP7 by default. On this car those pins are already used by
   the line sensors (Buddy 3's hardware), so the IMU (Buddy 4's hardware)
   can't share them. Open that file and change the unit-1 pin setup to
   GP2/GP3 instead — `docs/HARDWARE.md` §1.2 has the exact lines to
   change.
2. **Status LED pin patch.** The port defaults its "liveness LED" pin
   (`BOARD_LED_PIN`) to GP16, but this project needs GP16 for the
   ultrasonic sensor's trigger pin (Buddy 5's hardware). Open
   `include/sys/sysdepend/pico_rp2040/sysdef.h` and change
   `BOARD_LED_PIN` to GP19 instead, then wire an external LED (+ resistor)
   to GP19 — the on-board Pico W LED can't be used here because it's
   wired to the WiFi radio chip, not a normal pin.

**Why this matters even if you're not Buddy 4 or Buddy 5:** if you skip
these and just try to build, the code will compile fine and look like
it works — the failure only shows up later as "the IMU never responds"
or "the LED never blinks," which is a much more confusing bug to chase
down than just doing the patch now.

#### 0.5.4 Build it

From inside the port repo (`mtk3smp-rp2040`), in your terminal:

```sh
cd build_make
make -j8
```

What this does, in plain terms: `make` reads the build recipe, and for
every `.c` file in this project it runs the ARM compiler to turn it into
machine code, then links all those pieces together into one program. The
`-j8` just means "use up to 8 CPU cores at once to go faster" — safe to
lower to `-j4` or drop entirely (just `make`) on a weaker laptop, it'll
just take longer.

**What success looks like:** the command finishes with **no errors and
zero warnings**, and you'll find a new file named something like
`mtk3pico_smp0_uart.uf2` inside `build_make/`. `SMP=0` means "single-core"
— the simplest configuration, and the one to always get working first.
Once that's solid, you can also try dual-core with:

```sh
make SMP=1 -j8
```

**If the build fails:** read the *first* error message, not the last —
one real error often causes a cascade of confusing follow-on errors below
it. The most common beginner mistakes are: `arm-none-eabi-gcc` not
installed/on `PATH` (§0.5.1), this repo not placed in the right folder
(§0.5.2), or a patch from §0.5.3 typed slightly wrong. If you truly get
stuck, screenshot the *first* error and ask a teammate or search the exact
error text — don't just retry the same command hoping it changes.

#### 0.5.5 Flash it onto the actual Pico

1. **Unplug** the Pico if it's connected.
2. **Hold down the BOOTSEL button** on the Pico board (don't let go yet).
3. While still holding it, **plug in the USB cable** (into your laptop).
4. **Let go of BOOTSEL.** Your laptop should now show a new removable
   drive named `RPI-RP2`, the same way a USB flash drive would appear.
   If nothing appears, unplug and repeat from step 1 — timing the button
   press right sometimes takes a couple of tries.
5. **Copy the `.uf2` file onto that drive** — drag-and-drop
   `build_make/mtk3pico_smp0_uart.uf2` onto `RPI-RP2` in your file
   explorer, the same as copying a file onto a USB stick.
6. The Pico will automatically reset and start running your program the
   moment the copy finishes — the `RPI-RP2` drive will disappear. That's
   normal, not an error.

If you ever want to reflash with new code, repeat this whole process —
there's no "uninstall" step, copying a new `.uf2` just overwrites what
was there.

#### 0.5.6 Watch it think — the serial console

The Pico has no screen, so the only window into what your program is
doing (print statements, error messages, sensor readings) is a **serial
console** — a stream of text sent over the USB-serial adapter's wire.

1. Wire the USB-serial adapter's RX/TX/GND pins to the Pico's UART0 pins:
   GP0 (TX) and GP1 (RX) — **cross them**: the adapter's RX goes to the
   Pico's TX (GP0) and the adapter's TX goes to the Pico's RX (GP1). Also
   connect GND to GND. (Getting RX/TX backwards is the single most common
   reason "nothing shows up" — if you see nothing, try swapping those two
   wires first.)
2. Plug the USB-serial adapter into your laptop. It should appear as a
   new "COM port" (Windows, e.g. `COM5`) or device file (Mac/Linux, e.g.
   `/dev/tty.usbserial-XXXX`).
3. Open a serial terminal program pointed at that port, set to **115200
   baud, 8 data bits, no parity, 1 stop bit** (often written "115200
   8N1" — this is the default assumed almost everywhere, so most tools
   just need the baud rate set). Common free options:
   - Windows: **PuTTY** (select connection type "Serial"), or the
     built-in serial monitor in VS Code's PlatformIO/Arduino extensions
     if you already have one installed.
   - Mac/Linux: `screen /dev/tty.usbserial-XXXX 115200` from a terminal,
     or a GUI tool like CoolTerm.
4. Reset the Pico (unplug/replug its power, or press its reset button if
   it has one) and you should see boot text appear, e.g. lines like
   `[init] event bus ok`, matching what `app/app_main.c`'s `step()`
   helper prints for each subsystem as it starts up. If every `step()`
   line says `ok` and you reach `[init] ready, starting run`, the whole
   framework initialized successfully — you're ready to start the
   hardware bring-up checklist below.

**Note:** the USB cable that powers the Pico during flashing and the
USB-serial adapter for the console are two separate connections — you'll
typically have both plugged in at once during development (one for power
and reflashing, one to read output), unless you build with
`CONSOLE=usb_cdc` (see `docs/HARDWARE.md` §5) to put the console on the
same USB port as flashing.

#### 0.5.7 Now bring the hardware up, one piece at a time

Read `docs/HARDWARE.md` §1 first — it lists **four pin/hardware conflicts
that will each cost you a day if nobody warns you**: the I²C collision and
LED collision from §0.5.3 above, a collision between the RTOS's internal
timer and the PWM pins used for motors/servos (already handled by this
codebase, just know it exists), and the ultrasonic sensor's ECHO pin
being 5V — **if you wire that pin directly to the Pico without the
resistor divider described in HARDWARE.md §4.5, you will permanently
destroy the board.** This is the one genuinely destructive mistake
possible in this whole setup — double-check that wiring before you ever
power it up.

Then work through the **bring-up order** in `docs/HARDWARE.md` §7 — a
numbered checklist that tests one thing at a time, in this order: blink
LED → console print → motors spin the right way → encoders count cleanly
→ closed-loop speed control works → line sensors flip crossing the line
→ line following on a straight/curved line → servo sweeps cleanly →
ultrasonic distance matches a tape measure → a full obstacle scan
completes → IMU reads level → hump detection → barcode decoding →
telemetry over console → telemetry over network → the full mission
end-to-end. **Do not skip ahead or wire everything at once.** Each step
is deliberately small so that when something doesn't work, you know
exactly which piece to suspect — testing everything at once means a
single bad wire could be hiding anywhere.

#### 0.5.8 Already comfortable with the VS Code Raspberry Pi Pico extension? Start here instead

If you've already made a Pico project with the official **Raspberry Pi
Pico VS Code extension** (the one with "New C/C++ Project", "Compile
Project" and "Run Project (USB)" buttons), you already know how to build
and flash *a* Pico project — but that extension's buttons assume a
**Pico SDK + CMake + Ninja** project (a `CMakeLists.txt` file, the
extension's own build folder, etc.). **This project has none of that.**
It's built by the RTOS's own hand-written `Makefile`s instead — no CMake,
no Ninja, no `CMakeLists.txt` anywhere in this tree. So the extension's
one-click buttons simply won't find anything to build here, and that's
expected, not a sign something's broken. Here's how what you already know
maps onto this project:

- **"Where's my toolchain?"** You don't need to reinstall
  `arm-none-eabi-gcc` — the Pico extension already installed one for you,
  under `%USERPROFILE%\.pico-sdk\toolchain\<version>\bin\` on Windows
  (the extension keeps its own private copies of the compiler, CMake,
  Ninja, OpenOCD and `picotool`, one versioned folder per tool, so it
  never depends on anything being on your system `PATH`). First check
  whether it's already reachable from a plain terminal:
  ```sh
  arm-none-eabi-gcc --version
  ```
  If that works, you're done — skip straight to §0.5.2. If it says
  "command not found," either add that `bin` folder to your `PATH`, or
  open the **"Pico - Developer Command Prompt"** shortcut the extension
  adds to your Start Menu — it's a terminal window pre-configured with
  every one of those tool paths already set, and it works fine for typing
  the `make` commands below even though this isn't a CMake project.
- **Instead of "New C/C++ Project from Pico SDK,"** follow §0.5.2/§0.5.3
  above: clone the `mtk3smp-rp2040` RTOS port repo, drop this project's
  folder into it, and apply the two hardware patches. There's no
  extension wizard for this step — it's just `git clone` and copying a
  folder.
- **Instead of clicking "Compile Project,"** open a terminal (the Pico
  Developer Command Prompt above, or VS Code's own integrated terminal —
  ``Ctrl+` ``) in the `mtk3smp-rp2040/build_make` folder and run
  `make -j8`, exactly as in §0.5.4. It's the same compiler under the
  hood, just driven by a `Makefile` instead of the extension's CMake
  integration.
- **Instead of clicking "Run Project (USB),"** which uses `picotool`/CMake
  wiring that doesn't exist in this tree, either **drag-and-drop the
  `.uf2` file onto the `RPI-RP2` drive** in BOOTSEL mode as described in
  §0.5.5, or, if `picotool` is already on your `PATH` from the extension
  install, you can flash from the terminal instead once the Pico is in
  BOOTSEL mode:
  ```sh
  picotool load -f build_make/mtk3pico_smp0_uart.uf2
  picotool reboot
  ```
  Either way gets you the same result — there just isn't a single button
  for it here.
- **Instead of the extension's built-in Serial Monitor panel** — that
  panel watches the Pico's *own* USB port, which by default carries
  nothing on this project, because the console defaults to a separate
  UART on GP0/GP1 (see §0.5.6) rather than USB. You have two options:
  1. Wire a USB-serial adapter to GP0/GP1 as described in §0.5.6, and
     point any serial tool (including the extension's Serial Monitor
     panel — it isn't picky about *which* COM port you select) at that
     adapter's COM port instead of the Pico's own one, at 115200 baud.
  2. **Or**, since you already have the Pico SDK installed locally (the
     extension put it under `%USERPROFILE%\.pico-sdk\sdk\<version>\`),
     rebuild with the console routed over the Pico's own USB port
     instead of a separate UART:
     ```sh
     make CONSOLE=usb_cdc PICO_SDK_PATH=C:/Users/<you>/.pico-sdk/sdk/<version> -j8
     ```
     Flash the resulting `.uf2` the same way as above. Now the Pico
     shows up as a normal USB-serial device the moment it boots, and the
     extension's own Serial Monitor panel (or any serial tool) will work
     against that port with no extra adapter needed — this is the closest
     this project gets to the one-cable, no-extra-hardware experience the
     extension normally gives you. The only catch: per `docs/HARDWARE.md`
     §4.1 this is the one part of the RTOS port explicitly flagged as
     **less battle-tested** than the UART path, so if console output ever
     looks flaky, fall back to a real UART adapter (option 1) to rule
     that out before assuming your own code is at fault.

Everything after this point — bring-up order, per-buddy work — is
identical no matter which path you used to get the `.uf2` flashed.

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

This is a line-by-line-style tour of `sub_telemetry.h` and
`sub_telemetry.c`. It assumes you've read §0 above (event bus, lanes,
non-blocking pattern) and explains everything else — pointers, structs,
casts, the works — the first time it shows up. The source files
themselves now also carry inline comments next to almost every
non-obvious line; this section is the "why," the source is the "what,
right here."

#### Jargon you'll hit immediately

- **Pointer**: a variable that stores a memory *address* rather than a
  value directly — think of it as a sticky note with someone else's
  house address written on it, instead of a photocopy of what's inside
  that house. You "follow" a pointer to get at the real data. C spells
  "pointer to a `foo`" as `foo *`.
- **Struct**: a bundle of named fields glued into one value, like a
  form with labelled boxes (name, age, address). C has no
  classes/objects — a struct is as close as it gets, and this module
  uses structs a lot.
- **Function pointer**: a pointer whose address points at *code*
  instead of data — a variable that "remembers a function" so it can be
  called later, possibly by code that has no idea which specific
  function it ends up calling. This is C's version of a "callback" and
  is the trick the whole pluggable-transport design rests on.
- **Cast**: writing `(int)x` or `(unsigned long)x` tells the compiler
  "treat this value as this other type for this one expression." It
  doesn't change the underlying number, it just satisfies a function
  that expects a specific type (very common when formatting numbers
  into text, as this file does constantly).
- **`static`** on something at file scope means "private to this .c
  file" — no other file can see or link to it. It's the nearest C gets
  to encapsulation without classes.
- **`const`** on a pointer parameter is a promise: "this function will
  only read through this pointer, never write through it."
- **JSON** (JavaScScript Object Notation): plain text laid out as
  `{"key":value, "key2":value2}`. It's human-readable in a debug
  console and every network/monitoring tool already understands it,
  which is exactly why this module uses it for message bodies.

#### `sub_telemetry.h` — the public menu

```c
typedef struct {
    const char *name;
    rc_result_t (*open)(void);
    rc_result_t (*publish)(const char *topic, const char *payload, uint16_t len);
    rc_result_t (*close)(void);
    bool        (*is_up)(void);
} sub_telemetry_sink_t;
```
This is a **struct of function pointers** — the single most important
idea in this module, so it's worth slowing down on. Every field after
`name` is not a value but the *shape* of a function: "a function taking
no arguments and returning an `rc_result_t`," for instance. Nothing here
says *which* function — that gets filled in later, once per transport
(see `console_sink` below). Once a `sub_telemetry_sink_t` is fully
filled in, it behaves like a plug: the rest of this file can call
`sink->open()`, `sink->publish(...)`, `sink->close()`, `sink->is_up()`
without caring at all whether it's actually talking to the debug
console, a UDP socket, or (eventually) an MQTT client. This is why
Buddy 1's real job — building UDP and MQTT transports — never requires
touching `send()`, `publish_state()`, or any of the scheduling logic:
build a new file that fills in one of these structs, hand it to
`sub_telemetry_set_sink()`, done. `open`/`publish`/`close` all return
`rc_result_t` (this project's shared success/failure enum — see
`rc_types.h`) so failures propagate the same way everywhere else in the
codebase; `is_up` returns a plain `bool` because it's a pure status
check, not an action that can itself fail.

```c
typedef void (*sub_telemetry_cmd_cb_t)(rc_nav_cmd_t cmd, int32_t arg, void *ctx);
```
This line defines a *type*, not a function — read it as "the name
`sub_telemetry_cmd_cb_t` now means: a pointer to a function that takes
an `rc_nav_cmd_t`, an `int32_t`, and a `void *`, and returns nothing."
`rc_nav_cmd_t` is the shared enum of driving commands (turn left, turn
right, U-turn, straight, stop — see `rc_types.h`, also used by Buddy 3's
barcode decoder and Buddy 5's obstacle planner). `void *ctx` is a
generic "carry whatever you like" pointer: whoever registers a callback
supplies it once via `sub_telemetry_on_command(cb, ctx)`, and it comes
back unchanged every time the callback fires later, letting the callback
find its own state without needing a global variable. This whole type
exists for the *receiving* side of telemetry — commands coming in from
outside the robot — which is declared but not yet wired to anything real
(see TODOs).

```c
rc_result_t sub_telemetry_init(void);
rc_result_t sub_telemetry_set_sink(const sub_telemetry_sink_t *sink);
rc_result_t sub_telemetry_on_command(sub_telemetry_cmd_cb_t cb, void *ctx);
rc_result_t sub_telemetry_publish_event(const rc_event_t *evt);
const sub_telemetry_sink_t *sub_telemetry_console_sink(void);
```
These five are the module's entire public API — everything another file
is allowed to call. They're covered in detail below alongside their
implementations in the `.c` file, since the header only declares their
*shape*, not what they do.

#### `sub_telemetry.c` — constants

```c
#define TOPIC_MAX       (48)
#define PAYLOAD_MAX     (192)
#define TOPIC_BASE      "car/01"
```
`#define` is a **preprocessor macro** — before the compiler even sees
the code, a separate pass called the preprocessor does a literal
find-and-replace of the macro name with its text. So every later use of
`TOPIC_MAX` becomes the literal number `48`. Using named constants
instead of scattering bare numbers ("magic numbers") through the code
means the *meaning* of the number is documented at its one definition,
and changing it later only requires editing one line.

- **`TOPIC_MAX` (48 bytes)** — the size of the local buffer `send()`
  uses to build a topic string like `car/01/state`. C strings need one
  extra byte for the terminating `'\0'` (a zero byte marking "end of
  string" — C strings don't carry their length separately, unlike, say,
  Python strings), so this has to be big enough for `TOPIC_BASE` plus
  the longest leaf name (`"heartbeat"`, `"obstacle"`, etc.) plus a `/`
  plus that terminator.
- **`PAYLOAD_MAX` (192 bytes)** — the size of the local buffer used to
  build one JSON message body before sending it. If a future message
  needs more fields than fit in 192 bytes, `rc_snprintf` will simply
  truncate it rather than overflow the buffer — bump this constant first
  if you add fields and messages start looking cut off.
- **`TOPIC_BASE` ("car/01")** — the shared prefix for every topic this
  car publishes, e.g. `car/01/state`, `car/01/barcode`. If this project
  ever ran multiple cars on one shared MQTT broker, each car would get a
  different number here so a broker subscription like `car/+/state`
  (the `+` is an MQTT wildcard for "any one topic segment") could
  address all of them, or `car/01/#` (the `#` wildcard, "everything
  under this prefix") could address just this one. This constant is
  exactly the kind of thing "#define STEER_KP (2)" is a stand-in for in
  the task brief — a single named knob other code (here, `send()`) reads
  by name instead of retyping the string.

#### File-scope state

```c
static const sub_telemetry_sink_t *sink;
static ID       telem_tskid;
static uint32_t seq;
static uint32_t tx_count;
static uint32_t tx_fail;
static sub_telemetry_cmd_cb_t cmd_cb;
static void    *cmd_ctx;
```
All `static` at file scope, meaning private to this file but alive for
the whole program run (unlike a variable declared inside a function,
which is thrown away when that function returns). `sink` is the pointer
to whichever transport struct is currently active — everything in this
file routes messages *through* this one pointer, which is the entire
mechanism behind swapping transports without touching the rest of the
code. `telem_tskid` is the RTOS's handle for the background task created
in `sub_telemetry_init()`. `seq`, `tx_count`, `tx_fail` are simple
running counters reported in the outgoing messages themselves — useful
for a human watching the console to notice, say, that `tx_fail` is
climbing. `cmd_cb`/`cmd_ctx` hold whatever was registered through
`sub_telemetry_on_command()`, waiting for a receiving transport to
actually call them one day.

```c
static volatile int32_t  st_speed_l;
static volatile int32_t  st_speed_r;
static volatile uint32_t st_dist_mm;
static volatile bool     st_line_l;
static volatile bool     st_line_r;
```
This little block is the module's cache of "the latest known sensor
readings," kept fresh by the `on_odometry()`/`on_line()` event
subscriptions further down and read out whenever `publish_state()` needs
to build the next status message. `volatile` here tells the compiler
"this value can change at any moment for reasons outside this function's
own code (a different task's callback wrote to it), so never assume a
previously read value is still valid — always re-read it." Without
`volatile`, an optimizing compiler would be technically allowed to
"cache" one of these values in a CPU register and never notice a
callback updated it. The comment above them in the source explains why
no lock/mutex is needed despite two different tasks touching them: each
field is written by exactly one callback and read by exactly one task,
and on this 32-bit architecture reading/writing a single `int32_t` or
`bool` can't be interrupted halfway through, so there's no way to
observe a "torn" half-updated value.

#### The console sink — `console_open`/`console_publish`/`console_close`/`console_is_up`, `console_sink`, `sub_telemetry_console_sink()`

These four small functions are the console transport's implementation
of the `sub_telemetry_sink_t` interface described above. `console_open`
and `console_close` do nothing (return `RC_OK` immediately) because
there's no connection to set up or tear down for printing to a screen.
`console_is_up` always returns `true` — there's no cable to unplug.
`console_publish` is the interesting one:

```c
static rc_result_t console_publish(const char *topic, const char *payload, uint16_t len)
{
    (void)len;
    tm_printf((UB *)"[telem] %s %s\n", topic, payload);
    return RC_OK;
}
```
`tm_printf` is the RTOS's own `printf`-alike for writing to the debug
console (the same place you see boot logs). `(void)len;` is a very
common C idiom meaning "yes, I know this parameter exists and I'm not
using it, don't warn me about it" — the console doesn't need an explicit
length because `tm_printf` can find the end of `payload` itself (it's a
normal null-terminated string), but the interface always hands `len`
along because a lower-level transport like raw UDP genuinely needs to
know how many bytes to send. `(UB *)` is a cast converting the string
literal's type to `UB *` ("unsigned byte pointer"), the type this
particular RTOS's `tm_printf` expects — it doesn't change any bytes,
just satisfies the compiler.

```c
static const sub_telemetry_sink_t console_sink = {
    "console", console_open, console_publish, console_close, console_is_up
};
```
This is where the "plug" actually gets built: a real
`sub_telemetry_sink_t` value with each function-pointer field pointing
at the matching function above. Writing a function's name without
parentheses (`console_open`, not `console_open()`) gives you its
*address* — exactly what a function-pointer field needs.
`sub_telemetry_console_sink()` just hands back `&console_sink` (the
address of that one shared instance) so other files can get at it — for
example, to pass explicitly to `sub_telemetry_set_sink()`, or as the
safe fallback when nothing else is configured.

#### `send()` — the shared mailroom

```c
static rc_result_t send(const char *leaf, const char *payload)
{
    char topic[TOPIC_MAX];
    rc_result_t res;

    if ((sink == NULL) || !sink->is_up()) {
        tx_fail++;
        return RC_ERR_STATE;
    }

    (void)rc_snprintf(topic, sizeof(topic), "%s/%s", TOPIC_BASE, leaf);

    res = sink->publish(topic, payload, (uint16_t)rc_strlen(payload));
    if (res == RC_OK) {
        tx_count++;
    } else {
        tx_fail++;
    }
    return res;
}
```
Every outgoing message in this module funnels through `send()`, so it's
the one place that (a) builds the full topic string, (b) makes sure
there's actually a working transport before trying, and (c) keeps the
`tx_count`/`tx_fail` counters accurate. `char topic[TOPIC_MAX]` is a
fixed-size buffer that lives on the stack (local to this function call —
it disappears the moment `send()` returns). The `if` guard handles two
different failure shapes: `sink == NULL` (nothing has been installed
yet, e.g. this got called before `sub_telemetry_init()`) and
`!sink->is_up()` (a real transport exists but currently reports itself
as down, e.g. WiFi dropped) — either way, count it as a failure and
bail out *before* touching the transport, rather than letting it fail
messily.

`rc_snprintf(topic, sizeof(topic), "%s/%s", TOPIC_BASE, leaf)` is this
project's own bounded string-formatting helper — like the C standard
library's `sprintf`, but because it's also told `sizeof(topic)` (the
buffer's actual capacity), it can never write past the end of the
buffer no matter how long the inputs are. That's what makes a
fixed-size `TOPIC_MAX`-byte buffer safe to use here. `"%s/%s"` is a
format string with two placeholders, filled in order by `TOPIC_BASE`
and `leaf` — e.g. `"car/01"` and `"state"` become `"car/01/state"`. The
leading `(void)` on this call and the next one discards the return
value (the count of characters that *would* have been written) — this
codebase consistently marks "I mean to ignore this" explicitly rather
than leaving an unchecked value that looks like an oversight.

`res = sink->publish(topic, payload, (uint16_t)rc_strlen(payload));` is
the payoff of the whole function-pointer design: this line calls
*through* whatever function `sink->publish` currently points at. It
never changes no matter which transport is active — that's the entire
point of the sink interface. `rc_strlen` is this project's `strlen`
equivalent (length of a string, not counting the terminating `'\0'`);
the cast to `uint16_t` matches the `len` parameter's declared type in
the sink interface. The final `if`/`else` updates whichever counter
matches the outcome, and `send()` returns that same result so its own
caller (`publish_state()`, `publish_heartbeat()`, or a case in
`sub_telemetry_publish_event()`) can react if needed too.

#### `publish_state()` — the per-tick status message

```c
static void publish_state(void)
{
    char buf[PAYLOAD_MAX];

    (void)rc_snprintf(buf, sizeof(buf),
        "{\"seq\":%lu,\"t\":%lu,\"spd_l\":%ld,\"spd_r\":%ld,"
        "\"dist\":%lu,\"m_l\":%d,\"m_r\":%d,\"line\":\"%c%c\","
        "\"cls\":%d,\"peak\":%u,\"bc\":\"%c\"}",
        (unsigned long)seq,
        (unsigned long)rc_time_ms(),
        (long)st_speed_l,
        (long)st_speed_r,
        (unsigned long)st_dist_mm,
        (int)drv_motor_get(RC_SIDE_LEFT),
        (int)drv_motor_get(RC_SIDE_RIGHT),
        st_line_l ? '1' : '0',
        st_line_r ? '1' : '0',
        (int)sub_terrain_motion_class(),
        (unsigned int)sub_terrain_max_peak_mm(),
        (sub_barcode_last() != '\0') ? sub_barcode_last() : '-');

    seq++;
    (void)send("state", buf);
}
```
This builds one JSON object by hand, using `rc_snprintf` the same way a
sentence gets built with fill-in-the-blanks: the big quoted string is
the template with a `%something` placeholder for each field, and the
arguments after it fill those blanks in order, left to right. It pulls
data from several other buddies' modules, which is a good map of how
this subsystem cross-cuts the whole project:

- `seq` — this module's own message counter, incremented after every
  send so a listener can notice gaps or reordering.
- `rc_time_ms()` — milliseconds since boot, from `core/rc_time.h`; used
  as a lightweight timestamp instead of a real wall-clock (the board has
  no RTC/network time source).
- `st_speed_l`/`st_speed_r`/`st_dist_mm` — the cached values described
  above, last updated by `on_odometry()` from Buddy 2's
  `RC_EVT_ODOMETRY` events.
- `drv_motor_get(RC_SIDE_LEFT/RIGHT)` — reads Buddy 2's motor driver
  directly (not via an event) to report the currently commanded duty
  cycle for each wheel.
- `st_line_l`/`st_line_r` — cached from `on_line()`, sourced from Buddy
  3's line-sensor events.
- `sub_terrain_motion_class()`/`sub_terrain_max_peak_mm()` — calls
  straight into Buddy 4's terrain module for the current motion category
  and tallest hump seen so far.
- `sub_barcode_last()` — calls into Buddy 3's barcode decoder for the
  most recently decoded character.

Every argument except the two `? :` ones is wrapped in a cast like
`(unsigned long)` or `(int)`. This is necessary because `rc_snprintf`'s
`%lu`/`%ld`/`%d`/`%u`/`%c` placeholders expect specific, exact C types,
and this project's own types (`uint32_t`, `int32_t`, etc.) don't
necessarily match those built-in types bit-for-bit on this platform — the
cast just relabels the value for this one call; it doesn't change the
number itself. `st_line_l ? '1' : '0'` is the **ternary/conditional
operator**: shorthand for "if `st_line_l` is true, use the character
`'1'`, otherwise use `'0'`" — a compact inline if/else that produces a
value rather than running a statement, handy inside an argument list
like this. The barcode field does the same trick to fall back to `'-'`
when nothing has been decoded yet (`sub_barcode_last()` returns `'\0'`,
the null character, to mean "nothing").

The two lines after the big `rc_snprintf` call — `seq++;` and
`send("state", buf);` — bump the counter for *next* time and hand the
finished buffer to the shared mailroom to actually go out on topic
`car/01/state`.

#### `publish_heartbeat()` — the less-frequent health summary

```c
static void publish_heartbeat(void)
{
    char buf[PAYLOAD_MAX];

    (void)rc_snprintf(buf, sizeof(buf),
        "{\"up\":%lu,\"tx\":%lu,\"fail\":%lu,"
        "\"drop_fast\":%lu,\"drop_slow\":%lu,\"sink\":\"%s\"}",
        (unsigned long)rc_time_ms(),
        (unsigned long)tx_count,
        (unsigned long)tx_fail,
        (unsigned long)rc_event_dropped(RC_LANE_FAST),
        (unsigned long)rc_event_dropped(RC_LANE_SLOW),
        (sink != NULL) ? sink->name : "none");

    (void)send("status", buf);
}
```
Same technique as `publish_state()`, different, less time-critical set
of numbers: uptime, total messages sent (`tx_count`) and failed
(`tx_fail`), and — interesting cross-link — `rc_event_dropped(RC_LANE_FAST)`
/`rc_event_dropped(RC_LANE_SLOW)`, which ask `core/rc_event.c` (the
event bus itself, shared by every module) how many events on each lane
were ever dropped because a subscriber's queue filled up before it could
keep up. `RC_LANE_FAST`/`RC_LANE_SLOW` are the same two lanes described
in §0.2 — seeing `drop_fast` above zero anywhere in the fleet is a real
warning sign, since that's the lane steering and obstacle response rely
on. `sink->name` reports which transport is currently active
("console" today; "udp"/"mqtt" once built) as plain text, reading the
`name` field set back in the `console_sink` struct literal. `send()` is
called with topic leaf `"status"`, landing on `car/01/status`.

#### Event subscriptions — `on_odometry`, `on_line`, `on_notable`

```c
static void on_odometry(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    st_speed_l = evt->u.odometry.speed_l_mm_s;
    st_speed_r = evt->u.odometry.speed_r_mm_s;
    st_dist_mm = (evt->u.odometry.dist_l_mm + evt->u.odometry.dist_r_mm) / 2U;
}
```
This is a **subscriber callback** — a function registered against one
event ID via `rc_event_subscribe()` (see `sub_telemetry_init()` below),
which the event bus then calls automatically every time that kind of
event is published anywhere in the program. This module never calls
Buddy 2's motion code directly; it just reacts whenever Buddy 2's code
publishes an `RC_EVT_ODOMETRY` event, exactly the publish/subscribe
pattern from §0.2. `evt` is a *pointer* to the event data (passed by
pointer rather than by copy so the whole struct doesn't have to be
duplicated for every subscriber); `evt->u.odometry.speed_l_mm_s` uses
`->` instead of `.` specifically because `evt` is a pointer — `->` means
"follow the pointer, then reach into the field." The `u` in the middle
is a **union**: one block of memory that can be interpreted as different
shapes (`odometry`, `line`, `barcode`, `hump`, ...) depending on
`evt->id`, so one generic `rc_event_t` type can carry many different
payload shapes without wasting memory holding all of them at once — a
bit like one small parcel box that gets relabelled and repacked
differently depending on what's being shipped that day. This callback
does the least possible work: copy three numbers into the module's
cached state (`st_speed_l` etc., described above) and return — keeping
event-bus callbacks fast matters because the bus calls them directly,
and a slow subscriber can back up the lane it's on.

`on_line()` does the exact same thing for line-sensor state
(`st_line_l`/`st_line_r`), triggered by Buddy 3's `RC_EVT_LINE_SAMPLE`
events.

```c
static void on_notable(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    (void)sub_telemetry_publish_event(evt);
}
```
This one callback is reused for four different event IDs (barcode
decoded, hump end, obstacle profile, impact — see the subscriptions
below). Rather than caching anything, it just forwards the whole event
straight to `sub_telemetry_publish_event()` to build and send a message
*immediately*, rather than waiting for the next scheduled tick the way
`publish_state()` does. That's the "out of band" idea mentioned in the
header: a decoded barcode or a finished hump shouldn't have to wait up
to `RC_PERIOD_TELEM_MS` milliseconds to be reported.

#### `sub_telemetry_publish_event()` — build-and-send for one-off events

```c
rc_result_t sub_telemetry_publish_event(const rc_event_t *evt)
{
    char buf[PAYLOAD_MAX];

    switch (evt->id) {
    case RC_EVT_BARCODE_DECODED:
        (void)rc_snprintf(buf, sizeof(buf),
            "{\"sym\":\"%c\",\"cmd\":%d,\"rev\":%d}",
            evt->u.barcode.symbol,
            (int)evt->u.barcode.command,
            evt->u.barcode.reversed ? 1 : 0);
        return send("barcode", buf);
    ...
```
A `switch` statement picks one of several code paths based on a single
value — here, `evt->id`, the event's type tag. Each `case` reaches into
a different member of the `u` union described above (`evt->u.barcode`,
`evt->u.hump`, `evt->u.profile`) because each event type carries
different data, and builds its own small JSON shape and its own topic
leaf:

- `RC_EVT_BARCODE_DECODED` → `car/01/barcode`, carrying the decoded
  character, the navigation command it maps to (Buddy 3's decoder
  output), and whether the pattern was read reversed.
- `RC_EVT_HUMP_END` → `car/01/hump`, carrying Buddy 4's estimated peak
  height, how long the hump took, and the maximum pitch angle recorded.
- `RC_EVT_OBSTACLE_PROFILE` → `car/01/obstacle`, carrying Buddy 5's scan
  results: point count, closest distance/angle, estimated width, and
  clearance on each side.
- `RC_EVT_IMPACT` → `car/01/impact`, with a fixed `{"impact":1}` body
  since there's nothing further to report — the event firing at all
  *is* the information.
- `default:` — any event ID this function doesn't recognise returns
  `RC_ERR_PARAM`. This shouldn't happen in practice, since
  `sub_telemetry_init()` only ever hands `on_notable()` (which calls
  this function) the four IDs handled above, but it's a safe, explicit
  fallback rather than silently doing nothing.

This function is exposed in the header specifically so other code
*besides* `on_notable()` could call it directly too, if some future
module wanted to push a telemetry message immediately without going
through the event bus at all.

#### `telemetry_task()` — the background loop

```c
static void telemetry_task(INT stacd, void *exinf)
{
    uint32_t ticks = 0U;
    (void)stacd;
    (void)exinf;

    if (sink != NULL) {
        (void)sink->open();
    }

    for (;;) {
        tk_dly_tsk(RC_PERIOD_TELEM_MS);
        /* TODO Buddy 1: connection recovery belongs here. ... */
        publish_state();
        ticks++;
        if ((ticks % 8U) == 0U) {
            publish_heartbeat();
        }
    }
}
```
This is the function that actually runs as a **task** — in this RTOS, a
task is like a thread: an independent stream of execution the kernel
schedules alongside every other subsystem's task (motor control, line
following, scanning, ...). Its parameter shape `(INT stacd, void
*exinf)` is fixed by the RTOS's task API — it's what `tk_cre_tsk()`
below expects to be able to call, whether or not this particular task
needs those parameters (it doesn't, hence the `(void)stacd; (void)exinf;`
"ignore these" lines). `for (;;)` is a deliberate infinite loop: this
task runs for the entire lifetime of the program, the same as every
other subsystem's background task.

`tk_dly_tsk(RC_PERIOD_TELEM_MS)` is the key line for the non-blocking
philosophy in §0.3: it's an RTOS kernel call that puts *this task*
(only this one) to sleep for `RC_PERIOD_TELEM_MS` milliseconds — 250ms
by default, defined in `core/rc_config.h`, the same shared tunables file
every subsystem's periods and priorities live in (Buddy 4's IMU sampling
period, `RC_PERIOD_IMU_MS`, lives right next to it, following the same
pattern) — while every *other* task keeps running normally. This is what
makes the telemetry loop periodic without a busy `while` loop wasting
CPU cycles the motor-control task needs.

The `TODO` comment marks Buddy 1's second real task: today, once the
sink is opened at task start, nothing ever checks whether it's still
connected. The fix belongs right here, once per loop iteration — call
`sink->is_up()`, and if it reports down, call `sink->close()`, wait a
bit using `tk_dly_tsk` with a growing delay each retry (a "backoff"), then
try `sink->open()` again. The comment specifically warns against a
"retry loop" — a tight loop that keeps trying without yielding — because
that would violate the non-blocking rule from §0.3 and stall the whole
task (and delay every message after it) while waiting on a dead network
link.

`publish_state()` runs every single tick (every 250ms by default).
`ticks % 8U == 0U` uses the **modulo operator** `%` (remainder after
division) to run `publish_heartbeat()` only on every 8th tick — 250ms ×
8 = 2 seconds — a cheap way to put something on a slower sub-schedule
without needing a second timer or task.

#### `sub_telemetry_init()` — wiring it all up at boot

```c
rc_result_t sub_telemetry_init(void)
{
    T_CTSK ctsk;
    sink = &console_sink;

    (void)rc_event_subscribe(RC_EVT_ODOMETRY, RC_LANE_SLOW, on_odometry, NULL);
    (void)rc_event_subscribe(RC_EVT_LINE_SAMPLE, RC_LANE_SLOW, on_line, NULL);
    (void)rc_event_subscribe(RC_EVT_BARCODE_DECODED, RC_LANE_SLOW, on_notable, NULL);
    (void)rc_event_subscribe(RC_EVT_HUMP_END, RC_LANE_SLOW, on_notable, NULL);
    (void)rc_event_subscribe(RC_EVT_OBSTACLE_PROFILE, RC_LANE_SLOW, on_notable, NULL);
    (void)rc_event_subscribe(RC_EVT_IMPACT, RC_LANE_SLOW, on_notable, NULL);

    ctsk.exinf   = NULL;
    ctsk.itskpri = RC_PRI_TELEMETRY;
    ctsk.stksz   = RC_STACK_SZ;
    ctsk.task    = telemetry_task;
    ctsk.tskatr  = TA_HLNG | TA_RNG3;

    telem_tskid = tk_cre_tsk(&ctsk);
    if (telem_tskid <= E_OK) {
        return RC_ERR_HARDWARE;
    }
    (void)tk_sta_tsk(telem_tskid, 0);
    return RC_OK;
}
```
Called once, from the app's startup code (`usermain`, in `app/`), this
is the module's entry point. First it defaults `sink` to the console
transport so messages have somewhere safe to go from the very first
tick, even before anyone calls `sub_telemetry_set_sink()`.

Then it makes six calls to `rc_event_subscribe(event_id, lane, callback,
ctx)` — one line per kind of announcement this module cares about. Every
single one uses `RC_LANE_SLOW` deliberately: telemetry reporting must
never be allowed to sit in front of, or delay, the fast lane used for
steering and obstacle-avoidance reactions (§0.2's golden rule). `NULL`
is passed as the `ctx` argument on each because none of these callbacks
need any extra context beyond the event data itself — everything they
need is either in the event or in this module's own file-scope state.

The rest of the function creates the background task. `T_CTSK ctsk;`
declares a **configuration struct** — the RTOS's way of describing "the
task I want" before actually creating it, the same settings-struct
pattern used for the sink interface above, just for a different purpose.
Each field is filled in: no extra start data (`exinf`), the priority and
stack size read from the shared config constants in `rc_config.h`
(`RC_PRI_TELEMETRY` = 10, `RC_STACK_SZ` = 4096 bytes — the same stack
size constant every other subsystem's task uses, so nobody has to
reinvent a number), which function the task should actually run
(`telemetry_task`, described above), and attribute flags required by
this particular RTOS (`TA_HLNG` — written in a high-level language, i.e.
C, not assembly; `TA_RNG3` — runs at the least-privileged CPU protection
ring). `tk_cre_tsk(&ctsk)` is the kernel call that actually creates the
task from those settings and hands back either a small positive integer
ID or a negative error code; `telem_tskid <= E_OK` checks for that
failure case (`E_OK` is the kernel's "no error" sentinel value — a
successful task ID is always greater than it). Finally
`tk_sta_tsk(telem_tskid, 0)` actually starts it running — `tk_cre_tsk`
only creates a task in a dormant state, so without this call
`telemetry_task()` would never begin executing at all.

#### `sub_telemetry_set_sink()` and `sub_telemetry_on_command()` — the two remaining public entry points

```c
rc_result_t sub_telemetry_set_sink(const sub_telemetry_sink_t *new_sink)
{
    if (sink != NULL) {
        (void)sink->close();
    }
    sink = (new_sink != NULL) ? new_sink : &console_sink;
    return sink->open();
}
```
This is the function Buddy 1 will call from application startup code
once a UDP or MQTT sink is built: close whatever transport was active
before (if any), switch the shared `sink` pointer to the new one — or
back to `&console_sink` if `NULL` was passed in, a deliberate safe
fallback — and open it. Because every other function in this file reads
messages out through the same `sink` pointer rather than hard-coding a
transport, this one function is all it takes to change where every
future message goes.

```c
rc_result_t sub_telemetry_on_command(sub_telemetry_cmd_cb_t cb, void *ctx)
{
    cmd_cb  = cb;
    cmd_ctx = ctx;
    return RC_OK;
}
```
The simplest function in the file: it just remembers the callback and
context pointer it was given, for later. Nothing in the current codebase
ever actually calls `cmd_cb` — that's the other half of the "receiving
commands from outside the robot" TODO. Once a UDP or MQTT sink's receive
path exists and decodes an incoming command, it would call `cmd_cb(cmd,
arg, cmd_ctx)` to hand it off to whatever registered here (most likely
`sub_nav.c`, the shared mission state machine from §0.4).

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

This is a big module, so it's broken down file by file, and inside each
file, constant by constant and function by function — the same bar of
detail as "`#define STEER_KP (2)` — what it does, why, any links to other
code" that the rest of this walkthrough follows. Jargon is explained the
first time it comes up; skip ahead once you recognise a term.

#### `drv_motor.h` / `drv_motor.c` — the "pure output" layer

This layer knows nothing about speed; it just turns electrical power on
and off in a direction. It never reads a sensor. Think of it as the gas
pedal and gearstick, with no speedometer attached — `sub_motion.c` below
is the part that actually watches the speedometer and decides how hard to
press.

**Jargon you need first:**
- **PWM (pulse-width modulation)** — the only way to get a variable
  "amount of power" out of a pin that can only be fully on or fully off.
  The pin is flipped on and off very fast (20,000 times a second here —
  `MOTOR_PWM_HZ`), and the *fraction* of each cycle spent "on" — the
  **duty cycle** — controls the average power delivered. It's the same
  trick as flicking a light switch faster than your eye can follow: flick
  it on 30% of the time and the bulb just looks dimmer, it doesn't
  visibly blink.
- **H-bridge** — the chip on the Robo Pico board that actually pushes
  current through the motor. It has four internal switches arranged so
  that, depending on which pair is turned on, current flows through the
  motor one way (forward) or the other (reverse). Robo Pico exposes that
  as *two* PWM pins per motor (`pin_a` and `pin_b`) instead of one PWM pin
  plus a separate direction wire — driving `pin_a` spins the motor one
  way, driving `pin_b` spins it the other way, and driving both at once
  would short the H-bridge, which the code goes out of its way to avoid.

**Constants:**
- `#define MOTOR_PWM_HZ (20000UL)` — the PWM switching frequency, 20 kHz.
  Chosen because it's above the top of human hearing (so the motor
  doesn't audibly whine at partial power, the way a slower PWM frequency
  would) and comfortably inside what the Robo Pico's output stage can
  switch cleanly. If your motors whine anyway, the comment right above it
  suggests dropping to 10 kHz.
- `#define DUTY_MAX (1000)` — the largest legal duty value in "permille"
  units (parts-per-thousand — this project's integer stand-in for a
  percentage with one extra digit of precision, used everywhere instead
  of a fractional 0.0–1.0 float, because the RP2040 has no FPU — no
  hardware floating-point unit, so real division of fractional numbers is
  slow/unsupported and the whole codebase avoids it — see §2 of this
  guide, "No floating point anywhere"). `drv_motor_set()` clamps every
  incoming duty to ±`DUTY_MAX` so nothing downstream can ever ask for more
  than 100% power.

**The `motor_t` struct** — one instance per motor, kept in the private
`motors[2]` array indexed by `rc_side_t` (`RC_SIDE_LEFT`/`RC_SIDE_RIGHT`):
`pin_a`/`pin_b` are which two GPIO pins drive this motor (numbers come
from `rc_config.h`, the one file all pin numbers live in — see §2);
`last` is the most recently commanded signed duty, kept purely so
`drv_motor_get()` can report it later — this is what lets
`drv_encoder.c` infer which way a wheel is *supposed* to be
spinning, since the encoder hardware itself can't sense direction (more
on that below).

**Functions:**
- `drv_motor_init(void)` — boot-time setup. Loops over both motors,
  configures PWM on both of their pins at `MOTOR_PWM_HZ`, and sets duty to
  zero on all four pins so the car can't lurch the instant power comes on.
  Call once, before anything else in this module.
- `drv_motor_set(rc_side_t side, int16_t duty_permille)` — the lowest-level
  "make a wheel spin" call in the whole project. Clamps the requested
  duty into ±1000, then — this is the important bit — always drops the
  *opposite* pin to zero before raising the driving pin, so the H-bridge
  is never told to energize both directions at once (see the H-bridge
  explanation above). Positive `duty_permille` drives forward through
  `pin_a`; negative drives reverse through `pin_b`.
- `drv_motor_set_pair(int16_t left_permille, int16_t right_permille)` —
  convenience wrapper that calls `drv_motor_set()` twice. `sub_motion.c`
  uses this almost exclusively, since nearly every control decision
  updates both wheels together.
- `drv_motor_stop(rc_motor_stop_t mode)` — stops both wheels one of two
  ways, chosen by the `rc_motor_stop_t` enum (a **enum**, short for
  "enumeration," is just a named list of integer constants — using
  `RC_MOTOR_COAST`/`RC_MOTOR_BRAKE` instead of raw `0`/`1` makes the call
  site read like English instead of a magic number). `RC_MOTOR_COAST` cuts
  power entirely so the wheel free-spins down under its own friction;
  `RC_MOTOR_BRAKE` drives *both* pins of the H-bridge high at once, which
  intentionally shorts the motor windings together — that's an
  electrical brake, not a physical one, and it stops the wheel much
  faster than coasting.
- `drv_motor_get(rc_side_t side)` — returns whatever duty was last
  commanded (not a live measurement — just "what we last told it to do").
  This one function is the load-bearing link between this file and
  `drv_encoder.c`: because the wheel's spin sensor can't tell direction
  on its own, `drv_encoder_speed_mm_s()` calls `drv_motor_get()` and uses
  its sign to decide whether the measured speed should be reported as
  positive or negative.

#### `drv_encoder.h` / `drv_encoder.c` — turning wheel clicks into speed and distance

Each wheel has a slotted disc attached to its axle and a small optical
sensor next to it (an ST188 or similar). Every time a slot in the disc
passes between the sensor's emitter and detector, the sensor's output
pin pulses. This is exactly the trick behind counting clicks on a
bicycle's spoke card as the wheel spins — count clicks per second and you
know speed; count total clicks and (knowing how far one click represents)
you know distance travelled. That whole idea — clicks in, distance/speed
out — is what this codebase calls **odometry**, and it's the numeric
backbone this entire module (and Buddy 4's terrain code, and telemetry)
relies on.

**Jargon you need first:**
- **ISR (Interrupt Service Routine)** — a tiny function the processor
  jumps to *immediately* the instant a watched pin changes voltage,
  pausing whatever normal code was running, running the ISR, then
  resuming exactly where it left off. It exists so time-critical events
  (like "the wheel just clicked") get handled with microsecond precision
  instead of waiting for the next time some loop happens to check the pin.
  See TEAM_GUIDE.md §0.2 for the project-wide "golden rule" about what an
  ISR is and isn't allowed to do.
- **`volatile`** — a C keyword you'll see on every field of the `enc_t`
  struct below. It tells the compiler "this variable can change at any
  moment from outside the normal flow of this function (here: from an
  interrupt), so never assume a cached copy in a CPU register is still
  correct — always re-read it from memory." Without `volatile`, an
  optimizing compiler could quietly break this code by reusing a stale
  value.
- **Critical section (`DI`/`EI`)** — `DI(sts)` disables interrupts and
  saves the previous interrupt-enabled state into `sts`; `EI(sts)`
  restores it. Code between them can't be interrupted, which matters
  whenever it reads more than one `volatile` field that the ISR could
  update — without this, you could read a fresh timestamp paired with a
  stale period, a "torn read."
- **Debounce** — real slotted-opto sensors don't always switch cleanly;
  a slot edge moving slowly through the sensor can make the output
  flicker (bounce) a few times instead of switching once. "Debouncing"
  means ignoring any second edge that arrives suspiciously soon after the
  first, since it's almost certainly noise from the same physical slot,
  not a second slot.

**Constants:**
- `#define STALL_TIMEOUT_US (1000000UL)` — if a full second passes with
  no new edge on a wheel, `drv_encoder_period_us()` reports `0`
  ("stopped") instead of letting the "time since last edge" number keep
  growing forever, which would otherwise make a stationary wheel look
  like it's moving at an ever-decreasing crawl rather than not moving at
  all. One second is deliberately generous; tighten it once your car's
  slowest useful speed is known.
- `#define DEBOUNCE_US (500UL)` — the debounce window described above.
  The comment beside it shows the reasoning: at 20 slots per revolution
  and 600 RPM, slots are 5 ms apart, so rejecting anything closer than
  500 µs together has a wide safety margin without risking throwing away
  a real click.

**The `enc_t` struct** — one instance per wheel, in the private `encs[2]`
array: `count` is the running total of accepted clicks since boot
(monotonic — only ever increases); `last_us` is the timestamp of the most
recent accepted edge; `period_us` is the microsecond gap between the two
most recent accepted edges — this is what speed gets computed from;
`base_count` is a snapshot of `count` taken at the last
`drv_encoder_reset()`, so "distance since reset" is just
`count - base_count`; `published` is the `count` value already turned
into an event, so the bottom-half worker (below) doesn't re-publish
unchanged data every cycle; `pin` is the GPIO this wheel's sensor is
wired to. Every field except `pin` is `volatile`, because the ISR writes
them and normal code reads them (see jargon above).

**`encoder_isr(pin, level, t_us, ctx)`** — the actual interrupt handler,
attached to rising edges only (`drv_encoder_init`, below, explains why
"rising only" rather than both edges). `ctx` arrives as a generic
`void *` — an untyped pointer — and is cast back to `enc_t *`; this is
the standard C pattern for giving one shared handler function its own
per-instance data (here, "which wheel is this edge for") without writing
two nearly-identical handler functions. Inside, it does the absolute
minimum: compute the gap since the last accepted edge (`delta`), bail out
if that gap is smaller than `DEBOUNCE_US` (noise, ignore it), otherwise
record the new timestamp, store the new period, increment `count`, and
call `rc_defer_signal_i(defer_h)` to wake the bottom-half task. It does
**not** build or publish an event itself — see TEAM_GUIDE.md §0.2's rule
that an ISR may only record state and hand off, never do real work,
because the two encoder pins, the ultrasonic echo pin, and the barcode
pin all share the same interrupt hardware bank; time spent in one ISR is
time another pin's edge is waiting to be noticed, and for the barcode
reader specifically, edge *timing* is the actual data being measured, so
delay there directly corrupts a reading.

**`encoder_drain(ctx)`** — the "bottom half" (deferred work), which runs
later in normal task context rather than inside the interrupt, so it's
safe to do more here, including publishing on the event bus. For each
wheel, it copies `count`/`period_us` out inside a `DI`/`EI` critical
section (see jargon above), skips publishing if nothing changed since the
last drain (`count == published`), and otherwise publishes an
`RC_EVT_ENCODER_EDGE` event carrying the current count and period. Note
this deliberately *coalesces* rapid edges — if three clicks arrive before
the drain task next runs, one event reports the latest state rather than
three separate events replaying each click — because downstream code
cares about "current count and period," not "receiving one event per
physical click," and coalescing keeps a fast-spinning wheel from flooding
the event bus.

**`drv_encoder_init(void)`** — boot-time setup: registers `encoder_drain`
as the deferred worker, zeroes every field of both `enc_t` structs, and
attaches `encoder_isr` to each encoder pin for **rising edges only**. The
comment in the code explains why not both edges: counting both edges
would double the resolution, but a real slotted disc's "solid" and "gap"
segments aren't the same width, so the timing between a rising and the
next falling edge isn't the same as between two risings — mixing them
would make the period measurement (and therefore speed) noisy and wrong.

**`drv_encoder_count(side)`** — raw lifetime click count for one wheel,
no unit conversion, never resets.

**`drv_encoder_period_us(side)`** — returns the microsecond gap between
the two most recent accepted edges on one wheel, reading both fields
inside a critical section (see jargon above) so they can't be torn apart
by an interrupt mid-read, and returns `0` if the wheel has been silent
longer than `STALL_TIMEOUT_US`.

**`drv_encoder_speed_mm_s(side)`** — the function the PID loop in
`sub_motion.c` reads every control cycle; this is the "actual speed" half
of the cruise-control comparison described under `sub_motion.c` below.
It converts the raw period into millimetres/second:
`speed = (RC_ENC_UM_PER_TICK * 1000UL) / period`. `RC_ENC_UM_PER_TICK`
(defined in `rc_config.h`, not owned by this module but central to it) is
how many micrometres of wheel travel one click represents, computed from
wheel circumference divided by slots-per-revolution
(`(31416 * RC_WHEEL_DIAM_MM) / (10 * RC_ENC_SLOTS_PER_REV)` — 31416 is
1000×π to three decimal places, again to stay in integers). Multiplying
by 1000 before dividing shifts the units from metres/second to
millimetres/second while the whole calculation stays in whole-number
(integer) arithmetic the entire way through — there's no FPU on this
chip, so a division can never be left half-finished as a fraction; it has
to land on a usable integer immediately. Since a single-channel encoder
disc physically cannot tell *which way* the wheel is turning (there's no
second, phase-offset sensor the way a proper quadrature encoder would
have), the function borrows direction from `drv_motor_get(side)` in
`drv_motor.c` instead — if the motor was last told to run in reverse, the
speed is reported negative. This is correct almost all the time, but
briefly wrong for the fraction of a second right after a direction
reversal while the wheel is still physically coasting the old way — worth
remembering when tuning PID gains, since a reversal can look like a
sudden, real speed error to the controller.

**`drv_encoder_distance_mm(side)`** — `(count - base_count)` converted to
millimetres using the same `RC_ENC_UM_PER_TICK` constant. This is the
number `sub_motion.c`'s `MODE_DISTANCE` state watches every cycle to know
when a "drive forward 300 mm" move has actually covered 300 mm.

**`drv_encoder_reset(void)`** — snapshots the current lifetime count of
both wheels into `base_count`, inside a critical section. This does *not*
reset the lifetime `count` — it just moves the zero-point for
`drv_encoder_distance_mm()`, so each new move in `sub_motion.c` can
measure "how far did *this* move go" starting fresh from zero, called at
the top of `start_move()` below.

#### `sub_motion.h` / `sub_motion.c` — the brain that ties it together

Where the previous two files only know "spin this hard" and "here's how
fast that wheel is currently going," this file is the layer that actually
decides *what* duty to send, moment to moment, to make the car do what
the rest of the team asked for — "drive forward 300 mm," "turn 90°,"
"drive with this much steering bias." The public API is non-blocking (see
§0.3 up top): calling `sub_motion_forward_mm(300, callback, ctx)` returns
immediately with a move ID while the car drives in the background on its
own RTOS task; when the move finishes, your callback fires and an
`RC_EVT_MOTION_DONE` event is published on the event bus for anyone else
listening.

**The `mode_t` enum** — the car's current "what am I doing right now"
state: `MODE_IDLE` (motors off, nothing queued), `MODE_DISTANCE` (driving
straight toward a distance goal under PID speed control),
`MODE_TURN` (turning toward a target angle — currently a stub, see the
TODO below), `MODE_CONTINUOUS` (open-ended drive+steer with no goal,
the mode the line follower uses almost the entire time the car is
running).

**The `pid_t` struct and PID gain constants** — before the struct itself,
two `#define`s that shape every number inside it:
- `#define PID_SCALE (256)` — since there's no FPU, a PID gain that
  "should" be something like `2.5` is instead stored as a whole integer
  that's been multiplied by 256 (so `2.5` becomes `640`), and `pid_step()`
  divides the final combined result by `PID_SCALE` once at the end to
  undo the scaling. This is called **fixed-point arithmetic**: using
  ordinary integers to represent fractional values by agreeing, everywhere
  in the code, on an implicit scale factor.
- `#define INTEGRAL_CLAMP (200000)` — caps how large the PID integral
  term's running total (explained below) is allowed to grow in either
  direction. Without this, a wheel that's stalled or physically blocked
  would let the integral term climb without limit while the error stays
  nonzero — a problem called **integral windup** — and the instant the
  wheel is freed, all that pent-up correction would slam it with a sudden
  burst of power. Two independent instances of `pid_t` exist,
  `pid_l`/`pid_r`, because each wheel can need a slightly different
  amount of push to hit the same target speed (motor variance, friction,
  floor grip all differ side to side).

Fields inside `pid_t`: `kp`/`ki`/`kd` are the three tuning gains
(fixed-point, scaled by `PID_SCALE` as above); `integral` is the running
memory of past error, accumulated over time; `prev_err` is last cycle's
error, kept so the controller can tell how fast the error is changing.

**`pid_reset(p)`** — zeroes `integral` and `prev_err`. Called whenever a
fresh move starts (inside `start_move()` below), so leftover memory from
a *previous* move can't bleed into a brand-new one.

**`pid_set_gains(p, kp, ki, kd)`** — sets the three gains and calls
`pid_reset()`. Called from `sub_motion_init()` with the current
placeholder values — see the TODO section below for the tuning procedure.

**`pid_step(p, err, dt_ms)` — the PID control math itself.** This is the
heart of the whole module, so it's worth walking through line by line.
The analogy: a car's cruise control doesn't know in advance exactly how
much throttle produces exactly 60 km/h — that number changes with hills,
wind, and load. Instead it constantly measures your *actual* speed,
compares it to the speed you *asked for*, and nudges the throttle based
on the gap. `err` (computed by the caller as `target speed - actual
speed`) is that gap for this one wheel this one cycle: positive means
"going too slow, push harder," negative means "going too fast, ease off."
Each cycle combines three separate reactions to that same error:

1. **Proportional (P, uses `kp`)** — react to how wrong we are *right
   now*. This term alone is just `kp * err`: bigger error, bigger
   correction, applied immediately, with no memory of the past. On its
   own, a pure-P controller typically settles a little short of the
   target (because as the error shrinks, so does the push correcting
   it) — which is exactly the gap the next term exists to close.
2. **Integral (I, uses `ki`)** — react to how long/consistently we've
   been wrong. In code: `p->integral += (err * dt_ms)`, then clamped to
   `±INTEGRAL_CLAMP` as described above. If a wheel is *always* running a
   little slow even with a steady proportional push (say, because of
   constant extra friction the P term alone can't fully cancel), this
   running total keeps growing and keeps adding extra push until that
   steady, lingering gap disappears. It's the term that catches "small
   but persistent" errors that the proportional term, by design, can
   never fully close on its own.
3. **Derivative (D, uses `kd`)** — react to how fast the error is
   *changing*, to avoid overshoot. In code:
   `d = (err - p->prev_err) * 1000 / dt_ms` — "how much did the error
   change" divided by "how much time passed," scaled from
   per-millisecond to per-second while staying in integer math (guarded
   against `dt_ms == 0` to avoid a divide-by-zero). If the error is
   closing very quickly — the wheel is racing toward the target speed —
   this term leans *against* the proportional/integral push, the same way
   a driver eases off the accelerator just before reaching their target
   speed instead of slamming straight past it and having to correct back
   the other way.

The three terms are then combined and un-scaled in one line:
`out = ((kp * err) + ((ki * integral) / 1000) + (kd * d)) / PID_SCALE`.
The integral term gets an extra `/1000` because it accumulates
`err * dt_ms` (a larger-scale running total) rather than a single
instantaneous `err`, so it needs its own extra scale-down before joining
the other two terms. `out` is the new motor duty for that wheel, handed
straight to `drv_motor_set_pair()` back in `motion_task()`.

**`finish_move(completed)`** — the shared cleanup path used both when a
move finishes normally and when it's cut short (stopped, or pre-empted by
a newer move request). It copies the current callback/context/move-id out
to local variables *before* clearing the globals (because calling the
callback might indirectly trigger a brand-new move that would otherwise
overwrite those globals while this function is still using them);
averages both wheels' `drv_encoder_distance_mm()` into one "how far did
the car actually go" figure (averaging smooths out one wheel having
slipped slightly more than the other); resets `mode` to `MODE_IDLE`;
brakes the motors; publishes `RC_EVT_MOTION_DONE` on the event bus (so
*any* subsystem can react to a move finishing, not just whoever started
it — e.g. telemetry logging it); and finally calls the caller's own
callback function, if one was given.

**`start_move(m, target, cb, ctx)`** — shared setup used by every
"queue a move" entry point. If a move is already running it's pre-empted
first (`finish_move(false)` — "no, that one didn't complete normally, a
newer request took over"). It then resets the encoders
(`drv_encoder_reset()`, so this move's distance starts at zero — see
`drv_encoder.c` above) and both wheels' PID state (`pid_reset()`, so
stale integral/derivative memory from a previous move can't leak in),
records the caller's target/callback/context, hands out a fresh move ID
(wrapping past `0`, since `0` is reserved to mean "no active move"), and
finally switches `mode` into the requested state — at which point
`motion_task()`, described next, takes over.

**`motion_task(stacd, exinf)` — the move state machine.** This is the
background RTOS task created in `sub_motion_init()` that does *all* of
the actual driving; every public function above just sets state that
this loop reads. It's an infinite `for (;;)` loop that calls
`tk_dly_tsk(RC_PERIOD_MOTION_MS)` at the top of every iteration —
`RC_PERIOD_MOTION_MS` is 20 ms (50 Hz) from `rc_config.h`, and
`tk_dly_tsk` is a genuine RTOS sleep: the task is fully parked and other
tasks get the CPU during that 20 ms, not a busy-wait loop (see §0.3's
non-blocking rule, applied here at the task level rather than the
API-call level).

Every single cycle, regardless of mode, the task reads both wheels'
current speed via `drv_encoder_speed_mm_s()` (this is the same function
Buddy 4's terrain-detection code also reads, to help classify whether the
car is turning — see that module's section for how) and publishes a
fresh `RC_EVT_ODOMETRY` event with both wheels' speed and distance, so
anything listening (telemetry, terrain detection) always has current
numbers even while the car is simply idling.

Then it branches on `mode`:

- **`MODE_DISTANCE`** — averages both wheels' distance-so-far; if that's
  reached `goal_mm`, calls `finish_move(true)` (a real, successful
  completion) and stops. Otherwise it runs one `pid_step()` call *per
  wheel* — not once for an averaged speed — with `err = target_mm_s -
  measured speed` for that wheel, and sends the two results straight to
  `drv_motor_set_pair()`. Running PID independently per wheel (rather
  than once for the pair) is what keeps the two wheels matched in speed
  even when one side has more friction or a weaker motor than the other —
  each wheel gets exactly the push it individually needs.
- **`MODE_TURN`** — currently a stub: it just calls `finish_move(true)`
  immediately without actually turning the wheels or measuring an angle.
  The block comment directly above it in the source spells out the real
  plan: arc length for one wheel spinning about the car's centre over
  `deg` degrees is `RC_WHEEL_BASE_MM * pi * deg / 360`
  (`RC_WHEEL_BASE_MM`, from `rc_config.h`, is the centre-to-centre
  distance between the two wheels — needed here because during an
  in-place turn, each wheel effectively travels along a circle of that
  radius); convert that arc length to encoder ticks using
  `RC_ENC_UM_PER_TICK` (the same constant `drv_encoder.c` uses for
  straight-line distance); drive the two wheels in opposite directions;
  stop once both sides have accumulated that many ticks; and because
  wheels slip when spinning in place (unlike rolling forward), expect the
  real turn to fall consistently short of the pure-geometry number and
  fold a measured correction factor in rather than trusting the math
  alone. This is flagged as a required TODO — see below.
- **`MODE_CONTINUOUS`** — the mode the line follower uses almost
  constantly. Steering is applied as a simple **open-loop** differential
  on top of the closed-loop base speed: `out_l = base_cmd - steer_cmd`,
  `out_r = base_cmd + steer_cmd`. "Open-loop" here means no feedback
  corrects the *steering* itself (unlike the PID speed control in
  `MODE_DISTANCE`, which *is* closed-loop) — the code just computes the
  two duty values directly and sends them. A positive `steer_cmd` (steer
  right) speeds up the left wheel and slows the right one, pivoting the
  car rightward, and vice versa. The code comment explains why steering
  isn't *also* closed-loop here: independently closing the speed loop on
  each wheel while simultaneously trying to steer them apart fights
  itself (each wheel's PID would fight the steering bias, treating it as
  an error to correct away) — this simpler arrangement avoids that fight
  and works well enough in practice.
- **`MODE_IDLE`** (and the `default` case) — does nothing; the motors are
  already stopped.

**Public API, briefly (each wraps the machinery above):**
- `sub_motion_init(void)` — seeds both wheels' PID gains with the current
  placeholder values (`512, 64, 16` — see the TODO below), sets `mode` to
  idle, and creates + starts `motion_task` as an RTOS task at priority
  `RC_PRI_MOTION`. Must run once at boot before anything else here.
- `sub_motion_set_speed(mm_s)` — updates the cruise-control "set speed"
  dial used by `MODE_DISTANCE` (`target_mm_s`); has no effect on
  `MODE_CONTINUOUS`, which is driven directly by whatever
  `sub_motion_drive()` was last called with.
- `sub_motion_forward_mm` / `sub_motion_backward_mm(mm, cb, ctx)` — both
  currently call `start_move(MODE_DISTANCE, mm, cb, ctx)` — see the TODO
  below about backward moves not yet having real sign handling.
- `sub_motion_turn_deg(deg, cb, ctx)` — takes the absolute value of `deg`
  (via a ternary: `mag = (deg < 0) ? -deg : deg`) and calls
  `start_move(MODE_TURN, mag, cb, ctx)`; backed by the `MODE_TURN` stub
  above, so the request is accepted but the car doesn't actually turn yet.
- `sub_motion_drive(base_permille, steer_permille)` — refuses to interrupt
  an in-progress `MODE_DISTANCE` move (returns `RC_ERR_BUSY`), otherwise
  stores the two values and switches to `MODE_CONTINUOUS`.
- `sub_motion_stop(brake)` — pre-empts any running move via
  `finish_move(false)`, then calls `drv_motor_stop()` with either
  `RC_MOTOR_BRAKE` or `RC_MOTOR_COAST` depending on the flag.
- `sub_motion_busy(void)` — true only during `MODE_DISTANCE` or
  `MODE_TURN`; false during continuous drive or idle, since neither of
  those has a defined "finished" point to be busy toward.

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

This is the longest walkthrough in the guide because this module leans on
several C ideas (pointers, bitwise operators, function pointers, ISRs,
ring buffers) that nobody on the team is expected to already know. Every
time one of those shows up below, it's explained in plain terms the first
time, then referred back to afterwards. If you haven't yet, skim
TEAM_GUIDE.md §0 (event bus, publish/subscribe, the two lanes, the
non-blocking pattern, and why everything uses "permille" whole numbers
instead of fractions) — this section assumes you know those already and
won't re-explain them.

#### `drv_ir.h` / `drv_ir.c` — the hardware layer

This is the bottom layer: it only talks to the physical sensors and knows
nothing about "line following" or "barcodes" as concepts — that
intelligence lives one layer up, in `sub_line.c` and `sub_barcode.c`.

**`IR_ACTIVE_HIGH`** (`drv_ir.c`, near the top)
```c
#define IR_ACTIVE_HIGH      (1)
```
What it does: a compile-time switch. `#define` creates a *macro* — a
find-and-replace rule the compiler applies before it even starts
compiling. Anywhere the code writes `IR_ACTIVE_HIGH`, the compiler
substitutes `(1)` first. Further down, `drv_ir_on_line()` uses
`#if IR_ACTIVE_HIGH ... #else ... #endif` — that's a *preprocessor*
conditional, resolved before compilation, not a runtime `if`. Whichever
branch doesn't match gets deleted before the compiler even sees it, so
there's no runtime cost to supporting both sensor wirings.
Why: different IR sensor boards report "on the line" in opposite ways —
some pull the output pin HIGH over black, some pull it LOW. Rather than
hard-code one behaviour, this macro lets you flip a single `1`→`0` to
match your hardware.
Links to other code: read by `drv_ir_on_line()` immediately below it.
Getting this wrong is called out explicitly in "How to get started" step
2 above as the single most common way to make the car drive off the
table with total confidence, because every other function in this module
trusts whatever `drv_ir_on_line()` reports.

**`ADC_DEVNAME`**
```c
#define ADC_DEVNAME         ((UB *)"adca")
```
What it does: names the ADC (analogue-to-digital converter — the chip
hardware that turns a sensor's continuously-variable voltage into a
plain number the CPU can read and compare) device this RTOS should open.
`(UB *)` in front of the string is a *cast* — a explicit instruction to
the compiler "treat this piece of data as if it were this other type."
Here it tells the compiler to treat the text `"adca"` (normally a
`char *`) as a `UB *` (unsigned byte pointer), because that's the type
this particular RTOS's device-open function expects. A cast doesn't
change the underlying bytes, only how the compiler is willing to use
them.
Why: needed once, to open the device in `drv_ir_init()`.
Links to other code: used only by `tk_opn_dev(ADC_DEVNAME, ...)` inside
`drv_ir_init()`.

**Module-level `static` variables** (`adc_dd`, `bar_cb`, `bar_ctx`,
`bar_last_us`, `bar_enabled`, `defer_h`)
What they do: hold this module's private, persistent state — the ADC
device handle, an optional extra callback for raw barcode edges, the
timestamp of the last barcode edge, whether the barcode interrupt is
currently armed, and a handle for the deferred "bottom half" worker
(explained below).
Why `static`: at file scope (outside any function), `static` means "this
name is only visible inside this .c file." It's how C fakes "private"
member variables without having classes — no other file can accidentally
read or clobber `bar_enabled`, for example. This matters a lot in a
codebase with five people editing five different files: `static` is what
stops your file's `enabled` from colliding with `sub_line.c`'s `enabled`.
Why `volatile` on some of them (`bar_last_us`, `bar_enabled`, and the
ring buffer indices further down): these are written by an ISR
(interrupt handler, explained below) and read by ordinary code, or vice
versa, at moments the compiler can't predict from the plain flow of the
program. `volatile` tells the compiler "never assume this value hasn't
changed since you last checked it — always re-read it from memory,
don't cache it in a CPU register or optimize the read away." Without
`volatile` here, an aggressive compiler optimization could make code
that waits on `bar_enabled` loop forever, because it "proved" the
variable's value inside the loop never changes — except it does, from
inside the interrupt.

**The barcode edge ring buffer** (`BAR_RING_SZ`, `BAR_RING_MASK`,
`bar_edge_rec_t`, `bar_ring`, `bar_head`, `bar_tail`, `bar_overrun`)
```c
#define BAR_RING_SZ     (32U)
#define BAR_RING_MASK   (BAR_RING_SZ - 1U)
```
What it does: sets up a *ring buffer* (also called a circular buffer) — a
fixed-size array used as a queue. A "head" index marks where the next
item gets written; a "tail" index marks where the next item gets read
from. Both wrap back to 0 once they reach the end of the array, so the
same 32 slots get reused forever instead of the array growing.
Why: the barcode ISR (interrupt) discovers new bar/space edges instantly,
but decoding them is comparatively slow work that must not happen inside
an interrupt (see the ISR explanation below). The ring buffer is the
hand-off point — the ISR writes into it in microseconds, and the
"bottom half" task drains it at its own pace, moments later, without the
two ever needing to be perfectly synchronized.
Why `BAR_RING_SZ` is a power of two (32, not e.g. 30): so that wrapping
an index can be done with a single bitwise-AND (`index & BAR_RING_MASK`)
instead of a division (`index % BAR_RING_SZ`). `BAR_RING_MASK` is
`32 - 1 = 31`, which in binary is `0b00011111` — ANDing any number with
that keeps only its lowest 5 bits, which is mathematically identical to
`% 32` but much cheaper for a small microcontroller like the RP2040 to
compute (no division instruction needed). This bit trick — "AND with
size-minus-one instead of modulo" — only works when the size is a power
of two, which is why 32 was chosen rather than a rounder-sounding 30.
The bitwise operators here: `&` (AND) keeps bits that are 1 in *both*
operands; you'll see `|` (OR, sets a bit), `<<` (left shift, "multiply by
2 and make room for a new low bit"), and `>>` (right shift) elsewhere in
this module and elsewhere in the codebase — think of shifting as sliding
all the bits sideways, with zeros filling in behind.
`bar_edge_rec_t` is a `struct` — a bundle of related fields (here,
`width_us` and `level`) given one name so they can be stored and passed
around together, like a labelled envelope instead of two loose sheets of
paper.
Links to other code: written by `barcode_isr()`, read by
`barcode_drain()`, both below.

**`barcode_isr()`** — the barcode interrupt handler
What it literally does, line by line: computes `width` as the time
elapsed (in microseconds) since the previous edge; stores the new
timestamp as the baseline for next time; computes where the *next* ring
write would land (wrapping with the `& BAR_RING_MASK` trick above)
without committing to it yet; if that landing spot is the tail (meaning
the ring is completely full and the consumer hasn't caught up), counts
an overrun and bails out rather than overwriting unread data; otherwise
writes the width and level into the current head slot, advances head,
and calls `rc_defer_signal_i(defer_h)` to wake up the bottom-half task.
Why it's written this way: this is an ISR (Interrupt Service Routine) —
a tiny function the RP2040 jumps to automatically, pausing whatever else
the chip was doing, the instant the barcode sensor's output pin changes
voltage. TEAM_GUIDE.md §0.2's "golden rule" applies in full force here:
an ISR may only record a timestamp, flip a couple of registers, and hand
off — never loop, never publish an event directly, never do anything
that could take an unpredictable amount of time. Bar width *is* the
actual barcode data (see the Code 39 explanation below), so if this
handler were delayed by other work, the very timestamps it exists to
capture would be corrupted. Keeping the ISR to "a subtraction, a ring
write, and a signal" is what keeps that timing trustworthy.
Links to other code: signals `barcode_drain()` (the bottom half) via
`rc_defer_signal_i()`, whose `_i` suffix is this RTOS's naming
convention for "safe to call from inside an interrupt" (an ordinary
kernel call here could corrupt the scheduler's internal state, since the
scheduler wasn't expecting to be interrupted mid-operation).

**`barcode_drain()`** — the "bottom half"
What it does: loops `while (bar_tail != bar_head)`, meaning "while there
are unread entries in the ring," reading one entry at a time, advancing
the tail, building an `RC_EVT_BARCODE_EDGE` event from it, and
publishing that event — plus calling the optional extra `bar_cb`
callback if one was registered.
Why: this runs in ordinary task context, not inside an interrupt, so it
is allowed to do the comparatively slow work of building and publishing
an event — the exact work `barcode_isr()` was forbidden from doing
directly. This producer/consumer split (ISR produces fast, task consumes
whenever it gets scheduled) is the standard pattern for "I need
microsecond timing but also need to do real work with the data" on a
microcontroller.
Links to other code: every published `RC_EVT_BARCODE_EDGE` event is
what `sub_barcode.c`'s `on_edge()` subscribes to and reacts to — this is
the seam between the hardware layer and the decoding layer.

**`drv_ir_init()`**
What it does: registers `barcode_drain` as the deferred worker, sets both
line-sensor pins as plain digital inputs, attaches `barcode_isr` to the
barcode pin's interrupt but leaves it masked (disabled) until armed, and
opens the ADC device (allowed to fail without aborting init, since the
digital line-following path doesn't need it).
Why the barcode interrupt starts masked: see the reasoning under
`drv_ir_barcode_enable()` below — it's the same "don't decode track
noise as a fake barcode" concern.
Links to other code: called once at boot, before any other function in
this module is used.

**`drv_ir_on_line()`**
What it does: reads one sensor's raw digital pin value with
`gpio_get_val(pin)`, then applies the `IR_ACTIVE_HIGH` polarity
correction explained above to turn "raw HIGH/LOW" into "is this
genuinely over the black line, yes or no."
Links to other code: called by `drv_ir_sample_line()` below (for the two
line sensors) and directly by anyone testing sensor wiring (see "How to
get started" step 2).

**`drv_ir_read_raw()`**
What it does: reads the raw ADC brightness value (0–4095, a 12-bit
range) from the barcode sensor's analogue output, but only for
`RC_IR_BARCODE` — the two line sensors aren't wired to any ADC pin in
this design, so calling it for them just returns 0. `buf & 0x0FFFU` at
the end masks off everything except the lowest 12 bits, in case the ADC
hardware returns extra bits the code doesn't want.
Why: used purely for trim-pot calibration — reading actual brightness
numbers over white vs. black track to set the sensor's threshold.
Links to other code: also called internally by `drv_ir_sample_line()`
to attach the barcode channel's raw brightness onto every line-sample
event, for logging/telemetry even though it isn't used for steering
decisions.

**`drv_ir_sample_line()`**
What it does: reads both line sensors via `drv_ir_on_line()`, then
converts the two true/false readings into a single signed "position"
number using this table (also documented directly above the function in
the source):

| Left | Right | Meaning | position |
|---|---|---|---|
| off | off | line lost | keeps the previous sign (handled by sub_line.c, not here) |
| on | off | drifting right | -500 |
| off | on | drifting left | +500 |
| on | on | centred, or a junction | 0 |

...then publishes an `RC_EVT_LINE_SAMPLE` event carrying both raw
booleans, the position number, and the barcode sensor's raw brightness.
Why only four possible values: with two purely digital (on/off) sensors,
there are exactly four combinations of two booleans, so the "position"
this driver can report is necessarily coarse — this is the "4-state
position logic" the task description calls out, and it's exactly what
the TODO in `sub_line.c` (and in TEAM_GUIDE.md's "Missing/TODO" list) is
about upgrading to something continuous.
Links to other code: called every 5ms (`RC_PERIOD_LINE_MS`) from the
sensing task; its published event is what `sub_line.c`'s `on_sample()`
reacts to — this is the seam between the hardware layer and the
steering layer, exactly parallel to the barcode edge seam above.

**`drv_ir_on_barcode_edge()`**
What it does: stores an optional extra callback function, called
directly (from interrupt context — so it must be just as fast as the ISR
itself) for every raw edge, in addition to the event that's always
published. Most code should subscribe to the event instead; this exists
for anything needing lower latency than an event-bus hop.

**`drv_ir_barcode_enable()`**
What it does: when turning the barcode interrupt ON, first resets the
timing baseline (`bar_last_us`) and both ring indices to zero, so no
stale timestamp or leftover ring data from a previous arm/disarm cycle
can leak into the first reading; then enables (or disables) the actual
hardware interrupt via `rc_gpioirq_enable()`.
Why: the barcode path is deliberately left masked except when a barcode
read is actually expected (armed by `sub_line.c` reporting a junction —
see below). Leaving it always-on would mean ordinary shiny patches of
track, or the car simply idling, could generate spurious edges that the
decoder would try to interpret as a real barcode character.
Links to other code: called both directly, and indirectly via
`sub_barcode_arm()` in `sub_barcode.c`, which is itself triggered by
`sub_line.c` reaching `RC_LINE_JUNCTION` state.

#### `sub_line.h` / `sub_line.c` — steering decisions

**The `rc_line_state_t` enum** (`sub_line.h`)
```c
typedef enum {
    RC_LINE_TRACKING = 0,
    RC_LINE_LOST,
    RC_LINE_SEARCHING,
    RC_LINE_JUNCTION
} rc_line_state_t;
```
What it is: an `enum` (enumeration) — a small set of named whole-number
constants. `RC_LINE_TRACKING` is 0 (because it's written `= 0`
explicitly), and the rest count upward automatically (1, 2, 3).
Why use an enum instead of plain numbers: `state == RC_LINE_LOST` reads
as English; `state == 1` would force anyone reading the code to go look
up what "1" means. It also lets the compiler catch typos that plain
integers wouldn't (assigning an unrelated enum type to this variable is
a compile error; assigning a random int usually isn't caught).
The four states, in plain terms:
- `RC_LINE_TRACKING` — normal driving, at least one sensor currently
  sees the line, actively steering.
- `RC_LINE_LOST` — neither sensor has seen the line for
  `LOST_THRESHOLD` samples in a row; the car is now arcing, searching.
- `RC_LINE_SEARCHING` — the car deliberately left the line on purpose
  (e.g. driving around an obstacle under `sub_scan`'s control) and is
  watching for it to reappear, without treating the absence as an
  error.
- `RC_LINE_JUNCTION` — both sensors have seen black for
  `JUNCTION_THRESHOLD` samples in a row, which almost always means the
  car has reached a junction or the start of a barcode, not that it's
  perfectly centred on a normal line.
Links to other code: `sub_barcode_arm()` gets called (from the shared
`sub_nav.c` mission logic — see TEAM_GUIDE.md §0.4) when this state
becomes `RC_LINE_JUNCTION`, arming the barcode decoder at exactly the
moment it's likely to be needed.

**`LOST_THRESHOLD` and `JUNCTION_THRESHOLD`** (`sub_line.c`)
```c
#define LOST_THRESHOLD      (20U)
#define JUNCTION_THRESHOLD  (6U)
```
What they do: `LOST_THRESHOLD` is how many consecutive "neither sensor
sees the line" samples must happen before the code gives up and declares
the line genuinely lost, rather than reacting to a single blip.
`JUNCTION_THRESHOLD` is the same idea for "both sensors see the line" —
how many consecutive samples before that's treated as a real junction
rather than a momentary coincidence.
Why these specific values: the comment above `LOST_THRESHOLD` in the
source works the math out explicitly — at 5ms per sample (the line
sensor poll rate), 20 samples is 100ms, and at a typical 250mm/s driving
speed that's about 25mm of travel: short enough that the car reacts
quickly to a genuinely lost line, but long enough to smoothly ride over
small gaps or momentary sensor noise without false-triggering a
"searching" arc. `JUNCTION_THRESHOLD` at 6 samples (30ms) is shorter,
because a junction should be recognized promptly so the barcode decoder
gets armed with plenty of lead time before the barcode itself arrives.
This "count N consecutive matching samples before reacting" pattern is
called *debouncing* — the same idea used for encoder pulses in Buddy 2's
module and button presses in general embedded programming, to filter out
noise that only ever lasts one or two samples.
The `U` suffix on `20U` / `6U`: tells the compiler this is an *unsigned*
integer literal, matching the `uint32_t lost_count` / `both_count`
counters it gets compared against. Comparing a signed and unsigned
number in C can silently produce wrong results in edge cases (the signed
one gets converted to unsigned, and a negative number becomes a huge
positive one) — matching the suffix to the variable type sidesteps that
whole class of bug.

**`STEER_KP`** — this is the exact constant flagged for special
attention, so here is the full explanation at the requested level of
detail:
```c
#define STEER_KP        (2)
```
- **What it does / what KP means:** "KP" stands for "K, proportional" —
  the conventional name in control theory for the multiplier on a
  proportional-control term. This is the same idea as Buddy 2's PID
  speed controller (`sub_motion.c`, `pid_step()`) simplified down to just
  the "P" — there's no integral or derivative term here (see the TODO
  below). The general idea of proportional control: the further off you
  are from where you want to be, the harder you correct, in direct
  proportion to how far off you are. A real-world analogy: steering a
  shopping cart back toward the centre of an aisle — drift a little, nudge
  the cart gently; drift a lot, turn the wheel harder. `STEER_KP` is the
  "how hard do I nudge, per unit of drift" number.
- **What it's multiplied by:** the `position` value reported by
  `drv_ir_sample_line()` in `drv_ir.c` — the raw sideways error described
  in the 4-state table above (-500, 0, or +500, since there are only two
  digital sensors). This happens inside `steer_from_position()`:
  ```c
  int16_t steer = (int16_t)((int32_t)position * STEER_KP / 2);
  ```
  Reading this line: `position` gets cast up to `int32_t` (a wider,
  32-bit signed integer) before multiplying, specifically so that
  `position * STEER_KP` — which could reach 500 × 2 = 1000, still small,
  but the pattern matters for future larger gains — cannot silently
  overflow a 16-bit value before being scaled back down. This is a real,
  common C bug: multiplying two `int16_t` values together happens in
  16-bit arithmetic and can wrap around to a wrong (even negative-looking)
  result long before you'd expect it to, so widening to `int32_t` first
  is a defensive habit worth learning early. The `/ 2` at the end scales
  the result back down into the same -1000..1000 "permille" range that
  `sub_motion_drive()`'s steering parameter expects (see below). Only
  after all that arithmetic is safely back in range does the final
  `(int16_t)` cast narrow it back down to the type `steer_from_position()`
  hands off.
- **Why the value is 2:** it was picked as a small whole number that
  produces a firm, noticeable steering correction without saturating the
  output. At the sensors' maximum reported drift (`position = 500`) with
  `STEER_KP = 2`, the formula above gives `steer = 500 * 2 / 2 = 500` —
  half of the full ±1000 permille steering range: assertive, but not
  maxed out, leaving headroom. Like every constant in this codebase, it
  must be a whole number because the RP2040 has no hardware
  floating-point unit (see TEAM_GUIDE.md §2) — a "gain of 2.0" happens to
  be convenient here because it's already a whole number, but note the
  formula divides by 2 afterward specifically so that non-whole-feeling
  gains could still be approximated with integer tricks if this ever
  needed tuning to something like "1.5×" (e.g. `STEER_KP = 3`, still
  divided by 2, gives an effective ×1.5).
- **What happens if you raise it:** the car reacts more aggressively —
  it snaps back toward centre faster when it drifts, which sounds
  purely good, but a P-only controller with too high a gain overshoots:
  it corrects so hard that it swings past centre onto the *other* side,
  then has to correct back, then overshoots again — visible on the track
  as the car weaving side to side even on straight, well-marked line. This
  is exactly the "classic gain-too-high" behaviour that a derivative term
  (see the TODO comment directly above `STEER_KP` in the source) exists
  to damp out in a full PID controller; this module doesn't have one yet.
- **What happens if you lower it:** the correction becomes gentler and
  smoother in a straight line, but the car takes longer to recover from
  drift and may run wide (cut the corner too shallow, or drift off the
  outside) on sharp curves, because it isn't steering hard enough soon
  enough.
- **How it connects to `steer_from_position()` / `on_sample()`:**
  `steer_from_position(position)` is the only place `STEER_KP` is used.
  It's called from `on_sample()` in two situations: every time exactly
  one sensor reports the line (the normal tracking branch), and during a
  brief dropout (fewer than `LOST_THRESHOLD` all-dark samples so far) —
  in that second case it's called with `last_position`, i.e. "keep
  steering as if the last known reading still holds," which lets the car
  ride over small physical gaps in the line without over-reacting.
  `steer_from_position()`'s output, `steer`, is passed straight into
  `sub_motion_drive(base_permille, steer)` — Buddy 2's module (see
  TEAM_GUIDE.md's Buddy 2 section) — which biases the left and right
  wheel motor speeds against each other to actually turn the car.
  `sub_line.c` never touches a motor directly; it only ever decides *how
  hard* to ask Buddy 2's module to steer.

**`steer_from_position()`**
What it literally does: computes `position * STEER_KP / 2` in widened
32-bit arithmetic (explained fully under `STEER_KP` above), narrows the
result back to `int16_t`, and calls `sub_motion_drive(base_permille,
steer)`, ignoring its return value with a `(void)` cast (a standard C
idiom meaning "I know this returns something, I'm deliberately not
checking it here" — it silences a compiler warning about an unused
result, not an error).
Why it's a separate function rather than inlined: it's called from two
different places in `on_sample()` (normal tracking, and brief dropouts),
so factoring it out avoids repeating the formula.

**`on_sample()`** — the core decision function, called automatically
every ~5ms via the event bus (see TEAM_GUIDE.md §0.2) whenever
`drv_ir_sample_line()` publishes a new `RC_EVT_LINE_SAMPLE`.
Walking through it branch by branch:
1. **`if (!enabled) return;`** — if `sub_line_enable(false)` was called
   (e.g. because obstacle avoidance currently owns the wheels), do
   nothing at all — not even update the internal counters. This is what
   lets `sub_scan.c` take over driving cleanly without `sub_line.c`
   fighting it for control of the motors.
2. **Both sensors on the line** (`s->on_line_l && s->on_line_r`) — bumps
   `both_count`, resets `lost_count` to 0 (we clearly haven't lost the
   line), and once `both_count` reaches `JUNCTION_THRESHOLD`, transitions
   into `RC_LINE_JUNCTION` state. Regardless of whether the threshold has
   been reached yet, it drives straight (`steer = 0`) through — a
   sensible default while it's still ambiguous whether this is a real
   junction or just a wide bit of line.
3. **Exactly one sensor on the line** (`s->on_line_l || s->on_line_r`,
   having already ruled out "both") — the normal tracking case. Resets
   both counters (neither "junction" nor "lost" applies right now), and
   if the state was previously `RC_LINE_SEARCHING`, publishes
   `RC_EVT_LINE_REACQUIRED` first (telling e.g. `sub_scan.c` that the
   search for the line succeeded) before switching to
   `RC_LINE_TRACKING`. It remembers this position as `last_position` (in
   case the line vanishes on the very next sample) and calls
   `steer_from_position()` with the live reading.
4. **Neither sensor on the line** (the final `else`) — resets
   `both_count`, increments `lost_count`. If the car is deliberately
   `RC_LINE_SEARCHING`, it returns immediately without steering or
   declaring anything lost — whatever `sub_scan.c` commanded the motors
   to do stays in effect. Otherwise, once `lost_count` reaches
   `LOST_THRESHOLD`, it transitions to `RC_LINE_LOST` (publishing
   `RC_EVT_LINE_LOST`, but only on the *transition* into that state —
   the `if (state != RC_LINE_LOST)` guard stops the event from firing
   again every single sample while still lost) and starts arcing: half
   speed, steering hard (±400 permille) toward whichever side
   `last_position` was leaning, using the ternary operator
   `condition ? a : b` (a compact inline if/else that evaluates to one
   value or the other) to pick the direction. The reasoning, straight
   from the source comment: a car that drives dead straight after losing
   the line rarely finds it again by luck, but one that arcs back toward
   where it last saw the line usually does — because the line is a
   continuous path, and the car was on it a moment ago. If `lost_count`
   hasn't reached the threshold yet, it's treated as a brief dropout and
   handled the same as a normal sample, just using `last_position`
   instead of a fresh reading — this is what lets the car ride smoothly
   over small physical gaps in the printed line without over-reacting.

**`sub_line_init()` / `sub_line_enable()` / `sub_line_begin_search()` /
`sub_line_state()` / `sub_line_set_base()`**
- `sub_line_init()` subscribes `on_sample()` to `RC_EVT_LINE_SAMPLE` on
  the fast lane (steering must react quickly — see TEAM_GUIDE.md §0.2 on
  why fast vs. slow lanes exist) and resets state. Call once at boot.
- `sub_line_enable(on)` is the on/off switch for actually steering —
  state tracking continues either way, only the motor commands stop.
- `sub_line_begin_search()` tells the module "we left the line on
  purpose," switching straight into `RC_LINE_SEARCHING` so a deliberate
  detour isn't misread as an error.
- `sub_line_state()` is a simple getter other modules (`sub_nav.c`,
  telemetry) poll or react to.
- `sub_line_set_base(permille)` changes the cruising speed used while
  tracking — in permille (parts per thousand of full speed) rather than
  a percentage-with-decimals, again because there's no floating point on
  this chip.

#### `sub_barcode.h` / `sub_barcode.c` — Code 39 decoding

**Why widths, not a clock** (from the header's own top comment, worth
restating): the car's speed varies as it crosses a barcode, so the
absolute duration of a bar in milliseconds is meaningless on its own.
What stays constant regardless of speed is the *ratio* between a "wide"
bar and a "narrow" bar — Code 39 always keeps that ratio at roughly 2:1
or 3:1. Because the decoder works entirely off ratios rather than
absolute time, it works correctly whether the car crosses the barcode
fast or slow. This is what "ratio-based decoding" means, and it's the
single idea the rest of this section builds on.

**`sub_barcode_cb_t`** (`sub_barcode.h`)
```c
typedef void (*sub_barcode_cb_t)(char symbol, rc_nav_cmd_t cmd, void *ctx);
```
What it is: a *function pointer type*. Reading C function-pointer syntax
is genuinely one of the odder corners of the language — the trick is to
read from the inside out: `(*sub_barcode_cb_t)` says "`sub_barcode_cb_t`
is a pointer to a function," and the surrounding pieces say that function
takes `(char, rc_nav_cmd_t, void *)` and returns nothing (`void`). Once
declared, any ordinary function matching that exact shape — like
`symbol_to_cmd`'s caller pattern, or any handler another subsystem
writes — can be stored in a variable of this type and *called through
it* later, without the code that stores it needing to know in advance
which specific function it will be.
Why: this is how `sub_barcode.c` lets other subsystems (`sub_nav.c`,
telemetry, or your own test code) register their own "a character was
decoded" handler without `sub_barcode.c` needing a compile-time
`#include` of, or any knowledge about, those other files. It's the same
role a callback plays in JavaScript (`button.addEventListener(...)`) or
Python (passing a function as an argument) — C just needs an explicit
type declaration to describe the function's "shape" first.
Links to other code: the type of the `cb` parameter to
`sub_barcode_on_decode()`, and of the `user_cb` variable that gets called
inside `on_edge()`.

**`ELEMENTS` and `WINDOW`**
```c
#define ELEMENTS        (9U)
#define WINDOW          (10U)
```
What they mean: Code 39 encodes one character as 9 elements — alternating
bars and spaces — of which exactly 3 are "wide" and 6 are "narrow."
`ELEMENTS` is that count. `WINDOW` (10) is one bigger: the code keeps a
sliding buffer of up to 10 recent widths so that after a failed decode
attempt it can discard just the oldest one and retry with the next 9,
rather than throwing away everything and waiting for a completely fresh
batch — this "slide by one and retry" strategy handles the case where the
code's buffer just happened to start one element early or late.

**`ELEM_MIN_US` / `ELEM_MAX_US`**
```c
#define ELEM_MIN_US     (800UL)
#define ELEM_MAX_US     (200000UL)
```
What they do: any measured width outside this microsecond range (800µs
to 200,000µs = 0.8ms to 200ms) is treated as implausible — not a real
bar or space at all — and the whole collection window is reset. Why:
these bounds filter out two different kinds of bad data: electrical
noise causing extremely short spurious edges (below `ELEM_MIN_US`), and
long idle stretches of plain track where nothing is happening (above
`ELEM_MAX_US`, which would otherwise sit in the buffer looking like one
enormous "element"). The `UL` suffix marks these as `unsigned long`
(32-bit unsigned) literals, matching the `uint32_t width_us` values they
get compared against — the same signed/unsigned-matching discipline
explained under `LOST_THRESHOLD` above.

**`code39_t` struct and `table[]`**
```c
typedef struct {
    char     symbol;
    uint16_t pattern;
} code39_t;
```
What it is: bundles one printable character together with the 9-bit
number that represents its wide/narrow pattern. `table[]` is an array of
these — the lookup table the decoder searches. `TABLE_N` computes the
array's length at compile time as `sizeof(table) / sizeof(table[0])`
(total size in bytes, divided by one entry's size), so adding rows to
the table later automatically keeps the count correct without anyone
needing to update a separate number by hand.
**Important, flagged as a TODO both here and in the file:** only `'*'`
(the mandatory start/stop guard character) and `'A'` have their real,
correct 9-bit patterns in the table right now. `'B'`, `'C'`, and `'D'`
are explicitly marked `TODO: placeholder` in the source and are simply
wrong — some even duplicate `'A'`'s pattern by coincidence. Decoding
will silently fail or produce the wrong character for these three until
someone looks up the real, standard Code 39 bit patterns and replaces
them. Do this before testing any barcode that includes B, C, or D.

**`symbol_to_cmd()`**
What it does: a `switch` statement (a clean way to write "check one
value against several possibilities," clearer here than a chain of
`if`/`else if`) mapping each decoded character to a driving instruction:
`'A'` → turn left, `'B'` → turn right, `'C'` → go straight, `'D'` →
U-turn, anything else → no command. This mapping comes directly from the
project brief, not from anything about Code 39 itself — Code 39 just
carries characters; what those characters *mean* to this particular car
is this function's decision.

**`classify()`** — the heart of ratio-based decoding, explained step by
step, matching what the earlier inline comments in `sub_barcode.c` now
say:
1. **Find the shortest and longest width** in the current window of 9
   buffered widths (`min_w`, `max_w`). The code doesn't know the car's
   speed in advance, so it can't compare against some fixed "anything
   over 20ms is wide" rule — instead it works out short vs. long
   *relative to this specific window*, which is what makes the decoder
   speed-independent.
2. **Sanity check the spread:** if the longest element isn't at least
   double the shortest (`max_w < min_w * 2`), this window doesn't look
   like a real character yet (maybe it's straddling two characters, or
   it's noise) — bail out and let the caller slide the window and retry.
3. **Pick a threshold:** the midpoint between shortest and longest,
   `mid = (min_w + max_w) / 2`. Anything above the midpoint counts as
   "wide," anything at or below counts as "narrow."
4. **Classify and pack into one number:**
   ```c
   for (i = 0U; i < ELEMENTS; i++) {
       pattern <<= 1;
       if (widths[i] > mid) {
           pattern |= 1U;
           wide_count++;
       }
   }
   ```
   Reading this: `pattern <<= 1` shifts every bit already in `pattern`
   one place to the left (making room at the bottom for a new bit — the
   same left-shift idea explained under the ring buffer above, just used
   here to build up a value bit by bit instead of to wrap an index).
   Then, if this element was wide, `pattern |= 1U` (bitwise OR) turns on
   just the new bottom bit, leaving every bit already shifted in above it
   untouched. Doing this once per element, in order, packs all 9
   wide/narrow yes-or-no answers into the low 9 bits of a single 16-bit
   number — this is *bit-packing*: representing several small pieces of
   information as individual bits inside one larger number instead of as
   9 separate variables, so the whole character can be compared and
   looked up as one value. The very first element ends up in the
   highest of the 9 used bits (because it gets shifted left the most
   times by the elements that follow it), matching the "bit 8 is the
   first element" comment on the `pattern` field in `code39_t`.
5. **Final sanity check:** a legal Code 39 character always has exactly
   3 wide elements out of 9. If `wide_count` isn't exactly 3, the
   classification is rejected — this single check catches most
   misreads that made it past step 2.
Returns `false` (and leaves the output untouched) if either sanity check
fails; the caller (`on_edge()`) then slides the window forward by one
width and tries again, rather than giving up entirely.

**`lookup()`**
What it does: searches `table[]` for an exact match to the given
pattern. If found, writes the matching character through the `out`
pointer and sets `*reversed = false`. If not found, it builds the
*bit-reversed* version of the pattern (flips the order of all 9 bits —
so the element that was first becomes last and vice versa) and searches
again.
Why reversed matching matters: the car can physically cross a printed
barcode moving in either direction depending on approach angle, so the
sequence of bar widths it measures could come out either forwards or
backwards relative to how the barcode was printed. Without also checking
the reversed pattern, the decoder would silently fail roughly half the
time in a way that looks exactly like a sensor or wiring problem, which
would be a frustrating thing to debug.
`out` and `reversed` as pointer parameters: C functions can only
directly return one value (via the `return` keyword), so when a
function needs to hand back two or more results, it takes pointers to
the caller's own variables and writes into them via `*out = ...`
instead. This is a very common C pattern worth recognizing on sight.
The bit-reversal loop uses the same `1U << b` "build a mask with only
bit b set" technique explained under the ring buffer's bitmask above,
combined with `pattern & (1U << b)` to test whether that specific bit is
currently set.

**`shift_window()`**
What it does: drops the oldest buffered width and shifts every remaining
one down by one slot, so the window "slides" forward by exactly one
element rather than being wiped completely. Called after any failed
`classify()` or `lookup()`, on the assumption that the buffer might
simply be misaligned with the true start of the character (one element
too early or late) rather than genuinely full of garbage.

**`on_edge()`** — ties it all together, called automatically via the
event bus every time `drv_ir.c`'s barcode ISR records and publishes one
more bar/space width:
1. If decoding isn't currently `armed`, do nothing.
2. If the width is outside `ELEM_MIN_US..ELEM_MAX_US`, treat it as
   implausible and reset the whole window (`n_widths = 0`) rather than
   trying to build on top of noisy data.
3. Otherwise append the width to the buffer (sliding out the oldest one
   first via `shift_window()` if the buffer's already full at `WINDOW`),
   and if fewer than `ELEMENTS` (9) widths have been collected yet,
   simply wait for more.
4. Once 9 or more are buffered, call `classify()`; on failure, slide the
   window and return, ready to try again on the next edge.
5. On a successful classification, call `lookup()`; on failure, likewise
   slide and return.
6. On a full success: reset the window to start collecting the *next*
   character cleanly, remember the symbol for `sub_barcode_last()`,
   publish an `RC_EVT_BARCODE_DECODED` event (for anyone subscribed, such
   as `sub_nav.c`'s mission state machine), and — if one was registered
   via `sub_barcode_on_decode()` — call the user's callback function
   directly too. The `if (user_cb != NULL)` check before calling through
   the function pointer matters: `user_cb` starts out as `NULL` (meaning
   "points at nothing") until someone registers a real callback, and
   calling through a `NULL` function pointer would crash the program by
   trying to jump to invalid memory address zero.

**`sub_barcode_init()` / `sub_barcode_arm()` / `sub_barcode_on_decode()` /
`sub_barcode_last()`**
- `sub_barcode_init()` zeroes the width buffer, starts disarmed, and
  subscribes `on_edge()` to `RC_EVT_BARCODE_EDGE` on the fast lane. Call
  once at boot.
- `sub_barcode_arm(on)` turns decoding on/off *and* forwards that to
  `drv_ir_barcode_enable()` so the underlying hardware interrupt is
  masked too while disarmed — belt-and-braces protection against track
  noise being decoded as a phantom navigation command. In the full
  system, this gets called automatically when `sub_line.c` reports
  `RC_LINE_JUNCTION` (see `sub_line.h`'s enum explanation above) — the
  barcode decoder only listens right when a barcode is actually likely.
- `sub_barcode_on_decode(cb, ctx)` is how another subsystem registers its
  own handler, storing both the function pointer and an arbitrary `ctx`
  pointer that gets handed back to that function unchanged every time
  it's called — a way of giving a plain C function "memory" of which
  caller registered it, without needing objects or classes.
- `sub_barcode_last()` just returns the most recently decoded character,
  mainly useful for Buddy 1's telemetry module to report on progress.

#### How the three files fit together end to end

1. `drv_ir_sample_line()` polls the two line sensors every 5ms and
   publishes `RC_EVT_LINE_SAMPLE`.
2. `sub_line.c`'s `on_sample()` reacts to that event, steers the car via
   `sub_motion_drive()` (Buddy 2's module), and tracks whether the car is
   tracking / lost / searching / at a junction.
3. Reaching `RC_LINE_JUNCTION` (via the shared `sub_nav.c` mission logic,
   TEAM_GUIDE.md §0.4) arms `sub_barcode_arm(true)`, which in turn enables
   `drv_ir`'s barcode hardware interrupt.
4. As the car rolls over the barcode, `barcode_isr()` in `drv_ir.c` times
   every bar/space edge and queues the widths through the ring buffer;
   `barcode_drain()` publishes each one as `RC_EVT_BARCODE_EDGE`.
5. `sub_barcode.c`'s `on_edge()` collects those widths, classifies and
   looks up a character via `classify()`/`lookup()`, and publishes
   `RC_EVT_BARCODE_DECODED` with both the character and the navigation
   command it maps to (turn left/right, go straight, or U-turn).
6. `sub_nav.c` (shared mission logic) picks that command up and moves the
   car into `RC_NAV_EXECUTING_CMD`, handing off to Buddy 2's motion module
   to actually perform the turn.

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

This is the detailed, line-by-line-style companion to the "How to get
started" walkthrough above. It assumes zero prior coding or physics
background, so the first time a term like "gravity vector," "pitch,"
"accelerometer," "endianness," or "bit-shifting" shows up, it's explained
in plain language. It does **not** repeat the event-bus/publish-subscribe
or non-blocking explanations from §0 of this guide — read those first if
you haven't.

#### The two sensors, in plain terms

The board (a GY-511, built around a chip called the LSM303DLHC) is
actually two separate sensors glued onto one little circuit board:

- **Accelerometer** — feels acceleration, i.e. any push or pull on it,
  along three directions at once (X = forward/back, Y = side to side, Z =
  up/down). Here's the trick this whole module is built on: even when the
  car is sitting perfectly still, the accelerometer *still* feels
  something — gravity, pulling everything toward the centre of the
  Earth, at a constant 1 g ("g" is just the name for "how hard gravity
  pulls," used as a unit). Since gravity always points straight down,
  when the car is level, all of that "1 g of downward pull" shows up on
  the Z axis and nothing shows up on X or Y. But tilt the car — say, the
  front end rides up over a speed hump — and gravity doesn't change, but
  the car's *orientation relative to gravity* does: part of that same
  constant downward pull now leaks onto the X axis instead. The more the
  car tilts, the more of that pull shifts from Z onto X. This constant
  "which way is down, and by how much on each axis" reading is called the
  **gravity vector**, and reading how it's split between X and Z is
  exactly how this module measures tilt without ever needing a
  gyroscope.
- **Magnetometer** — a digital compass. It senses the Earth's (very weak)
  magnetic field to estimate which way is magnetic north. It's wired up
  and read in this codebase, but not currently used for navigation — see
  the warning in `drv_imu.h`, quoted below.

**The gyroscope this board doesn't have** would normally measure *rate of
rotation* directly (e.g. "you are currently rotating at 30 degrees per
second") — the textbook way to get an angle is to add up ("integrate")
that rate over time. Without one, this module has to derive tilt from the
gravity vector instead (accelerometer) and derive turning from how much
faster one wheel is spinning than the other (encoders, Buddy 2's
territory) instead of from the IMU at all. `drv_imu.h` opens with exactly
this warning:

> THIS PART HAS NO GYROSCOPE... Tilt must come from the gravity vector in
> the accelerometer, not from integrating a rate. Turn rate has no direct
> source, use differential encoder counts from Buddy 2. Hump height
> cannot come from double-integrating acceleration; the drift over a two
> second climb is larger than the hump.

Keep that constraint in mind — nearly every unusual-looking piece of math
below exists specifically to work around it.

#### `drv_imu.c` / `drv_imu.h` — talking to the chip

This is the "hardware layer": it only knows how to read raw numbers out
of the sensor over **I²C** (pronounced "eye-squared-C") — a simple
two-wire communication protocol (one wire carries data, one carries a
clock signal) that lets several chips share the same two Raspberry Pi
Pico pins, each chip only responding when its own numeric address is sent
first. This file has no opinion about what a "hump" is — that judgment
call belongs entirely to `sub_terrain.c`.

**Registers, and the `#define` constants at the top.** A "register" is
just a numbered memory slot inside the sensor chip — think of it as one
labelled cell in a tiny built-in spreadsheet that you can either write a
setting into, or read a measurement out of. The `#define`s near the top
of `drv_imu.c` give readable names to those slot numbers and to specific
bit patterns written into them, e.g.:

- `A_CTRL_REG1 = 0x20` and `A_ODR_100HZ = 0x57` — writing the value
  `0x57` (hexadecimal, i.e. base-16 — a compact way to write binary bit
  patterns) into register `0x20` turns the accelerometer on, at 100
  samples per second, with all three axes active. The specific bits that
  make up `0x57` are defined by the chip's datasheet, not invented here.
- `A_CTRL_REG4 = 0x23` and `A_FS_2G_HR = 0x08` — configures the
  accelerometer's measurement range to ±2 g (as opposed to a wider range
  like ±8 g). The trade-off: a narrower range means each measurable step
  ("LSB," least-significant bit — the smallest change the sensor can
  detect) represents a smaller amount of tilt, about a thousandth of a g,
  which is the precision hump detection needs. A wider range would let
  the car survive a bigger jolt without the reading "clipping" (maxing
  out), but at the cost of coarser resolution for gentle tilts.
- `M_CRA_REG`, `M_CRB_REG`, `M_MR_REG` — the magnetometer's equivalent
  rate/gain/mode registers, set to 75 Hz, a specific gain, and continuous
  read mode.
- `AUTO_INC = 0x80` — explained in detail below, in the accelerometer
  register-trap section.

**`drv_imu_init(void)`.** Opens the I²C bus device by name, then writes
the configuration bytes above into both chips' registers. If the very
first open or the accelerometer writes fail, it returns an error
immediately (`RC_ERR_HARDWARE`) rather than limping on — this is
deliberate, so a board that isn't wired up yet fails loudly at boot
instead of silently reporting all-zero tilt forever. The magnetometer
writes are allowed to fail without aborting startup, since the
magnetometer isn't required for the hump/hazard features this module is
graded on.

**`drv_imu_read_accel(int16_t *x, int16_t *y, int16_t *z)` — register
trap #1: left-justified data.** `x`, `y`, `z` here are **pointers** —
instead of the function returning one number, the caller hands it the
*addresses* of three variables it owns (using `&x`, the "address-of"
operator, at the call site), and the function writes results directly
into them via `*x = ...`. This is how C hands back more than one value
from a single function call. The function reads 6 raw bytes back from the
chip (2 bytes per axis) and has to reassemble them into three signed
16-bit numbers. Two traps lurk here, both explained with inline comments
at the exact lines in `drv_imu.c`:

1. **Byte order ("endianness").** The accelerometer sends the *low* byte
   of each 16-bit value first and the *high* byte second — the opposite
   of how you'd write the number on paper. So the code does
   `((uint16_t)b[1] << 8) | b[0]` — the `<<` is a **left shift**: it
   slides `b[1]`'s bits 8 positions to the left, i.e. into the *upper*
   half of a 16-bit slot, making room below it; the `|` is a **bitwise
   OR**, which merges two sets of bits together (any bit that's 1 in
   either input becomes 1 in the result) — together they rebuild the
   correct 16-bit value from two separately-received bytes.
2. **Left-justified data.** The accelerometer's actual measurement is
   only 12 bits of precision, but it's placed in the *top* 12 bits of
   that 16-bit slot instead of the bottom 12 — like writing a 3-digit
   number but padding it with extra zero digits on the right instead of
   the left. So after reassembling the 16-bit value, the code shifts it
   right by 4 (`>> 4`, a **right shift**, sliding all the bits down 4
   places) to bring the real measurement back down into the normal
   range. Skip this shift and every single accelerometer reading comes
   out exactly 16 times too large — a bug that's easy to "fix" by luck
   (scale your thresholds by 16 too) and very hard to notice is wrong
   until you compare against a real angle.

The final `(int16_t)` in each line is a **cast** — an explicit instruction
to treat the bits as a particular type, here a signed 16-bit integer,
because tilt can be positive (nose up) or negative (nose down) and the
raw bytes alone don't carry that sign information on their own.

This function calls `reg_read_burst()`, which requests `A_OUT_X_L |
AUTO_INC` as the starting register. `AUTO_INC` (`0x80`) is another
bitwise OR trick: setting the top bit of the register address tells the
chip "after sending me one register's byte, automatically advance to the
next register" — so one request streams back all 6 bytes (X, Y, Z) in
one go, instead of needing 6 separate one-byte requests. Forget this bit
and a multi-byte read comes back with only the first two bytes correct
and garbage after that.

Because this function *blocks* — it waits for the I²C transaction to
finish before returning — it must only be called from a normal task, and
never from an **ISR** (Interrupt Service Routine, a tiny piece of code
the processor jumps to instantly when a hardware event happens, e.g. a
pin changing state — see §0.2 of this guide for the full explanation and
the "golden rule" about what an ISR is and isn't allowed to do).

**`drv_imu_read_mag(int16_t *x, int16_t *y, int16_t *z)` — register trap
#2: opposite endianness *and* scrambled axis order.** Same output-via-
pointer pattern as above, but two things are different from the
accelerometer, and both are exactly the kind of mistake that's invisible
until you're debugging a car that swerves the wrong way:

1. This chip is **big-endian** — the *opposite* byte order from the
   accelerometer's little-endian data. Here the *high* byte comes first,
   so the code does `((uint16_t)b[0] << 8) | b[1]` — b[0] shifted into
   the upper half this time, not b[1]. There's also no `>> 4` shift here,
   because unlike the accelerometer, this chip's value isn't
   left-justified — it already fills the full 16 bits.
2. The registers come out in the order **X, then Z, then Y** — not X, Y,
   Z like you'd naturally assume. Read them in the "obvious" order and
   the car's sideways and up/down magnetic readings get silently
   swapped, which would look exactly like a bug somewhere else entirely.

**`drv_imu_sample(void)`.** The function the sensing task calls on a
timer, roughly 100 times a second (`RC_PERIOD_IMU_MS`, defined in
`rc_config.h`). It calls the two read functions above, subtracts the
calibration bias measured at start-up from each accelerometer axis, and
publishes an `RC_EVT_IMU_SAMPLE` event — the one piece of data every
other function in this module ultimately reacts to. One deliberate
detail: Z's bias correction only removes the *error* found during
calibration, not the whole 1 g gravity signal itself — because the pitch
math (see below) specifically needs that constant "1 g on Z when level"
reference to still be there; removing all of Z would delete the very
signal this whole module depends on.

**`drv_imu_calibrate(uint16_t n)`.** Every real accelerometer has a small
built-in offset from manufacturing tolerances and from however it's
actually mounted on your particular car — even sitting dead level, it
won't read *exactly* zero on X and Y. This function takes `n` readings
with the car sitting still and level, adds them up (in a wider 32-bit
running total, since adding many 16-bit numbers can overflow a 16-bit
total), divides by how many readings actually succeeded, and stores the
result as `bias_x`/`bias_y`/`bias_z` — to be subtracted from every future
reading. It sleeps briefly between samples (`tk_dly_tsk`, a kernel
"pause this task, let others run" call) rather than looping tight, in
keeping with the non-blocking philosophy from §0.3 — though since this
only runs once before the mission starts, that cost is essentially free
either way. **Skip calling this and every pitch reading for the rest of
the run will be silently offset by whatever tilt the board happened to be
mounted at.**

**`drv_imu_pitch_ddeg(void)` — the small-angle approximation, explained
with the actual numbers.** This is the single most important function in
the whole module: it turns the last accelerometer X reading into a pitch
angle. "Pitch" means how much the car is tilted nose-up or nose-down,
like one end of a see-saw. The proper trigonometric formula for this
would use `asin()` (inverse sine) — but the RP2040's CPU (a Cortex-M0+)
has **no FPU** (floating-point unit — the hardware circuitry that makes
`3.14 * 2.0`-style decimal math fast). Without one, real trig functions
have to be emulated entirely in software ("soft-float"), which would cost
thousands of CPU cycles *per call*, far too slow to run 100 times a
second. So the code uses a shortcut instead: for angles under about 25
degrees (which covers every realistic speed hump), the *small-angle
approximation* says `sin(angle) ≈ angle`, as long as the angle is
measured in radians (radians are simply a different unit for measuring
angles, the one this formula needs — 2π radians = 360 degrees). Since
`ax` (the accelerometer's X reading) is already proportional to
`sin(pitch)` once you divide by gravity, this collapses the whole
calculation down to simple integer division and multiplication:

```c
return (int16_t)(((int32_t)last_ax * 573) / 1000);
```

Walking through where `573` comes from: `last_ax` is in milli-g
(thousandths of a g), so dividing by 1000 converts it to a fraction of
gravity, which (via the small-angle trick) is approximately the pitch in
radians. Multiplying by 573 then converts radians to *tenths of a
degree* in one step: 1 radian = 180/π degrees ≈ 57.3 degrees ≈ 573 tenths
of a degree. The result stays a plain whole number throughout — the
project's "no floating point anywhere" rule (see §2 of this guide) in
action. The one caveat: this only measures tilt correctly when the car
isn't *also* accelerating or braking hard at the same moment, since a
hard brake pushes on the X axis too and would be indistinguishable from
tilt using this method alone.

#### `sub_terrain.c` / `sub_terrain.h` — deciding what the numbers mean

If `drv_imu.c` is "the ears and inner ear," `sub_terrain.c` is "the part
of the brain that decides what you just felt." It never touches I²C
directly — it only reacts to `RC_EVT_IMU_SAMPLE` events.

**The threshold constants.** `PITCH_ENTER_DDEG` (40, i.e. 4.0°) is how far
nose-up the car must pitch before the code believes a hump has actually
started, and `PITCH_EXIT_DDEG` (20, i.e. 2.0°) is the narrower band the
pitch must settle back inside before the hump is considered finished.
Deliberately using two *different* thresholds instead of one shared value
is a standard trick called **hysteresis** — it stops the reading
flickering rapidly between "hump" and "not hump" if the pitch happens to
sit right at the edge of a single threshold. `HUMP_MIN_MS` (150 ms)
throws out anything too quick to be a physically real hump (more likely
sensor jitter); `HUMP_MAX_MS` (5000 ms) gives up on a climb that's taken
too long, treating it as a long gentle slope instead of a hump.
`IMPACT_MG` (2200, i.e. 2.2 g) is the jolt size on X that means "the car
just hit something," not "the car is just accelerating hard."

**The hump state machine, in `on_sample()`.** This function runs every
single time a new IMU sample arrives (about 100 times/second) and
advances a small state machine with three phases, defined as a C `enum`
(a way of giving readable names to a small fixed set of whole-number
states — `H_FLAT` is really just the number 0 under the hood, `H_CLIMBING`
is 1, and so on, but the name is what a human reads):

- **`H_FLAT`** — the default, "nothing happening" state. Every sample,
  it checks whether pitch has crossed above `PITCH_ENTER_DDEG`; if so, it
  switches to `H_CLIMBING`, records the current timestamp, seeds the
  "peak pitch so far" with this first reading, snapshots the current
  wheel-odometry distance (`dist_at_hump_start`, via
  `drv_encoder_distance_mm()` — again Buddy 2's territory), and publishes
  `RC_EVT_HUMP_BEGIN`.
- **`H_CLIMBING`** — keeps updating the running peak-pitch-so-far
  (needed later to compute height), and watches for pitch to swing over
  to clearly nose-down, which means the car has crested the top and moved
  to `H_DESCENDING`. If it's been climbing longer than `HUMP_MAX_MS`
  without cresting, it gives up and resets straight back to `H_FLAT`
  instead.
- **`H_DESCENDING`** — waits for pitch to settle back within the narrow
  exit band. Once it does, if the whole climb-to-settle episode lasted at
  least `HUMP_MIN_MS`, it computes the hump's height (see next section),
  updates the run's tallest hump if this one's bigger, publishes
  `RC_EVT_HUMP_END` with the height/duration/peak-pitch, and calls the
  optional user-registered callback (set via `sub_terrain_on_hump()`) if
  one was registered.

**`pitch_to_height_mm()` — the geometry, worked through with the real
numbers.** This is the cleverest piece of this module, and the reason a
gyroscope-free board can still estimate how tall a hump is. The naive
approach — integrate acceleration twice to get position — genuinely
doesn't work here: over the roughly two seconds a climb takes, small
sensor errors compound (this is called "drift") into a height estimate
bigger than the hump itself. Instead, the code treats the car as a rigid
ruler of known length: the **wheelbase**, the front-to-back distance
between the axles (`RC_WHEEL_BASE_MM`, measured and set in
`rc_config.h`, Buddy 2's calibration constant). While the front wheels
are already up on the ramp and the rear wheels are still on flat ground,
the whole car body tilts by exactly the pitch angle, and basic geometry
(picture a ramp forming the hypotenuse of a right triangle, with the
wheelbase as that hypotenuse) says:

```
rise = wheelbase * sin(pitch)
```

and — the same small-angle trick used in `drv_imu_pitch_ddeg()` — for the
small angles a speed hump produces, `sin(pitch) ≈ pitch` in radians. In
code, `peak_ddeg` (tenths of a degree) is first converted to
milliradians (thousandths of a radian, another fixed-point scaling trick
to avoid ever needing a fraction) via `* 1745 / 1000` — 1745/1,000,000 is
the degrees-in-tenths-to-radians conversion factor (π/1800) approximated
as a whole-number fraction — and then multiplied by `RC_WHEEL_BASE_MM`
and divided by 1000 (undoing the earlier milliradian scaling) to get the
rise in millimetres. That rise **is** the estimated hump height: how tall
the ramp must be for the car's known wheelbase to have tilted that much.
It is explicitly an *estimate*, not a direct measurement — see the TODOs
below for how to validate and cross-check it.

**`classify()` — deciding the one-word motion label every sample.** This
function is checked in strict priority order, and the first matching
condition wins:

1. A hard jolt on X beyond `IMPACT_MG` in either direction →
   `RC_MOTION_IMPACT`, published immediately as `RC_EVT_IMPACT`.
2. Otherwise, if the hump state machine above is currently `H_CLIMBING`
   or `H_DESCENDING` → `RC_MOTION_CLIMBING` / `RC_MOTION_DESCENDING`.
3. Otherwise, if both wheel-encoder speeds read zero →
   `RC_MOTION_STATIONARY`.
4. Otherwise, if the left and right encoder speeds differ by more than
   150 mm/s → `RC_MOTION_TURNING`. This is the module's honest
   workaround for having no gyroscope: it falls back entirely to
   `drv_encoder_speed_mm_s()` from **Buddy 2's** motion module — the same
   way a tank or a wheelchair turns by driving its two sides at different
   speeds, a difference between the wheels is direct evidence of turning,
   and it's a more reliable signal than trying to infer rotation from
   this IMU's limited data.
5. Otherwise, a strong forward/backward push on X classifies as
   `RC_MOTION_ACCELERATING` or `RC_MOTION_DECELERATING`.
6. Otherwise, `RC_MOTION_CRUISING` (steady speed, nothing notable
   happening).

An event (`RC_EVT_MOTION_CLASS`) is only published when the class
actually *changes* — not every single sample — so subscribers like Buddy
1's telemetry module aren't flooded with "still cruising" a hundred times
a second.

**The public API in `sub_terrain.h`.** `sub_terrain_init()` is called
once at boot; it resets internal state and subscribes `on_sample()` to
`RC_EVT_IMU_SAMPLE` on the bus's **fast lane** (see §0.2 — hump/impact
detection needs to react without delay, the same reasoning as steering).
`sub_terrain_on_hump(cb, ctx)` lets any other module register a plain
function to be called the moment a hump finishes, as an alternative to
subscribing to the event bus directly. `sub_terrain_max_peak_mm()` and
`sub_terrain_motion_class()` are simple getters returning the module's
internally-tracked state. `sub_terrain_reset()` clears everything back to
"just booted, car still" — useful for starting a fresh run without a
full reboot.

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

Jargon up front — these terms show up constantly in this module, so get
comfortable with them before reading the code:

- **ISR (Interrupt Service Routine)** — a tiny piece of code the chip
  jumps to *automatically*, the instant some hardware event happens (a
  pin's voltage changes, a timer reaches zero), pausing whatever else was
  running to do it. Think of it like a phone ringing: you drop what
  you're doing, answer briefly, and get back to what you were doing — an
  ISR is expected to be that quick.
- **PWM (Pulse Width Modulation)** — a way to control something (a servo's
  position, a motor's speed) using only an on/off digital pin, by
  controlling *how long* the pulse stays "on" within each repeating time
  slice.
- **Duty cycle** — the fraction of each PWM cycle spent "on." Higher duty
  = more average power/signal.
- **TIMER alarm interrupt** — the RP2040 chip has hardware timers you can
  tell "interrupt me in X microseconds," and then simply forget about —
  no code sits in a loop counting down. When the time's up, the chip
  jumps straight to an ISR you registered ahead of time. This module uses
  two of these (see below) instead of software delay loops.
- **Pointer** — a variable that holds a memory *address* rather than a
  value directly — "where to find the data," not the data itself.
  Function pointers (pointers that hold the address of a *function*) are
  how this codebase implements callbacks: "call me back later," instead
  of "wait right here until you're done."
- **`struct`** — a way to bundle several related values into one named
  unit, like stapling several related fields onto one form instead of
  keeping them as loose separate notes.
- **`static`** (at file scope) — means "only visible inside this one .c
  file." It's how C fakes "private" module state without needing classes.
- **`const`** — a promise to the compiler (and to anyone reading the code)
  that a function won't modify whatever a pointer points to.
- **State machine (`enum` + `switch`/`if` chains)** — a way of writing
  code that can only ever be in one of a small, named set of "modes" at a
  time (e.g. "idle," "moving," "waiting for echo"), and reacts differently
  depending which mode it's currently in. This whole module is built as
  nested state machines — see below.

#### `drv_servo.h` / `drv_servo.c` — aiming the sensor

This is the smaller of the two driver files, and does one job: point the
ultrasonic sensor at a given angle, and estimate how long it'll take to
get there. There is no way to *ask* the servo where it currently is — a
cheap hobby servo just accepts a "go here" pulse and trusts you to know
when it's arrived, the way a fax machine has no receipt showing the other
side actually printed the page.

- **`#define SERVO_HZ (50UL)`** — the PWM frame rate for the servo signal:
  one pulse every 1/50th of a second (20 ms). This isn't a tunable
  performance knob, it's just what hobby servos expect by convention —
  fixed by the hardware, not chosen for this project.
- **`#define PULSE_MIN_US (500UL)` / `#define PULSE_MAX_US (2400UL)`** —
  the pulse widths (in microseconds, within each 20 ms frame) that a
  typical hobby servo reads as "go to 0°" and "go to 180°." Everything
  between those two angles is a straight-line interpolation between these
  two pulse widths. These numbers come from the servo's own
  datasheet/convention, and are what `drv_servo_set_angle()` converts
  every angle into.
- **`#define SWEEP_DEG (RC_SERVO_ANGLE_MAX - RC_SERVO_ANGLE_MIN)`** — total
  degrees of travel (180), used as the divisor in that same interpolation.
- **`#define US_PER_DEG (2200UL)` and `#define SETTLE_FIXED_MS (25UL)`** —
  this is the pair called out explicitly in the "how to get started"
  section above, and it deserves the "what it does, why, what links to
  it" treatment the brief asks for:
  - **What it does:** `US_PER_DEG` estimates how many microseconds of
    travel time the servo needs *per degree* of movement; divided by 1000
    inside `drv_servo_settle_ms()`, it becomes milliseconds-per-degree.
    `SETTLE_FIXED_MS` is a flat number of extra milliseconds tacked on
    top, to let the mechanical arm stop physically wobbling ("ringing" —
    like a diving board bouncing after someone jumps off) once it
    technically arrives.
  - **Why:** because there's no feedback sensor telling the code "the
    servo has arrived," these two constants are the *entire* basis for
    knowing when a distance reading taken at a new angle can be trusted.
    Too short, and `sub_scan.c` pings before the sensor has physically
    settled, producing readings tagged with the wrong angle or blurred
    by motion. Too long, and every scan takes needlessly longer — a full
    5-point coarse scan takes roughly `5 × (servo travel + 30ms echo
    flight)`, so this pair of constants directly controls how long the
    car sits still scanning instead of driving.
  - **Links to other code:** `drv_servo_settle_ms()` (which uses both
    constants) is called from `step_to()` in `sub_scan.c`, which is the
    single function both the coarse and fine sweep loops use for every
    angle they visit. Get this wrong, and it shows up as a symptom in
    `sub_scan.c`'s scan timing, not in this file.
- **`drv_servo_init(void)`** — sets up the 50 Hz PWM signal and centers
  the sensor at 90° (straight ahead). Call once at boot, after the
  bracket has been mechanically aligned (see "How to get started" step 2
  above — the code's idea of "90° = straight ahead" only means anything
  if the bracket was bolted on correctly).
- **`drv_servo_set_angle(int16_t deg)`** — the actual "point it here"
  function. First it clamps `deg` to `RC_SERVO_ANGLE_MIN`/`MAX` (0–180)
  so a bug elsewhere can never ask for an angle outside what the bracket
  can physically reach. Then it does the pulse-width math:
  ```c
  pulse_us = PULSE_MIN_US
           + (((PULSE_MAX_US - PULSE_MIN_US) * (uint32_t)deg) / SWEEP_DEG);
  ```
  Read this as: "0° maps to `PULSE_MIN_US`, 180° maps to `PULSE_MAX_US`,
  and everything in between is proportionally in between" — the same
  logic as reading a value off a ruler between two marked ends. It's
  written as multiply-then-divide (not divide-then-multiply) specifically
  to lose as little precision as possible while only using whole-number
  (integer) arithmetic — there's no `float`/`double` anywhere in this
  codebase because the RP2040 chip has no hardware floating-point unit
  (FPU), and doing real decimal math in software would be slow. Finally
  it remembers `last_angle` and hands the pulse width to the PWM driver.
  It returns immediately — it does **not** wait for the servo to actually
  get there.
- **`drv_servo_get_angle(void)`** — returns whatever was last commanded.
  Again: a *command history*, not a live measurement.
- **`drv_servo_settle_ms(int16_t from, int16_t to)`** — see the detailed
  walkthrough of `US_PER_DEG`/`SETTLE_FIXED_MS` above; this function is
  where those two constants actually get combined: distance travelled
  (in degrees) × time-per-degree, plus the fixed settle allowance.
- **`drv_servo_release(void)`** — stops sending PWM pulses entirely, which
  lets the servo go mechanically limp and stop drawing current. This
  matters because the servo and the drive motors share one battery — a
  servo move happening at the exact moment the drive motors draw a big
  current spike (e.g. starting from a stop) can brown out (momentarily
  starve of power) the whole Pico.

#### `drv_ultrasonic.h` / `drv_ultrasonic.c` — the "shout and listen"

This is the most technically interesting file in the module, because of
*how* it avoids blocking the rest of the car while a measurement is in
flight. It's worth understanding the naive alternative first, because the
whole design here is a direct reaction to it.

**The problem it avoids.** A textbook Arduino-style ultrasonic driver
looks like this:

```c
digitalWrite(TRIG, HIGH); delayMicroseconds(10);
digitalWrite(TRIG, LOW);  pulseIn(ECHO, HIGH);
```

Both `delayMicroseconds` and `pulseIn` are **busy-waits** — the processor
sits in a tight loop doing nothing else until the operation finishes.
`pulseIn` in particular can hold the CPU hostage for up to 30
milliseconds if nothing echoes back (sound has to travel out and back, at
about 343 m/s, and the sensor's own max range takes a while to time out).
On this car, the wheel motor control loop needs to run roughly every 20
milliseconds to keep the car driving straight and smooth (see Buddy 2's
PID loop in `sub_motion.c`). A 30ms stall means a missed control update —
visible as a stutter or swerve exactly when the sensor is trying to look
for obstacles, which is precisely the worst possible moment for the car
to lose control.

**The fix: a 5-step interrupt relay.** Instead of one function that waits
the whole way through, the measurement is broken into five tiny pieces,
each one doing only a few microseconds of work and then handing off to
the *next* piece via a hardware interrupt — nothing in software ever sits
and waits. Here's each step, in order, with what hardware event triggers
it and exactly what it does:

1. **`drv_ultrasonic_ping(int16_t tag_angle_deg)`** — called from ordinary
   code (specifically `step_to()` in `sub_scan.c`), not from an
   interrupt. This is where a measurement *begins*. It:
   - Briefly disables all interrupts (`DI(sts)` / `EI(sts)` — Disable
     Interrupts / Enable Interrupts) just long enough to check "is a
     measurement already running?" and, if not, mark one as starting.
     This tiny "critical section" exists so an ISR can never sneak in
     between the check and the set and see the state half-updated — a
     classic bug called a race condition.
   - Records `tag_angle_deg` — the servo angle this measurement is being
     taken at — so that once a result eventually comes back, it can be
     matched to *which direction* it was looking, without extra
     bookkeeping elsewhere.
   - Raises the TRIG pin high (this is the "shout") and arms **TIMER
     alarm 1** to fire again in `RC_ULTRA_TRIG_US` (12 microseconds) —
     the standard trigger pulse length the HC-SR04 datasheet asks for.
   - Returns immediately. Total time spent: a handful of microseconds.

2. **`trig_done_handler` — triggered by TIMER alarm 1 firing**, exactly
   12µs after step 1 armed it. This is the chip's own hardware clock
   jumping straight to this function; nothing was sitting and counting
   down in software. It:
   - Drops TRIG back to low — the trigger pulse is over, and the HC-SR04
     has (by now) started emitting its 40kHz "click" and preparing to
     listen for the echo.
   - **Un-masks (enables) the ECHO pin's interrupt.** This is the key
     idea that makes the *listening* half interrupt-driven too: from this
     instant on, any voltage change on the ECHO pin will itself
     automatically trigger step 3/4 below — no polling loop needed.
   - Arms **TIMER alarm 2** as a timeout safety net (step 5), in case the
     echo never comes back at all.

3. **`echo_isr` (rising edge) — triggered the instant the ECHO pin goes
   high.** The HC-SR04 raises ECHO the moment it starts listening for the
   sound to bounce back, so the pin itself is effectively a stopwatch
   button. This handler just records the current microsecond timestamp
   into `t_rise` and moves the internal state machine on to "waiting for
   the echo to end."

4. **`echo_isr` (falling edge) — triggered the instant the ECHO pin drops
   back to low**, meaning the sensor detected the reflected sound. It
   computes `t_us - t_rise` — exactly how long ECHO stayed high, in
   microseconds — and hands that duration off to `finish_i()`. Crucially,
   the actual *division* needed to turn microseconds into millimetres
   (via the `US_TO_MM` macro) does **not** happen here — it's deferred to
   a "bottom half" function, `ultra_drain()`, that runs later in normal
   task context. The golden interrupt rule from TEAM_GUIDE.md §0.2
   applies directly here: an ISR may only record a timestamp and hand
   off, never do the heavier work (division, publishing an event, calling
   a user callback) itself. `finish_i()` also does the two things that
   genuinely *can't* wait — masking the ECHO pin's interrupt back off
   (so a noisy, ringing 5V-to-3.3V voltage divider can't keep firing
   spurious interrupts while the car is busy doing something else) and
   cancelling the now-unnecessary timeout alarm.
5. **`timeout_handler` — triggered by TIMER alarm 2 firing**, if steps 3
   and 4 never happened within `RC_ULTRA_TIMEOUT_US` of step 2 arming it.
   This is the safety net: if nothing echoes back at all (nothing is in
   front of the sensor, or it's too far away, too soft to reflect sound,
   or angled away), this fires instead and reports an *invalid* reading —
   so a scan can never get stuck waiting forever for a result that will
   never arrive.

**Why two TIMER alarms and not one?** Because two independent countdowns
are needed simultaneously at different points: one to time the outgoing
12µs trigger pulse (alarm 1), and a separate one to time the maximum
allowed wait for a returning echo (alarm 2). They're both RP2040 hardware
peripherals reserved specifically because this project's underlying RTOS
port doesn't otherwise use them (see the comment at the top of
`drv_ultrasonic.h`).

**The math constant worth knowing:**
```c
#define US_TO_MM(us)  (((us) * 343UL) / 2000UL)
```
Speed of sound is roughly 343 metres/second at room temperature. The echo
travels *out and back* (to the obstacle and then all the way back to the
sensor), so the raw one-way distance is half of what the round-trip time
would suggest — that's where the extra factor of 2 comes from (`/2000`
instead of `/1000`, folding the unit conversion and the "divide by two
for round trip" into one division). It's written as a macro (compiler
text-substitution, not a real function call) so it's essentially free to
use, and it works directly on whatever integer width is handed in.

**Result delivery — `ultra_drain()` and `RC_EVT_ULTRA_RESULT`.** Once
`finish_i()` has stashed a raw measurement, it calls
`rc_defer_signal_i()` to request that `ultra_drain()` run soon, in normal
task context. `ultra_drain()` converts the width to millimetres, checks
it against `RC_ULTRA_MIN_MM`/`RC_ULTRA_MAX_MM` (flagging it invalid if
out of the sensor's honest working range), then publishes
`RC_EVT_ULTRA_RESULT` on the shared event bus (see TEAM_GUIDE.md §0.2)
and — if one was registered via `drv_ultrasonic_on_result()` — calls a
direct callback too. `sub_scan.c`'s `on_range()` is exactly that
callback: it's what stitches a raw distance reading back into the
angle-tagged `points[]` array a scan is building up.

#### `sub_scan.h` / `sub_scan.c` — scanning, profiling, and planning

This file is where the two drivers above get orchestrated into an actual
behaviour: sweep, look at what was found, decide what to do about it. It
is built as **two nested state machines** — the top-level scan state
(idle / watching / coarse-moving / coarse-pinging / fine-moving /
fine-pinging) plus the ping/echo state machine inside
`drv_ultrasonic.c` underneath it.

**The event-flag "doorbell" pattern.** Before the state machine itself, a
quick note on the plumbing: `scan_flgid` is a kernel "event flag" object,
which is like a doorbell a task can go to sleep listening for (via
`tk_wai_flg`), instead of constantly checking a mailbox in a loop. Other
code rings that doorbell (`tk_set_flg`) with one of three named bits:

- **`FLG_RESULT`** — "an ultrasonic ping just produced a result," rung by
  `on_range()` (the ultrasonic driver's registered callback) every time a
  measurement completes.
- **`FLG_START`** — "please begin a scan," rung by `sub_scan_start()`.
- **`FLG_ABORT`** — "stop whatever's running," rung by `sub_scan_abort()`.

This is how `scan_task` (the background task driving the whole module)
can sleep almost the entire time a scan is happening, without polling
anything.

**The scan state machine.** `scan_state_t` defines the named states:
`S_IDLE` (nothing happening), `S_WATCH` (slow continuous forward ping —
see below), `S_COARSE_MOVE`/`S_COARSE_PING` (the broad first sweep),
`S_FINE_MOVE`/`S_FINE_PING` (the zoomed-in re-scan), and an unused
`S_DONE` marker. Walking through what actually happens, function by
function:

- **`sub_scan_init(void)`** — creates the event flag and the background
  `scan_task`, and registers `on_range()` with the ultrasonic driver.
  Call once at boot, after `drv_servo_init()` and `drv_ultrasonic_init()`.
- **`sub_scan_start(void)`** — the entry point everything else calls
  (typically `sub_nav.c`, the shared mission state machine, when it
  decides an obstacle needs a proper look). Returns `RC_ERR_BUSY` if a
  scan is already running, otherwise rings `FLG_START` and returns
  **immediately** — this is the "start it and register a callback"
  non-blocking pattern from TEAM_GUIDE.md §0.3 in action. The actual
  scanning work happens later, on `scan_task`'s own schedule.
- **`scan_task(INT stacd, void *exinf)`** — the background task, created
  once and running forever (`for (;;)`, a loop with no exit — the normal
  C idiom for "a task that just keeps doing its job"). Each time through
  the loop:
  - If in `S_WATCH`, it pings straight ahead once, waits for the result,
    then sleeps for `RC_PERIOD_SCAN_MS` before looping again — this is
    the slow, continuous "is something getting close while I'm just line
    following" background check (see `sub_scan_set_watch` below).
  - Otherwise it sleeps (via `tk_wai_flg`) until `FLG_START` is rung (or
    100ms passes, just so the loop can re-check things periodically).
  - Once started, it calls `run_coarse()`, then — if something was found
    — `find_fine_window()` followed by `run_fine()`, then
    `publish_and_finish()` to report the result. If a watch was requested
    meanwhile, it drops back into `S_WATCH` instead of `S_IDLE` once done.
- **`step_to(int16_t angle)`** — the one function both sweep loops call
  for every angle they visit: ask `drv_servo_settle_ms()` how long this
  particular move will take, command the servo there, **sleep for that
  long via `tk_dly_tsk()`** (a genuine kernel sleep — the task is
  completely off the CPU, not spinning), then start one ultrasonic ping.
  This single function is where the "non-blocking but still correctly
  timed" trick lives: from the outside it looks like ordinary sequential
  code (move, wait, ping), but because the wait is a kernel sleep and not
  a busy loop, every other task on the system — especially the motion PID
  loop — keeps running at full rate the entire time.
- **`wait_result(void)`** — sleeps until either `FLG_RESULT` or
  `FLG_ABORT` is rung, or 100ms passes as a safety timeout (protecting
  against a genuinely wedged measurement, not expected to fire in normal
  operation since the ultrasonic driver has its own timeout — see step 5
  above).
- **`run_coarse(void)`** — the broad first pass: steps the servo from
  `RC_SCAN_COARSE_START` to `RC_SCAN_COARSE_END` in
  `RC_SCAN_COARSE_STEP`-degree increments (all defined in
  `rc_config.h`), pinging and collecting a result at each stop. This is
  the "slowly pan the security camera across the whole visible arc once"
  pass.
- **`find_fine_window(void)`** — looks back over everything `run_coarse()`
  collected, finds the single closest hit that counted as an obstacle
  (closer than `OBSTACLE_MM`), and computes a narrow angular window
  (`fine_from`..`fine_to`) centred on it, clamped to the servo's physical
  travel limits. This is the "spotted something, now swing back and get a
  closer look at just that spot" step. Returns `false` (skipping the fine
  scan entirely) if nothing close enough was found.
- **`run_fine(void)`** — re-scans just that narrow window at the tighter
  `RC_SCAN_FINE_STEP` spacing, adding these extra, more closely-packed
  points on top of what the coarse pass already collected. This sharpens
  the eventual width/clearance estimate around the one obstacle that
  matters, without paying the cost of scanning the *entire* arc that
  finely.
- **`on_range(uint16_t range_mm, bool valid, void *ctx)`** — registered
  with the ultrasonic driver as its result callback, so this runs in
  **interrupt context**. It does the bare minimum allowed: store the
  reading (angle, range, valid) into the next free slot of the `points[]`
  array — guarded by a bounds check so a scan can never write past the
  end of the fixed-size array, a classic C bug this code deliberately
  avoids — then rings `FLG_RESULT` to wake `scan_task`.

**The obstacle-profiling trigonometry — `build_profile()`.** Once a scan
finishes, this function walks every collected `(angle, range)` point once
and boils the whole scan down into five numbers: closest obstacle
distance and its bearing, an estimated width, and clearance (open space)
on the left and right of straight-ahead. The interesting part is the
width estimate:

```c
int32_t span_ddeg = ((int32_t)last_hit - (int32_t)first_hit) * 10;
int32_t span_mrad = (span_ddeg * 1745) / 1000;
p->width_mm = (uint16_t)(((int32_t)closest * span_mrad) / 1000);
```

The underlying geometry: `width ≈ 2 × range × tan(span / 2)`, where
`span` is the angular width (in degrees) the obstacle occupied across the
sweep (`first_hit` to `last_hit` — the first and last angle at which a
reading counted as "close"), and `range` is the distance to it. Picture
the sensor at the point of a narrow triangle, with the obstacle's two
edges as the triangle's other two corners — this is exactly the same
trigonometry an old-fashioned land surveyor uses to estimate the width of
something across a field just from one distance and two angles.

The code doesn't call a real `tan()` function, though — and this is the
key trick, called the **small-angle approximation**: for angles under
about 60 degrees, `tan(x)` (measured in radians) is close enough to `x`
itself that the error is smaller than the HC-SR04's own ~15° beam width
(the sensor's own fuzziness already swamps this shortcut's error). That
approximation lets the whole computation be done with plain integer
multiply/divide instead of needing a real trigonometric function — which
matters because the RP2040 chip has **no hardware floating-point unit
(FPU)**, so calling a software `tan()` would be slow, and would also
require `float`/`double` math, which this entire codebase avoids on
principle (see TEAM_GUIDE.md §2, "No floating point anywhere" — Buddy 4's
pitch-to-height calculation in `sub_terrain.c` uses this exact same
small-angle trick for the same reason). Concretely: `span_ddeg` converts
the angular span into tenths of a degree (keeping it a whole number even
though the real span may not be a multiple of 10), `span_mrad` converts
that into milliradians (thousandths of a radian, using the constant 1745
≈ 1000 × π/180 × 10, i.e. "tenths of a degree to milliradians"), and the
final line multiplies that angle (in "radians × 1000") by the distance
and divides back down by 1000 — which works out to exactly `range ×
angle_in_radians`, standing in for the full `2 × range × tan(span/2)`
formula (the factor of 2 and the "half the span" cancel out
algebraically, which is why neither shows up explicitly in the code).

**Clearance counting.** `clearance_left_mm`/`clearance_right_mm` are
*not* a measured gap width — they're a count of how many sampled bearings
on each side of straight-ahead came back "clear" (farther than
`OBSTACLE_MM`), multiplied by the angular spacing between samples
(`RC_SCAN_COARSE_STEP`) to turn "N clear bearings" into a rough linear
distance. This is a deliberately crude stand-in: a single narrow ranging
beam, sampled in discrete angular steps, can honestly only report "this
direction looked open," not a smooth, precise gap width.

**The decision — `build_plan()`.** Takes the profile above and turns it
into one of: go straight (nothing close, or the nearest thing is further
than `OBSTACLE_MM`), stop (both `clearance_left_mm` and
`clearance_right_mm` are below `CAR_WIDTH_MM` — nowhere safe to fit), or
turn toward whichever side reported more clearance (`>=` breaks a tie
toward turning left, an arbitrary but consistent choice). When it decides
to turn, it also computes how far sideways (`lateral_mm`) and forward
(`forward_mm`) to steer — half the car's width plus half the obstacle's
width sideways, and the obstacle's width plus a 100mm margin forward —
simple geometric estimates for clearing the obstacle's far edge before
straightening back out. `build_plan()` only *decides*; it's
`sub_motion.c` (Buddy 2's module) that actually drives the resulting
turn/distance commands.

**Reporting the result — `publish_and_finish()`.** Builds the profile and
plan, then publishes both on the shared event bus:
`RC_EVT_OBSTACLE_PROFILE` and `RC_EVT_AVOIDANCE_PLAN`. `sub_nav.c` (the
shared mission state machine everyone's module plugs into — see
TEAM_GUIDE.md §0.4) subscribes to `RC_EVT_AVOIDANCE_PLAN` to know when
it's time to leave `RC_NAV_AVOIDING` mode and hand off to `sub_motion.c`'s
actual turn/drive commands; `sub_telemetry.c` (Buddy 1's module)
subscribes to `RC_EVT_OBSTACLE_PROFILE` to report what was found. Any
direct callback registered via `sub_scan_on_complete()` is also called
here, and finally the state machine resets to idle (or back to `S_WATCH`
if a continuous watch was requested).

**Continuous forward watch — `sub_scan_set_watch(bool on, uint16_t
trigger_mm)`.** A lighter-weight mode than a full scan: while the car is
just line-following, one ping straight ahead fires every
`RC_PERIOD_SCAN_MS` so the car can notice something getting close without
running a full sweep constantly — like a security camera's motion sensor
staying on low-power standby between active sweeps. `sub_nav.c` subscribes
to `RC_EVT_ULTRA_RESULT` directly (not through this module) and decides
when a watch reading is close enough to escalate into a real
`sub_scan_start()`.

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

- **No ISR in this codebase calls `tm_printf`/`rc_snprintf`/any print
  function, and none should.** This was checked across every interrupt
  handler in the tree (`encoder_isr`, `timeout_handler`, `barcode_isr`,
  `trig_done_handler`, `echo_isr`, `gpio_bank0_handler`) — all of the
  `tm_printf` calls in the codebase live in ordinary tasks (`app_main.c`'s
  startup sequence, `sub_nav.c`'s state-change logging, and
  `sub_telemetry.c`'s console sink), never inside an ISR. Why this
  matters: printing text is *slow* — even this project's small
  `rc_snprintf` (see §0 and Buddy 1's walkthrough) has to walk the whole
  format string and write out each character, and `tm_printf` on top of
  that waits on the UART/USB hardware to actually send the bytes. An ISR
  is only allowed a handful of microseconds (see the interrupt rule in
  `SKILL.md`) before it starts blocking the other interrupts sharing its
  vector — the two encoder pins, the ultrasonic echo pin and the barcode
  pin all currently share one. A `tm_printf` call inside any of those
  handlers would very likely corrupt a barcode reading or drop an encoder
  tick on the very next interrupt. **If you're ever debugging and tempted
  to add a print statement inside an ISR to see what's happening, don't**
  — instead, either print from the "bottom half" task the ISR hands off
  to (e.g. `encoder_drain`, `ultra_drain`, `barcode_drain` — these already
  run in task context and are safe to print from), or record the value
  into a variable the ISR already has and read it back later from a task.
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
