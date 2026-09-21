# Team Guide — Autonomous RoboCar

Welcome. This guide exists so that **anyone on the team — even with zero
embedded-systems or C background — can open it and know exactly what to do
next.** It is organized by role ("Buddy 1" through "Buddy 5"), matching the
ownership table in the [README](README.md) and [SKILL.md](SKILL.md). Read
your own section fully before touching code. Skim §0 too — §0.2 explains the
event bus and §0.6 walks through every file in `core/`, the plumbing every
module sits on top of; you will bump into it no matter which piece you own.
§0.7 shows every physical part with a picture and who owns it.
Each Buddy section then opens with your hardware and a "How your files connect" table showing
exactly which `core/` services and which other buddies' files yours talk to.

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
core/         the plumbing everyone shares (event bus, timing, interrupts, PWM) — see §0.6
drivers/      one file per physical part (motor, encoder, servo, ultrasonic sensor, IR sensor, IMU)
subsystems/   one file per team member's "job", plus the shared mission logic
app/          the startup code that wires everything together and boots the car,
              plus app_bench.c — the per-buddy bench tests (§0.8)
docs/         HARDWARE.md — wiring, pin numbers, and calibration steps; img/hw/ — part pictures used in §0.7
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
- **Serial console over USB** — the Pico has no screen, so it "talks" by
  streaming text over its own USB cable, which your laptop sees as a
  serial port (a "COM port" on Windows). Any serial-terminal program can
  open it. Because it's USB, the baud rate you pick doesn't matter.

#### 0.5.1 Install the tools (once per laptop)

The easy path: **install the Raspberry Pi Pico extension in VS Code** and
let it run its first-time setup. It downloads the ARM compiler, the Pico
SDK and `picotool` into `%USERPROFILE%\.pico-sdk\` (`~/.pico-sdk` on
Mac/Linux), and this project's build script finds them there
automatically. Then you need only two more things:

1. **Git** — downloads the code. Windows: "Git for Windows" from
   git-scm.com, which also gives you **Git Bash**, the terminal to use for
   every command in this guide. Mac: `brew install git`. Linux:
   `sudo apt install git`.
2. **GNU `make`** — the tool that runs the compiler in the right order.
   The Pico extension does *not* install it.
   - Windows: **nothing to do** — the first time you build, `build.sh`
     downloads a small portable copy into `build/tools/` (about 400 kB)
     and uses it from then on. If that download is blocked, install it
     yourself with `winget install ezwinports.make` and reopen Git Bash.
   - Mac: `xcode-select --install`. Linux: `sudo apt install make`.

If you'd rather not use the extension, install `arm-none-eabi-gcc` (13.2
or newer) and `picotool` yourself, clone `raspberrypi/pico-sdk` somewhere,
and tell the build where it is with `PICO_SDK_PATH=/path/to/pico-sdk`.
Only TinyUSB out of the SDK is used — it's what puts the console on the
Pico's USB port — but that part is mandatory here because the UART pins
belong to the left wheel encoder.

You do **not** need a host C++ compiler (the build uses `picotool` for
the last step instead of the port's own tool) and you do **not** need a
USB-serial adapter (the console rides the same USB cable you flash over).

#### 0.5.2 Get the code — one clone, both repositories

Two pieces of code end up on the Pico: this repo (the car) and the
**micro T-Kernel port** (`mtk3smp-rp2040` — the operating system plus the
`make`-based build system). You don't download the port separately: it's
wired in as a **git submodule** at `external/mtk3smp-rp2040`, pinned to
one exact commit so everyone on the team builds the identical kernel.

Open Git Bash, go to the folder you want the project in, and:

```sh
git clone --recurse-submodules https://github.com/Dysteriaaaa/INF2004-IS07-AutonomousRoboCar.git
cd INF2004-IS07-AutonomousRoboCar
```

Already cloned it without `--recurse-submodules`? Then the
`external/mtk3smp-rp2040` folder is empty — fix with:

```sh
git submodule update --init
```

#### 0.5.3 Set up — patches the port and wires our code into its build

```sh
./build/setup.sh
```

**Where to type this — and every `./build/...` command in this guide:**
in **Git Bash**, with the repo folder as the current directory (the one
that contains `build/`, `core/`, `drivers/`…; after the clone in §0.5.2
you're already there, and `cd INF2004-IS07-AutonomousRoboCar` gets you
back). Or skip the terminal entirely and use the VS Code tasks in §0.5.9.

> **Not WSL.** If you have Ubuntu/WSL installed, typing `bash` into
> PowerShell or CMD opens *that*, and inside it none of the Pico tools
> exist — so the scripts fail with "not found" errors. Use the **Git
> Bash** app from the Start Menu (it came with Git for Windows), or the
> VS Code terminal with the *Git Bash* profile. The script detects the
> WSL mistake and tells you.

One command, run once after cloning (and again any time the submodule is
updated). It does three things you'd otherwise have to do by hand:

1. **Fetches the kernel port** if the submodule isn't there yet.
2. **Patches five lines of the port** so it stops claiming pins this car
   needs. The stock port switches GP0/GP1 to a UART console (that's our
   left encoder), GP8/GP9 to I²C (that's the right motor), parks GP27/GP28
   as analogue inputs (barcode DO and right-encoder B), and puts the
   status LED on GP16 (line sensor 1). Each patch is explained in
   `docs/HARDWARE.md` §1 and applied by `build/patch_port.py`; running it
   twice is harmless.
3. **Installs the build hook.** The port only compiles one flat
   `app_program/` folder, so it can't see our `core/`, `drivers/`,
   `subsystems/` and `app/`. `build/robocar.mk` teaches it to; `setup.sh`
   copies that file into the port. The port's own demo program is then
   simply not built — our `app/app_main.c` supplies `usermain()` instead.

**Why this matters even if you're not touching hardware:** skip it and
the build still *succeeds* — you get a robot whose IMU never answers,
whose LED never blinks and whose left wheel never counts. Far more
confusing to chase later than to run one script now.

#### 0.5.4 Build it

```sh
./build/build.sh          # single core — always start here
./build/build.sh smp      # dual core, once single core works
```

(Git Bash, from the repo folder — see the note in §0.5.3. In VS Code:
**Terminal → Run Build Task**, or `Ctrl+Shift+B`, does the first line.)

What this does, in plain terms: it finds the ARM compiler and Pico SDK
(on `PATH`, or under `~/.pico-sdk` where the VS Code extension put them),
runs the port's `make` with the settings this car needs, and turns the
result into a `.uf2` file with `picotool`. Every `.c` file in this project
gets compiled to machine code and linked together with the kernel into one
program.

Two settings are baked into the script on purpose, so you can never
forget them: **`CONSOLE=usb_cdc`** (GP0/GP1 are the left encoder, so the
console must use USB — `docs/HARDWARE.md` §1.5) and **`E2U=`** (skips the
port's own ELF-to-UF2 tool, which would need a C++ compiler you probably
don't have; `picotool` does that job instead).

**What success looks like:** the last line says
`== flash this : build/out/mtk3pico_smp0_usb_cdc.uf2`, and above it there
were **no errors and zero warnings**. That `.uf2` is the file you put on
the Pico in the next step. (Verified: both profiles build clean with the
15.2 toolchain.)

**If the build fails:** read the *first* error message, not the last —
one real error often causes a cascade of confusing follow-on errors below
it. The most common beginner mistakes are: `make` not installed or Git
Bash not reopened after installing it (§0.5.1), the submodule folder
empty (§0.5.2), or `setup.sh` not run (§0.5.3 — the script tells you if
so). If you truly get stuck, screenshot the *first* error and ask a
teammate or search the exact error text — don't just retry the same
command hoping it changes.

#### 0.5.5 Flash it onto the actual Pico

1. **Unplug** the Pico if it's connected.
2. **Hold down the BOOTSEL button** on the Pico board (don't let go yet).
3. While still holding it, **plug in the USB cable** (into your laptop).
4. **Let go of BOOTSEL.** Your laptop should now show a new removable
   drive named `RPI-RP2`, the same way a USB flash drive would appear.
   If nothing appears, unplug and repeat from step 1 — timing the button
   press right sometimes takes a couple of tries.
5. **Copy the `.uf2` file onto that drive** — drag-and-drop
   `build/out/mtk3pico_smp0_usb_cdc.uf2` onto `RPI-RP2` in your file
   explorer, the same as copying a file onto a USB stick. (Or, with the
   Pico in BOOTSEL mode, run `./build/flash.sh` — same result, no
   dragging.)
6. The Pico will automatically reset and start running your program the
   moment the copy finishes — the `RPI-RP2` drive will disappear. That's
   normal, not an error.

If you ever want to reflash with new code, repeat this whole process —
there's no "uninstall" step, copying a new `.uf2` just overwrites what
was there.

#### 0.5.6 Watch it think — the serial console

The Pico has no screen, so the only window into what your program is
doing (print statements, error messages, sensor readings) is a **serial
console** — a stream of text. On this car it comes out of the **same USB
cable you flash over**: the `CONSOLE=usb_cdc` build makes the Pico show
up to your laptop as a serial port the moment it boots. No extra adapter,
no extra wires.

1. With the Pico plugged in and running your program (not in BOOTSEL
   mode), look for a new "COM port" (Windows, e.g. `COM5`) or device file
   (Mac/Linux, e.g. `/dev/tty.usbmodemXXXX`). It appears a second or two
   after the `RPI-RP2` drive disappears.
2. Open a serial terminal program on that port. Because it's USB, the
   baud rate genuinely doesn't matter — pick anything. Common free
   options:
   - Windows: **PuTTY** (connection type "Serial"), or the Serial Monitor
     panel inside VS Code's Raspberry Pi Pico extension.
   - Mac/Linux: `screen /dev/tty.usbmodemXXXX` from a terminal, or a GUI
     tool like CoolTerm.
3. Reset the Pico (unplug/replug, or the RST button on the Robo Pico) and
   you should see boot text appear, e.g. lines like `[init] event bus
   ok`, matching what `app/app_main.c`'s `step()` helper prints for each
   subsystem as it starts up. If every `step()` line says `ok` and you
   reach `[init] ready, starting run`, the whole framework initialized
   successfully — you're ready to start the hardware bring-up checklist
   below.

**If nothing appears:** the port only exists while the program is
running, so it vanishes during BOOTSEL/flashing and comes back after —
close and reopen your terminal program after each reflash. And if the
port never appears at all, the image wasn't built by `build/build.sh`
(which always sets `CONSOLE=usb_cdc`); rebuild with it (§0.5.4).

#### 0.5.7 Now bring the hardware up, one piece at a time

**Before you plug in a single wire, look at the two pin diagrams in
[`docs/img/hw/`](img/hw/):**

- [`robopico_pin_map.png`](img/hw/robopico_pin_map.png) — the Robo Pico
  drawn as it looks on the bench, with colour-coded chips fanning out from
  every connector we use: the GPIO number, the function in use (I²C / ADC
  / PWM) and the device on that socket. Find the socket on the picture,
  follow its chips, and you know what plugs in and what `GP3` means in
  the code. This is the one in the design review.
- [`robopico_board_view.png`](img/hw/robopico_board_view.png) — the same
  information drawn onto a picture of the actual board. Use this when you
  are holding the board and want to know which socket a device plugs
  into. Grove 1 is on the left edge, Grove 7 on the right edge, Grove 2–6
  along the bottom, and the motor terminals and servo header along the
  top.

In both, green means "this project uses it", grey means "broken out but
spare", and amber means "there was a conflict here and it had to be
patched — read the note underneath".

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
numbered checklist that tests one thing at a time. Each step has a
matching **bench image** (§0.8) that runs just that piece and prints its
readings, so you never have to debug the full mission to test one sensor.
In this order: blink
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
expected, not a sign something's broken. The good news: the extension
already installed everything the build needs except `make`, and the
scripts in `build/` know where it put them. Here's how what you already
know maps onto this project:

- **"Where's my toolchain?"** Already installed — the extension keeps its
  own copies of the compiler, the SDK and `picotool` under
  `%USERPROFILE%\.pico-sdk\`, one versioned folder per tool.
  `build/build.sh` looks there automatically (newest version wins), so you
  don't need to touch your `PATH`. The one thing to add is GNU `make`:
  `winget install ezwinports.make`, then reopen Git Bash.
- **Instead of "New C/C++ Project from Pico SDK,"** clone this repo with
  `--recurse-submodules` and run `./build/setup.sh` (§0.5.2–§0.5.3). The
  kernel port is a submodule, so there's nothing to download or copy by
  hand.
- **Instead of clicking "Compile Project,"** press **Ctrl+Shift+B** —
  the repo's `.vscode/tasks.json` wires the build to VS Code's build task
  (§0.5.9) — or run `./build/build.sh` in the integrated terminal, which
  this workspace sets to Git Bash. Same compiler under the hood, driven
  by a `Makefile` instead of the extension's CMake integration.
- **Instead of clicking "Run Project (USB),"** put the Pico in BOOTSEL
  mode and run `./build/flash.sh` (it calls the extension's own
  `picotool`), or drag `build/out/mtk3pico_smp0_usb_cdc.uf2` onto the
  `RPI-RP2` drive. Same result — there just isn't a single button for it.
- **The extension's built-in Serial Monitor panel works as-is.** The
  console on this car is routed over the Pico's own USB port (that's what
  the `CONSOLE=usb_cdc` setting baked into `build.sh` does), which is
  exactly the port the Serial Monitor panel watches. One caveat from
  `docs/HARDWARE.md` §4.1: the USB console is the part of the RTOS port
  flagged as less battle-tested. If output ever looks garbled, suspect
  that before suspecting your own code — but there is no UART fallback on
  this car, because GP0/GP1 are the left encoder.

Everything after this point — bring-up order, per-buddy work — is
identical no matter which path you used to get the `.uf2` flashed.

---

#### 0.5.9 No terminal at all: VS Code tasks

The repo ships a `.vscode/` folder that turns every build/flash command
into a menu item, so you never have to remember a directory or a shell:

1. Open the repo folder in VS Code (**File → Open Folder…**, pick
   `INF2004-IS07-AutonomousRoboCar`). Accept the prompt to install the
   recommended extensions (the Raspberry Pi Pico extension and C/C++).
2. **Terminal → Run Task…** and pick one of:
   - `RoboCar: setup` — once, after cloning (and after a submodule update).
   - `RoboCar: build mission image` — also bound to **Ctrl+Shift+B**.
   - `RoboCar: build bench image…` — a picker asks which buddy's bench.
   - `RoboCar: flash …` — with the Pico in BOOTSEL mode.
   - `RoboCar: clean`.
3. Output appears in the Terminal panel. Then open the Pico extension's
   **Serial Monitor** (or any serial tool) on the Pico's USB port to watch
   the console, exactly as in §0.5.6.

The tasks run the same `build/*.sh` scripts, always through Git Bash
(`.vscode/tasks.json` points at `C:\Program Files\Git\bin\bash.exe`),
and `.vscode/settings.json` makes Git Bash this workspace's default
terminal — so even the integrated terminal is the right shell. If Git is
installed somewhere unusual, edit that one path in `tasks.json`.

The Pico extension's own **Compile / Run** buttons still don't apply
(they need a CMake project, §0.5.8); the tasks are the VS Code way to
drive this build.

---

### 0.6 The `core/` toolbox — the nine shared files and who plugs into them

**First, a common confusion cleared up.** Two separate piles of code end up
on the Pico:

- **The RTOS port** (`mtk3smp-rp2040`, written by our lecturer). This is
  the *operating system*: it knows how to start the chip, run several
  tasks "at once", and provides every function whose name starts with
  `tk_` (`tk_cre_tsk`, `tk_wai_flg`, `tk_dly_tsk`…) or `tm_`
  (`tm_printf`). We never edit it — we only apply the two small patches
  from §0.5.3.
- **This project** (`core/`, `drivers/`, `subsystems/`, `app/`). Every
  file here was written by us for this car. Nothing in it is copied from
  the RTOS.

`core/` is the part of *our* code that everyone else's code leans on. Think
of it as the toolbox in the middle of the workshop: it doesn't build the
car, but every buddy reaches into it. It wraps the raw Pico chip and the
RTOS into a handful of simple services (a stopwatch, a doorbell
switchboard, a notice board, a dimmer switch…) so that the drivers and
subsystems can be written in plain terms like "publish this event" or "set
this pin to 40% power" instead of poking at hardware registers.

**Who uses what — the connection map**

| `core/` file | What it is (one line) | Used directly by | Which buddy that serves |
|---|---|---|---|
| `rc_prelude.h` | The mandatory first `#include` in every `.c` file | every `.c` file in the tree | everyone |
| `rc_types.h` | The shared dictionary: result codes, command names, event IDs, the event "parcel" | every file (pulled in by the other headers) | everyone |
| `rc_config.h` | The one settings sheet: every pin number and tuning constant | every driver, subsystem and `app_main.c` | everyone |
| `rc_time.*` | Microsecond stopwatch read straight from the chip | `drv_encoder`, `drv_ir`, `drv_ultrasonic`, `sub_terrain`, `sub_telemetry` | 2, 3, 4, 5, 1 |
| `rc_gpioirq.*` | The doorbell switchboard: routes the chip's single GPIO interrupt to the right driver | `drv_encoder` (GP0/GP7), `drv_ir` (GP27), `drv_ultrasonic` (GP3) | 2, 3, 5 |
| `rc_defer.*` | The "do it in a moment" list: lets an interrupt hand real work to a task | `drv_encoder`, `drv_ir`, `drv_ultrasonic` | 2, 3, 5 |
| `rc_event.*` | The notice board (publish / subscribe) that connects all subsystems | every driver and subsystem | everyone |
| `rc_pwm.*` | The dimmer switch: turns "40 % power" or "1500 µs pulse" into a PWM signal | `drv_motor` (20 kHz), `drv_servo` (50 Hz) | 2, 5 |
| `rc_fmt.*` | A tiny, safe text formatter for building telemetry messages | `sub_telemetry` | 1 |

Read the table by row: *"`rc_pwm` is the dimmer switch; the motor driver
and the servo driver use it; so it serves Buddy 2 and Buddy 5."* Each
buddy's section further down repeats just the rows that matter to them.

Below, each file in plain language. You do not need to understand the
insides to use them — the point is to know what each one *offers* and
which of your files calls it.

---

#### `rc_prelude.h` — "include me first"

**What it is.** A 50-line header whose only job is to be the first line of
every `.c` file. It fixes a naming clash: the RTOS defines a type called
`size_t` one way and the standard C library (which we need for
`memset`/`strlen`-style helpers) defines it another way. If both
definitions meet in one file, the compiler stops with
`conflicting types for 'size_t'`. It's like two colleagues named "Sam" in
one email thread — someone has to say which Sam we mean. `rc_prelude.h`
tells the RTOS "don't define `size_t`, let the C library do it" *before*
any RTOS header is read. That's why the rule is "first line, always".

**Who uses it.** Every `.c` file. If you create a new file and forget it,
the very first `#include <string.h>` will fail to compile.

#### `rc_types.h` — the shared dictionary

**What it is.** The definitions everyone has to agree on so the files can
talk to each other:

- `rc_result_t` — the answer every function gives back: `RC_OK`, or a
  named reason for failure (`RC_ERR_BUSY`, `RC_ERR_TIMEOUT`,
  `RC_ERR_HARDWARE`…). Every driver and subsystem function returns one of
  these instead of a bare number, so a failure is readable at a glance.
- `rc_nav_cmd_t` — the four barcode commands plus stop:
  `RC_CMD_TURN_LEFT`, `RC_CMD_TURN_RIGHT`, `RC_CMD_GO_STRAIGHT`,
  `RC_CMD_U_TURN`, `RC_CMD_STOP`. Buddy 3 produces these; `sub_nav`
  consumes them.
- `rc_evt_id_t` — the list of every event that can appear on the notice
  board (`RC_EVT_ODOMETRY`, `RC_EVT_LINE_SAMPLE`, `RC_EVT_HUMP_END`…).
  Adding a new kind of message to the car means adding one line here.
- `rc_event_t` — the **parcel** itself. It has a label (`id`), a
  timestamp (`t_us`, stamped automatically when published), and a
  compartment `u` that holds *one* of many possible contents: `u.odometry`
  for speed/distance, `u.line` for the two IR sensor bits, `u.ultra` for a
  range reading, and so on. In C this "one box, many possible contents" is
  called a *union*: the box is only as big as the largest item, and the
  `id` label tells the reader which item is inside. A subscriber reads
  `evt->id` first and then the matching `evt->u.xxx`.

**Who uses it.** Everyone, automatically — every other `core/` header
includes it.

#### `rc_config.h` — the settings sheet

**What it is.** A single header containing every GPIO pin number, every
timing period, every task priority and every mechanical measurement
(wheel diameter, encoder slots, wheel base). Nothing else in the tree is
allowed to contain a bare pin number. Changing a pin, a sample rate or a
priority means editing exactly one line here.

The file is split into blocks: *fixed by the Robo Pico board* (motors,
servo ports, buzzer, buttons — do not touch), *chosen by us* (encoders,
IR sensors, ultrasonic, IMU — move if your wiring differs), *timing*,
*task priorities*, *mechanical constants*, and per-subsystem tuning
(ultrasonic limits, scan geometry, event ring sizes).

**Who uses it.** Every driver, every subsystem, `app_main.c`, and three of
the `core/` files themselves. If you're looking for "which pin is the
right encoder on?" or "how often does the PID run?" the answer is here.

#### `rc_time.c` / `rc_time.h` — the stopwatch

**What it is.** A way to read *microseconds* (millionths of a second)
since the chip powered on. The RTOS's own clock only ticks every 1 ms,
which is far too coarse for two things this car does: measuring how long
an ultrasonic echo takes to come back (1 ms of error is 17 cm of range)
and measuring how wide a barcode bar is as it slides past the sensor. The
RP2040 has a free-running 1 MHz counter in hardware, and `rc_time_us()`
simply reads it — no interrupt, no RTOS call, so it is safe to read from
inside an interrupt.

Three functions:

- `rc_time_us()` — the raw microsecond count.
- `rc_time_since(then)` — microseconds elapsed since an earlier reading.
  Use this rather than `now - then` by hand: the counter is 32-bit and
  wraps back to zero every ~71 minutes, and this function is written so
  the wrap doesn't produce a wrong answer.
- `rc_time_ms()` — the same clock in milliseconds, for less precise uses
  like "how long did the hump last" or telemetry timestamps.

**Who uses it.** `drv_encoder` (time between clicks → speed), `drv_ir`
(time between barcode edges → bar width), `drv_ultrasonic` (echo flight
time → distance), `sub_terrain` (hump duration), `sub_telemetry`
(message timestamps). `rc_event` also stamps every parcel with it.

#### `rc_gpioirq.c` / `rc_gpioirq.h` — the doorbell switchboard

**What it is.** The RP2040 has 30 GPIO pins but only *one* interrupt line
for all of them together. Four of our pins need to trigger code the
instant they change (left encoder A on GP0, right encoder A on GP7,
ultrasonic echo GP3, barcode sensor GP27). Picture an apartment block with one shared
doorbell: when it rings, someone has to look at the panel to see which
flat was pressed and go tell *that* tenant. `rc_gpioirq` is that
concierge. It claims the one interrupt from the RTOS, and when it fires it
reads the chip's status register, works out which pin(s) changed, and
calls the small handler the owning driver registered for that pin —
passing the pin number, whether it went high or low, and a microsecond
timestamp taken at the door.

Three functions a driver uses:

- `rc_gpioirq_attach(pin, edges, pull, handler, ctx)` — "ring my handler
  when this pin goes up / down / either way". Also sets the pin up as an
  input with the requested pull resistor.
- `rc_gpioirq_enable(pin, on/off)` — temporarily mute one pin without
  forgetting the registration. The ultrasonic driver uses this so echo
  edges are only listened to while a ping is actually in flight.
- `rc_gpioirq_detach(pin)` — undo attach.

The handler you register runs *inside the interrupt*, so the rule in §0.2
applies: timestamp, update a counter, ring the defer bell, return. Nothing
else. On the dual-core build the interrupt is owned by core 0 only, as the
port requires.

**Who uses it.** `drv_encoder` (rising edges on GP0/GP7, reading GP1/GP28
for direction inside the handler), `drv_ir` (both edges on GP27),
`drv_ultrasonic` (both edges on GP3).

#### `rc_defer.c` / `rc_defer.h` — the "do it in a moment" list

**What it is.** The partner to `rc_gpioirq`. An interrupt handler is only
allowed a few microseconds and may not do anything that could wait, loop
or call into another subsystem. But the *result* of an interrupt — "a
barcode edge arrived, decode it", "the echo came back, publish the
range" — is real work. `rc_defer` is how the interrupt hands that work to
a normal task without doing it itself.

Each driver registers a **drain** function once at start-up
(`rc_defer_register(my_drain, ctx)` → returns a small handle number). Then,
inside its interrupt handler, the driver calls
`rc_defer_signal_i(handle)` — a single RTOS call that sets one bit in an
event flag, the RTOS equivalent of pressing a "please come" button. A
dedicated **Defer task** (priority 4, the highest in the car) is sleeping
on that flag; it wakes, sees which bits are set, and calls the matching
drain functions in task context, where publishing events and calling
callbacks is allowed. Because the Defer task sits above both event
dispatchers, the drain normally runs within microseconds of the interrupt
that asked for it.

Up to 16 drains can be registered; the car uses three. `rc_defer_count()`
tells you how many times drains have been requested, handy for spotting a
sensor that's "ringing the bell" far more often than it should.

**Who uses it.** `drv_encoder` (`encoder_drain`), `drv_ir`
(`barcode_drain`), `drv_ultrasonic` (`ultra_drain`). `app_main.c` calls
`rc_defer_init()` before anything else so the task exists before any
driver registers with it.

#### `rc_event.c` / `rc_event.h` — the notice board

**What it is.** §0.2 already explains the idea (publish / subscribe, two
lanes). This is what's inside:

- Each lane owns a **ring buffer** of 64 parcel slots — a fixed circle of
  pigeon-holes. Publishing copies your `rc_event_t` into the next free
  slot and moves the "write" pointer on; the dispatcher reads from the
  "read" pointer and moves it on. When write catches up with read the ring
  is full and the parcel is *dropped and counted* (`rc_event_dropped()`),
  never blocked on. Bounded and predictable is the whole point.
- Each lane has a **dispatcher task** (FAST at priority 5, SLOW at
  priority 9). It sleeps on an event flag; every publish sets the flag; the
  task wakes, pops parcels one by one and calls every subscriber for that
  parcel's `id`, in the order they subscribed. Up to 32 subscriptions per
  lane.
- Two ways to publish: `rc_event_publish()` from a task, and
  `rc_event_publish_i()` from inside an interrupt (it uses the
  interrupt-safe lock). Both fill in the timestamp for you.
- `rc_event_subscribe(id, lane, callback, ctx)` returns a handle;
  `rc_event_unsubscribe(handle)` removes it.

The protection around the ring is a few-microsecond critical section:
interrupts masked on single core, plus a spinlock on the dual-core build.
There are no mutexes anywhere on the control path — subsystems never share
variables, they only pass parcels.

**Who uses it.** Everything. Drivers publish raw readings
(`ENCODER_EDGE`, `LINE_SAMPLE`, `IMU_SAMPLE`, `ULTRA_RESULT`,
`BARCODE_EDGE`); subsystems subscribe to those and publish their
conclusions (`ODOMETRY`, `LINE_LOST`, `BARCODE_DECODED`, `HUMP_END`,
`OBSTACLE_PROFILE`, `AVOIDANCE_PLAN`); `sub_nav` and `sub_telemetry`
subscribe to the conclusions. `app_main.c` calls `rc_event_init()` second,
straight after `rc_defer_init()`, because every other `init` subscribes or
publishes.

#### `rc_pwm.c` / `rc_pwm.h` — the dimmer switch

**What it is.** A motor can't be told "run at 40 %". What you *can* do is
switch its power on and off very fast — say 20,000 times a second — and
leave it on for 40 % of each cycle. The motor's inertia smooths that into
40 % of full speed. That trick is **PWM** (pulse-width modulation). A
hobby servo uses the same idea at a different speed: 50 pulses a second,
and the *width* of each pulse (1000–2000 µs) tells the servo which angle
to hold.

The RTOS port gives us the raw PWM registers but no way to set the slow
frequency a servo needs; `rc_pwm` adds that one missing register write
and wraps everything in four calls:

- `rc_pwm_init_pin(pin, freq_hz)` — set the pin's PWM channel to this
  frequency and start it at 0 %.
- `rc_pwm_set_duty(pin, permille)` — 0..1000, i.e. tenths of a percent.
  (We use "permille" everywhere instead of "percent" so we never need a
  decimal point — there's no floating point on this chip.)
- `rc_pwm_set_pulse_us(pin, us)` — set the pulse width directly in
  microseconds; what servos want.
- `rc_pwm_enable(pin, on/off)`.

One catch worth knowing: the RP2040's PWM hardware is organised in
**slices** of two pins each, and both pins in a slice must share the same
frequency. The pin map in `rc_config.h` was chosen so that the two pins of
each motor (GP8/GP9, GP10/GP11) and the scan servo (GP15, slice 7) each
land on their own slice. Don't move a motor pin onto the servo's slice.

**Who uses it.** `drv_motor` (both motors at 20 kHz — above hearing, so
no whine), `drv_servo` (50 Hz, pulse in µs).

#### `rc_fmt.c` / `rc_fmt.h` — the tiny text printer

**What it is.** Telemetry needs to build text like
`{"state":2,"speed":250,"dist":1830}`. The standard C way is `snprintf`,
but on this chip that drags in 10–20 kB of library code and, in some
configurations, calls `malloc` from inside a periodic task — both things
the resource-efficiency marking scheme penalises. `rc_snprintf` is our own
150-line replacement that supports only what telemetry needs
(`%d %u %ld %lu %c %s %%`), never allocates memory, and always leaves the
output properly terminated even when the buffer is too small.
`rc_strlen` is the matching string-length helper.

**Who uses it.** `sub_telemetry` only. If you ever need to format text
elsewhere, use this rather than `snprintf` so the rest of the tree stays
consistent.

---

#### How `app/app_main.c` wires the toolbox together (start-up order)

The order in `usermain()` is not arbitrary; each step needs the one before
it:

1. `rc_defer_init()` — the Defer task must exist before any driver
   registers a drain.
2. `rc_event_init()` — the notice board (and `rc_time_init()`, which it
   calls) must exist before anyone subscribes or publishes.
3. `rc_gpioirq_init()` — the switchboard must own the interrupt before
   any driver attaches a pin.
4. Drivers: motor, encoder, servo, ultrasonic, IR, then IMU (the IMU is
   allowed to fail — the car still runs, it just can't measure humps).
5. Subsystems: motion, line, barcode, terrain, scan, telemetry, and
   finally `sub_nav`, because it subscribes to all the others' events.
6. Two housekeeping tasks (Sense at priority 7, Blink at 10), IMU
   calibration, then `sub_nav_start()`.

If you add a new driver or subsystem, slot its `init` into the same
pattern: after the `core/` services it needs, before anything that
subscribes to it.

---

### 0.7 Hardware — who owns which physical part (with pictures)

Every part in the kit has exactly one buddy responsible for wiring it,
calibrating it and writing its driver — except the handful of *shared*
parts that everyone's code runs through. Use this section to recognise
each part on the bench and to know who to ask about it.

Where the pictures come from: the Robo Pico, Pico W, HC-SR04, GY-511 and
TCRT5000 images are lifted from the datasheets in `documents/`. The
motor, encoder, servo, IR module and battery have no datasheet in the
folder, so those are labelled drawings of the standard part — **when the
kit is in front of you, photograph the real thing and drop it into
`docs/img/hw/` under the same filename**; this guide will pick it up
without any other edit.

**Ownership at a glance**

| Part | Qty | Owner | Plugs into | Picture |
|---|---|---|---|---|
| Raspberry Pi Pico W | 1 | **shared** | Robo Pico's Pico socket | [Pico W pinout](#shared-parts) |
| Cytron Robo Pico carrier board | 1 | **shared** | — (everything plugs into it) | [Robo Pico](#shared-parts) |
| Single-cell LiPo battery | 1 | **shared** | Robo Pico LiPo socket | [LiPo](#shared-parts) |
| Chassis, wheels, castor, Grove cables, status LED | — | **shared** | see `docs/HARDWARE.md` §8 | — |
| DC gear motor + wheel | 2 | **Buddy 2** | Left → MOTOR 2 terminal (GP10/GP11), right → MOTOR 1 terminal (GP8/GP9) | [motor](#buddy-2-hardware) |
| Wheel encoder, two-channel A/B | 2 | **Buddy 2** | Grove 1 → GP0/GP1 (left), Grove 7 → GP7/GP28 (right) | [encoder](#buddy-2-hardware) |
| MH-Sensor-Series IR module (TCRT5000 + LM393) | 3 | **Buddy 3** | Grove 4 → GP16, Grove 5 → GP6 (line); Grove 6 → GP27 + GP26/ADC0 (barcode) | [IR module](#buddy-3-hardware) |
| GY-511 breakout (LSM303DLHC accel + magnetometer) | 1 | **Buddy 4** | Grove 3 → I2C0 on GP4 (SDA) / GP5 (SCL) | [GY-511](#buddy-4-hardware) |
| HC-SR04 ultrasonic ranger | 1 | **Buddy 5** | Grove 2 → GP2 (TRIG), GP3 (ECHO via divider) | [HC-SR04](#buddy-5-hardware) |
| SG90-class servo + pan bracket | 1 | **Buddy 5** | Robo Pico servo port 4 (GP15) | [servo](#buddy-5-hardware) |
| 1 kΩ + 2 kΩ resistors (ECHO divider) | 1 each | **Buddy 5** | inline on the ECHO wire | — |
| WiFi radio (CYW43439, on the Pico W itself) | — | **Buddy 1** | nothing to wire | [Pico W pinout](#shared-parts) |

Two rules that follow from the table:

1. **If it's shared, don't change it without telling everyone.** Moving a
   jumper on the Robo Pico, re-flashing with a different pin patch, or
   swapping the battery affects every buddy's testing.
2. **If it's yours, it's yours end to end** — mounting, wiring, the
   `drivers/` file, calibration, and the "is it plugged in?" question at
   integration time.

<a id="shared-parts"></a>
#### Shared parts

**Raspberry Pi Pico W** — the computer. A green board with a silver RP2040
chip in the middle and a metal-canned WiFi module (the CYW43439) near the
USB end. It sits in the Robo Pico's socket, USB connector facing outwards.
Everything in this repo runs on it. The pinout below is the one you'll
keep coming back to: our pin numbers (GP2, GP16, …) are the green
`GPn` labels, *not* the physical pin numbers 1–40.

<img src="docs/img/hw/pico_w_pinout.jpg" width="720" alt="Pico W pinout">

**Cytron Robo Pico** — the purple carrier board the Pico W plugs into. It
provides the motor driver (so the Pico's 3.3 V pins can drive 6 V
motors), four servo ports, seven Grove connectors, a LiPo charger and
power switch, a buzzer, two buttons and a Neopixel LED. Nobody "owns" it
because every buddy's part plugs into it somewhere — but Buddy 2 uses it
most heavily (the motor terminals *are* the Robo Pico's motor driver).

<img src="docs/img/hw/robo_pico_photo.jpg" width="380" alt="Robo Pico photo">  <img src="docs/img/hw/robo_pico_labelled.jpg" width="440" alt="Robo Pico labelled">

Where the fixed pins live on it (from `core/rc_config.h`): motors on
GP8–GP11, servo ports on GP12–GP15, buzzer GP22, buttons GP20/GP21,
Neopixel GP18. Those cannot be moved — they're wired on the board.

**Battery** — a single-cell 3.7 V LiPo pouch with a 2-pin JST plug that
goes into the Robo Pico's LiPo socket. The Robo Pico charges it over USB
and its power switch turns the whole car on and off. Note the HC-SR04
wants 5 V and will lose range on a sagging battery — see
`docs/HARDWARE.md` §3.

<img src="docs/img/hw/lipo_battery.png" width="360" alt="LiPo battery">

**Also shared, no picture needed:** the chassis and castor wheel; seven
Grove cables (one per socket); the external liveness LED on GP19 with its
330 Ω resistor. There is no USB-serial adapter — the console rides the
Pico's own USB cable (§0.5.6).

<a id="buddy-2-hardware"></a>
#### Buddy 2 — Motion: motors and encoders

**DC gear motor + wheel (×2).** The classic yellow "TT motor": a yellow
plastic gearbox with a silver motor can on the back and two wires. Each
one screws into a Robo Pico motor terminal (left → M1, right → M2). The
Robo Pico drives each motor with two PWM pins, so "forward", "reverse"
and "brake" are all done in `drv_motor.c` by choosing which pin gets the
duty.

<img src="docs/img/hw/motor_wheel.png" width="460" alt="TT gear motor with wheel and encoder disc">

**Wheel encoder, two-channel (×2).** Sits on the motor's rear axle and
gives two pulse outputs, `A` and `B`, that are the same train of pulses
shifted by a quarter of a step. Every step of the wheel gives one pulse
on `A` → speed and distance in `drv_encoder.c`. And because `B` is
shifted, whether `B` is low or high at the instant `A` rises tells you
which way the wheel is turning — so direction is *measured*, not guessed.
Four wires: `GND`, `VCC`, `A`, `B`, which is exactly one Grove cable —
left encoder into **Grove 1** (`A` → GP0, `B` → GP1), right encoder into
**Grove 7** (`A` → GP7, `B` → GP28). If a wheel reads backwards, swap
that encoder's `A` and `B` wires.

<img src="docs/img/hw/encoder_module.png" width="460" alt="wheel encoder module">

<a id="buddy-3-hardware"></a>
#### Buddy 3 — Line following and barcode: IR reflective modules

**IR reflective module (×3).** A small board with a sensor element at one
end that points *down* at the floor: it shines infrared light and
measures how much bounces back — a lot from white, very little from
black. An on-board LM393 comparator with a trim pot turns that into a
clean `DOUT` high/low ("black / not black"); `AOUT` gives the raw
analogue level. Two of these are the line sensors, wired on `DO` only
(sensor 1 → GP16 on Grove 4, sensor 2 → GP6 on Grove 5); the third is the
barcode reader on Grove 6, wired on *both* outputs (`DO` → GP27 for edge
timing, `AO` → GP26 for the analogue path). **On Grove 5 leave line sensor
2's `AO` pin unconnected** — that socket's second signal pin is also GP26,
and two analogue outputs tied together read as nonsense. Your kit has the
MH-Sensor-Series board (a TCRT5000 element + LM393 comparator; the
Waveshare ST188 board is pin-compatible); the TCRT5000 element itself is
the small black block with two domes
shown on the right.

<img src="docs/img/hw/ir_module.png" width="460" alt="IR reflective module">  <img src="docs/img/hw/tcrt5000_element.png" width="180" alt="TCRT5000 element">

<a id="buddy-4-hardware"></a>
#### Buddy 4 — IMU: GY-511 breakout

**GY-511 (LSM303DLHC) breakout (×1).** A small blue board, about the size
of a fingernail, with an 8-pin header labelled `VIN 3.3V GND SCL SDA
INT2 INT1 DRDY`. Only four wires are used: 3.3 V → `VIN`, GND, SDA → GP4,
SCL → GP5 (Robo Pico Grove 3 — note SDA is the *lower* pin number, that
is fixed by the chip). It contains an accelerometer (tilt — how far the
nose is up or down, which is how humps are detected) and a magnetometer
(compass). **It has no gyroscope**, despite what many online tutorials
for "GY-511" assume — see `docs/HARDWARE.md` §1.1. Mount it flat and
firmly; a wobbling IMU reports a wobbling road.

<img src="docs/img/hw/gy511_lsm303dlhc.jpg" width="380" alt="GY-511 LSM303DLHC breakout">

<a id="buddy-5-hardware"></a>
#### Buddy 5 — Scanning: ultrasonic ranger and servo

**HC-SR04 ultrasonic ranger (×1).** The blue board with two silver
"eyes": one speaker (T) that shouts an inaudible click, one microphone
(R) that listens for the echo. Four pins on Grove 2: `VCC` → 3V3, `Trig` →
GP2, `Echo` → GP3, `GND`. **`Echo` outputs 5 V and the Pico's pins are 3.3 V
only** — the 1 kΩ / 2 kΩ resistor divider on that wire is not optional.
Range 2 cm – 4 m; the flight time of the echo is what `drv_ultrasonic.c`
measures with the microsecond stopwatch.

<img src="docs/img/hw/hcsr04.jpg" width="360" alt="HC-SR04 ultrasonic sensor">

**SG90-class servo + pan bracket (×1).** The small blue plastic servo with
a white horn and a three-wire lead (brown/black = GND, red = V+ i.e. battery voltage,
orange/yellow = signal). It plugs straight into Robo Pico servo port 4
(GP15, the right-most of the four) — the port has the 3-pin header in the
right order. The HC-SR04
bolts to a bracket on the horn so the scan task can point it from 30° to
150°. Centre it at 90° *before* attaching the bracket.

<img src="docs/img/hw/servo_sg90.png" width="400" alt="SG90 servo">

#### Buddy 1 — no physical part to wire

The WiFi radio is the CYW43439 module already on the Pico W (the metal
can at the USB end in the pinout picture above). Your hardware work is
limited to making sure every build carries `CONSOLE=usb_cdc` so the
console rides the USB cable (GP0/GP1 belong to the left encoder) and,
later, confirming the radio comes up through the port's `libwifi`. If a
laptop hotspot or router is needed for the demo, that's yours to bring.

---

### 0.8 Running just *your* module — bench modes

The normal image runs the whole mission: line following, barcodes,
avoidance, everything at once. That's the wrong tool for bring-up. If the
car misbehaves, five people's code is a suspect. So the build has **bench
modes**: an image that initialises everything exactly as the mission
does, but then — instead of starting the mission — runs *one* buddy's
subsystem on its own and prints what it sees to the USB console, forever.

You pick the bench when you build, and the image gets a matching name so
nobody flashes the wrong one by accident:

```sh
./build/build.sh bench=imu          # -> build/out/mtk3pico_smp0_usb_cdc_bench-imu.uf2
./build/flash.sh bench=imu          # Pico in BOOTSEL first
```

Run those from the repo folder in **Git Bash** (§0.5.3), or in VS Code:
**Terminal → Run Task → RoboCar: build bench image…** then **…flash bench
image…** and pick the bench from the list (§0.5.9).

Then open the USB serial port (§0.5.6). The boot log ends with
`[init] ready, BENCH MODE: imu` instead of `starting run`, followed by a
banner saying what the bench expects you to do.

| Bench | Buddy | Motors? | What it does |
|---|---|---|---|
| `motion` | 2 | **yes** | Open-loop spin with encoder readout, then one closed-loop 300 mm move, then hand-turn mode |
| `line` | 3 | no | Prints both line sensors, position estimate and follower state 10× a second |
| `follow` | 3 | **yes** | Same, but the line follower is enabled — the car drives along the line |
| `barcode` | 3 | no | Decoder armed permanently; prints every bar/space width and every decoded symbol |
| `imu` | 4 | no | Prints accelerometer, pitch, motion class and max hump peak 5× a second; announces humps |
| `ultra` | 5 | no | Servo parked at 90°, one ping every 100 ms, range printed |
| `scan` | 5 | no | Full coarse + fine sweep every few seconds, every ping printed, then the profile and plan |
| `telemetry` | 1 | no | Everything sensing, car stationary; the `[telem]` stream at 4 Hz plus dropped-event counts |

All the bench code is in one file, `app/app_bench.c`, one function per
bench, deliberately plain — read yours before running it, and edit it
freely (a different duty, a longer move, an extra field printed). It is
*your* test harness. The mission image never contains any of it: the
selection is a compile-time constant, so an unselected bench is
dead-stripped.

Each buddy's section below has a **"Run your module"** box with the
command, what good output looks like, and what to do when it isn't.

**Which is the right one to start with?** Follow `docs/HARDWARE.md` §7 in
order: `motion` (steps 3–5) → `line` then `follow` (6–7) → `ultra` then
`scan` (8–10) → `imu` (11–12) → `barcode` (13) → `telemetry` (14) → the
mission image (16).

---

## 1. Roles at a glance

| Buddy | Files you own | What you're building |
|---|---|---|
| 1 | `subsystems/sub_telemetry.*` | Reporting the car's status over WiFi (and receiving commands, eventually) |
| 2 | `subsystems/sub_motion.*`, `drivers/drv_motor.*`, `drivers/drv_encoder.*` | Making the wheels move the right speed/distance/angle |
| 3 | `subsystems/sub_line.*`, `subsystems/sub_barcode.*`, `drivers/drv_ir.*` | Staying on the black line, reading barcodes |
| 4 | `subsystems/sub_terrain.*`, `drivers/drv_imu.*` | Detecting speed humps and classifying how the car is moving |
| 5 | `subsystems/sub_scan.*`, `drivers/drv_ultrasonic.*`, `drivers/drv_servo.*` | Scanning for obstacles and planning a way around them |

Each buddy's section is its own file, so you can keep just yours open:

| Buddy | Your guide | Files you own |
|---|---|---|
| 1 | [`docs/guide/buddy1-telemetry.md`](docs/guide/buddy1-telemetry.md) | `subsystems/sub_telemetry.*` |
| 2 | [`docs/guide/buddy2-motion.md`](docs/guide/buddy2-motion.md) | `subsystems/sub_motion.*`, `drivers/drv_motor.*`, `drivers/drv_encoder.*` |
| 3 | [`docs/guide/buddy3-line-barcode.md`](docs/guide/buddy3-line-barcode.md) | `subsystems/sub_line.*`, `subsystems/sub_barcode.*`, `drivers/drv_ir.*` |
| 4 | [`docs/guide/buddy4-imu-terrain.md`](docs/guide/buddy4-imu-terrain.md) | `subsystems/sub_terrain.*`, `drivers/drv_imu.*` |
| 5 | [`docs/guide/buddy5-scan-avoidance.md`](docs/guide/buddy5-scan-avoidance.md) | `subsystems/sub_scan.*`, `drivers/drv_ultrasonic.*`, `drivers/drv_servo.*` |

Each one has the same shape: what the module does, **Run your module**
(the bench command and what good output looks like), how to get started,
a function-by-function code walkthrough, and the TODO list that is your
graded work.

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
- **Build and check for warnings after every change**: `./build/build.sh`
  (single core) and `./build/build.sh smp` (dual core). Both must stay at
  zero warnings.
