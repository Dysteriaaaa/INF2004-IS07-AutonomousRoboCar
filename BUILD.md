# Build & Test Guide — from zero to a driving car

This is the "how do I actually run it" guide. It assumes you have **never
used a Raspberry Pi Pico, never written C, and have never built code from a
terminal**. Every step is here, in order, with the exact menu item or
button to click.

**The environment this guide is written for is Visual Studio Code with the
Raspberry Pi Pico extension, on Windows.** Every step below can be done
from inside VS Code. You do **not** need WSL / Ubuntu for anything — in
fact WSL gets in the way (see §7). If you already know your way around a
terminal, §6 gives the equivalent commands.

The other two guides:

- [`TEAM_GUIDE.md`](TEAM_GUIDE.md) — how the code is organised, the shared
  code everyone runs on, the shared hardware and who owns which part.
- [`docs/buddyN-…/`](docs/) — one folder per buddy (guide + datasheets): your hardware, your
  wiring, your code explained line by line, your TODOs.

---

## 0. Words you'll see, defined once

- **Pico** — the Raspberry Pi Pico W, the small green board that is the
  car's computer. It has no screen or keyboard; the only way to see what it
  is doing is the *console* (below).
- **Robo Pico** — the purple carrier board the Pico plugs into. All sensors
  and motors connect to *it*.
- **Firmware / image** — the program we compile, as one file that the Pico
  runs. Ours end in `.uf2`.
- **Build / compile** — turning the `.c` source files into that `.uf2`.
- **Flash** — copying the `.uf2` onto the Pico so it runs on every power-up.
- **BOOTSEL** — the small white button on the Pico. Holding it while
  plugging in USB makes the Pico show up as a USB drive named `RPI-RP2`,
  ready to receive a `.uf2`.
- **Console** — text the Pico prints (boot messages, sensor readings). On
  this car it comes out of the Pico's own USB cable and appears in a
  serial-monitor window on your laptop.
- **Bench image** — a build that runs *one* buddy's part on its own and
  prints its readings, instead of the whole mission. This is how you test
  your piece without everyone else's code being involved.
- **Repo folder** — the folder you get when you clone the project,
  `INF2004-IS07-AutonomousRoboCar`. Everything in this guide happens with
  that folder open in VS Code.

---

## 1. Install the tools (once per laptop, ~15 minutes)

You need exactly three things. The Pico extension does most of the work.

### 1.1 Visual Studio Code + the Raspberry Pi Pico extension

1. Install VS Code from code.visualstudio.com.
2. Open VS Code → the Extensions view (the four-squares icon on the left,
   or `Ctrl+Shift+X`) → search **Raspberry Pi Pico** → Install (publisher:
   Raspberry Pi).
3. The first time the extension activates it downloads the ARM compiler,
   the Pico SDK, `picotool` and OpenOCD into
   `C:\Users\<you>\.pico-sdk\`. Let it finish (a few minutes). Our build
   script finds everything there automatically — there is nothing to
   configure and nothing to add to your PATH.

### 1.2 Git for Windows

Install from git-scm.com, accepting the defaults. This gives VS Code the
ability to clone the project and, importantly, installs **Git Bash**, the
small shell our build scripts run in. You will not need to open Git Bash
yourself — VS Code will run it for you — but it must be installed.

### 1.3 That's it

No `make`, no C++ compiler, no serial adapter, no WSL. The one remaining
tool (GNU `make`) is downloaded automatically by the build script the
first time you build (about 400 kB into `build/tools/`).

> **Mac / Linux:** install VS Code + the Pico extension the same way, then
> `xcode-select --install` (Mac) or `sudo apt install git make` (Linux).
> Everything else below is identical.

---

## 2. Get the code into VS Code

1. In VS Code press `Ctrl+Shift+P`, type **Git: Clone**, press Enter.
2. Paste the repository URL:
   `https://github.com/Dysteriaaaa/INF2004-IS07-AutonomousRoboCar.git`
   and press Enter.
3. Choose a folder to put it in (your Desktop is fine). When VS Code asks
   **"Would you like to open the cloned repository?"** click **Open**.
4. VS Code will suggest installing the *recommended extensions* for this
   workspace (the Pico extension and C/C++). Click **Install**.

You now have the repo folder open. On the left you'll see `app/`, `core/`,
`drivers/`, `subsystems/`, `docs/`, `build/` and an empty
`external/mtk3smp-rp2040/` — the empty one is filled in the next step.

---

## 3. First-time setup (once, ~1 minute)

The car runs on a small operating system (micro T-Kernel) that lives in
a separate repository. This step fetches it, applies five small patches so
it stops using pins our sensors need, and tells its build system where
our code is.

1. Menu **Terminal → Run Task…**
2. Pick **`RoboCar: setup (first time / after submodule update)`**.
3. A terminal panel opens at the bottom and prints what it did. It ends
   with `done. Build with: ./build/build.sh`.

What just happened, in plain terms: `external/mtk3smp-rp2040/` was
downloaded (that's the OS), five lines in it were changed (the patches —
each one is explained in `docs/HARDWARE.md` §1), and one small makefile
was dropped into it so its build compiles our `core/ drivers/ subsystems/
app/` folders. Run this task again whenever someone updates the OS
version (you'll be told).

---

## 4. Build, flash, and watch the console

### 4.1 Build the mission image

Press **`Ctrl+Shift+B`** (or **Terminal → Run Build Task**). This runs
`RoboCar: build mission image`.

The terminal fills with `Building file: …` lines — one per source file —
and, on the very first build, a short `downloading a portable copy of
make` message. It takes about a minute. **Success looks like this at the
end:**

```
== flash this : build/out/mtk3pico_smp0_usb_cdc.uf2   (BOOTSEL + copy to RPI-RP2, or ./build/flash.sh)
```

and **no lines containing `error` or `warning`** above it. That `.uf2` is
your firmware.

If it fails, read the *first* error, not the last — one real error causes
a cascade of follow-on ones. §7 lists the usual causes.

### 4.2 Flash it onto the Pico

1. Unplug the Pico's USB cable if it's connected.
2. Hold down the **BOOTSEL** button on the Pico.
3. Still holding it, plug the USB cable into your laptop.
4. Let go. A USB drive called **`RPI-RP2`** appears in File Explorer. (If
   it doesn't, unplug and try again — the timing takes a couple of goes.)
5. Either:
   - **Terminal → Run Task… → `RoboCar: flash mission image`**, or
   - drag `build/out/mtk3pico_smp0_usb_cdc.uf2` onto the `RPI-RP2` drive
     in File Explorer, the same as copying a file to a USB stick.
6. The drive disappears and the Pico restarts running your program.
   That's success, not an error.

Reflashing later is the same six steps — a new `.uf2` simply replaces the
old one.

### 4.3 Watch the console (Serial Monitor)

The Pico has no screen. Everything it "says" is text over the USB cable.

1. A second or two after flashing, the Pico appears to Windows as a
   serial port (something like `COM5`).
2. In VS Code open the Pico extension's **Serial Monitor**: click the
   Pico icon in the left bar → **Serial Monitor**, or `Ctrl+Shift+P` →
   **Serial Monitor: Focus on Serial Monitor View**.
3. Pick the Pico's port from the **Port** dropdown (it's the one that
   appeared when you plugged in — unplug/replug to see which). Baud rate:
   any value works, it's USB.
4. Click **Start Monitoring**.
5. Press the **RST** button on the Robo Pico (or unplug/replug) to see the
   boot messages from the start. You should see:

```
=== robotic car: bring-up ===
[init] defer ok
[init] event bus ok
[init] gpio irq ok
[init] motor ok
[init] encoder ok
[init] servo ok
[init] ultrasonic ok
[init] ir ok
[init] imu ok
[init] motion ok
...
[init] hold still, calibrating IMU...
[init] ready, starting run
```

Every `ok` is one piece of the car initialising. `[init] imu FAILED` is the
only one the car tolerates (it carries on without hump detection) — and it
means the IMU is miswired, see the Buddy 4 guide. Any other `FAILED` stops
the boot.

**The port vanishes while the Pico is in BOOTSEL mode and comes back after
the flash.** So: flash first, then click Start Monitoring. If Serial Monitor
says the port is busy, nothing else may have it open (PuTTY, another VS
Code window).

---

## 5. Test one part at a time — bench images

Don't start with the whole mission. If the car misbehaves then, five
people's code are suspects. Instead, **each buddy has a bench image**: it
initialises everything exactly as the mission does, but then runs *only
that buddy's part* and prints what it sees, forever. You flash it, open the
Serial Monitor, and compare what you see with the "good output" below.

### 5.1 How to build and flash any bench image

1. **Terminal → Run Task… → `RoboCar: build bench image…`**
2. A list appears — pick your bench (each says which buddy it is for).
3. Wait for `== flash this : build/out/…_bench-<name>.uf2`.
4. Put the Pico in BOOTSEL mode (§4.2 steps 1–4).
5. **Terminal → Run Task… → `RoboCar: flash bench image…`**, pick the
   same bench (or drag the `.uf2` onto `RPI-RP2`).
6. Open the Serial Monitor (§4.3). The boot log now ends with
   `[init] ready, BENCH MODE: <name>` followed by a banner telling you what
   the bench expects you to do.

The bench code itself is one plain file, `app/app_bench.c`, one function
per bench. It's *your* test harness — read yours, and change it freely
(a different motor duty, an extra printed value). The mission image
contains none of it.

| Bench | Buddy | Motors? | Tests bring-up step (`docs/HARDWARE.md` §7) |
|---|---|---|---|
| `motion` | 2 | **yes** | 3–5: motors, encoders, closed-loop speed |
| `line` | 3 | no | 6: line sensors |
| `follow` | 3 | **yes** | 7: line following — the car drives |
| `barcode` | 3 | no | 13: barcode decoding |
| `imu` | 4 | no | 11–12: IMU, hump |
| `ultra` | 5 | no | 9: ultrasonic ranging |
| `scan` | 5 | no | 8, 10: servo sweep, full scan |
| `telemetry` | 1 | no | 14–15: telemetry stream |

Do them in the order the car needs to work: **motion → line → follow →
ultra → scan → imu → barcode → telemetry**, then the mission (§5.10).

### 5.2 `motion` — Buddy 2: motors and encoders

**Wheels off the ground for phase 1** (prop the chassis on a box). The
bench runs three phases and announces each:

1. *Open loop, 30 % duty, 4 s* — prints
   `L cnt=… spd=… dir=… | R cnt=… spd=… dir=…` every 250 ms. **Good:** both
   counts climb steadily and **both speeds are positive**.
   - A count that never moves → that encoder isn't wired or powered.
   - A count that jumps in bursts, or climbs while the wheel is still →
     electrical noise on the encoder wires. Check the encoder's GND and
     3V3, and keep its wires away from the motor wires. (Raising
     `DEBOUNCE_US` would hide it, but also throw away real ticks at
     speed.)
   - A **negative** speed → that encoder's A and B wires are swapped.
     Swap them; don't negate in code.
   - A wheel turning the wrong way → that *motor's* two wires are
     swapped at the terminal. Swap them.
2. *Closed-loop moves* — **put the car on the floor** with 1 m clear in
   front (you get 5 s). It drives forward 500 mm, backward 500 mm, turns
   right 90°, left 90° and does a U-turn, pausing 4 s after each so you
   can measure: a tape for distances, a protractor for turns. Each move
   prints `completed`/`ABORTED` and how far the wheels went.
   - The forward move also prints `t_ms,tgt_l,spd_l,duty_l,tgt_r,spd_r,duty_r`
     every 40 ms: each wheel's target and measured speed and its duty.
     That is the PID step response for your tuning report — paste it
     into a spreadsheet and plot speed against time.
   - The distance error is what you fold into `RC_ENC_UM_PER_TICK`; the
     turn error goes into `TURN_SLIP_PERMILLE` in `sub_motion.c`.
   - `ABORTED` with `[motion] … wheel not turning` means that wheel
     stalled or its encoder isn't counting; `move took too long` means
     the motors can't reach the speed.
   - Until `RC_ENC_TICKS_PER_REV` is measured (phase 3), these distances
     are meaningless — each move stops almost at once.
3. *Motors off* — turn a wheel by hand; only that side's count should
   change. Catches cross-wired encoders. **This is also where you
   measure `RC_ENC_TICKS_PER_REV`:** put a tape mark on the tyre, turn the
   wheel slowly exactly 10 turns in one direction, and divide the change
   in `cnt` by 10. Do both wheels, put the number in `rc_config.h`, and
   rebuild.

Edit `bench_motion()` in `app/app_bench.c` as tuning progresses — that's
where your PID step-response logs for the report come from.

### 5.3 `line` — Buddy 3: line sensors (motors off)

Prints `L=1 R=0 pos=-500 barcode_raw=… state=…` ten times a second.
`1` means "sees black". Slide the car sideways across the line by hand:

- Each bit must flip cleanly right at the line's edge. Flicker at the
  edge → adjust that module's trim pot (`docs/HARDWARE.md` §4.3).
- `state` goes `TRACKING` → `LOST` when you lift the car off the line,
  and `JUNCTION` when both sensors sit on black.
- A sensor that reads `1` over *white* has the opposite polarity: flip
  `IR_ACTIVE_HIGH` at the top of `drv_ir.c`.

### 5.4 `follow` — Buddy 3: the line follower (the car DRIVES)

Put the car on the line first. The follower is enabled and steers through
`sub_motion_drive()`. It should track a straight, then a curve. If it
weaves side to side, that's the missing derivative term — Buddy 3's TODO.
Base speed is `sub_line_set_base(350)` in `app/app_main.c`.

### 5.5 `barcode` — Buddy 3: the decoder (motors off)

The decoder is armed permanently. Pull a printed Code 39 barcode under
the sensor by hand, at two different speeds. You get one line per edge —
`edge -> black after 4120 us` — and `[bench] DECODED 'A' cmd=1` on a
match. **Good:** wide bars are consistently 2–3× the narrow ones
*regardless* of how fast you pulled, and `overruns=0` in the 5-second
status line.

- No edges at all → the sensor's DO isn't on GP27, or the trim pot is
  off.
- Edges but no decode → widths are outside 0.8–200 ms per element (car
  too fast or too slow over the code, or the trim pot is marginal so
  some bars are missed). Print `widths[]` from `on_edge()` to see.

### 5.6 `imu` — Buddy 4: accelerometer, pitch, humps (motors off)

Keep the car **level and still while it boots** — that's when
`drv_imu_calibrate()` runs. Then five times a second:
`acc x=… y=… z=… mg  pitch=… deg  class=…  max peak=… mm`.

- **Good:** level reads `z` ≈ 1000 mg and `pitch` ≈ 0.0; lift the nose
  and pitch goes **positive**; `class` is `STATIONARY` while it sits.
- Push the car over a book: `[bench] hump BEGIN`, then
  `hump END peak=… mm`, and `max peak` updates.
- `[init] imu FAILED` at boot → I²C wiring. SDA is GP4, SCL is GP5;
  swapping them is the usual cause.
- Pitch stuck at a constant non-zero value → the board isn't flat.

This bench is also how you log flat-ground pitch noise to choose
`PITCH_ENTER_DDEG` in `sub_terrain.c`.

### 5.7 `ultra` — Buddy 5: ranging straight ahead (motors off)

The servo parks at 90° and the sensor pings every 100 ms, printing
`90 deg: 312 mm`. Hold a book at 100, 300 and 1000 mm (tape measure):
readings should be within a few mm and steady.

- `no echo` every time → TRIG/ECHO swapped, or the 1 kΩ / 2 kΩ divider
  is missing.
- Wildly jumping values → the servo is buzzing against an end-stop, or
  the sensor is looking at the floor.

### 5.8 `scan` — Buddy 5: the full sweep (motors off)

Every few seconds: the servo steps 30° → 150°, each ping prints
`<angle> deg: <mm>`, then if something is close it re-scans around it in
6° steps, and finally the profile and plan print —
`closest 240 mm at 60 deg, width 90 mm, clearance L=… R=…` and
`plan: TURN_RIGHT lateral=… forward=…`.

- Readings at a new angle that look like the *previous* angle → the
  servo hadn't settled; raise the settle constants in `drv_servo.c`.
- The console must never pause during a sweep — that's the PID loop
  proving it keeps running (bring-up step 10).

### 5.9 `telemetry` — Buddy 1: the reporting stream (motors off)

Everything senses, the car stays still. **Good:** a
`[telem] car/01/state {...}` line four times a second whose values change
when you move the car by hand, a `[telem] car/01/status` heartbeat every
~2 s, and `[bench] dropped fast=0 slow=0` every 5 s. A non-zero dropped
count means a consumer is too slow for the event ring — your first real
bug. With WiFi enabled (§5.11) this is also where you point a `netcat`
listener (UDP sink) or `mosquitto_sub` (MQTT sink) and watch the same
lines arrive over the air.

### 5.10 The whole car — the mission image

When every bench above passes, build the mission image (§4.1), flash it
(§4.2), put the car on the track, and watch the console (§4.3). Boot ends
with `[init] ready, starting run`; from then on `[nav] …` lines announce
each mission-state change (following → reading barcode → executing →
avoiding → recovering). `TEAM_GUIDE.md` §2.2 explains those states.

The mission uses **every** buddy's part at once. If something is wrong,
go back to the bench that covers it rather than debugging the whole car.

### 5.11 Turning on WiFi telemetry (`RC_NET_ENABLE`)

By default telemetry goes to the **USB console** and the radio is off. That
is the reliable demo baseline — you do not need WiFi for anything above,
and the graded mission image builds with the radio compiled out.

Turning it **on** sends the exact same messages over **WiFi/MQTT** instead,
and lets the car receive commands on `car/01/cmd`. It takes three steps:

1. **Give the port a WiFi build.** The Pico W radio, lwIP and the MQTT
   client are a *development profile* of the `mtk3smp-rp2040` port — they
   are not linked in the normal build. Enable them once as described in
   [`docs/buddy1-telemetry/MQTT.md`](docs/buddy1-telemetry/MQTT.md)
   ("Enabling the radio path"). This is the only fiddly step, and it is a
   one-time change to the port, not to the car code.

2. **Fill in your network.** In `core/rc_config.h`, set `RC_WIFI_SSID`,
   `RC_WIFI_PASS`, `RC_WIFI_COUNTRY`, and the broker/listener addresses
   (`RC_MQTT_BROKER_IP`, `RC_UDP_DEST_IP`) to your demo hotspot and laptop.

3. **Flip the switch and build.** In `core/rc_config.h` change
   `#define RC_NET_ENABLE (0)` to `(1)` (or build with
   `RC_CFLAGS="-DRC_NET_ENABLE=1"`), then build the mission image as in
   §4.1. On boot you should see `[wifi] up` then `[mqtt] connected`.

**Quick test** — on the laptop running the broker (`mosquitto -v`):

```bash
mosquitto_sub -h <broker-ip> -t 'car/01/#' -v   # watch telemetry
mosquitto_pub  -h <broker-ip> -t car/01/cmd -m L # send "turn left"
mosquitto_pub  -h <broker-ip> -t car/01/cmd -m X # send "stop"
```

If WiFi is not up (or you skipped step 1), the car quietly falls back to
the console sink and keeps running — nothing else breaks. Full topic and
message details are in [`MQTT.md`](docs/buddy1-telemetry/MQTT.md).

---

## 6. Doing the same things from a terminal (optional)

Everything the tasks do is a script in `build/`. If you'd rather type:

1. Open VS Code's integrated terminal (**Terminal → New Terminal**, or
   ``Ctrl+` ``). This workspace sets it to **Git Bash**, which is the shell
   the scripts need. It opens in the repo folder already.
2. Run:

```sh
./build/setup.sh                  # once, after cloning
./build/build.sh                  # mission image  -> build/out/mtk3pico_smp0_usb_cdc.uf2
./build/build.sh bench=motion     # a bench image  -> build/out/..._bench-motion.uf2
./build/build.sh smp              # dual-core mission image (later)
./build/flash.sh                  # flash the mission image (Pico in BOOTSEL)
./build/flash.sh bench=motion     # flash that bench image
./build/build.sh clean            # start over
```

The scripts work from any current directory as long as you run them by
that path. Outside VS Code, use the **Git Bash** app from the Start Menu
and `cd` into the repo folder first.

---

## 7. When it doesn't work

| Symptom | Cause | Fix |
|---|---|---|
| The task terminal says `This is WSL (Ubuntu), but the Pico tools are installed on Windows` — or every tool is "not found" | You're in WSL. Typing `bash` into PowerShell/CMD opens Ubuntu, not Git Bash. **No step in this project needs WSL.** | Use the VS Code tasks, or the VS Code terminal (Git Bash profile), or the Git Bash app. |
| `bash.exe` not found when running a task | Git for Windows isn't installed, or is installed somewhere unusual | Install Git for Windows (§1.2). If it's elsewhere, edit the one path in `.vscode/tasks.json`. |
| `arm-none-eabi-gcc not found on PATH or under ~/.pico-sdk/toolchain` | The Pico extension hasn't finished its first-time download, or isn't installed | Open the extension once, let it finish, retry. |
| `Pico SDK with TinyUSB not found` | Same as above (the SDK is part of that download) | Same. Or point at your own SDK: `PICO_SDK_PATH=/c/path/to/pico-sdk ./build/build.sh` |
| `application hook missing: run ./build/setup.sh` | Setup never ran, or the OS submodule was updated | Run the **setup** task. |
| `downloading a portable copy of make … download failed` | Your network blocks SourceForge | Install make yourself: `winget install ezwinports.make`, then restart VS Code. |
| `RPI-RP2` never appears | BOOTSEL wasn't held while plugging in, or a charge-only USB cable | Retry the timing; use a data cable. |
| No serial port after flashing | The image wasn't built with the USB console (only happens if you bypass `build.sh`) | Build with the tasks / `build.sh` — it always sets `CONSOLE=usb_cdc`. |
| Serial Monitor shows nothing until you press RST | Normal — the boot log was printed before you started monitoring | Press **RST** on the Robo Pico. |
| Boot prints `[init] imu FAILED` | IMU not wired, or SDA/SCL swapped | Buddy 4 guide, wiring table. |
| Left wheel never counts in `motion` | GP0/GP1 driven by the OS's UART | Run the **setup** task again (it applies the patch that turns UART0 off), rebuild. |
| The car drives off the table in `follow` | IR polarity backwards | §5.3: flip `IR_ACTIVE_HIGH`. |

When stuck, copy the **first** red line from the terminal and search for
it, or ask a teammate — don't re-run the same task hoping it changes.
