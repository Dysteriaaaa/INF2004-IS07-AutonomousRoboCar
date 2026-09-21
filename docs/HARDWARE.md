# Hardware and Driver Setup Guide

Autonomous robotic car, Raspberry Pi Pico W on a Cytron Robo Pico carrier,
running µT-Kernel 3.0 from the `mtk3smp-rp2040` port.

Everything below is checked against the datasheets in `OKY3231-2.zip` and
against the port's own source, not against a generic Pico tutorial. Where the
port disagrees with what you would expect from the Pico SDK, that is called
out.

---

## 1. Read this first: four things that will cost you a day each

These are not edge cases. Each one is a conflict you will hit in the first
week if nobody tells you.

### 1.1 The IMU has no gyroscope

The `OKY3231-2` part is a **GY-511 / LSM303DLHC**: a 3-axis accelerometer and
a 3-axis magnetometer, on two separate dice with two separate I²C addresses.
There is no gyro on this board.

Three of Buddy 4's listed tasks assume one:

| Task in the brief | What you actually have to do |
|---|---|
| Tilt detection | Derive from the gravity vector in the accelerometer |
| Turn-rate monitoring | Use the encoder difference from Buddy 2. The magnetometer will read heading, but the motors sit centimetres away and will swamp it |
| Peak hump measurement | Do **not** double-integrate acceleration. Over a two-second climb the bias drift exceeds the hump height. Estimate peak pitch and convert with the wheelbase |

`sub_terrain.c` implements the pitch-and-wheelbase approach and documents the
trade-off. Validate it against a ruler on three known humps and quote the
error in your terrain analysis report.

### 1.2 The stock I²C driver collides with the left motor

`device/i2c/sysdepend/rp2040/i2c_rp2040.c` hardcodes its pins:

- **I²C0 → GP8 / GP9**. On Robo Pico those are **M1A and M1B**, the left motor.
- **I²C1 → GP6 / GP7**. Usable, but GP6 and GP7 sit on *different* Grove ports
  (5 and 7), so you cannot connect them with one Grove cable.

Fix: patch the I²C1 pin table to **GP2 / GP3**, which is Robo Pico's Grove 2
and the QWIIC / Stemma QT Maker port. In `i2c_rp2040.c`, change the unit-1
branch:

```c
/* was GP6/GP7 */
out_w(GPIO_CTRL(2), GPIO_CTRL_FUNCSEL_I2C);
out_w(GPIO(2), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
out_w(GPIO_CTRL(3), GPIO_CTRL_FUNCSEL_I2C);
out_w(GPIO(3), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
```

Per the Robo Pico datasheet, GP2 is `SDA1` and GP3 is `SCL1`, so this is a
legal mapping.

### 1.3 The liveness LED default sits on a pin you need

The port defaults `BOARD_LED_PIN` to **GP16**, which is half of Grove 4 and
which this framework uses for the ultrasonic trigger. You cannot fall back to
the Pico W's on-board LED, because it hangs off the CYW43439 radio rather than
an RP2040 pin.

Fix: change `BOARD_LED_PIN` in
`include/sys/sysdepend/pico_rp2040/sysdef.h` to **GP19** (matching
`RC_PIN_STATUS_LED`), and wire an external LED with a series resistor.

### 1.4 The kernel's physical timer is built on the PWM block

`INTNO_PTIM` is `INTNO_PWM`, and `StartPhysicalTimer` claims a PWM *slice*.
Any slice used for motors or servos is therefore unavailable as a physical
timer.

- Slice 4 (GP8/GP9): left motor
- Slice 5 (GP10/GP11): right motor
- Slice 6 (GP12/GP13): servos

Slices 0–3 and 7 remain free. This framework does not use `StartPhysicalTimer`
at all; it uses the RP2040 **TIMER** block's alarms instead, which are
untouched by the kernel.

---

## 2. Pin map

![Robo Pico hardware integration pin map](img/hw/robopico_pin_map.png)

Pin-by-pin reference, in the style of the official Pico pinout: one row per
physical header pin, showing the Pico function, the Robo Pico connector that
pin is wired to, and what this project connects there. Green is in use, grey
is spare, amber flags a conflict covered in §1.

![Robo Pico board view](img/hw/robopico_board_view.png)

Board-level view of the same wiring, for when you are holding the board and
want to know which socket a device goes into: Grove 1 on the left edge,
Grove 7 on the right edge, Grove 2–6 along the bottom, motor terminals and
the four-way servo header along the top.

Both are generated — edit the scripts, never the PNGs, then re-run:

| Script | Output | Copy for the design review |
|---|---|---|
| `docs/img/hw/gen_pin_map.py` | `robopico_pin_map.png` | `documents/week6_diagrams/d2_pin_layout.png` |
| `docs/img/hw/gen_board_view.py` | `robopico_board_view.png` | `documents/week6_diagrams/d2b_board_view.png` |

Slice number is `(GPIO >> 1) & 7`. Grove port numbers are from the Robo Pico
datasheet.

| GPIO | Use | Owner | Fixed? | Notes |
|---|---|---|---|---|
| GP0 | UART0 TX, console | port | board | Grove 1. Needed for `tm_printf` on a UART build |
| GP1 | UART0 RX, console | port | board | Grove 1 |
| GP2 | I²C1 SDA → IMU | Buddy 4 | chosen | Grove 2 / Maker port, after the §1.2 patch |
| GP3 | I²C1 SCL → IMU | Buddy 4 | chosen | Grove 2 / Maker port |
| GP4 | Left encoder | Buddy 2 | chosen | Grove 3 |
| GP5 | Right encoder | Buddy 2 | chosen | Grove 3 |
| GP6 | IR line sensor, left (DOUT) | Buddy 3 | chosen | Grove 5 pin 1 |
| GP7 | IR line sensor, right (DOUT) | Buddy 3 | chosen | Grove 7 pin 1 |
| GP8 | Motor M1A | Buddy 2 | **board** | Slice 4A |
| GP9 | Motor M1B | Buddy 2 | **board** | Slice 4B |
| GP10 | Motor M2A | Buddy 2 | **board** | Slice 5A |
| GP11 | Motor M2B | Buddy 2 | **board** | Slice 5B |
| GP12 | Scan servo | Buddy 5 | **board** | Servo port 1, slice 6A |
| GP13 | Spare servo | — | **board** | Servo port 2, slice 6B |
| GP14, GP15 | Servo ports 3, 4 | — | **board** | Unused |
| GP16 | Ultrasonic TRIG | Buddy 5 | chosen | Grove 4. See §1.3 |
| GP17 | Ultrasonic ECHO | Buddy 5 | chosen | Grove 4. **Needs a divider**, see §4.5 |
| GP18 | NeoPixel | — | **board** | Unused |
| GP19 | Status LED | shared | chosen | External LED + resistor |
| GP20, GP21 | Buttons | — | **board** | Useful as a run/stop switch |
| GP22 | Piezo buzzer | — | **board** | Has a mute switch on the board |
| GP23, GP24, GP25, GP29 | **Radio reserved** | — | **board** | CYW43439. Do not touch |
| GP26 | IR barcode, AOUT | Buddy 3 | chosen | ADC0. Grove 5/6 |
| GP27 | IR barcode, DOUT | Buddy 3 | chosen | Grove 6 |
| GP28 | free / VBAT sense | — | board | ADC2. VBAT only if you solder the jumper |

> The Robo Pico datasheet lists GP26 on both Grove 5 and Grove 6. Verify
> against the silkscreen on your actual board before you cable it up.

Everything above is defined in one place, `core/rc_config.h`. No other file
hard-codes a GPIO number.

---

## 3. Power

| Item | Value |
|---|---|
| Input range (Vin, LiPo or USB) | 3.6 – 6 V |
| Total +3V3 available to Grove ports | 300 mA |
| Motor voltage at full speed | equals the supply voltage |

Two consequences worth planning around:

1. **The HC-SR04 wants 5 V.** On a single-cell LiPo the rail is 3.7–4.2 V.
   Most HC-SR04 modules work down to about 3.3 V with reduced range; if yours
   does not, feed it from USB 5 V during bench testing and accept a shorter
   range on battery, or fit a boost module.
2. **The servo and the motors share the supply.** A servo step during a
   motor-current spike can brown out the Pico. `drv_servo_release()` exists so
   you can stop driving the servo when it is not scanning. If you still see
   resets, add a bulk capacitor (470 µF or more) across the servo supply.

---

## 4. Per-buddy hardware, wiring and drivers

### 4.1 Buddy 1 — WiFi, command and telemetry

**Hardware:** none beyond the Pico W's built-in CYW43439.

**The problem you need to know about up front.** The port's own
`docs/PORT_RP2040.md` is explicit:

- WiFi and lwIP (DHCP, DNS, UDP, TCP) exist as `WIFI_*` build knobs, are
  described as a **development profile**, and are **not part of the qualified
  release**. Association, DHCP and a repeating 64-packet UDP echo have run on
  hardware; static addressing and TCP have had less exposure.
- **No MQTT client ships.** MQTT is listed under "Not ported", with the note
  that transports are present and anything layered on top is left as an
  exercise.

So if you write the telemetry subsystem directly against an MQTT client, you
have nothing to demo until that client exists. `sub_telemetry.h` puts the
transport behind a `sub_telemetry_sink_t` interface instead, so the framing,
topic design, message structures and publish scheduling all work today over
the console sink.

**Recommended order:**

1. **Console sink** (shipped). Gives you a working telemetry framework on day
   one and lets the other four buddies see their data.
2. **UDP sink.** Uses the port's lwIP UDP profile, which is the best-tested
   network path it has. Enable the `WIFI_*` build knobs, put credentials in
   `config/wifi_credentials.h` (copy the `.example.h`), and point it at a
   `netcat` listener on your laptop.
3. **MQTT sink.** Bring in a client. lwIP's own `apps/` sources are in the
   Pico SDK checkout, and `LWIP_ALTCP` in `lib/libnet/lwip/include/lwipopts.h`
   is the switch some of them need.

**Drivers required:** `lib/libwifi` (CYW43439 bring-up, in the port),
`lib/libnet/lwip` (in the port), an MQTT client (you supply).

**Ownership rule that will bite you:** the CYW43439 PIO-SPI, DMA, pins and
state are owned by **processor 1** (physical core 0). Its service task must be
pinned `TP_PRC1`. Other tasks may only read its published status. Same for
TinyUSB: any task may call `tm_printf`, but nothing may call TinyUSB directly.

---

### 4.2 Buddy 2 — Motion control

**Hardware:** 2 × DC gear motors into the Robo Pico terminals, 2 × slotted
optical encoders.

**Wiring:**

| Signal | Pin | Note |
|---|---|---|
| Left motor | M1 terminal | M1A = GP8, M1B = GP9 |
| Right motor | M2 terminal | M2A = GP10, M2B = GP11 |
| Left encoder OUT | GP4 | Grove 3 |
| Right encoder OUT | GP5 | Grove 3 |

Robo Pico uses **two PWM pins per motor**, not PWM-plus-direction. Forward is
PWM on MxA with MxB low; reverse swaps them; brake is both high; coast is both
low. `drv_motor.c` handles this. Actual direction depends on how you wired the
motor terminals — if a wheel spins backwards, swap that motor's two wires
rather than adding a sign in software.

**Encoder limitation:** these are single-channel, so **direction is not
measurable from the disc**. `drv_encoder.c` takes direction from the last
commanded motor value, which is correct except during the moment a wheel is
still coasting after a reversal. Keep that in mind when tuning.

**Drivers required:** `rc_pwm` (in this tree, adds the missing clock divider),
`rc_gpioirq` (in this tree). No kernel device driver.

**Setup and calibration, in order:**

1. Measure your wheel diameter and slot count. Set `RC_WHEEL_DIAM_MM`,
   `RC_ENC_SLOTS_PER_REV` and `RC_WHEEL_BASE_MM` in `rc_config.h`.
2. Wheels off the ground. Command a fixed duty, log `drv_encoder_speed_mm_s`,
   and confirm it is stable and roughly linear in duty.
3. Tune the PID gains in `sub_motion_init()`. The shipped values are
   placeholders. Record the step responses — that is your PID tuning report.
4. On the ground: command `sub_motion_forward_mm(1000)` ten times, measure
   with a tape, and fold the average error into `RC_ENC_UM_PER_TICK`.
5. Calibrate turns the same way. Expect the measured turn to fall **short** of
   the geometric one because of slip. Apply a correction factor rather than
   trusting the geometry.

---

### 4.3 Buddy 3 — IR line following and barcode

**Hardware:** 3 × reflective IR modules. The bundle has datasheets for both
the Waveshare **ST188 + LM393** board and the **TCRT5000**; either works.
Two watch the line, one reads barcodes.

**Wiring:**

| Sensor | DOUT | AOUT | Note |
|---|---|---|---|
| Line, left | GP6 | — | Digital, polled at 200 Hz |
| Line, right | GP7 | — | Digital, polled at 200 Hz |
| Barcode | GP27 | GP26 (ADC0) | Digital is edge-interrupt driven |

Each module has a **trim pot** setting the LM393 comparator threshold, and
exposes the raw divider on AOUT.

**Polarity warning.** On the Waveshare board, DOUT goes **LOW** over a
reflective (white) surface and **HIGH** over a non-reflective (black) one, so
HIGH means on-line. `IR_ACTIVE_HIGH` at the top of `drv_ir.c` controls this.
Getting it backwards is the single most common cause of a car that
confidently drives off the table.

**Calibration:**

1. Hold the sensor over white track. Read AOUT with `drv_ir_read_raw()`.
2. Hold it over the black line. Read again.
3. Set the trim pot so DOUT flips at roughly the midpoint of the two.
4. Repeat under the lighting you will actually demo in. IR ambient from
   fluorescent tubes and sunlight are very different.

**Why the barcode sensor is interrupt-driven and the line sensors are not.**
At 200 Hz a poll is cheaper than an interrupt, and line timing does not need
to be tight. Barcode is the opposite: bar widths at speed are tens of
milliseconds, and what carries the data is the **ratio** between wide and
narrow bars, not the absolute width. So each transition needs a microsecond
stamp. `drv_ir.c` publishes a width with every edge, and `sub_barcode.c`
decodes purely in ratios — which is what makes it speed-independent, and what
the brief means by "robust operation at varying speeds".

**Drivers required:** `rc_gpioirq` (in this tree), `device/adc` (the port's
ADC driver, opened as `"adca"`, channel number passed as the read start
position).

**Known gap:** the Code 39 pattern table in `sub_barcode.c` has real values
for `*` and `A` only. `B`, `C` and `D` are marked `TODO` placeholders. Fill
them from a Code 39 reference before you test decoding.

---

### 4.4 Buddy 4 — IMU and terrain

**Hardware:** GY-511 (LSM303DLHC) breakout.

**Wiring:** I²C1 after the §1.2 patch. SDA → GP2, SCL → GP3, VCC → 3V3,
GND → GND. The module accepts 3–5 V.

| Die | I²C address |
|---|---|
| Accelerometer | `0x19` |
| Magnetometer | `0x1E` |

Two register-level traps that catch nearly everyone:

- The **accelerometer** is little-endian and **left-justified**: the 12-bit
  result sits in the top bits, so shift down by 4. Multi-byte reads need the
  MSB of the sub-address set (`0x80`) for auto-increment.
- The **magnetometer** is **big-endian**, and the axis order in the register
  block is **X, Z, Y** — not X, Y, Z.

`drv_imu.c` handles both.

**Drivers required:** `device/i2c` (the port's driver, opened as `"iicb"` for
unit 1), patched per §1.2.

**Setup:**

1. Configure `A_CTRL_REG1 = 0x57` (100 Hz, all axes) and
   `A_CTRL_REG4 = 0x08` (±2 g, high resolution). At ±2 g one LSB is about
   1 mg, which is the resolution hump detection needs.
2. Mount the board **rigidly**. A breakout on a wobbly standoff will register
   its own vibration as terrain.
3. Run `drv_imu_calibrate(100)` with the car level and still. This zeroes X
   and Y and stores the deviation of Z from 1 g.
4. Set your hump thresholds empirically: drive hard on flat ground, log the
   pitch, and set `PITCH_ENTER_DDEG` in `sub_terrain.c` above the peak you see
   there. Cornering and braking both tilt the car.

---

### 4.5 Buddy 5 — Ultrasonic scanning and obstacle avoidance

**Hardware:** HC-SR04 + one RC servo (SG90 class) on a pan bracket.

**Wiring:**

| Signal | Pin | Note |
|---|---|---|
| Servo | GP12 | Robo Pico servo port 1 |
| TRIG | GP16 | Grove 4 |
| ECHO | GP17 | Grove 4, **through a divider** |

> **ECHO is a 5 V output and the RP2040 is not 5 V tolerant.** Do not connect
> it directly. Use a divider: 1 kΩ from ECHO to GP17, 2 kΩ from GP17 to GND.
> That gives 3.33 V at the pin. A level shifter works too. Skipping this is
> the fastest way to destroy a Pico on this project.

From the datasheet: TRIG needs a pulse of **at least 10 µs**; the module then
sends eight 40 kHz bursts and holds ECHO high for the round-trip time. Range
is 2 cm to 4 m, beam width about 15°, and accuracy about 3 mm.

That 15° beam width is the real limit on obstacle profiling. Below about 15°
of angular separation the sensor cannot tell two bearings apart, so
`RC_SCAN_FINE_STEP` of 6° is oversampling the beam — useful for smoothing,
not for genuinely finer resolution. Say so in your report rather than claiming
6° accuracy.

**Drivers required:** `rc_pwm` and `rc_gpioirq` (this tree). Two RP2040 TIMER
alarm interrupts (IRQ 1 and IRQ 2), claimed directly in
`drv_ultrasonic.c` — these are unused by the kernel, which was verified
against the port source.

**Why the driver looks the way it does.** The usual Arduino approach:

```c
digitalWrite(TRIG, HIGH); delayMicroseconds(10);
digitalWrite(TRIG, LOW);  pulseIn(ECHO, HIGH);
```

Both of those are busy-waits, and `pulseIn` holds the CPU for up to 30 ms when
nothing echoes back. On a car running a 50 Hz PID loop that is a missed
control update and a visible swerve. `drv_ultrasonic.c` instead spreads one
measurement across five interrupts and costs a few microseconds of CPU:

| Step | What happens |
|---|---|
| `ping()` | Raise TRIG, arm TIMER alarm 1 for +12 µs, return |
| Alarm 1 ISR | Drop TRIG, unmask the ECHO edge, arm alarm 2 as the timeout |
| ECHO rise ISR | Stamp the start |
| ECHO fall ISR | Compute width, publish the result |
| Alarm 2 ISR | Publish an invalid result if no echo came back |

The 30 ms of flight time costs nothing.

**Setup:**

1. Centre the servo mechanically at 90° before you bolt the bracket on.
2. Measure `drv_servo_settle_ms`'s constants for your servo. They dominate
   total scan time — a full coarse scan is roughly 5 × (travel + 30 ms).
3. Set `CAR_WIDTH_MM` in `sub_scan.c` from your car's widest point plus
   40 mm of margin.

---

## 5. Build and flash

```sh
git clone https://github.com/sirfonzie/mtk3smp-rp2040.git
cd mtk3smp-rp2040
# apply the §1.2 I2C patch and the §1.3 BOARD_LED_PIN change
# drop this tree into app_program/ and add it to the makefile

cd build_make
make -j8              # single core, the conservative default
make SMP=1 -j8        # dual core
```

Hold **BOOTSEL** while plugging in the Pico, then copy
`build_make/mtk3pico_smp0_uart.uf2` to the `RPI-RP2` drive.

Console is **UART0 on GP0/GP1, 115200 8N1**, so you need a USB-serial adapter.
`make CONSOLE=usb_cdc PICO_SDK_PATH=...` puts the console on the Pico's own USB
port instead; that is the only part of the tree needing the Pico SDK, and it
uses TinyUSB only, not the SDK's CMake build.

Toolchain: `arm-none-eabi-gcc` (baseline 13.2.1) and a host `g++`.

**Start on `SMP=0`.** Get the car working on one core first. The single-core
build is the port's default and its conservative profile, and every bug you
hit will be your bug rather than a cross-core one.

---

## 6. SMP rules, if you go dual-core

Taken from the port's `PORT_RP2040.md`. These are qualification boundaries,
not tuning advice.

1. **SIO hardware spinlocks 0–2 are kernel-reserved.** Do not claim them.
2. **A shared peripheral IRQ must have one owner core.** `rc_gpioirq.c`
   enables IO_IRQ_BANK0 in core 0's NVIC only, so all GPIO handlers run on
   processor 1. Keep it that way.
3. **Publish shared data before making another core observe it.** `volatile`
   alone is not inter-core synchronisation; use the port's atomics.
4. **Treat UART, I²C, ADC, DMA and PWM as single-owner resources.** Only the
   monitor USB transmit path is qualified for concurrent cross-core use.
5. **Task-to-processor assignment is static.** There is no dynamic affinity.
6. **There is no FPU.** Every calculation in this tree is integer or fixed
   point, deliberately. A soft-float `atan2` at 100 Hz would cost thousands of
   cycles.

A reasonable split if you do go SMP: pin the radio and telemetry to processor
1 (mandatory for the radio anyway) and let the control tasks migrate.

---

## 7. Bring-up order

Do not wire everything and flash once. In this order, each step is testable on
its own:

| Step | Test | Pass condition |
|---|---|---|
| 1 | Blink GP19 | LED blinks at 1 Hz |
| 2 | `tm_printf` over UART | Text appears at 115200 |
| 3 | Motors, open loop | Both wheels spin the right way at a fixed duty |
| 4 | Encoders | Counts rise smoothly; no bursts (that is bounce — raise `DEBOUNCE_US`) |
| 5 | Closed-loop speed | Commanded mm/s matches measured within 10 % |
| 6 | IR sensors | DOUT flips crossing the line; AOUT differs clearly black vs white |
| 7 | Line following | Car tracks a straight line, then a curve |
| 8 | Servo | Sweeps 30° to 150° without buzzing at the ends |
| 9 | Ultrasonic | Range matches a tape measure at 100, 300, 1000 mm |
| 10 | Scan | A full coarse scan completes and the PID loop keeps running through it |
| 11 | IMU | Pitch reads ~0 level, positive nose-up |
| 12 | Hump | Peak estimate within 20 % of a ruler on a known hump |
| 13 | Barcode | Consistent decode of one symbol at two different speeds |
| 14 | Telemetry, console | Messages at 4 Hz with no dropped-event counts |
| 15 | Telemetry, network | Same messages arriving on your laptop |
| 16 | Full mission | End to end |

Steps 1–7 are the critical path. Everything else can proceed in parallel once
the car drives.

---

## 8. Parts checklist

| Item | Qty | Notes |
|---|---|---|
| Raspberry Pi Pico W | 1 | GP23/24/25/29 reserved by the radio |
| Cytron Robo Pico | 1 | Motor driver, servo ports, Grove breakouts |
| DC gear motors + wheels | 2 | |
| Slotted optical encoders | 2 | Single channel |
| IR reflective module (ST188 or TCRT5000) | 3 | 2 line + 1 barcode |
| HC-SR04 | 1 | Needs a 5 V supply and a divider on ECHO |
| SG90-class servo + pan bracket | 1 | |
| GY-511 / LSM303DLHC | 1 | Accel + mag, **no gyro** |
| 1 kΩ and 2 kΩ resistors | 1 each | ECHO divider |
| LED + 330 Ω resistor | 1 | Liveness on GP19 |
| Single-cell LiPo | 1 | 3.6–6 V input range |
| USB-serial adapter | 1 | For the UART console |
| Bulk capacitor, 470 µF+ | 1 | Servo supply, if you see brownouts |
| Grove cables | 4+ | Ports 2, 3, 4 and jumpers for 5/6/7 |
