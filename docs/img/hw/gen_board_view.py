"""
Generate the Robo Pico board-layout view for INF2004-IS07.

Companion to gen_pin_map.py: that one is the pin-by-pin reference in the
style of the official Pico pinout; this one shows where each device
physically plugs into the board.

Physical port positions follow the Cytron Robo Pico datasheet mechanical
drawing (88 mm x 72 mm board, Grove 1 on the left edge, Grove 7 on the
right edge, Grove 2-6 along the bottom edge, motor terminals and the
4-way servo header along the top edge).

Signal assignments follow core/rc_config.h and section 2 of the Week 6
design review.

Run:  python docs/img/hw/gen_pin_map.py
Out:  docs/img/hw/robopico_board_view.png
      docs/img/week6/d2b_board_view.png (copy, for the Word doc)
"""

import os
from PIL import Image, ImageDraw, ImageFont

W, H = 2000, 1600

# ----------------------------------------------------------------- palette
BG          = "#ffffff"
INK         = "#1a1a1a"
MUTED       = "#5b5b5b"

BOARD_FILL  = "#efe9f7"    # Robo Pico is a purple PCB
BOARD_EDGE  = "#5e35b1"

USED_FILL   = "#d9ead3"    # port carrying a signal this project uses
USED_EDGE   = "#38761d"

FREE_FILL   = "#eeeeee"    # on-board but unused by us
FREE_EDGE   = "#999999"

DEV_FILL    = "#ffffff"    # external device / module
DEV_EDGE    = "#444444"

WARN_FILL   = "#fff2cc"    # conflict that had to be patched
WARN_EDGE   = "#bf9000"

PICO_FILL   = "#dce9f7"
PICO_EDGE   = "#3d6fa5"

LEAD        = "#6b6b6b"

FONTDIR = r"C:\Windows\Fonts"


def font(name, size):
    for cand in (name, "arial.ttf"):
        try:
            return ImageFont.truetype(os.path.join(FONTDIR, cand), size)
        except OSError:
            continue
    return ImageFont.load_default()


F_TITLE  = font("arialbd.ttf", 42)
F_SUB    = font("arial.ttf", 23)
F_LEG    = font("arial.ttf", 21)
F_BOXT   = font("arialbd.ttf", 23)
F_BOXB   = font("arial.ttf", 20)
F_BOXS   = font("arial.ttf", 18)
F_PORT   = font("arialbd.ttf", 18)
F_PIN    = font("arialbd.ttf", 17)
F_TINY   = font("arial.ttf", 16)
F_NOTE   = font("arial.ttf", 20)
F_NOTEB  = font("arialbd.ttf", 20)
F_BOARD  = font("arialbd.ttf", 26)

img = Image.new("RGB", (W, H), BG)
d = ImageDraw.Draw(img)


def text_w(s, f):
    return d.textbbox((0, 0), s, font=f)[2]


def ctext(cx, y, s, f, fill=INK):
    d.text((cx - text_w(s, f) / 2, y), s, font=f, fill=fill)


def box(x0, y0, x1, y1, fill, edge, r=10, width=2):
    d.rounded_rectangle([x0, y0, x1, y1], radius=r, fill=fill, outline=edge, width=width)


def devbox(x0, y0, x1, y1, title, lines, fill=DEV_FILL, edge=DEV_EDGE):
    """External device / annotation box: bold title then body lines."""
    box(x0, y0, x1, y1, fill, edge)
    cx = (x0 + x1) / 2
    y = y0 + 12
    ctext(cx, y, title, F_BOXT)
    y += 32
    for ln in lines:
        f = F_BOXB if not ln.startswith("~") else F_BOXS
        ctext(cx, y, ln.lstrip("~"), f, MUTED if ln.startswith("~") else INK)
        y += 26 if not ln.startswith("~") else 23
    return y


def lead(pts):
    d.line(pts, fill=LEAD, width=2)


# ----------------------------------------------------------------- heading
ctext(W / 2, 34, "Robo Pico \u2014 Board Wiring View", F_TITLE)
ctext(W / 2, 88,
      "INF2004-IS07 autonomous robotic car   \u00b7   port positions per the Cytron Robo Pico datasheet   \u00b7   "
      "signal assignments per core/rc_config.h",
      F_SUB, MUTED)

# legend
leg = [("Port / pin used by this project", USED_FILL, USED_EDGE),
       ("On-board, reserved or unused", FREE_FILL, FREE_EDGE),
       ("External device or module", DEV_FILL, DEV_EDGE),
       ("Conflict \u2014 patched, see notes", WARN_FILL, WARN_EDGE)]
lx = 300
for label, fl, eg in leg:
    d.rounded_rectangle([lx, 132, lx + 34, 158], radius=5, fill=fl, outline=eg, width=2)
    d.text((lx + 46, 133), label, font=F_LEG, fill=INK)
    lx += 60 + text_w(label, F_LEG) + 42

# ----------------------------------------------------------------- board
BX, BY, BW, BH = 600, 430, 800, 655


def bx(f):
    return BX + f * BW


def by(f):
    return BY + f * BH


box(BX, BY, BX + BW, BY + BH, BOARD_FILL, BOARD_EDGE, r=18, width=3)
# silkscreen branding + orientation, in the clear strip left of the socket
ctext(bx(0.105), by(0.165), "Cytron", F_TINY, "#7e57c2")
ctext(bx(0.105), by(0.195), "ROBO PICO", F_BOARD, "#5e35b1")
ctext(bx(0.105), by(0.245), "88 \u00d7 72 mm", F_TINY, "#7e57c2")
ctext(bx(0.105), by(0.272), "top view, USB left", F_TINY, "#7e57c2")

# ---- Pico W socket (centre)
px0, py0, px1, py1 = bx(0.20), by(0.30), bx(0.82), by(0.64)
box(px0, py0, px1, py1, PICO_FILL, PICO_EDGE)
ctext((px0 + px1) / 2, py0 + 18, "Raspberry Pi Pico W socket", F_BOXT, "#22496f")
ctext((px0 + px1) / 2, py0 + 50, "GP23 / GP24 / GP25 / GP29  \u2192  CYW43439 radio (WiFi telemetry)", F_TINY, "#22496f")
ctext((px0 + px1) / 2, py0 + 74, "flanked by the 2 \u00d7 20 GPIO breakout headers", F_TINY, MUTED)
# USB end marker
d.rectangle([px0 - 16, (py0 + py1) / 2 - 16, px0 + 2, (py0 + py1) / 2 + 16],
            fill="#cfd8dc", outline="#607d8b", width=2)
d.text((px0 - 14, (py0 + py1) / 2 + 22), "USB", font=F_TINY, fill=MUTED)
# breakout header strips
for yy in (py0 - 22, py1 + 6):
    d.rectangle([px0 + 10, yy, px1 - 10, yy + 16], fill="#37474f", outline="#263238")

# GP19 status LED is taken off the breakout header, not a dedicated port
box(bx(0.125), by(0.325), bx(0.195), by(0.405), USED_FILL, USED_EDGE, r=5)
ctext(bx(0.16), by(0.335), "GP19", F_PIN, USED_EDGE)
ctext(bx(0.16), by(0.368), "LED", F_TINY, USED_EDGE)

# ---- top edge: power, motor terminals, servo header
# LiPo + VIN
box(bx(0.10), by(0.03), bx(0.17), by(0.10), FREE_FILL, FREE_EDGE, r=5)
ctext(bx(0.135), by(0.045), "LiPo", F_TINY, MUTED)
box(bx(0.19), by(0.02), bx(0.30), by(0.11), "#c8e6c9", "#2e7d32", r=5)
ctext(bx(0.245), by(0.035), "VIN", F_PORT, "#2e7d32")
ctext(bx(0.245), by(0.070), "3.6-6 V", F_TINY, "#2e7d32")

# MOTOR 2 terminal (left motor: M2B = GP11, M2A = GP10)
box(bx(0.35), by(0.025), bx(0.49), by(0.125), USED_FILL, USED_EDGE, r=5)
ctext(bx(0.42), by(0.035), "MOTOR 2", F_PORT, USED_EDGE)
ctext(bx(0.42), by(0.072), "M2B  M2A", F_TINY, USED_EDGE)
ctext(bx(0.42), by(0.096), "GP11  GP10", F_PIN, USED_EDGE)

# MOTOR 1 terminal (right motor: M1B = GP9, M1A = GP8)
box(bx(0.545), by(0.025), bx(0.685), by(0.125), USED_FILL, USED_EDGE, r=5)
ctext(bx(0.615), by(0.035), "MOTOR 1", F_PORT, USED_EDGE)
ctext(bx(0.615), by(0.072), "M1B  M1A", F_TINY, USED_EDGE)
ctext(bx(0.615), by(0.096), "GP9   GP8", F_PIN, USED_EDGE)

# SERVO header: four 3-pin ports, only port 4 (GP15) is used
box(bx(0.78), by(0.03), bx(0.95), by(0.145), FREE_FILL, FREE_EDGE, r=5)
ctext(bx(0.865), by(0.038), "SERVO", F_PORT, MUTED)
_sx = bx(0.792)
for _n, _gp in enumerate(("12", "13", "14", "15")):
    _w, _g = 26, 6
    _x0 = _sx + _n * (_w + _g)
    _used = (_n == 3)
    d.rounded_rectangle([_x0, by(0.078), _x0 + _w, by(0.135)], radius=4,
                        fill=USED_FILL if _used else "#ffffff",
                        outline=USED_EDGE if _used else FREE_EDGE, width=2)
    ctext(_x0 + _w / 2, by(0.092), _gp, F_TINY, USED_EDGE if _used else MUTED)

# ---- left edge: GROVE 1
box(BX - 4, by(0.52), bx(0.145), by(0.665), USED_FILL, USED_EDGE, r=5)
ctext(bx(0.070), by(0.535), "GROVE 1", F_TINY, USED_EDGE)
ctext(bx(0.070), by(0.567), "GP0 / GP1", F_PIN, USED_EDGE)
ctext(bx(0.070), by(0.602), "enc L  A / B", F_TINY, MUTED)
ctext(bx(0.070), by(0.630), "GND 3V3", F_TINY, MUTED)

# ---- right edge: GROVE 7
box(bx(0.855), by(0.52), BX + BW + 4, by(0.665), USED_FILL, USED_EDGE, r=5)
ctext(bx(0.930), by(0.535), "GROVE 7", F_TINY, USED_EDGE)
ctext(bx(0.930), by(0.567), "GP7 / GP28", F_PIN, USED_EDGE)
ctext(bx(0.930), by(0.602), "enc R  A / B", F_TINY, MUTED)
ctext(bx(0.930), by(0.630), "3V3 GND", F_TINY, MUTED)

# ---- buzzer + mute switch (right of the socket)
box(bx(0.845), by(0.41), bx(0.935), by(0.49), FREE_FILL, FREE_EDGE, r=5)
ctext(bx(0.89), by(0.425), "BUZZER", F_TINY, MUTED)
ctext(bx(0.89), by(0.452), "GP22", F_PIN, MUTED)

# ---- buttons row
btns = [(0.29, "RST"), (0.43, "GP21"), (0.57, "GP20")]
for f, lbl in btns:
    d.rounded_rectangle([bx(f), by(0.695), bx(f + 0.095), by(0.745)], radius=16,
                        fill=FREE_FILL, outline=FREE_EDGE, width=2)
    ctext(bx(f + 0.0475), by(0.705), lbl, F_TINY, MUTED)

# ---- RGB neopixels on GP18, one at each bottom corner
for f in (0.035, 0.94):
    d.ellipse([bx(f), by(0.70), bx(f + 0.035), by(0.745)], fill="#bbdefb", outline="#1565c0", width=2)
ctext(bx(0.052), by(0.755), "GP18 RGB", F_TINY, MUTED)
ctext(bx(0.925), by(0.755), "GP18 RGB", F_TINY, MUTED)

# ---- MAKER port (QWIIC / Stemma QT) shares GP2/GP3 with Grove 2, which is
# the ultrasonic TRIG/ECHO here, so it must stay empty.
box(bx(0.042), by(0.86), bx(0.132), by(0.95), FREE_FILL, FREE_EDGE, r=5)
ctext(bx(0.087), by(0.872), "MAKER", F_TINY, MUTED)
ctext(bx(0.087), by(0.900), "GP2/3", F_TINY, MUTED)

# ---- bottom edge Grove ports 2..6
groves = [
    (0.152, 0.256, "GROVE 2", "GP2 / GP3",   True),
    (0.320, 0.432, "GROVE 3", "GP4 / GP5",   True),
    (0.488, 0.600, "GROVE 4", "GP16 / GP17", True),
    (0.656, 0.768, "GROVE 5", "GP6 / GP26",  True),
    (0.824, 0.936, "GROVE 6", "GP26 / GP27", True),
]
grove_anchor = {}
for f0, f1, name, pins, used in groves:
    fill, edge = (USED_FILL, USED_EDGE) if used else (FREE_FILL, FREE_EDGE)
    x0, x1 = bx(f0), bx(f1)
    box(x0, by(0.845), x1, by(0.975), fill, edge, r=5)
    ctext((x0 + x1) / 2, by(0.858), name, F_PORT, edge)
    ctext((x0 + x1) / 2, by(0.898), pins, F_PIN, edge)
    # connector shroud
    d.rectangle([x0 + 10, by(0.935), x1 - 10, by(0.972)], fill="#fafafa", outline="#9e9e9e", width=2)
    grove_anchor[name] = ((x0 + x1) / 2, by(0.975))

# ================================================================= devices
# ---- top row device boxes
devbox(300, 196, 660, 356, "Power input",
       ["LiPo 1S or Vin 3.6\u20136 V", "~3V3 rail: 300 mA total to Grove ports",
        "~motors + servo share the pack \u2014", "~add 470 \u00b5F if the servo browns out"])

devbox(700, 196, 960, 356, "Left motor",
       ["DC gear motor + wheel", "M2A GP10 / M2B GP11", "~PWM slice 5 ch A/B, 20 kHz",
        "~drv_motor \u2022 Motion task"])

devbox(980, 196, 1240, 356, "Right motor",
       ["DC gear motor + wheel", "M1A GP8 / M1B GP9", "~PWM slice 4 ch A/B, 20 kHz",
        "~drv_motor \u2022 Motion task"])

devbox(1260, 196, 1560, 356, "Scan servo (SG90)",
       ["Pan bracket for the HC-SR04", "GP15 \u2014 servo port 4", "~PWM slice 7 ch B, 50 Hz",
        "~drv_servo \u2022 Scan task"])

lead([(480, 356), (480, 400), (bx(0.245), 400), (bx(0.245), by(0.02))])
lead([(830, 356), (830, 404), (bx(0.42), 404), (bx(0.42), by(0.025))])
lead([(1110, 356), (1110, 392), (bx(0.615), 392), (bx(0.615), by(0.025))])
lead([(1410, 356), (1410, 380), (bx(0.925), 380), (bx(0.925), by(0.03))])

# ---- left device box: Grove 1 console
devbox(40, 700, 560, 860, "Left wheel encoder (A/B)",
       ["A \u2192 GP0 (edge IRQ), B \u2192 GP1", "GND, 3V3, A, B \u2014 GROVE 1, one cable",
        "~B sampled in the ISR for direction", "~Motion task @ 50 Hz \u2022 drv_encoder"],
       fill=WARN_FILL, edge=WARN_EDGE)
lead([(560, 780), (BX - 4, 780)])

# ---- left lower: status LED
devbox(40, 895, 560, 1035, "Status LED (external)",
       ["LED + 330 \u03a9 on GP19", "taken from the GPIO breakout header",
        "~Pico W's own LED hangs off the CYW43", "~radio, so it cannot be used here"],
       fill=WARN_FILL, edge=WARN_EDGE)
lead([(560, 960), (575, 960), (575, by(0.365)), (bx(0.125), by(0.365))])

# ---- right device box: Grove 7 line sensor right
devbox(1440, 700, 1960, 860, "Right wheel encoder (A/B)",
       ["A \u2192 GP7 (edge IRQ), B \u2192 GP28", "GND, 3V3, A, B \u2014 GROVE 7, one cable",
        "~B sampled in the ISR for direction", "~Motion task @ 50 Hz \u2022 drv_encoder"])
lead([(1440, 780), (BX + BW + 4, 780)])

# ---- right lower: on-board unused
devbox(1440, 895, 1960, 1035, "On-board, not used by this project",
       ["Piezo buzzer GP22  \u00b7  2 \u00d7 RGB LED GP18", "Buttons GP20 / GP21  \u00b7  servo ports 1\u20133",
        "~GP12/GP13/GP14 free \u00b7 GP17 free \u00b7 MAKER port", "~console is USB, so no UART pins"],
       fill=FREE_FILL, edge=FREE_EDGE)

# ---- bottom device boxes, one per Grove port
bottom = [
    ("GROVE 2", "HC-SR04 ultrasonic", ["TRIG GP2 / ECHO GP3", "ECHO is 5 V \u2014 needs 1k/2k divider",
                                       "~TIMER alarms end TRIG and time out ECHO",
                                       "~Scan task \u2022 drv_ultrasonic"], WARN_FILL, WARN_EDGE),
    ("GROVE 3", "LSM303DLHC IMU", ["Accelerometer + magnetometer", "SDA GP4 / SCL GP5 \u2014 I\u00b2C0",
                                   "~0x19 accel \u00b7 0x1E mag \u00b7 no gyro",
                                   "~Sense task @ 100 Hz \u2022 drv_imu"], WARN_FILL, WARN_EDGE),
    ("GROVE 4", "IR line sensor 1 (left)", ["MH-Sensor-Series TCRT5000 + LM393", "DO \u2192 GP16  (AO unused)",
                                            "~polled by the Sense task @ 200 Hz",
                                            "~drv_ir \u2022 sub_line"], WARN_FILL, WARN_EDGE),
    ("GROVE 5", "IR line sensor 2 (right)", ["MH-Sensor-Series TCRT5000 + LM393", "DO \u2192 GP6  \u2014  AO MUST stay unwired",
                                             "~Grove 5 pin 2 is GP26 = barcode AO",
                                             "~drv_ir \u2022 sub_line"], WARN_FILL, WARN_EDGE),
    ("GROVE 6", "IR barcode sensor", ["AO \u2192 GP26 (ADC0)", "DO \u2192 GP27, both-edge IRQ",
                                      "~every edge timestamped in the ISR",
                                      "~bar/space widths \u2192 Code 39 \u2022 sub_barcode"], DEV_FILL, DEV_EDGE),
]

bw_, gap = 370, 22
x = (W - (5 * bw_ + 4 * gap)) / 2
for port, title, lines, fl, eg in bottom:
    devbox(x, 1128, x + bw_, 1300, title, lines, fill=fl, edge=eg)
    ax, ay = grove_anchor[port]
    lead([(ax, ay), (ax, 1100), (x + bw_ / 2, 1100), (x + bw_ / 2, 1128)])
    x += bw_ + gap

# ================================================================= notes
ny = 1332
d.text((60, ny), "Conflicts handled (details in docs/HARDWARE.md \u00a71):", font=F_NOTEB, fill=INK)
ny += 32
notes = [
    "1.  GP0/GP1 are UART0, the port's default console, and also the left encoder \u2014 so every build MUST be CONSOLE=usb_cdc. "
    "The console comes out of the Pico's own USB port.",
    "2.  GP4 is I\u00b2C0 SDA and GP5 is I\u00b2C0 SCL in silicon (fixed). The port's I\u00b2C0 driver defaults to GP8/GP9 (left motor), so its unit-0 "
    "pin table is re-pinned to GP4/GP5 and the IMU opens \"iica\". Wire SDA\u2192GP4, SCL\u2192GP5.",
    "3.  HC-SR04 ECHO outputs 5 V and the RP2040 is not 5 V tolerant \u2014 1 k\u03a9 from ECHO to GP3 and 2 k\u03a9 from GP3 to GND (3.33 V at the pin). "
    "Connecting it directly destroys the board.",
    "4.  BOARD_LED_PIN moved from GP16 to GP19 in sysdef.h \u2014 GP16 is line sensor 1. The Pico W's on-board LED is wired to the "
    "CYW43439 radio, not to an RP2040 pin, so an external LED is required.",
    "5.  GP26 is on BOTH Grove 6 pin 1 and Grove 5 pin 2. Line sensor 2 on Grove 5: wire GND, VCC and DO only, or its AO shorts onto the barcode "
    "sensor's AO. MAKER shares GP2/GP3 with Grove 2 — leave it empty.",
    "6.  PWM slices 4, 5 and 7 are taken by the motors and the servo, so the kernel's PWM-based StartPhysicalTimer cannot use them. "
    "This tree uses the RP2040 TIMER alarms instead, which the kernel never touches.",
]
for n in notes:
    d.text((60, ny), n, font=F_NOTE, fill=MUTED)
    ny += 30

ny += 6
d.text((60, ny),
       "Grove 1 sits on the left edge and Grove 7 on the right edge; Grove 2\u20136 run along the bottom edge. "
       "Every GPIO number above is defined once in core/rc_config.h and nowhere else.",
       font=F_NOTE, fill=INK)

# ================================================================= output
here = os.path.dirname(os.path.abspath(__file__))
root = os.path.abspath(os.path.join(here, "..", "..", ".."))
out1 = os.path.join(here, "robopico_board_view.png")
img.save(out1)
print("wrote", out1)

out2 = os.path.join(root, "docs", "img", "week6", "d2b_board_view.png")
if os.path.isdir(os.path.dirname(out2)):
    img.save(out2)
    print("wrote", out2)
