"""
Generate the INF2004-IS07 pin map in the style of the official Raspberry Pi
Pico pinout diagram: the board in the centre, one row per physical header
pin, and colour-coded chips fanning out to the left and right.

The Robo Pico breaks every Pico pin out on its 2 x 20 female headers, so the
row order here is the real 40-pin Pico header order (1-20 down the left,
40-21 down the right). Each row adds two columns the stock Pico diagram does
not have:

  * which Robo Pico connector that pin is wired to (Grove port, motor
    terminal, servo port, on-board peripheral), from the Cytron Robo Pico
    datasheet; and
  * what this project connects there, from core/rc_config.h and section 2
    of the Week 6 design review.

Run:  python docs/img/hw/gen_pin_map.py
Out:  docs/img/hw/robopico_pin_map.png
      documents/week6_diagrams/d2_pin_layout.png (copy, for the Word doc)
"""

import os
from PIL import Image, ImageDraw, ImageFont

# ----------------------------------------------------------------- palette
BG        = "#ffffff"
INK       = "#1a1a1a"
MUTED     = "#5b5b5b"

C_GP      = ("#a8d08d", "#1b3d10")      # GPIO number chip, as on the Pico diagram
C_PIN     = ("#dcdcdc", "#2b2b2b")      # physical pin number
C_GND     = ("#3a3f44", "#ffffff")
C_PWR     = ("#e8453c", "#ffffff")
C_PWR2    = ("#f7c0bd", "#6d1512")      # 3V3_EN / RUN
C_UART    = ("#8f7cc0", "#ffffff")
C_I2C     = ("#5ba3d9", "#08304d")
C_PWM     = ("#e0579a", "#ffffff")
C_ADC     = ("#2f7d4f", "#ffffff")
C_GPIO    = ("#c7d3dd", "#1d2b36")

USED      = ("#d9ead3", "#2f6b1c")      # this project uses it
FREE      = ("#efefef", "#6d6d6d")      # broken out, spare
WARN      = ("#ffe9b0", "#8a6200")      # conflict, see notes

BOARD_F   = "#efe9f7"
BOARD_E   = "#5e35b1"
SOCK_F    = "#2f7d4f"
SOCK_E    = "#1d5133"

FONTDIR = r"C:\Windows\Fonts"


def font(name, size):
    for cand in (name, "arial.ttf"):
        try:
            return ImageFont.truetype(os.path.join(FONTDIR, cand), size)
        except OSError:
            continue
    return ImageFont.load_default()


F_TITLE = font("arialbd.ttf", 44)
F_SUB   = font("arial.ttf", 24)
F_LEG   = font("arial.ttf", 20)
F_DEV   = font("arial.ttf", 21)
F_DEVB  = font("arialbd.ttf", 21)
F_PORT  = font("arialbd.ttf", 19)
F_IF    = font("arial.ttf", 18)
F_GP    = font("arialbd.ttf", 21)
F_NUM   = font("arialbd.ttf", 18)
F_NOTE  = font("arial.ttf", 21)
F_NOTEB = font("arialbd.ttf", 21)
F_BRAND = font("arialbd.ttf", 30)
F_SMALL = font("arial.ttf", 17)
F_HEAD  = font("arialbd.ttf", 18)

# ----------------------------------------------------------------- geometry
W, H      = 2400, 1760
ROW_H     = 54
ROWS      = 20
TOP       = 268
BX0, BX1  = 950, 1450          # centre board

PIN_W, GP_W, IF_W, PORT_W, DEV_W = 46, 96, 132, 174, 346
GAP       = 8
CH        = 38                 # chip height

img = Image.new("RGB", (W, H), BG)
d = ImageDraw.Draw(img)


def tw(s, f):
    return d.textbbox((0, 0), s, font=f)[2]


def ctext(cx, cy, s, f, fill=INK):
    d.text((cx - tw(s, f) / 2, cy - f.size * 0.72), s, font=f, fill=fill)


def chip(x0, cy, w, label, pair, f=F_IF, r=7, border=None):
    """One rounded label chip, vertically centred on cy."""
    if not label:
        return
    fill, txt = pair
    d.rounded_rectangle([x0, cy - CH / 2, x0 + w, cy + CH / 2], radius=r,
                        fill=fill, outline=border or fill, width=2)
    ctext(x0 + w / 2, cy, label, f, txt)


def state_pair(state):
    return {"used": USED, "warn": WARN, "free": FREE}[state]


def state_edge(state):
    return {"used": "#2f6b1c", "warn": "#bf9000", "free": "#b5b5b5"}[state]


# ----------------------------------------------------------------- rows
# (pin, kind, gp, iface, iface_colour, robo-pico connector, our device, state)
G = "gnd"
P = "pwr"
IO = "io"

left = [
    (1,  IO, "GP0",  "GPIO in, IRQ", C_GPIO, "GROVE 1",      "Left encoder A",                "warn"),
    (2,  IO, "GP1",  "GPIO in",      C_GPIO, "GROVE 1",      "Left encoder B (direction)",    "warn"),
    (3,  G,  "",     "GND",          C_GND,  "",             "",                              "free"),
    (4,  IO, "GP2",  "GPIO out",     C_GPIO, "GROVE 2",      "HC-SR04 TRIG",                  "used"),
    (5,  IO, "GP3",  "GPIO in, IRQ", C_GPIO, "GROVE 2",      "HC-SR04 ECHO \u2014 5 V!",      "warn"),
    (6,  IO, "GP4",  "I\u00b2C0 SDA", C_I2C, "GROVE 3",      "IMU SDA (LSM303DLHC)",          "warn"),
    (7,  IO, "GP5",  "I\u00b2C0 SCL", C_I2C, "GROVE 3",      "IMU SCL (LSM303DLHC)",          "warn"),
    (8,  G,  "",     "GND",          C_GND,  "",             "",                              "free"),
    (9,  IO, "GP6",  "GPIO in",      C_GPIO, "GROVE 5",      "IR line sensor 2 (R) \u2014 DO", "warn"),
    (10, IO, "GP7",  "GPIO in, IRQ", C_GPIO, "GROVE 7",      "Right encoder A",               "used"),
    (11, IO, "GP8",  "PWM 4A",       C_PWM,  "MOTOR 1  M1A", "Left motor A",                  "used"),
    (12, IO, "GP9",  "PWM 4B",       C_PWM,  "MOTOR 1  M1B", "Left motor B",                  "used"),
    (13, G,  "",     "GND",          C_GND,  "",             "",                              "free"),
    (14, IO, "GP10", "PWM 5A",       C_PWM,  "MOTOR 2  M2A", "Right motor A",                 "used"),
    (15, IO, "GP11", "PWM 5B",       C_PWM,  "MOTOR 2  M2B", "Right motor B",                 "used"),
    (16, IO, "GP12", "PWM 6A",       C_PWM,  "SERVO 1",      "spare",                         "free"),
    (17, IO, "GP13", "PWM 6B",       C_PWM,  "SERVO 2",      "spare",                         "free"),
    (18, G,  "",     "GND",          C_GND,  "",             "",                              "free"),
    (19, IO, "GP14", "PWM 7A",       C_PWM,  "SERVO 3",      "spare",                         "free"),
    (20, IO, "GP15", "PWM 7B",       C_PWM,  "SERVO 4",      "Scan servo SG90 (HC-SR04 pan)", "used"),
]

right = [
    (40, P,  "",     "VBUS",         C_PWR,  "",             "5 V from USB",                  "free"),
    (39, P,  "",     "VSYS",         C_PWR,  "",             "LiPo / Vin rail, 3.6\u20136 V", "free"),
    (38, G,  "",     "GND",          C_GND,  "",             "",                              "free"),
    (37, P,  "",     "3V3_EN",       C_PWR2, "",             "",                              "free"),
    (36, P,  "",     "3V3 (OUT)",    C_PWR,  "",             "3V3 rail \u2014 300 mA total",  "free"),
    (35, P,  "",     "ADC_VREF",     C_PWR2, "",             "",                              "free"),
    (34, IO, "GP28", "GPIO in",      C_GPIO, "GROVE 7",      "Right encoder B (direction)",   "used"),
    (33, G,  "",     "AGND",         C_GND,  "",             "",                              "free"),
    (32, IO, "GP27", "GPIO 2-edge",  C_GPIO, "GROVE 6",      "IR barcode \u2014 DO",          "used"),
    (31, IO, "GP26", "ADC0",         C_ADC,  "GROVE 6 (+5)", "IR barcode \u2014 AO",          "warn"),
    (30, P,  "",     "RUN",          C_PWR2, "",             "reset",                         "free"),
    (29, IO, "GP22", "GPIO / PWM",   C_GPIO, "on-board",     "Piezo buzzer (not used)",       "free"),
    (28, G,  "",     "GND",          C_GND,  "",             "",                              "free"),
    (27, IO, "GP21", "GPIO in",      C_GPIO, "on-board",     "Button 2 (not used)",           "free"),
    (26, IO, "GP20", "GPIO in",      C_GPIO, "on-board",     "Button 1 (not used)",           "free"),
    (25, IO, "GP19", "GPIO out",     C_GPIO, "breakout hdr", "Status LED + 330 \u03a9",       "warn"),
    (24, IO, "GP18", "PIO",          C_GPIO, "on-board",     "2 \u00d7 WS2812 RGB (not used)", "free"),
    (23, G,  "",     "GND",          C_GND,  "",             "",                              "free"),
    (22, IO, "GP17", "GPIO",         C_GPIO, "GROVE 4",      "spare (line 1's AO, unused)",   "free"),
    (21, IO, "GP16", "GPIO in",      C_GPIO, "GROVE 4",      "IR line sensor 1 (L) \u2014 DO", "warn"),
]

# ----------------------------------------------------------------- heading
ctext(W / 2, 62, "Robo Pico \u2014 Hardware Integration Pin Map", F_TITLE)
ctext(W / 2, 108,
      "INF2004-IS07 autonomous robotic car   \u00b7   every Pico pin as broken out by the Cytron Robo Pico   \u00b7   "
      "assignments from core/rc_config.h",
      F_SUB, MUTED)

legend = [("Used by this project", USED[0], "#2f6b1c"),
          ("Broken out, spare", FREE[0], "#b5b5b5"),
          ("Conflict \u2014 see notes below", WARN[0], "#bf9000")]
lx = 690
for lbl, fl, eg in legend:
    d.rounded_rectangle([lx, 146, lx + 32, 172], radius=6, fill=fl, outline=eg, width=2)
    d.text((lx + 44, 147), lbl, font=F_LEG, fill=INK)
    lx += 44 + tw(lbl, F_LEG) + 62

# column headings
hdr_y = 232
lx_pin = BX0 - 20 - PIN_W
lx_gp = lx_pin - GAP - GP_W
lx_if = lx_gp - GAP - IF_W
lx_pt = lx_if - GAP - PORT_W
lx_dev = lx_pt - GAP - DEV_W

rx_pin = BX1 + 20
rx_gp = rx_pin + PIN_W + GAP
rx_if = rx_gp + GP_W + GAP
rx_pt = rx_if + IF_W + GAP
rx_dev = rx_pt + PORT_W + GAP

for x, w, lbl in ((lx_dev, DEV_W, "What we connect"), (lx_pt, PORT_W, "Robo Pico port"),
                  (lx_if, IF_W, "Function"), (lx_gp, GP_W, "GPIO"), (lx_pin, PIN_W, "Pin")):
    ctext(x + w / 2, hdr_y, lbl, F_HEAD, MUTED)
for x, w, lbl in ((rx_pin, PIN_W, "Pin"), (rx_gp, GP_W, "GPIO"), (rx_if, IF_W, "Function"),
                  (rx_pt, PORT_W, "Robo Pico port"), (rx_dev, DEV_W, "What we connect")):
    ctext(x + w / 2, hdr_y, lbl, F_HEAD, MUTED)

# ----------------------------------------------------------------- board
BTOP, BBOT = TOP - 30, TOP + ROWS * ROW_H + 16
d.rounded_rectangle([BX0, BTOP, BX1, BBOT], radius=22, fill=BOARD_F, outline=BOARD_E, width=3)
ctext((BX0 + BX1) / 2, BTOP + 34, "Cytron", F_SMALL, "#7e57c2")
ctext((BX0 + BX1) / 2, BTOP + 66, "ROBO PICO", F_BRAND, "#5e35b1")
ctext((BX0 + BX1) / 2, BTOP + 96, "carrier board", F_SMALL, "#7e57c2")

# Pico W socket in the middle of the carrier
sx0, sx1 = BX0 + 118, BX1 - 118
sy0, sy1 = BTOP + 132, BBOT - 150
d.rounded_rectangle([sx0, sy0, sx1, sy1], radius=14, fill=SOCK_F, outline=SOCK_E, width=3)
# USB shield at the top of the socket
d.rounded_rectangle([(sx0 + sx1) / 2 - 42, sy0 - 26, (sx0 + sx1) / 2 + 42, sy0 + 20],
                    radius=8, fill="#d7dade", outline="#8c9196", width=3)
ctext((sx0 + sx1) / 2, sy0 + 46, "USB = console", F_SMALL, "#ffffff")
# RP2040
d.rounded_rectangle([(sx0 + sx1) / 2 - 62, (sy0 + sy1) / 2 - 62, (sx0 + sx1) / 2 + 62, (sy0 + sy1) / 2 + 62],
                    radius=8, fill="#20242a", outline="#0e1013", width=2)
ctext((sx0 + sx1) / 2, (sy0 + sy1) / 2 - 16, "RP2040", F_PORT, "#e8e8e8")
ctext((sx0 + sx1) / 2, (sy0 + sy1) / 2 + 14, "+ CYW43439", F_SMALL, "#b9c0c8")
ctext((sx0 + sx1) / 2, (sy0 + sy1) / 2 + 42, "radio", F_SMALL, "#b9c0c8")
ctext((sx0 + sx1) / 2, sy1 - 78, "Raspberry Pi Pico W", F_SMALL, "#ffffff")
ctext((sx0 + sx1) / 2, sy1 - 50, "GP23 / 24 / 25 / 29 \u2192 radio,", F_SMALL, "#cfe6d6")
ctext((sx0 + sx1) / 2, sy1 - 26, "not on the header", F_SMALL, "#cfe6d6")

ctext((BX0 + BX1) / 2, BBOT - 104, "2 \u00d7 20 GPIO breakout headers", F_SMALL, "#5e35b1")
ctext((BX0 + BX1) / 2, BBOT - 78, "every Pico pin is re-exposed on the", F_SMALL, "#7e57c2")
ctext((BX0 + BX1) / 2, BBOT - 56, "carrier's Grove / motor / servo ports", F_SMALL, "#7e57c2")

# ----------------------------------------------------------------- pin rows
def draw_side(rows, side):
    for i, (pin, kind, gp, iface, ifcol, port, dev, state) in enumerate(rows):
        cy = TOP + i * ROW_H + ROW_H / 2

        # header pad + stub into the board
        pad_x = BX0 + 12 if side == "L" else BX1 - 12
        d.ellipse([pad_x - 9, cy - 9, pad_x + 9, cy + 9], fill="#f3c969", outline="#8a6b18", width=2)
        if side == "L":
            d.line([(BX0 - 20, cy), (pad_x - 9, cy)], fill="#9aa0a6", width=2)
        else:
            d.line([(pad_x + 9, cy), (BX1 + 20, cy)], fill="#9aa0a6", width=2)

        if side == "L":
            xp, xg, xi, xt, xd = lx_pin, lx_gp, lx_if, lx_pt, lx_dev
        else:
            xp, xg, xi, xt, xd = rx_pin, rx_gp, rx_if, rx_pt, rx_dev

        chip(xp, cy, PIN_W, str(pin), C_PIN, F_NUM, r=5)

        if kind == IO:
            chip(xg, cy, GP_W, gp, C_GP, F_GP)
            chip(xi, cy, IF_W, iface, ifcol, F_IF)
        else:
            # power / ground rails span the GPIO + function columns
            chip(xg, cy, GP_W + GAP + IF_W, iface, ifcol, F_IF)

        if port:
            chip(xt, cy, PORT_W, port, state_pair(state), F_PORT, border=state_edge(state))
        if dev:
            f = F_DEVB if state == "used" else F_DEV
            chip(xd, cy, DEV_W, dev, state_pair(state), f, border=state_edge(state))


draw_side(left, "L")
draw_side(right, "R")

# ----------------------------------------------------------------- notes
ny = TOP + ROWS * ROW_H + 52
d.text((70, ny), "Conflicts handled (full detail in docs/HARDWARE.md \u00a71):", font=F_NOTEB, fill=INK)
ny += 33
notes = [
    "1.  GP0 / GP1 \u2014 these are also UART0, the port's default console. The left encoder lives here, so every build MUST be CONSOLE=usb_cdc: "
    "the console comes out of the Pico's own USB port and no UART pin is driven.",
    "2.  GP4 / GP5 \u2014 in RP2040 silicon GP4 is I\u00b2C0 SDA and GP5 is I\u00b2C0 SCL, fixed. The port's I\u00b2C0 driver defaults to GP8/GP9 (the left motor), "
    "so its unit-0 pin table is re-pinned to GP4/GP5 in i2c_rp2040.c and the IMU opens \"iica\". Wire SDA\u2192GP4, SCL\u2192GP5.",
    "3.  GP3 \u2014 the HC-SR04 ECHO line idles at 5 V and the RP2040 is not 5 V tolerant. Fit 1 k\u03a9 from ECHO to GP3 and 2 k\u03a9 from GP3 to GND (3.33 V at the pin). "
    "Connecting it directly destroys the board.",
    "4.  GP16 / GP19 \u2014 the port defaults BOARD_LED_PIN to GP16, which is line sensor 1 here, so it is moved to GP19 in sysdef.h. "
    "The Pico W's on-board LED is wired to the CYW43439 radio rather than to an RP2040 pin, so an external LED is required.",
    "5.  GP26 / GP6 \u2014 the Robo Pico routes GP26 to BOTH Grove 6 pin 1 and Grove 5 pin 2. Line sensor 2 sits on Grove 5: connect only its DO wire (GP6) "
    "and leave its AO unconnected, or it is shorted onto the barcode sensor's AO.",
    "6.  PWM slices 4, 5 and 7 are consumed by the two motors and the scan servo, so the kernel's PWM-based StartPhysicalTimer cannot use them. "
    "This tree uses the RP2040 TIMER alarms instead, which the kernel never touches.",
]
def wrap(text, f, maxw):
    words, lines, cur = text.split(" "), [], ""
    for wd in words:
        trial = (cur + " " + wd).strip()
        if tw(trial, f) <= maxw:
            cur = trial
        else:
            lines.append(cur)
            cur = wd
    if cur:
        lines.append(cur)
    return lines


for n in notes:
    for k, ln in enumerate(wrap(n, F_NOTE, W - 70 - 80)):
        d.text((70 if k == 0 else 104, ny), ln, font=F_NOTE, fill=MUTED)
        ny += 29
    ny += 3

ny += 6
for ln in wrap("Pin numbers are the physical Pico header positions, which the Robo Pico re-exposes on its breakout headers "
               "and routes to its Grove, motor and servo connectors. For where each device physically plugs in, see the "
               "board-level view in robopico_board_view.png.", F_NOTE, W - 150):
    d.text((70, ny), ln, font=F_NOTE, fill=INK)
    ny += 29

# ----------------------------------------------------------------- output
here = os.path.dirname(os.path.abspath(__file__))
root = os.path.abspath(os.path.join(here, "..", "..", ".."))
out1 = os.path.join(here, "robopico_pin_map.png")
img.save(out1)
print("wrote", out1)

out2 = os.path.join(root, "documents", "week6_diagrams", "d2_pin_layout.png")
if os.path.isdir(os.path.dirname(out2)):
    img.save(out2)
    print("wrote", out2)
