"""
Generate the INF2004-IS07 pin map in the style of the official Raspberry Pi
Pico pinout poster: the Pico W drawn in the centre with numbered
castellations, one row per physical pin, and colour-coded function chips
fanning out left and right.

Reading a row from the board outward:

    pin# | GPx | SPI / I2C / UART alternates | PWM | ROBO PICO PORT | what we connect

The alternate-function chips are the RP2040's real pin functions in the same
order and colours as the official poster. On this car most are unused, so
they are drawn faded; the function a pin actually uses is drawn solid with a
dark outline. The two outermost chips are what the poster does not have: the
Robo Pico connector that pin is routed to (from the Cytron datasheet) and the
device this project plugs in there (from core/rc_config.h).

Run:  python docs/img/hw/gen_pin_map.py
Out:  docs/img/hw/robopico_pin_map.png
      documents/week6_diagrams/d2_pin_layout.png (copy, for the Word doc)
"""

import os
from PIL import Image, ImageDraw, ImageFont

# ----------------------------------------------------------------- palette (poster colours)
BG       = "#ffffff"
C_GP     = ("#a5d16b", "#1e3a10")     # GPIO number
C_PIN    = ("#d9d9d9", "#222222")     # physical pin number
C_GND    = ("#333333", "#ffffff")
C_PWR    = ("#e53935", "#ffffff")     # VBUS / VSYS / 3V3
C_PWR2   = ("#f4b7b3", "#7a1c17")     # 3V3_EN / RUN
C_UART   = ("#8e6fc1", "#ffffff")
C_I2C    = ("#4aa3df", "#ffffff")
C_SPI    = ("#e5559b", "#ffffff")
C_ADC    = ("#2e7d4f", "#ffffff")
C_PWM    = ("#ef8f2f", "#ffffff")     # not on the poster; added where this car uses PWM
C_PORT   = ("#6a3fb5", "#ffffff")     # Robo Pico connector (the board is purple)
C_DEV    = ("#ffffff", "#1a1a1a")     # what we connect
C_SPARE  = ("#f2f2f2", "#8a8a8a")

BOARD_F  = "#2e8b45"
BOARD_E  = "#1c5a2c"
PAD_F    = "#e8c85a"
PAD_E    = "#7d6417"

FONTDIR = r"C:\Windows\Fonts"


def font(name, size):
    for cand in (name, "arial.ttf"):
        try:
            return ImageFont.truetype(os.path.join(FONTDIR, cand), size)
        except OSError:
            continue
    return ImageFont.load_default()


F_CHIP  = font("arialbd.ttf", 19)
F_CHIPS = font("arial.ttf", 18)
F_DEV   = font("arial.ttf", 19)
F_DEVB  = font("arialbd.ttf", 19)
F_NUM   = font("arialbd.ttf", 16)
F_BOARD = font("arialbd.ttf", 18)
F_SMALL = font("arial.ttf", 16)
F_TITLE = font("arialbd.ttf", 34)
F_SUB   = font("arial.ttf", 20)

# ----------------------------------------------------------------- geometry
W, H     = 2680, 1560
ROW_H    = 46
ROWS     = 20
TOP      = 360
BX0, BX1 = 1145, 1535          # the Pico W body
CH       = 34                  # chip height
GAP      = 6

PIN_W, GP_W, FN_W, PORT_W, DEV_W = 42, 84, 110, 150, 312

img = Image.new("RGB", (W, H), BG)
d = ImageDraw.Draw(img)


def tw(s, f):
    return d.textbbox((0, 0), s, font=f)[2]


def ctext(cx, cy, s, f, fill):
    d.text((cx - tw(s, f) / 2, cy - f.size * 0.70), s, font=f, fill=fill)


def fade(hexcol, k=0.32):
    r, g, b = int(hexcol[1:3], 16), int(hexcol[3:5], 16), int(hexcol[5:7], 16)
    m = lambda c: int(c * k + 255 * (1 - k))
    return "#%02x%02x%02x" % (m(r), m(g), m(b))


def chip(x0, cy, w, label, pair, f=F_CHIP, faded=False, outline=None, r=6):
    fill, txt = pair
    if faded:
        fill, txt = fade(fill), fade(txt, 0.55)
    d.rounded_rectangle([x0, cy - CH / 2, x0 + w, cy + CH / 2], radius=r,
                        fill=fill, outline=outline or fill, width=3 if outline else 1)
    ctext(x0 + w / 2, cy, label, f, txt)


def vtext(text, f, fill, bg):
    """Text rendered on a strip and rotated 90 degrees (reads bottom-to-top)."""
    strip = Image.new("RGB", (tw(text, f) + 20, f.size + 12), bg)
    ImageDraw.Draw(strip).text((10, 4), text, font=f, fill=fill)
    return strip.rotate(90, expand=True)


# ----------------------------------------------------------------- pin table
# (pin, gp, alternates inner->outer as on the poster, in-use index, PWM chip,
#  Robo Pico port, device, used)
G, P = "gnd", "pwr"
UART, I2C, SPI, ADC = C_UART, C_I2C, C_SPI, C_ADC

left = [
    (1,  "GP0",  [("SPI0 RX", SPI), ("I2C0 SDA", I2C), ("UART0 TX", UART)],  None, None,     "GROVE 1",    "Left encoder A  (IRQ)",        True),
    (2,  "GP1",  [("SPI0 CSn", SPI), ("I2C0 SCL", I2C), ("UART0 RX", UART)], None, None,     "GROVE 1",    "Left encoder B  (direction)",  True),
    (3,  G,      [], None, None, "", "", False),
    (4,  "GP2",  [("SPI0 SCK", SPI), ("I2C1 SDA", I2C)],                     None, None,     "GROVE 2",    "HC-SR04 TRIG",                 True),
    (5,  "GP3",  [("SPI0 TX", SPI), ("I2C1 SCL", I2C)],                      None, None,     "GROVE 2",    "HC-SR04 ECHO  (5 V, divider)", True),
    (6,  "GP4",  [("SPI0 RX", SPI), ("I2C0 SDA", I2C), ("UART1 TX", UART)],  1,    None,     "GROVE 3",    "IMU SDA  (LSM303DLHC)",        True),
    (7,  "GP5",  [("SPI0 CSn", SPI), ("I2C0 SCL", I2C), ("UART1 RX", UART)], 1,    None,     "GROVE 3",    "IMU SCL  (LSM303DLHC)",        True),
    (8,  G,      [], None, None, "", "", False),
    (9,  "GP6",  [("SPI0 SCK", SPI), ("I2C1 SDA", I2C)],                     None, None,     "GROVE 5",    "IR line sensor 2 (R)  DO",     True),
    (10, "GP7",  [("SPI0 TX", SPI), ("I2C1 SCL", I2C)],                      None, None,     "GROVE 7",    "Right encoder A  (IRQ)",       True),
    (11, "GP8",  [("SPI1 RX", SPI), ("I2C0 SDA", I2C), ("UART1 TX", UART)],  None, "PWM 4A", "MOTOR1 M1A", "Left motor",                   True),
    (12, "GP9",  [("SPI1 CSn", SPI), ("I2C0 SCL", I2C), ("UART1 RX", UART)], None, "PWM 4B", "MOTOR1 M1B", "Left motor",                   True),
    (13, G,      [], None, None, "", "", False),
    (14, "GP10", [("SPI1 SCK", SPI), ("I2C1 SDA", I2C)],                     None, "PWM 5A", "MOTOR2 M2A", "Right motor",                  True),
    (15, "GP11", [("SPI1 TX", SPI), ("I2C1 SCL", I2C)],                      None, "PWM 5B", "MOTOR2 M2B", "Right motor",                  True),
    (16, "GP12", [("SPI1 RX", SPI), ("I2C0 SDA", I2C), ("UART0 TX", UART)],  None, None,     "SERVO 1",    "spare",                        False),
    (17, "GP13", [("SPI1 CSn", SPI), ("I2C0 SCL", I2C), ("UART0 RX", UART)], None, None,     "SERVO 2",    "spare",                        False),
    (18, G,      [], None, None, "", "", False),
    (19, "GP14", [("SPI1 SCK", SPI), ("I2C1 SDA", I2C)],                     None, None,     "SERVO 3",    "spare",                        False),
    (20, "GP15", [("SPI1 TX", SPI), ("I2C1 SCL", I2C)],                      None, "PWM 7B", "SERVO 4",    "Scan servo SG90",              True),
]

right = [
    (40, P, [("VBUS", C_PWR)],      None, None, "", "", False),
    (39, P, [("VSYS", C_PWR)],      None, None, "", "", False),
    (38, G, [], None, None, "", "", False),
    (37, P, [("3V3_EN", C_PWR2)],   None, None, "", "", False),
    (36, P, [("3V3(OUT)", C_PWR)],  None, None, "", "", False),
    (35, P, [("ADC_VREF", C_PWR2)], None, None, "", "", False),
    (34, "GP28", [("ADC2", ADC)],                                    None, None, "GROVE 7",  "Right encoder B  (direction)", True),
    (33, G, [("AGND", C_GND)], None, None, "", "", False),
    (32, "GP27", [("ADC1", ADC), ("I2C1 SCL", I2C)],                 None, None, "GROVE 6",  "IR barcode DO  (IRQ)",         True),
    (31, "GP26", [("ADC0", ADC), ("I2C1 SDA", I2C)],                 0,    None, "GROVE 6",  "IR barcode AO",                True),
    (30, P, [("RUN", C_PWR2)],      None, None, "", "", False),
    (29, "GP22", [],                                                 None, None, "on-board", "piezo buzzer (unused)",        False),
    (28, G, [], None, None, "", "", False),
    (27, "GP21", [("I2C0 SCL", I2C)],                                None, None, "on-board", "button (unused)",              False),
    (26, "GP20", [("I2C0 SDA", I2C)],                                None, None, "on-board", "button (unused)",              False),
    (25, "GP19", [("SPI0 TX", SPI), ("I2C1 SCL", I2C)],              None, None, "header",   "Status LED + 330 \u03a9",      True),
    (24, "GP18", [("SPI0 SCK", SPI), ("I2C1 SDA", I2C)],             None, None, "on-board", "2 \u00d7 WS2812 RGB (unused)", False),
    (23, G, [], None, None, "", "", False),
    (22, "GP17", [("SPI0 CSn", SPI), ("I2C0 SCL", I2C), ("UART0 RX", UART)], None, None, "GROVE 4", "spare",                False),
    (21, "GP16", [("SPI0 RX", SPI), ("I2C0 SDA", I2C), ("UART0 TX", UART)],  None, None, "GROVE 4", "IR line sensor 1 (L)  DO", True),
]

# ----------------------------------------------------------------- heading (top-left, clear of the radio tag)
d.text((70, 40), "Robo Pico \u2014 Pin Map, INF2004-IS07", font=F_TITLE, fill="#1a1a1a")
d.text((70, 92), "Pico W pinout as on the official poster.  Faded = function available but unused on this car; solid + outlined = in use.",
       font=F_SUB, fill="#666666")
d.text((70, 120), "Outer chips: the Robo Pico socket each pin is routed to, and what this car connects there.",
       font=F_SUB, fill="#666666")

# ----------------------------------------------------------------- board
BTOP = TOP - 60
BBOT = TOP + ROWS * ROW_H + 40
d.rounded_rectangle([BX0, BTOP, BX1, BBOT], radius=26, fill=BOARD_F, outline=BOARD_E, width=4)
d.rounded_rectangle([BX0 + 10, BTOP + 10, BX1 - 10, BBOT - 10], radius=20, outline="#3fa457", width=2)

# USB connector + radio tag above it (the poster's "LED (GP25)" spot)
ucx = (BX0 + BX1) / 2
d.rounded_rectangle([ucx - 44, BTOP - 36, ucx + 44, BTOP + 34], radius=8, fill="#d0d4d8", outline="#7d858c", width=3)
d.rectangle([ucx - 30, BTOP - 20, ucx + 30, BTOP + 6], fill="#eef0f2", outline="#9aa1a7")
ctext(ucx, BTOP + 56, "USB  (console)", F_SMALL, "#ffffff")
tag = vtext("WiFi  GP23/24/25/29", F_SMALL, C_GP[1], C_GP[0])
img.paste(tag, (int(ucx) - 70 - tag.width // 2, BTOP - 50 - tag.height))
d.line([(ucx - 70, BTOP - 50), (ucx - 70, BTOP - 36)], fill="#1e3a10", width=2)

# BOOTSEL
d.rounded_rectangle([ucx - 22, BTOP + 110, ucx + 22, BTOP + 150], radius=6, fill="#f2f2f2", outline="#9a9a9a", width=2)
ctext(ucx, BTOP + 172, "BOOTSEL", F_SMALL, "#ffffff")

# RP2040
mcy = (BTOP + BBOT) / 2 - 10
d.rounded_rectangle([ucx - 58, mcy - 58, ucx + 58, mcy + 58], radius=8, fill="#1f2328", outline="#0b0d0f", width=2)
ctext(ucx, mcy - 14, "RP2040", F_BOARD, "#ffffff")
ctext(ucx, mcy + 14, "Pico W", F_SMALL, "#c9d1d9")

# board name, rotated, between the left pin numbers and the chip
side = vtext("Raspberry Pi Pico W", F_BOARD, "#d9f0dd", BOARD_F)
img.paste(side, (BX0 + 78, int(mcy) + 80))

# pads + numbers
for i in range(ROWS):
    cy = TOP + i * ROW_H + ROW_H / 2
    for x in (BX0 + 22, BX1 - 22):
        d.ellipse([x - 9, cy - 9, x + 9, cy + 9], fill=PAD_F, outline=PAD_E, width=2)
        d.ellipse([x - 4, cy - 4, x + 4, cy + 4], fill="#fff6d0")
    ctext(BX0 + 50, cy, str(i + 1), F_NUM, "#ffffff")
    ctext(BX1 - 50, cy, str(40 - i), F_NUM, "#ffffff")

# DEBUG pads below the board
ctext(ucx, BBOT - 26, "DEBUG", F_SMALL, "#ffffff")
for k, (lbl, col) in enumerate((("SWCLK", "#ef8f2f"), ("GND", "#333333"), ("SWDIO", "#ef8f2f"))):
    x = ucx - 48 + k * 48
    d.ellipse([x - 8, BBOT - 8, x + 8, BBOT + 8], fill=PAD_F, outline=PAD_E, width=2)
    t = vtext(lbl, F_SMALL, "#ffffff", col).rotate(180)
    img.paste(t, (int(x) - t.width // 2, int(BBOT) + 14))


# ----------------------------------------------------------------- rows
def draw_side(rows, side):
    for i, (pin, gp, fns, use, pwm, port, dev, used) in enumerate(rows):
        cy = TOP + i * ROW_H + ROW_H / 2
        if side == "L":
            x = BX0 - 14
            def place(w):          # walk outward to the left
                nonlocal x
                x -= w
                x0 = x
                x -= GAP
                return x0
            d.line([(BX0 - 14, cy), (BX0 + 13, cy)], fill="#9aa0a6", width=2)
        else:
            x = BX1 + 14
            def place(w):          # walk outward to the right
                nonlocal x
                x0 = x
                x += w + GAP
                return x0
            d.line([(BX1 - 13, cy), (BX1 + 14, cy)], fill="#9aa0a6", width=2)

        chip(place(PIN_W), cy, PIN_W, str(pin), C_PIN, F_NUM)

        if gp == G:
            lbl = fns[0][0] if fns else "GND"
            chip(place(GP_W + GAP + FN_W), cy, GP_W + GAP + FN_W, lbl, C_GND, F_CHIP)
            continue
        if gp == P:
            lbl, col = fns[0]
            chip(place(GP_W + GAP + FN_W), cy, GP_W + GAP + FN_W, lbl, col, F_CHIP)
            continue

        chip(place(GP_W), cy, GP_W, gp, C_GP, F_CHIP,
             outline="#1e3a10" if (used and use is None and not pwm) else None)

        # three alternate slots (SPI, I2C, UART as on the poster) plus a fixed
        # PWM slot, so the outer columns line up on every row
        for k in range(3):
            x0 = place(FN_W)
            if k < len(fns):
                lbl, col = fns[k]
                in_use = (use == k)
                chip(x0, cy, FN_W, lbl, col, F_CHIPS, faded=not in_use, outline="#1a1a1a" if in_use else None)
        x0 = place(FN_W)
        if pwm:
            chip(x0, cy, FN_W, pwm, C_PWM, F_CHIPS, outline="#1a1a1a")

        if port:
            chip(place(PORT_W), cy, PORT_W, port, C_PORT, F_CHIP, faded=not used)
        else:
            place(PORT_W)
        if dev:
            chip(place(DEV_W), cy, DEV_W, dev, C_DEV if used else C_SPARE, F_DEVB if used else F_DEV,
                 outline="#2f6b1c" if used else "#c8c8c8")


draw_side(left, "L")
draw_side(right, "R")

# ----------------------------------------------------------------- legend + notes
ly = BBOT + 120
items = [("GPIO", C_GP), ("power", C_PWR), ("GND", C_GND), ("UART", C_UART), ("I\u00b2C", C_I2C), ("SPI", C_SPI),
         ("ADC", C_ADC), ("PWM", C_PWM), ("Robo Pico socket", C_PORT), ("this car", C_DEV)]
x = 70
for lbl, pair in items:
    w = tw(lbl, F_CHIPS) + 26
    chip(x, ly, w, lbl, pair, F_CHIPS, outline="#2f6b1c" if lbl == "this car" else None)
    x += w + 16
d.text((70, ly + 30),
       "Grove 1 is on the Robo Pico's left edge, Grove 7 on the right edge, Grove 2\u20136 along the bottom; each device is one Grove cable "
       "(GND, 3V3, two signals). GP26 is on both Grove 5 and Grove 6, so line sensor 2 on Grove 5 uses DO only.",
       font=F_SMALL, fill="#555555")
d.text((70, ly + 54),
       "Patched in the kernel port by build/setup.sh: I\u00b2C0 moved from GP8/9 to GP4/5, UART0 off GP0/1 (the console is USB), "
       "status LED GP16\u2192GP19, GP27/28 kept digital. Full detail: docs/HARDWARE.md \u00a71.",
       font=F_SMALL, fill="#555555")

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
