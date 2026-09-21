"""
Generate the INF2004-IS07 pin map: the Cytron Robo Pico drawn as it really
is (landscape, purple; motor terminals and the 4-way servo header along the
top, Grove 1 on the left edge, Grove 7 on the right edge, Grove 2-6 along
the bottom, the Pico socket and the two 20-way headers in the middle), with
poster-style colour-coded chips fanning out from every connector this car
uses.

Chip colours follow the official Pico pinout poster: green = GPIO number,
blue = I2C, dark green = ADC, orange = PWM, purple = Robo Pico connector,
white/green-outlined = the device this car connects. Faded chips are parts
of the board this car does not use.

Board geometry and connector pin-outs come from the Cytron Robo Pico
datasheet (documents/Robo_Pico.pdf); signal assignments from core/rc_config.h.

Run:  python docs/img/hw/gen_pin_map.py
Out:  docs/img/hw/robopico_pin_map.png
      documents/week6_diagrams/d2_pin_layout.png (copy, for the Word doc)
"""

import os
from PIL import Image, ImageDraw, ImageFont

# ----------------------------------------------------------------- palette
BG      = "#ffffff"
C_GP    = ("#a5d16b", "#1e3a10")
C_I2C   = ("#4aa3df", "#ffffff")
C_ADC   = ("#2e7d4f", "#ffffff")
C_PWM   = ("#ef8f2f", "#ffffff")
C_PWR   = ("#e53935", "#ffffff")
C_GND   = ("#333333", "#ffffff")
C_PORT  = ("#6a3fb5", "#ffffff")
C_TERM  = ("#3a3f44", "#ffffff")      # motor terminal pin (M1A ...)
C_DEV   = ("#ffffff", "#1a1a1a")
C_SPARE = ("#f2f2f2", "#8a8a8a")
LEAD    = "#6b6b6b"

PCB     = "#5c2d91"
PCB_E   = "#3b1a5e"
PCB_L   = "#6e3aa8"
WHITE   = "#f4f1ea"
GOLD    = "#d8b64a"

FONTDIR = r"C:\Windows\Fonts"


def font(name, size):
    for cand in (name, "arial.ttf"):
        try:
            return ImageFont.truetype(os.path.join(FONTDIR, cand), size)
        except OSError:
            continue
    return ImageFont.load_default()


F_TITLE = font("arialbd.ttf", 34)
F_SUB   = font("arial.ttf", 20)
F_CHIP  = font("arialbd.ttf", 19)
F_CHIPS = font("arial.ttf", 18)
F_DEVB  = font("arialbd.ttf", 19)
F_SILK  = font("arialbd.ttf", 15)     # silkscreen labels on the board
F_TINY  = font("arial.ttf", 12)
F_BRAND = font("arialbd.ttf", 30)
F_SMALL = font("arial.ttf", 16)

# ----------------------------------------------------------------- geometry
W, H = 2600, 1860
BX0, BY0, BW, BH = 750, 540, 1100, 900          # 88 x 72 mm -> 1.22
CH, GAP = 34, 6

img = Image.new("RGB", (W, H), BG)
d = ImageDraw.Draw(img)


def bx(f): return BX0 + f * BW
def by(f): return BY0 + f * BH


def tw(s, f): return d.textbbox((0, 0), s, font=f)[2]


def ctext(cx, cy, s, f, fill):
    d.text((cx - tw(s, f) / 2, cy - f.size * 0.70), s, font=f, fill=fill)


def fade(hexcol, k=0.32):
    r, g, b = int(hexcol[1:3], 16), int(hexcol[3:5], 16), int(hexcol[5:7], 16)
    m = lambda c: int(c * k + 255 * (1 - k))
    return "#%02x%02x%02x" % (m(r), m(g), m(b))


def chip(x0, cy, w, label, pair, f=F_CHIP, faded=False, outline=None):
    fill, txt = pair
    if faded:
        fill, txt = fade(fill), fade(txt, 0.55)
    d.rounded_rectangle([x0, cy - CH / 2, x0 + w, cy + CH / 2], radius=6,
                        fill=fill, outline=outline or fill, width=3 if outline else 1)
    ctext(x0 + w / 2, cy, label, f, txt)
    return x0 + w


def row(x0, cy, items, faded=False):
    """items: [(label, pair, width, outline)] laid out left to right."""
    x = x0
    for label, pair, w, outline in items:
        chip(x, cy, w, label, pair, F_DEVB if pair is C_DEV and not faded else F_CHIP,
             faded=faded, outline=None if faded else outline)
        x += w + GAP
    return x - GAP


def row_r(x1, cy, items, faded=False):
    """Same, but right-aligned so the last chip ends at x1."""
    total = sum(w for _, _, w, _ in items) + GAP * (len(items) - 1)
    return row(x1 - total, cy, items, faded)


def lead(pts):
    d.line(pts, fill=LEAD, width=2)


def vtext(text, f, fill, bg, ccw=True):
    strip = Image.new("RGB", (tw(text, f) + 8, f.size + 6), bg)
    ImageDraw.Draw(strip).text((4, 1), text, font=f, fill=fill)
    return strip.rotate(90 if ccw else -90, expand=True)


DEV_O = "#2f6b1c"   # outline of an in-use device chip


# ================================================================= the board
d.rounded_rectangle([BX0, BY0, BX0 + BW, BY0 + BH], radius=30, fill=PCB, outline=PCB_E, width=4)
for fx, fy in ((0.045, 0.16), (0.955, 0.16), (0.045, 0.79), (0.955, 0.79)):
    d.ellipse([bx(fx) - 13, by(fy) - 13, bx(fx) + 13, by(fy) + 13], fill=BG, outline="#c9c9c9", width=2)

# --- top-left power: LiPo JST, VIN terminal, switch, PWR led
d.rectangle([bx(0.095), by(0.03), bx(0.165), by(0.105)], fill=WHITE, outline="#b8b3a8", width=2)
ctext(bx(0.13), by(0.068), "LIPO", F_TINY, "#666666")
d.rectangle([bx(0.185), by(0.02), bx(0.30), by(0.115)], fill="#4caf50", outline="#2e7d32", width=2)
for fx in (0.215, 0.27):
    d.ellipse([bx(fx) - 11, by(0.058) - 11, bx(fx) + 11, by(0.058) + 11], fill="#dcdcdc", outline="#888888", width=2)
ctext(bx(0.243), by(0.14), "3.6-6V  VIN", F_TINY, WHITE)
d.rectangle([bx(0.03), by(0.165), bx(0.10), by(0.215)], fill=WHITE, outline="#b8b3a8", width=2)
ctext(bx(0.065), by(0.19), "OFF  ON", F_TINY, "#444444")
d.ellipse([bx(0.13) - 4, by(0.19) - 4, bx(0.13) + 4, by(0.19) + 4], fill="#4cff4c")
ctext(bx(0.13), by(0.165), "PWR", F_TINY, WHITE)

# --- motor terminals (top), test buttons, driver IC
for fx0, fx1, name, lb, la in ((0.345, 0.49, "MOTOR 2", "M2B  GP11", "GP10  M2A"),
                               (0.54, 0.685, "MOTOR 1", "M1B  GP9", "GP8  M1A")):
    d.rectangle([bx(fx0), by(0.025), bx(fx1), by(0.125)], fill="#2b2b2b", outline="#111111", width=2)
    for fx in (fx0 + 0.035, fx1 - 0.035):
        d.ellipse([bx(fx) - 13, by(0.072) - 13, bx(fx) + 13, by(0.072) + 13], fill="#d0d0d0", outline="#777777", width=2)
    d.rounded_rectangle([bx((fx0 + fx1) / 2) - 40, by(0.143) - 10, bx((fx0 + fx1) / 2) + 40, by(0.143) + 10],
                        radius=8, fill=WHITE)
    ctext(bx((fx0 + fx1) / 2), by(0.143), name, F_TINY, PCB)
    d.text((bx(fx0) - 2, by(0.155)), lb, font=F_TINY, fill=WHITE)
    d.text((bx(fx1) - tw(la, F_TINY) + 2, by(0.155)), la, font=F_TINY, fill=WHITE)
for fx, lbl in ((0.365, "M2B"), (0.425, "M2A"), (0.60, "M1B"), (0.66, "M1A")):
    d.rounded_rectangle([bx(fx) - 12, by(0.20) - 12, bx(fx) + 12, by(0.20) + 12], radius=4, fill=WHITE, outline="#b8b3a8")
    ctext(bx(fx), by(0.235), lbl, F_TINY, WHITE)
d.rectangle([bx(0.475), by(0.185), bx(0.555), by(0.225)], fill="#1a1a1a")
for fx in (0.735, 0.955, 0.735):
    pass
for fx, fy in ((0.735, 0.06), (0.955, 0.06), (0.955, 0.19), (0.72, 0.16)):
    d.ellipse([bx(fx) - 20, by(fy) - 20, bx(fx) + 20, by(fy) + 20], fill="#3a3a3a", outline="#8a8a8a", width=2)
    d.ellipse([bx(fx) - 12, by(fy) - 12, bx(fx) + 12, by(fy) + 12], fill="#c8c8c8")

# --- servo header (4 x 3 pins) with GP12..GP15
d.rectangle([bx(0.79), by(0.035), bx(0.935), by(0.145)], fill="#1a1a1a", outline="#000000")
for c in range(4):
    for r in range(3):
        cx, cy = bx(0.808 + c * 0.036), by(0.055 + r * 0.037)
        d.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], fill=GOLD)
for c, gp in enumerate(("12", "13", "14", "15")):
    t = vtext("GP" + gp, F_TINY, WHITE, PCB)
    img.paste(t, (int(bx(0.808 + c * 0.036)) - t.width // 2, int(by(0.035)) - t.height - 2))
ctext(bx(0.96), by(0.035) - 8, "SERVO", F_TINY, WHITE)
d.text((bx(0.945), by(0.05)), "S", font=F_TINY, fill=WHITE)
d.text((bx(0.945), by(0.087)), "+", font=F_TINY, fill=WHITE)
d.text((bx(0.945), by(0.124)), "\u2212", font=F_TINY, fill=WHITE)

# --- 2 x 20-way headers with silkscreen labels, Pico socket between them
TOP_LBL = ["VBUS", "VSYS", "GND", "3V3_EN", "3V3", "ADC_VREF", "GP28", "GND", "GP27", "GP26",
           "RUN", "GP22", "GND", "GP21", "GP20", "GP19", "GP18", "GND", "GP17", "GP16"]
BOT_LBL = ["GP0", "GP1", "GND", "GP2", "GP3", "GP4", "GP5", "GND", "GP6", "GP7",
           "GP8", "GP9", "GND", "GP10", "GP11", "GP12", "GP13", "GND", "GP14", "GP15"]
HX0, HX1 = bx(0.20), bx(0.82)
for fy, labels, above in ((0.305, TOP_LBL, True), (0.655, BOT_LBL, False)):
    d.rectangle([HX0, by(fy) - 18, HX1, by(fy) + 18], fill="#151515", outline="#000000")
    for k, lbl in enumerate(labels):
        cx = HX0 + (k + 0.5) * (HX1 - HX0) / 20
        d.ellipse([cx - 5, by(fy) - 5, cx + 5, by(fy) + 5], fill=GOLD)
        t = (vtext(lbl, F_TINY, C_GP[1], C_GP[0]) if (above and lbl == "GP19")
             else vtext(lbl, F_TINY, WHITE if lbl != "GND" else "#c9b6e4", PCB))
        y = int(by(fy)) - 22 - t.height if above else int(by(fy)) + 22
        img.paste(t, (int(cx) - t.width // 2, y))
d.rounded_rectangle([HX0, by(0.345), HX1, by(0.615)], radius=8, fill=PCB_L, outline="#7b48b8", width=1)
t = vtext("PICO", F_BRAND, WHITE, PCB_L)
img.paste(t, (int(HX1) - t.width - 12, int(by(0.46)) - t.height // 2))
d.rectangle([bx(0.235), by(0.42), bx(0.275), by(0.49)], fill="#cfd8dc", outline="#607d8b", width=2)
t = vtext("USB", F_SILK, WHITE, PCB_L)
img.paste(t, (int(bx(0.285)), int(by(0.455)) - t.height // 2))
d.text((bx(0.205), by(0.44)), "\u2192", font=F_SILK, fill=WHITE)
for c in range(6):
    for r in range(3):
        d.rectangle([bx(0.40 + c * 0.055), by(0.38 + r * 0.075), bx(0.40 + c * 0.055) + 10, by(0.38 + r * 0.075) + 20],
                    fill="#3f2a5a", outline="#8a7aa3")

# --- branding
d.text((bx(0.035), by(0.30)), "Cytron", font=F_SMALL, fill="#e8dcf7")
d.text((bx(0.035), by(0.33)), "ROBO", font=F_BRAND, fill=WHITE)
d.text((bx(0.035), by(0.375)), "PICO", font=F_BRAND, fill=WHITE)

# --- buzzer + mute switch
d.ellipse([bx(0.845), by(0.40), bx(0.93), by(0.51)], fill="#1f1f1f", outline="#4a4a4a", width=3)
ctext(bx(0.8875), by(0.455), "GP22", F_TINY, "#bdbdbd")
d.rectangle([bx(0.94), by(0.43), bx(0.975), by(0.48)], fill=WHITE, outline="#b8b3a8")

# --- Grove 1 (left edge) and Grove 7 (right edge)
d.rectangle([BX0 - 22, by(0.52), bx(0.055), by(0.66)], fill=WHITE, outline="#b8b3a8", width=2)
t = vtext("GROVE 1", F_TINY, "#555555", WHITE); img.paste(t, (int(bx(0.0)) - 2, int(by(0.59)) - t.height // 2))
for k, lbl in enumerate(("GND", "3V3", "GP0", "GP1")):
    d.text((bx(0.065), by(0.525 + k * 0.034)), lbl, font=F_TINY, fill=WHITE)
d.rectangle([bx(0.945), by(0.52), BX0 + BW + 22, by(0.66)], fill=WHITE, outline="#b8b3a8", width=2)
t = vtext("GROVE 7", F_TINY, "#555555", WHITE); img.paste(t, (int(bx(0.985)) - 2, int(by(0.59)) - t.height // 2))
for k, lbl in enumerate(("GP28", "GP7", "3V3", "GND")):
    d.text((bx(0.895), by(0.525 + k * 0.034)), lbl, font=F_TINY, fill=WHITE)

# --- buttons row, RGB LEDs
for fx, lbl in ((0.29, "RST"), (0.43, "GP21"), (0.57, "GP20")):
    d.rounded_rectangle([bx(fx), by(0.70), bx(fx + 0.095), by(0.75)], radius=14, fill=WHITE, outline="#b8b3a8", width=2)
    ctext(bx(fx + 0.0475), by(0.725), lbl, F_TINY, "#444444")
for fx, n in ((0.035, "0"), (0.94, "1")):
    d.ellipse([bx(fx), by(0.70), bx(fx + 0.035), by(0.745)], fill="#2196f3", outline="#0d47a1", width=2)
    ctext(bx(fx + 0.0175), by(0.775), "GP18", F_TINY, WHITE)

# --- MAKER port
d.rectangle([bx(0.045), by(0.87), bx(0.125), by(0.95)], fill=WHITE, outline="#b8b3a8", width=2)
ctext(bx(0.085), by(0.965), "MAKER", F_TINY, WHITE)

# --- Grove 2..6 along the bottom edge
GROVES = [("GROVE 2", 0.152, 0.256, ("GND", "3V3", "GP2", "GP3")),
          ("GROVE 3", 0.320, 0.432, ("GND", "3V3", "GP4", "GP5")),
          ("GROVE 4", 0.488, 0.600, ("GND", "3V3", "GP16", "GP17")),
          ("GROVE 5", 0.656, 0.768, ("GND", "3V3", "GP6", "GP26")),
          ("GROVE 6", 0.824, 0.936, ("GND", "3V3", "GP26", "GP27"))]
for name, f0, f1, pins in GROVES:
    d.rectangle([bx(f0), by(0.85), bx(f1), BY0 + BH + 22], fill=WHITE, outline="#b8b3a8", width=2)
    ctext(bx((f0 + f1) / 2), by(0.985), name, F_TINY, "#555555")
    for k, lbl in enumerate(pins):
        t = vtext(lbl, F_TINY, WHITE, PCB)
        img.paste(t, (int(bx(f0 + 0.014 + k * 0.027)) - t.width // 2, int(by(0.85)) - t.height - 4))
    for k in range(4):
        cx = bx(f0 + 0.014 + k * 0.027)
        d.ellipse([cx - 3, by(0.78) - 3, cx + 3, by(0.78) + 3], fill="#2979ff")

# ================================================================= fan-out chips
# --- TOP: motors and servo, stacked so no leader crosses another stack
DEV_W, TERM_W, GP_W, FN_W = 190, 72, 70, 90

def motor_rows(x1, y, side_name, pins):
    for k, (term, gp, pwm, sign) in enumerate(pins):
        cy = y + k * (CH + GAP)
        end = row_r(x1, cy, [(f"{side_name} {sign}", C_DEV, DEV_W, DEV_O), (term, C_TERM, TERM_W, None),
                             (gp, C_GP, GP_W, None), (pwm, C_PWM, FN_W, "#1a1a1a")])
    return end

# MOTOR 2 (right motor), lowest stack, right-aligned to its terminal
m2x = bx(0.42)
motor_rows(m2x + 40, 385, "Left motor", (("M2A", "GP10", "PWM 5A", "+"), ("M2B", "GP11", "PWM 5B", "\u2212")))
lead([(m2x + 40, 385 + CH / 2 + 4), (m2x + 40, 445), (m2x, 445), (m2x, by(0.025))])
# MOTOR 1 (left motor), above it
m1x = bx(0.615)
motor_rows(m1x + 40, 275, "Right motor", (("M1A", "GP8", "PWM 4A", "+"), ("M1B", "GP9", "PWM 4B", "\u2212")))
lead([(m1x + 40, 275 + CH / 2 + 4), (m1x + 40, 327), (m1x, 327), (m1x, by(0.025))])
# SERVO port 4, top right, left-aligned
sx = bx(0.808 + 3 * 0.036)
row(sx + 60, 165, [("PWM 7B", C_PWM, FN_W, "#1a1a1a"), ("GP15", C_GP, GP_W, None), ("SERVO 4", C_PORT, 100, None),
                   ("Scan servo SG90 (pan)", C_DEV, 220, DEV_O)])
row(sx + 60, 165 + CH + GAP, [("GP12 GP13 GP14", C_GP, 156, None), ("SERVO 1-3", C_PORT, 100, None),
                              ("spare", C_SPARE, 80, None)], faded=True)
lead([(sx + 60, 165), (sx, 165), (sx, by(0.035) - 30)])
# power in
row_r(bx(0.243) + 20, 500, [("LiPo or Vin  3.6\u20136 V", C_PWR, 200, None)])
lead([(bx(0.243), 500 + CH / 2), (bx(0.243), by(0.02))])

# --- LEFT: Grove 1 = left encoder
g1y = by(0.59)
row_r(BX0 - 60, g1y - 44, [("Left encoder A  (IRQ)", C_DEV, 268, DEV_O), ("GP0", C_GP, GP_W, "#1e3a10")])
row_r(BX0 - 60, g1y,      [("Left encoder B  (direction)", C_DEV, 268, DEV_O), ("GP1", C_GP, GP_W, "#1e3a10")])
row_r(BX0 - 60, g1y + 44, [("encoder GND", C_GND, 130, None), ("3V3", C_PWR, 60, None), ("GROVE 1", C_PORT, 106, None)])
lead([(BX0 - 60 + 8, g1y), (BX0 - 22, g1y)])
# maker port (shares GP2/GP3 with Grove 2 -> keep empty)
row_r(BX0 - 60, by(0.91), [("MAKER port \u2014 same GP2/GP3 as Grove 2, leave empty", C_SPARE, 470, None)], faded=True)
lead([(BX0 - 52, by(0.91)), (bx(0.045), by(0.91))])

# --- RIGHT: Grove 7 = right encoder; buzzer, RGB (unused); status LED
g7y = by(0.59)
row(BX0 + BW + 60, g7y - 44, [("GP7", C_GP, GP_W, "#1e3a10"), ("Right encoder A  (IRQ)", C_DEV, 268, DEV_O)])
row(BX0 + BW + 60, g7y,      [("GP28", C_GP, GP_W, "#1e3a10"), ("Right encoder B  (direction)", C_DEV, 268, DEV_O)])
row(BX0 + BW + 60, g7y + 44, [("GROVE 7", C_PORT, 106, None), ("3V3", C_PWR, 60, None), ("encoder GND", C_GND, 130, None)])
lead([(BX0 + BW + 22, g7y), (BX0 + BW + 52, g7y)])
# status LED off the top header (GP19 is the 16th pin)
gp19x = HX0 + 15.5 * (HX1 - HX0) / 20
row(BX0 + BW + 60, by(0.215), [("GP19", C_GP, GP_W, "#1e3a10"), ("Status LED + 330 \u03a9  (20-way header pin)", C_DEV, 372, DEV_O)])
lead([(BX0 + BW + 52, by(0.215)), (bx(0.985), by(0.215)), (bx(0.985), by(0.232)), (gp19x, by(0.232)), (gp19x, by(0.245))])

# --- BOTTOM: Grove 2..6 columns
COLS = {
    "GROVE 2": [("GP2", C_GP, "#1e3a10", None), ("HC-SR04 TRIG", C_DEV, DEV_O, False),
                ("GP3", C_GP, "#1e3a10", None), ("HC-SR04 ECHO  5 V\u21923V3", C_DEV, DEV_O, False)],
    "GROVE 3": [("GP4", C_GP, None, "I2C0 SDA"), ("IMU SDA  (LSM303DLHC)", C_DEV, DEV_O, False),
                ("GP5", C_GP, None, "I2C0 SCL"), ("IMU SCL  (LSM303DLHC)", C_DEV, DEV_O, False)],
    "GROVE 4": [("GP16", C_GP, "#1e3a10", None), ("Line sensor 1 (L)  DO", C_DEV, DEV_O, False),
                ("GP17", C_GP, None, None), ("not connected", C_SPARE, None, True)],
    "GROVE 5": [("GP6", C_GP, "#1e3a10", None), ("Line sensor 2 (R)  DO", C_DEV, DEV_O, False),
                ("GP26", C_GP, None, None), ("= barcode AO, leave unwired", C_SPARE, None, True)],
    "GROVE 6": [("GP26", C_GP, None, "ADC0"), ("IR barcode AO", C_DEV, DEV_O, False),
                ("GP27", C_GP, None, "ADC1"), ("IR barcode DO  (IRQ)", C_DEV, DEV_O, False)],
}
COL_W, COL_PITCH = 236, 252
for k, (name, f0, f1, pins) in enumerate(GROVES):
    cx = bx((f0 + f1) / 2)                       # the real connector
    colx = BX0 + BW / 2 + (k - 2) * COL_PITCH    # its column, fanned out wider
    x0 = colx - COL_W / 2
    y = BY0 + BH + 78
    lead([(cx, BY0 + BH + 22), (cx, BY0 + BH + 44), (colx, BY0 + BH + 44), (colx, y - CH / 2 - 2)])
    chip(x0, y, COL_W, f"{name}   GND \u00b7 3V3", C_PORT, F_CHIP)
    y += CH + GAP
    for lbl, pair, outline, extra in COLS[name]:
        if pair is C_GP:
            faded = (extra is None and outline is None)
            if extra:                                   # function chip beside the GP chip
                chip(x0, y, 78, lbl, C_GP, F_CHIP, faded=False)
                chip(x0 + 78 + GAP, y, COL_W - 78 - GAP, extra, C_I2C if "I2C" in extra else C_ADC, F_CHIPS, outline="#1a1a1a")
            else:
                chip(x0, y, COL_W, lbl, C_GP, F_CHIP, faded=faded, outline=outline)
        else:
            faded = bool(extra)
            chip(x0, y, COL_W, lbl, pair, F_DEVB if not faded else F_CHIPS, faded=faded,
                 outline=None if faded else outline)
        y += CH + GAP

# ================================================================= legend + notes
ly = H - 110
items = [("GPIO", C_GP), ("I\u00b2C", C_I2C), ("ADC", C_ADC), ("PWM", C_PWM), ("power", C_PWR), ("GND", C_GND),
         ("Robo Pico socket", C_PORT), ("terminal pin", C_TERM), ("this car", C_DEV)]
x = 60
for lbl, pair in items:
    w = tw(lbl, F_CHIPS) + 26
    chip(x, ly, w, lbl, pair, F_CHIPS, outline=DEV_O if lbl == "this car" else None)
    x += w + 14
d.text((60, ly + 30), "GP26 is wired to both Grove 5 and Grove 6, so line sensor 2 on Grove 5 must connect only GND, VCC and DO. "
                      "The MAKER port duplicates Grove 2's GP2/GP3 (the ultrasonic) and must stay empty.", font=F_SMALL, fill="#555555")
d.text((60, ly + 54), "Patched in the kernel port by build/setup.sh: I\u00b2C0 moved from GP8/9 to GP4/5, UART0 off GP0/1 "
                      "(console is USB), status LED GP16\u2192GP19, GP27/28 kept digital. Detail: docs/HARDWARE.md \u00a71.", font=F_SMALL, fill="#555555")

# ================================================================= output
img = img.crop((0, 110, W, H))          # the band the title used to occupy
here = os.path.dirname(os.path.abspath(__file__))
root = os.path.abspath(os.path.join(here, "..", "..", ".."))
out1 = os.path.join(here, "robopico_pin_map.png")
img.save(out1)
print("wrote", out1)
out2 = os.path.join(root, "documents", "week6_diagrams", "d2_pin_layout.png")
if os.path.isdir(os.path.dirname(out2)):
    img.save(out2)
    print("wrote", out2)
