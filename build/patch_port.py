"""
Apply this car's board patches to the mtk3smp-rp2040 submodule.

Run by build/setup.sh. Safe to run repeatedly: each patch is skipped when the
file already carries it. Every change below exists because the stock port
claims a pin at boot that this car uses for something else; the full story is
in docs/HARDWARE.md section 1.

    1. BOARD_LED_PIN 16 -> 19            GP16 is line sensor 1
    2. Boot pin table (hw_setting.c)      GP0/GP1 no longer muxed to UART0 (left encoder)
                                          I2C0 pins GP8/GP9 -> GP4/GP5   (motor pins vs Grove 3)
                                          GP27/GP28 no longer parked as ADC (barcode DO, right encoder B)
    3. I2C driver pin table               unit 0 GP8/GP9 -> GP4/GP5, same reason
    4. TM_CONSOLE_UART 1 -> 0             no UART0 mirror; the console is USB CDC only
    5. DEVCNF_USE_SER 1 -> 0              the "sera" UART device is not registered at all
"""

import io
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PORT = os.path.join(ROOT, "external", "mtk3smp-rp2040")

if not os.path.isfile(os.path.join(PORT, "build_make", "makefile")):
    sys.exit("submodule not checked out: run  git submodule update --init  first")


def patch(rel, old, new, label):
    path = os.path.join(PORT, rel)
    s = io.open(path, encoding="utf-8", newline="").read()
    if "\r\n" in s:                      # keep the file's own line endings
        old = old.replace("\n", "\r\n")
        new = new.replace("\n", "\r\n")
    if new in s and old not in s:
        print(f"  = {label} (already applied)")
        return
    if s.count(old) != 1:
        sys.exit(f"  ! {label}: expected exactly one match in {rel}, found {s.count(old)}. "
                 f"The port may have changed; update build/patch_port.py.")
    io.open(path, "w", encoding="utf-8", newline="").write(s.replace(old, new))
    print(f"  + {label}")


# 1. status LED off GP16 -------------------------------------------------------
patch("include/sys/sysdepend/pico_rp2040/sysdef.h",
      "#define\tBOARD_LED_PIN\t\t16",
      "#define\tBOARD_LED_PIN\t\t19",
      "BOARD_LED_PIN 16 -> 19")

# 2. boot-time pin function table ----------------------------------------------
patch("kernel/sysdepend/pico_rp2040/hw_setting.c",
      "\t/* P0,P1 : UART0 */\n"
      "\t{GPIO_CTRL(0),\tGPIO_CTRL_FUNCSEL_UART},\t/* P0 UART0-TX */\n"
      "\t{GPIO_CTRL(1),\tGPIO_CTRL_FUNCSEL_UART},\t/* P1 UART0-RX */\n",
      "\t/* P0,P1 : left wheel encoder A/B on this car, not UART0.\n"
      "\t   drv_encoder sets them up; the console is USB CDC only. */\n",
      "hw_setting.c: GP0/GP1 not muxed to UART0")

patch("kernel/sysdepend/pico_rp2040/hw_setting.c",
      "\t/* P27 : ADC1 */\n"
      "\t{GPIO_CTRL(27),\tGPIO_CTRL_FUNCSEL_NULL},\n"
      "\t{GPIO(27), GPIO_DRIVE_4MA | GPIO_SHEMITT},\t/* Disable input & pull-up & pull-down */\n"
      "\n"
      "\t/* P28 : ADC2 */\n"
      "\t{GPIO_CTRL(28),\tGPIO_CTRL_FUNCSEL_NULL},\n"
      "\t{GPIO(28), GPIO_DRIVE_4MA | GPIO_SHEMITT},\t/* Disable input & pull-up & pull-down */\n",
      "\t/* P27, P28 : digital inputs on this car (barcode DO, right encoder B),\n"
      "\t   set up by drv_ir / drv_encoder. Only P26 is an ADC input. */\n",
      "hw_setting.c: GP27/GP28 left as digital inputs")

patch("kernel/sysdepend/pico_rp2040/hw_setting.c",
      "\t/* P8 : I2C0_SDA */\n"
      "\t{GPIO_CTRL(8),\tGPIO_CTRL_FUNCSEL_I2C},\n"
      "\t{GPIO(8), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT},\t/* Pull-up */\n"
      "\n"
      "\t/* P9 : I2C0_SCL */\n"
      "\t{GPIO_CTRL(9),\tGPIO_CTRL_FUNCSEL_I2C},\n"
      "\t{GPIO(9), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT},\t/* Pull-up */\n",
      "\t/* P4 : I2C0_SDA (Robo Pico Grove 3; GP8/GP9 are the left motor) */\n"
      "\t{GPIO_CTRL(4),\tGPIO_CTRL_FUNCSEL_I2C},\n"
      "\t{GPIO(4), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT},\t/* Pull-up */\n"
      "\n"
      "\t/* P5 : I2C0_SCL */\n"
      "\t{GPIO_CTRL(5),\tGPIO_CTRL_FUNCSEL_I2C},\n"
      "\t{GPIO(5), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT},\t/* Pull-up */\n",
      "hw_setting.c: I2C0 on GP4/GP5")

# 3. I2C driver's own pin setup, unit 0 ----------------------------------------
patch("device/i2c/sysdepend/rp2040/i2c_rp2040.c",
      "\t\tout_w(GPIO_CTRL(8), GPIO_CTRL_FUNCSEL_I2C);\n"
      "\t\tout_w(GPIO(8), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);\n"
      "\t\tout_w(GPIO_CTRL(9), GPIO_CTRL_FUNCSEL_I2C);\n"
      "\t\tout_w(GPIO(9), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);\n",
      "\t\t/* Robo Pico Grove 3: GP4/GP5. GP8/GP9 are the left motor. */\n"
      "\t\tout_w(GPIO_CTRL(4), GPIO_CTRL_FUNCSEL_I2C);\n"
      "\t\tout_w(GPIO(4), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);\n"
      "\t\tout_w(GPIO_CTRL(5), GPIO_CTRL_FUNCSEL_I2C);\n"
      "\t\tout_w(GPIO(5), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);\n",
      "i2c_rp2040.c: unit 0 on GP4/GP5")

# 4. no UART0 console mirror ---------------------------------------------------
patch("config/config_tm.h",
      "#define\tTM_CONSOLE_UART\t\t(1)\t/* UART0 early/panic mirror */",
      "#define\tTM_CONSOLE_UART\t\t(0)\t/* no UART0 mirror: GP0/GP1 are the left encoder */",
      "TM_CONSOLE_UART 1 -> 0")

# 5. no UART device driver -----------------------------------------------------
patch("config/config_device.h",
      "#define DEVCNF_USE_SER\t\t1\t\t// Serial communication device ",
      "#define DEVCNF_USE_SER\t\t0\t\t// Serial communication device (off: GP0/GP1 are the left encoder)",
      "DEVCNF_USE_SER 1 -> 0")

print("port patched")
