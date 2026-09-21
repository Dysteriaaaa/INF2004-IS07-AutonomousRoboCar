#!/usr/bin/env bash
# Build the car firmware and produce a flashable .uf2 in build/out/.
#
#   ./build/build.sh            single core (SMP=0)  <- start here
#   ./build/build.sh smp        dual core   (SMP=1)
#   ./build/build.sh clean
#
# What it fixes for you so the port's own makefile just works on a laptop
# that has the VS Code Pico extension:
#
#   * finds arm-none-eabi-gcc and the Pico SDK under ~/.pico-sdk if they are
#     not already on PATH (set PICO_SDK_PATH / add the toolchain to PATH to
#     override);
#   * always builds CONSOLE=usb_cdc: GP0/GP1 are the left encoder on this car,
#     so the UART console does not exist (docs/HARDWARE.md section 1.5);
#   * converts the .elf to .uf2 with picotool instead of the port's elf2uf2,
#     which would need a host g++ that Windows machines rarely have.
#
# Needs GNU make. Windows: `winget install ezwinports.make`, then reopen Git
# Bash. (`make --version` should print 4.x.)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="$ROOT/external/mtk3smp-rp2040"
OUT="$ROOT/build/out"
PICO_HOME="${PICO_HOME:-$HOME/.pico-sdk}"

mode="${1:-}"
SMP=0
case "$mode" in
    ""|smp0) ;;
    smp|smp1|SMP=1) SMP=1 ;;
    clean) ;;
    *) echo "usage: $0 [smp|clean]" >&2; exit 2 ;;
esac

[ -f "$PORT/build_make/makefile" ] || { echo "run ./build/setup.sh first" >&2; exit 1; }
grep -q RC_ROOT "$PORT/build_make/mtkernel_3/app_program/subdir.mk" 2>/dev/null \
    || { echo "application hook missing: run ./build/setup.sh" >&2; exit 1; }

# newest <dir>/<version> under $1
newest() { ls -d "$1"/*/ 2>/dev/null | sort -V | tail -1 | sed 's#/$##'; }

# ---- toolchain -------------------------------------------------------------
if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    tc="$(newest "$PICO_HOME/toolchain")"
    [ -n "$tc" ] || { echo "arm-none-eabi-gcc not found on PATH or under $PICO_HOME/toolchain" >&2; exit 1; }
    export PATH="$tc/bin:$PATH"
fi
command -v make >/dev/null 2>&1 || { echo "GNU make not found. Windows: winget install ezwinports.make" >&2; exit 1; }

# ---- Pico SDK (TinyUSB for the USB console) ----------------------------------
if [ -z "${PICO_SDK_PATH:-}" ]; then
    PICO_SDK_PATH="$(newest "$PICO_HOME/sdk")"
fi
[ -f "$PICO_SDK_PATH/lib/tinyusb/src/tusb.c" ] \
    || { echo "Pico SDK with TinyUSB not found at '$PICO_SDK_PATH' (set PICO_SDK_PATH)" >&2; exit 1; }

# ---- picotool for elf -> uf2 -------------------------------------------------
PICOTOOL="${PICOTOOL:-$(command -v picotool 2>/dev/null || true)}"
if [ -z "$PICOTOOL" ]; then
    pt="$(newest "$PICO_HOME/picotool")"
    [ -x "$pt/picotool/picotool.exe" ] && PICOTOOL="$pt/picotool/picotool.exe"
    [ -x "$pt/picotool/picotool" ]     && PICOTOOL="$pt/picotool/picotool"
fi

cd "$PORT/build_make"

if [ "$mode" = clean ]; then
    make CONSOLE=usb_cdc PICO_SDK_PATH="$PICO_SDK_PATH" SMP=0 E2U= clean
    make CONSOLE=usb_cdc PICO_SDK_PATH="$PICO_SDK_PATH" SMP=1 E2U= clean
    rm -rf mtkernel_3/robocar "$OUT"
    exit 0
fi

echo "== toolchain : $(arm-none-eabi-gcc --version | head -1)"
echo "== pico sdk  : $PICO_SDK_PATH"
echo "== profile   : SMP=$SMP CONSOLE=usb_cdc"

# E2U= empties the port's elf2uf2 variable so the .elf rule neither builds
# nor runs that host tool; picotool does the conversion below.
make -j"$(nproc 2>/dev/null || echo 4)" CONSOLE=usb_cdc PICO_SDK_PATH="$PICO_SDK_PATH" SMP="$SMP" E2U=

elf="mtk3pico_smp${SMP}_usb_cdc.elf"
[ -f "$elf" ] || { echo "expected $elf was not produced" >&2; exit 1; }

mkdir -p "$OUT"
cp "$elf" "$OUT/"
if [ -n "$PICOTOOL" ]; then
    "$PICOTOOL" uf2 convert "$elf" "$OUT/${elf%.elf}.uf2" >/dev/null
    echo
    echo "== flash this : build/out/${elf%.elf}.uf2   (BOOTSEL + copy to RPI-RP2, or ./build/flash.sh)"
else
    echo
    echo "== built $OUT/$elf but picotool was not found, so no .uf2 was made."
    echo "   Install picotool (the VS Code Pico extension provides it) or set PICOTOOL=/path/to/picotool."
fi
