#!/usr/bin/env bash
# Build the car firmware and produce a flashable .uf2 in build/out/.
#
#   ./build/build.sh                  single core (SMP=0)  <- start here
#   ./build/build.sh smp              dual core   (SMP=1)
#   ./build/build.sh bench=motion     one buddy's bench test instead of the
#                                     mission; names: motion line follow
#                                     barcode imu ultra scan telemetry
#                                     (see app/app_bench.h). Combines: smp bench=imu
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
# Run it from Git Bash on Windows (or any POSIX shell on Mac/Linux), from any
# directory - the script finds the repo from its own location. Do NOT run it
# from WSL/Ubuntu: the Pico tools live on the Windows side.
#
# Needs GNU make. On Windows the script fetches a portable copy into
# build/tools/ the first time if none is on PATH.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="$ROOT/external/mtk3smp-rp2040"
OUT="$ROOT/build/out"
PICO_HOME="${PICO_HOME:-$HOME/.pico-sdk}"

# The classic trap: on Windows, `bash` typed into PowerShell/CMD opens WSL
# (Ubuntu), not Git Bash. The toolchain, SDK and picotool the VS Code Pico
# extension installed are on the Windows side, so a WSL shell cannot see them.
if grep -qi microsoft /proc/version 2>/dev/null && [ ! -d "$PICO_HOME" ]; then
    cat >&2 <<'EOF'
This is WSL (Ubuntu), but the Pico tools are installed on Windows.
Open "Git Bash" (installed with Git for Windows) - or the VS Code terminal
with the Git Bash profile - and run this script there.
EOF
    exit 1
fi

SMP=0
BENCH=none
mode=build
for arg in "$@"; do
    case "$arg" in
        smp|smp1|SMP=1) SMP=1 ;;
        smp0) SMP=0 ;;
        bench=*) BENCH="${arg#bench=}" ;;
        clean) mode=clean ;;
        *) echo "usage: $0 [smp] [bench=<name>] [clean]" >&2; exit 2 ;;
    esac
done
case "$BENCH" in
    none|motion|line|follow|barcode|imu|ultra|scan|telemetry) ;;
    *) echo "unknown bench '$BENCH'; see app/app_bench.h" >&2; exit 2 ;;
esac
BENCH_DEF="RC_BENCH_$(echo "$BENCH" | tr '[:lower:]' '[:upper:]')"

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
# ---- GNU make ----------------------------------------------------------------
# Windows has no make; fetch a portable ezwinports build into build/tools once.
if ! command -v make >/dev/null 2>&1; then
    tools="$ROOT/build/tools/make/bin"
    if [ ! -x "$tools/make.exe" ]; then
        case "$(uname -s)" in
            MINGW*|MSYS*) ;;
            *) echo "GNU make not found: install it (apt/brew install make)" >&2; exit 1 ;;
        esac
        echo "== GNU make not found; downloading a portable copy into build/tools/ (one-time, ~400 kB)"
        mkdir -p "$ROOT/build/tools"
        zip="$ROOT/build/tools/make.zip"
        curl -fsSL -o "$zip" "https://sourceforge.net/projects/ezwinports/files/make-4.4.1-without-guile-w32-bin.zip/download" \
            || { echo "download failed. Install make instead: winget install ezwinports.make, then reopen Git Bash" >&2; exit 1; }
        if command -v unzip >/dev/null 2>&1; then
            unzip -q "$zip" -d "$ROOT/build/tools/make"
        else
            python -c "import zipfile,sys; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" "$zip" "$ROOT/build/tools/make"
        fi
        rm -f "$zip"
    fi
    export PATH="$tools:$PATH"
    command -v make >/dev/null 2>&1 || { echo "still no make after bootstrap; winget install ezwinports.make" >&2; exit 1; }
fi

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
echo "== profile   : SMP=$SMP CONSOLE=usb_cdc bench=$BENCH"

# Barr-C style check (NFR9): flag any source line over 80 columns. A warning,
# not an error, so nobody is blocked - but fix them before you commit.
long_lines="$(LC_ALL=C.UTF-8 awk 'length($0) > 80 { print FILENAME ":" FNR }'     "$ROOT"/core/*.[ch] "$ROOT"/drivers/*.[ch]     "$ROOT"/subsystems/*.[ch] "$ROOT"/app/*.[ch])"
if [ -n "$long_lines" ]; then
    echo "== WARNING: lines over 80 columns (Barr-C):"
    echo "$long_lines" | sed 's/^/     /'
fi

# The bench selection is a -D on our own objects only. The port's stale-object
# guard keys on its profile, not on RC_BENCH, so drop app/ objects ourselves
# (two small files) to guarantee the right bench is linked.
rm -f mtkernel_3/robocar/app/*.o mtkernel_3/robocar/app/*.d

# E2U= empties the port's elf2uf2 variable so the .elf rule neither builds
# nor runs that host tool; picotool does the conversion below.
make -j"$(nproc 2>/dev/null || echo 4)" CONSOLE=usb_cdc PICO_SDK_PATH="$PICO_SDK_PATH" SMP="$SMP" E2U= RC_CFLAGS="-DRC_BENCH=$BENCH_DEF"

elf="mtk3pico_smp${SMP}_usb_cdc.elf"
[ -f "$elf" ] || { echo "expected $elf was not produced" >&2; exit 1; }

name="mtk3pico_smp${SMP}_usb_cdc"
hint=""
[ "$SMP" = 1 ]        && hint="$hint smp"
[ "$BENCH" != none ]  && { name="${name}_bench-${BENCH}"; hint="$hint bench=$BENCH"; }

mkdir -p "$OUT"
cp "$elf" "$OUT/$name.elf"
if [ -n "$PICOTOOL" ]; then
    "$PICOTOOL" uf2 convert "$elf" "$OUT/$name.uf2" >/dev/null
    echo
    echo "== flash this : build/out/$name.uf2   (BOOTSEL + copy to RPI-RP2, or ./build/flash.sh$hint)"
else
    echo
    echo "== built $OUT/$name.elf but picotool was not found, so no .uf2 was made."
    echo "   Install picotool (the VS Code Pico extension provides it) or set PICOTOOL=/path/to/picotool."
fi
