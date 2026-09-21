#!/usr/bin/env bash
# Flash the most recently built image with picotool.
#
#   ./build/flash.sh          single-core image
#   ./build/flash.sh smp      dual-core image
#
# Put the Pico in BOOTSEL mode first (hold BOOTSEL, plug in USB, release).
# If picotool is not found, just drag the .uf2 from build/out/ onto the
# RPI-RP2 drive instead - same result.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PICO_HOME="${PICO_HOME:-$HOME/.pico-sdk}"
SMP=0; [ "${1:-}" = smp ] && SMP=1
uf2="$ROOT/build/out/mtk3pico_smp${SMP}_usb_cdc.uf2"
[ -f "$uf2" ] || { echo "no image at $uf2 - run ./build/build.sh${SMP:+ smp} first" >&2; exit 1; }

PICOTOOL="${PICOTOOL:-$(command -v picotool 2>/dev/null || true)}"
if [ -z "$PICOTOOL" ]; then
    pt="$(ls -d "$PICO_HOME"/picotool/*/ 2>/dev/null | sort -V | tail -1 | sed 's#/$##')"
    [ -x "$pt/picotool/picotool.exe" ] && PICOTOOL="$pt/picotool/picotool.exe"
    [ -x "$pt/picotool/picotool" ]     && PICOTOOL="$pt/picotool/picotool"
fi
[ -n "$PICOTOOL" ] || { echo "picotool not found; copy $uf2 onto the RPI-RP2 drive by hand" >&2; exit 1; }

"$PICOTOOL" load -f "$uf2"
"$PICOTOOL" reboot
echo "flashed $(basename "$uf2"); the USB serial console appears in a second or two"
