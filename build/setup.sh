#!/usr/bin/env bash
# One-time setup after cloning (and again after `git submodule update`).
#
#   1. Fetch the mtk3smp-rp2040 kernel port into external/ (pinned commit).
#   2. Apply this car's board patches to it (build/patch_port.py).
#   3. Install build/robocar.mk as the port's application makefile so the
#      port's build compiles core/ drivers/ subsystems/ app/ from this repo.
#
# Run from Git Bash on Windows, or any POSIX shell:   ./build/setup.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="$ROOT/external/mtk3smp-rp2040"

echo "== 1/3 kernel port submodule"
git -C "$ROOT" submodule update --init --recursive
echo "   at $(git -C "$PORT" log --oneline -1)"

echo "== 2/3 board patches"
python "$ROOT/build/patch_port.py"

echo "== 3/3 application makefile hook"
cp "$ROOT/build/robocar.mk" "$PORT/build_make/mtkernel_3/app_program/subdir.mk"
echo "   installed build_make/mtkernel_3/app_program/subdir.mk"

echo
echo "done. Build with:   ./build/build.sh"
