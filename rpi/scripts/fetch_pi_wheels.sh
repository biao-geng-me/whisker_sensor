#!/usr/bin/env bash
# Download daq_server.py's Python deps as wheels for an offline install on the
# BlueOS Pi (no internet there). Run on any machine with pip + internet.
#
# BlueROV2 Pi as of 2026-10: Raspbian 11 (bullseye), armv7l, Python 3.9.2,
# no pip, and Debian strips distutils so pip can't run. The wheels are installed by
# unzipping them into a venv (see rpi/README.md). armv7l wheels come from
# piwheels.org (PyPI only has aarch64).
#
# Usage: ./fetch_pi_wheels.sh [outdir] [platform] [pyver]
#   defaults: pi_wheels linux_armv7l 39      -> then tar it up and scp to the Pi
set -euo pipefail
OUT=${1:-pi_wheels}
PLAT=${2:-linux_armv7l}
PYV=${3:-39}
pip download -d "$OUT" --platform "$PLAT" --python-version "$PYV" \
    --implementation cp --abi "cp$PYV" --only-binary=:all: \
    --index-url https://www.piwheels.org/simple --extra-index-url https://pypi.org/simple \
    pyserial pymavlink
ls -la "$OUT"
