#!/usr/bin/env bash
# Rsync the Pi-relevant subtrees from this repo to the Raspberry Pi.
#
# Designed for WSL on Windows (rsync is preinstalled). Run from anywhere;
# paths are resolved relative to this script's location.
#
# Usage:
#   ./sync.sh pi@<pi-ip> [--dest ~/whisker_sensor] [--port 22] [--dry-run]
#
# Examples:
#   ./sync.sh pi@192.168.1.42
#   ./sync.sh pi@raspberrypi.local --dest ~/work/whisker --port 2222
#   ./sync.sh pi@192.168.1.42 --dry-run

set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "usage: $0 user@host [--dest PATH] [--port N] [--dry-run]" >&2
    exit 2
fi

PI_HOST="$1"; shift
DEST='~/whisker_sensor'
PORT=22
DRY_RUN=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dest)    DEST="$2"; shift 2 ;;
        --port)    PORT="$2"; shift 2 ;;
        --dry-run) DRY_RUN=(-n); shift ;;
        *)         echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

for d in rpi hx711_array; do
    if [[ ! -d "$REPO_ROOT/$d" ]]; then
        echo "source not found: $REPO_ROOT/$d" >&2
        exit 1
    fi
done

# trailing slashes matter: copy each folder as a subdirectory under $DEST
rsync -avz --delete "${DRY_RUN[@]}" \
    -e "ssh -p $PORT" \
    --exclude '.git' \
    --exclude '__pycache__' \
    --exclude '*.dat' \
    --exclude '*.ino' \
    "$REPO_ROOT/rpi" "$REPO_ROOT/hx711_array" \
    "$PI_HOST:$DEST/"

echo
echo "Done. On the Pi:"
echo "  cd $DEST && python3 -u rpi/daq_bridge.py --port /dev/ttyACM0 --bind 0.0.0.0:5555"
