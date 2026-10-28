#!/usr/bin/env bash
# Rsync the Pi-relevant subtrees from this repo to the Raspberry Pi.
#
# Designed for WSL on Windows (rsync is preinstalled). Run from anywhere;
# paths are resolved relative to this script's location.
#
# Usage:
#   ./sync.sh pi@<pi-ip> [--dest ~/whisker_sensor] [--port 22] [--dry-run] [--restart]
#
#   --restart: afterwards `sudo systemctl restart whisker-daq` over ssh (asks for
#   the Pi's sudo password), so the service runs the new code
#
# Examples:
#   ./sync.sh pi@192.168.1.42
#   ./sync.sh pi@raspberrypi.local --dest ~/work/whisker --port 2222
#   ./sync.sh pi@192.168.1.42 --dry-run
#   ./sync.sh pi@192.168.2.2 --restart

set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "usage: $0 user@host [--dest PATH] [--port N] [--dry-run] [--restart]" >&2
    exit 2
fi

PI_HOST="$1"; shift
# shellcheck disable=SC2088  # ~ expands on the Pi, not here
DEST='~/whisker_sensor'
PORT=22
DRY_RUN=()
RESTART=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dest)    DEST="$2"; shift 2 ;;
        --port)    PORT="$2"; shift 2 ;;
        --dry-run) DRY_RUN=(-n); shift ;;
        --restart) RESTART=1; shift ;;
        *)         echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

for d in rpi hx711_array whisker_link; do
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
    "$REPO_ROOT/rpi" "$REPO_ROOT/hx711_array" "$REPO_ROOT/whisker_link" \
    "$PI_HOST:$DEST/"

echo
if [[ $RESTART -eq 1 && ${#DRY_RUN[@]} -eq 0 ]]; then
    echo "Restarting whisker-daq on $PI_HOST ..."
    ssh -t -p "$PORT" "$PI_HOST" "sudo systemctl restart whisker-daq && systemctl --no-pager --lines=5 status whisker-daq"
else
    echo "Done. The whisker-daq service (BlueROV) still runs the old code; restart it:"
    echo "  ssh -t $PI_HOST sudo systemctl restart whisker-daq     (or run this with --restart)"
    echo "  first time: ssh -t $PI_HOST sudo bash $DEST/rpi/scripts/install_service.sh"
    echo "  hx711 bridge: cd $DEST && python3 -u rpi/daq_bridge.py --port /dev/ttyACM0 --bind 0.0.0.0:5555"
fi
