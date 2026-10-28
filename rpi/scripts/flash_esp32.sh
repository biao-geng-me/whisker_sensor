#!/usr/bin/env bash
# Flash whisker_daq firmware onto the ESP32 from the BlueOS Pi — i.e. over the
# tether: ssh in, run this. The ESP32's USB-UART is local to the Pi here, so
# esptool's DTR/RTS auto-reset into the bootloader works.
#
# One-time: unzip the esptool wheels (armv7l, from piwheels) into a venv:
#   python3 -m venv --without-pip ~/esptool_venv
#   SITE=$(~/esptool_venv/bin/python -c "import sysconfig; print(sysconfig.get_paths()['purelib'])")
#   for w in ~/esp32_flash/esptool_wheels/*.whl; do ~/esptool_venv/bin/python -m zipfile -e "$w" "$SITE"; done
#
# Usage: rpi/scripts/flash_esp32.sh [merged.bin] [port]
#   default image: ~/esp32_flash/whisker_daq.ino.merged.bin (arduino-cli
#   compile --output-dir produces it; it goes at offset 0x0)
set -euo pipefail
BIN=${1:-$HOME/esp32_flash/whisker_daq.ino.merged.bin}
PORT=${2:-/dev/ttyUSB0}
PY=${ESPTOOL_PY:-$HOME/esptool_venv/bin/python}

[[ -f "$BIN" ]] || { echo "firmware image not found: $BIN" >&2; exit 1; }
if pgrep -f "daq_server.py" >/dev/null; then
    echo "daq_server.py is running and holds $PORT — stop it first: sudo systemctl stop whisker-daq" >&2
    echo "(or Ctrl-C a server started by hand; pkill does not help with the service, it restarts it)" >&2
    exit 1
fi
"$PY" -m esptool --chip esp32 --port "$PORT" --baud 460800 write_flash 0x0 "$BIN"
echo "Flashed. The ESP32 has been reset and is running the new firmware."
systemctl is-enabled --quiet whisker-daq 2>/dev/null && echo "Start the server again: sudo systemctl start whisker-daq"
