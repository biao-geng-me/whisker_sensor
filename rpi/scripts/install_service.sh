#!/usr/bin/env bash
# Install (or remove) the whisker-daq systemd service on the BlueOS Pi host.
#
# Run on the Pi, from the synced code:
#   sudo ~/whisker_sensor/rpi/scripts/install_service.sh              # install / update, enable, (re)start
#   sudo ~/whisker_sensor/rpi/scripts/install_service.sh --uninstall  # stop, disable, remove the unit
#   options: --user NAME (default: the user who ran sudo, else pi), --no-start
#
# Safe to run again: it rewrites the unit, keeps an existing env file
# (~/whisker_sensor.env, the server arguments) and restarts the service.

set -euo pipefail

NAME=whisker-daq
UNIT_DST=/etc/systemd/system/$NAME.service
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT_SRC=$SCRIPT_DIR/$NAME.service
DEFAULT_ARGS="--serial /dev/ttyUSB0 --mode wide --selects 0123457"

RUN_USER=${SUDO_USER:-pi}
[[ $RUN_USER == root ]] && RUN_USER=pi
UNINSTALL=0
START=1
while [[ $# -gt 0 ]]; do
    case "$1" in
        --uninstall) UNINSTALL=1; shift ;;
        --user)      RUN_USER="$2"; shift 2 ;;
        --no-start)  START=0; shift ;;
        -h|--help)   sed -n '2,11p' "$0"; exit 0 ;;
        *)           echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

if [[ $EUID -ne 0 ]]; then
    echo "run with sudo: sudo $0 $*" >&2
    exit 1
fi
command -v systemctl >/dev/null || { echo "no systemctl on this host" >&2; exit 1; }

warn() { echo "WARNING: $*" >&2; }

if [[ $UNINSTALL -eq 1 ]]; then
    systemctl disable --now "$NAME" 2>/dev/null || true
    rm -f "$UNIT_DST"
    systemctl daemon-reload
    systemctl reset-failed "$NAME" 2>/dev/null || true
    echo "removed $NAME (the env file and recordings are kept)"
    exit 0
fi

HOME_DIR=$(getent passwd "$RUN_USER" | cut -d: -f6)
[[ -n $HOME_DIR ]] || { echo "no such user: $RUN_USER" >&2; exit 1; }
ENV_FILE=$HOME_DIR/whisker_sensor.env
PY=$HOME_DIR/whisker_venv/bin/python
CODE=$HOME_DIR/whisker_sensor

[[ -f $UNIT_SRC ]] || { echo "unit not found: $UNIT_SRC" >&2; exit 1; }
[[ -x $PY ]] || warn "$PY missing: set up the venv first (rpi/README.md, one-time setup step 4)"
[[ -f $CODE/rpi/daq_server.py ]] || warn "$CODE/rpi/daq_server.py missing: sync the code first (sync.sh / sync.ps1)"

# the server arguments: create once, never overwrite (the user edits it)
if [[ ! -f $ENV_FILE ]]; then
    cat > "$ENV_FILE" <<EOF
# Arguments for rpi/daq_server.py run by the $NAME service.
# After editing: sudo systemctl restart $NAME
# Don't add --record here: recording is started from wavi_py (Rec) or a command.
DAQ_ARGS=$DEFAULT_ARGS
EOF
    chown "$RUN_USER": "$ENV_FILE"
    echo "wrote $ENV_FILE"
else
    echo "keeping $ENV_FILE"
fi
# systemd syntax, not shell (the value is unquoted): take the last DAQ_ARGS= line
DAQ_ARGS=$(sed -n 's/^[[:space:]]*DAQ_ARGS=//p' "$ENV_FILE" | tail -n 1 | sed 's/^"\(.*\)"$/\1/')
DAQ_ARGS=${DAQ_ARGS:-$DEFAULT_ARGS}
echo "server arguments: $DAQ_ARGS"

# the serial port in DAQ_ARGS (default /dev/ttyUSB0)
SERIAL=/dev/ttyUSB0
read -r -a _args <<< "$DAQ_ARGS"
for ((i = 0; i < ${#_args[@]}; i++)); do
    case "${_args[i]}" in
        --serial)   SERIAL=${_args[i+1]:-$SERIAL} ;;
        --serial=*) SERIAL=${_args[i]#--serial=} ;;
    esac
done

if [[ $SERIAL != none ]]; then
    # BlueOS's serial bridge (bridget) holds the port open; it must stay deleted
    if command -v curl >/dev/null; then
        bridges=$(curl -s --max-time 3 http://127.0.0.1/bridget/v1.0/bridges || true)
        if [[ $bridges == *"$SERIAL"* ]]; then
            warn "a BlueOS bridget serial bridge uses $SERIAL; it keeps the port from daq_server."
            warn "delete it (rpi/README.md, one-time setup step 2); bridges now: $bridges"
        fi
    fi
    if [[ -e $SERIAL ]]; then
        grp=$(stat -L -c %G "$SERIAL")
        [[ $grp == dialout ]] || warn "$SERIAL belongs to group $grp, not dialout: the service may not open it"
    else
        echo "note: $SERIAL not present now (fine: the server waits for the ESP32)"
    fi
fi

# a daq_server started by hand holds TCP 5600 and the serial port
if pgrep -f 'rpi/daq_server.py' >/dev/null; then
    own=$(systemctl show -p MainPID --value "$NAME" 2>/dev/null || echo 0)
    others=$(pgrep -f 'rpi/daq_server.py' | grep -vx "$own" || true)
    [[ -z $others ]] || warn "daq_server already running outside the service (pid $others): stop it"
fi

tmp=$(mktemp)
sed -e "s|/home/pi|$HOME_DIR|g" -e "s|^User=pi$|User=$RUN_USER|" "$UNIT_SRC" > "$tmp"
install -m 644 "$tmp" "$UNIT_DST"
rm -f "$tmp"
echo "installed $UNIT_DST (user $RUN_USER)"
systemctl daemon-reload
systemctl enable "$NAME" >/dev/null
echo "enabled: starts at boot"
if [[ $START -eq 1 ]]; then
    systemctl restart "$NAME"
    sleep 2
    systemctl --no-pager --lines=15 status "$NAME" || true
fi
echo
echo "follow the log: journalctl -u $NAME -f"
