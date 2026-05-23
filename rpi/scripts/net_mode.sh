#!/usr/bin/env bash
# Toggle the Pi between wifi modes:
#   hotspot  — bring up the AP profile (Pi broadcasts its own SSID, no internet)
#   client   — bring AP down; NetworkManager auto-reconnects to known wifi
#   status   — print current wlan0 state and any active wifi connection
#
# Usage:
#   sudo ./net_mode.sh hotspot [SSID]
#   sudo ./net_mode.sh client  [SSID]
#   ./net_mode.sh status
#
# SSID defaults to whisker-ap (must match what setup_hotspot.sh created).

set -euo pipefail

MODE="${1:-status}"
CON_NAME="${2:-whisker-ap}"
IFACE='wlan0'

if ! command -v nmcli >/dev/null 2>&1; then
    echo "Error: nmcli not found. See setup_hotspot.sh for NetworkManager install notes." >&2
    exit 1
fi

case "$MODE" in
    hotspot)
        if ! nmcli -t -f NAME connection show | grep -qx "$CON_NAME"; then
            echo "Error: connection '$CON_NAME' not found. Run setup_hotspot.sh first." >&2
            exit 1
        fi
        echo "Bringing up hotspot '$CON_NAME' on $IFACE..."
        sudo nmcli connection up "$CON_NAME"
        echo
        echo "Hotspot active. Clients connect to SSID '$CON_NAME' and reach the Pi at:"
        nmcli -g IP4.ADDRESS device show "$IFACE" | head -n1 || true
        echo "(MATLAB: wavi(transport='tcp', tcp_host='10.42.0.1', tcp_port=5555, ...))"
        ;;

    client)
        if nmcli -t -f NAME,STATE connection show --active | grep -qx "$CON_NAME:activated"; then
            echo "Bringing down hotspot '$CON_NAME'..."
            sudo nmcli connection down "$CON_NAME"
        else
            echo "Hotspot '$CON_NAME' was not active."
        fi
        echo "NetworkManager will reconnect $IFACE to a known wifi network."
        echo "Wait a moment, then run: $0 status"
        ;;

    status)
        echo "=== $IFACE state ==="
        nmcli device status | awk -v i="$IFACE" 'NR==1 || $1==i'
        echo
        echo "=== active connections ==="
        nmcli -f NAME,DEVICE,TYPE,STATE connection show --active
        echo
        echo "=== $IFACE IPv4 ==="
        nmcli -g IP4.ADDRESS device show "$IFACE" 2>/dev/null || echo "(none)"
        ;;

    *)
        echo "usage: $0 {hotspot|client|status} [SSID]" >&2
        exit 2
        ;;
esac
