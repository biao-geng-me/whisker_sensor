#!/usr/bin/env bash
# One-time setup: create a NetworkManager connection profile that turns
# wlan0 into a wifi access point. Idempotent — re-running updates the
# existing profile.
#
# Usage:
#   sudo ./setup_hotspot.sh [SSID]                       # prompts for password
#   sudo WIFI_PSK='<password>' ./setup_hotspot.sh [SSID]  # non-interactive
#
# Defaults: SSID = whisker-ap. Password must be 8..63 chars (WPA2 rule).
#
# After this runs, use net_mode.sh hotspot|client to toggle modes.

set -euo pipefail

SSID="${1:-whisker-ap}"
CON_NAME="$SSID"
IFACE='wlan0'

if ! command -v nmcli >/dev/null 2>&1; then
    cat >&2 <<'MSG'
Error: nmcli not found.
Hotspot setup uses NetworkManager, which is the default on Raspberry Pi OS
Bookworm (Oct 2023) and later. On older releases, either upgrade or install:
    sudo apt update && sudo apt install -y network-manager
    sudo systemctl enable --now NetworkManager
Note: enabling NM will displace dhcpcd-based networking, which may rename
your existing wifi connection.
MSG
    exit 1
fi

# Verify the wlan interface exists
if ! nmcli -t -f DEVICE device | grep -qx "$IFACE"; then
    echo "Error: interface $IFACE not found. Available devices:" >&2
    nmcli device status >&2
    exit 1
fi

# Get password
if [[ -z "${WIFI_PSK:-}" ]]; then
    read -rsp "Wifi password (8..63 chars): " WIFI_PSK
    echo
fi
PSK_LEN=${#WIFI_PSK}
if (( PSK_LEN < 8 || PSK_LEN > 63 )); then
    echo "Error: password must be 8..63 chars (got $PSK_LEN)" >&2
    exit 1
fi

# Add the connection if absent, then modify settings (idempotent either way)
if nmcli -t -f NAME connection show | grep -qx "$CON_NAME"; then
    echo "Updating existing connection: $CON_NAME"
else
    echo "Creating connection: $CON_NAME"
    sudo nmcli connection add \
        type wifi \
        ifname "$IFACE" \
        con-name "$CON_NAME" \
        ssid "$SSID" \
        autoconnect no
fi

sudo nmcli connection modify "$CON_NAME" \
    802-11-wireless.mode ap \
    802-11-wireless.band bg \
    ipv4.method shared \
    ipv6.method ignore \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$WIFI_PSK"

echo
echo "Done. Hotspot profile '$CON_NAME' is configured."
echo "Default AP gateway IP (clients connect to this): 10.42.0.1"
echo
echo "Activate it now with:"
echo "    sudo $(dirname "$0")/net_mode.sh hotspot"
