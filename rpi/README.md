# Raspberry Pi DAQ

The Pi sits between the Arduino (USB serial) and one or more PC clients (TCP). Conceptually it's a `tee` for the Arduino's byte stream: data flows out to whoever's listening, and the Pi can grow extra capabilities (parsing, recording, multi-client) on a separate branch later without touching the data path.

## Phase 1 — transparent byte pump ✅ done

[`daq_bridge.py`](daq_bridge.py) opens the Arduino serial port once and forwards bytes verbatim to a single TCP client. No parsing, no recording on the Pi. The MATLAB client ([wavi/wavi.m](../wavi/wavi.m) with `transport='tcp'`) handles framing and records `.dat` files locally exactly as it does over a direct serial link.

### Setup (once per fresh Pi)

```bash
sudo apt-get install python3-pip
pip3 install -r rpi/requirements.txt
# add yourself to dialout so /dev/ttyACM0 is readable without sudo
sudo usermod -aG dialout "$USER"
```

Log out and back in for the group change to take effect.

### Run

From the repo root on the Pi:

```bash
python3 -u rpi/daq_bridge.py --port /dev/ttyACM0 --bind 0.0.0.0:5555
```

The Arduino's `WAIT_LED_PIN` (pin 52) lights as soon as the bridge opens the serial port; it turns off when the MATLAB client connects and sends `N=<nsensor>` over TCP.

### Connect from MATLAB

```matlab
wavi(transport='tcp', tcp_host='<pi-ip>', tcp_port=5555, nsensor=9, Fs=80)
```

Then hit Connect in the UI.

### Find the Pi's IP

```bash
hostname -I        # on the Pi
ip -4 addr show
```

`raspberrypi.local` (mDNS) often works from the PC too. In Phase 2 (hotspot) the Pi's AP-side IP is fixed by the OS config — see below.

## Dev workflow (PC → Pi)

The Pi 4B is too weak for VS Code Remote-SSH agent work, so we edit on the PC and push from the host. Two wrappers, pick whichever fits your shell:

**WSL / Git Bash (rsync) — preferred when available:**

```bash
./rpi/scripts/sync.sh pi@<pi-ip>
# first-time only: chmod +x rpi/scripts/sync.sh
```

**PowerShell (scp, no extra install):**

```powershell
.\rpi\scripts\sync.ps1 -PiHost pi@<pi-ip>
```

Both push `rpi/` and the Python parts of `hx711_array/` (excluding `*.ino`) as subdirectories of `~/whisker_sensor/` on the Pi (override with `--dest` / `-Dest`). The `scp` path re-copies everything every time; rsync only sends diffs. Either is fast enough for these small folders.

## Phase 2 — wifi hotspot mode

Goal: untether the field setup from any router. The Pi becomes its own wifi AP; the PC/laptop joins the Pi's SSID and reaches the bridge directly. No code change to `daq_bridge.py` — it already binds `0.0.0.0`, so it accepts clients from any interface. Phase 2 is OS-level config only, driven by two small scripts.

### The Pi 4B wifi constraint

The built-in BCM4345 chip can do AP mode, but **AP + STA (station/client) on the same chip is unreliable**. Two practical layouts:

- **Mode switch** (recommended first): toggle the Pi between "client" (joins an existing wifi for internet/dev) and "hotspot" (broadcasts its own SSID, no internet). `net_mode.sh` does this in one command. Keep using ethernet for code sync during dev.
- **USB wifi dongle** (more flexible): `wlan0` (built-in) is permanently the AP; the dongle becomes a second interface (`wlan1`) running as a station for internet. ~$15. Worth it once mode-switching gets annoying.

### Setup (one-time, requires NetworkManager)

Raspberry Pi OS Bookworm (Oct 2023) and later use NetworkManager by default — nothing to install. On older releases, the [setup script](scripts/setup_hotspot.sh) prints install instructions when it can't find `nmcli`.

```bash
# Default SSID is whisker-ap; pass another as $1 if you want a different name.
sudo ./rpi/scripts/setup_hotspot.sh             # prompts for password
# or non-interactive:
sudo WIFI_PSK='<password>' ./rpi/scripts/setup_hotspot.sh
```

This creates a NetworkManager connection profile for the AP. `ipv4.method shared` makes NM run dnsmasq for DHCP automatically. The AP gateway IP defaults to **`10.42.0.1`** — that's what MATLAB connects to in field mode.

### Switching modes

```bash
sudo ./rpi/scripts/net_mode.sh hotspot   # bring up AP, drop internet
sudo ./rpi/scripts/net_mode.sh client    # bring AP down, NM rejoins known wifi
./rpi/scripts/net_mode.sh status         # show current wlan0 state and IP
```

In hotspot mode, MATLAB connects with:

```matlab
wavi(transport='tcp', tcp_host='10.42.0.1', tcp_port=5555, nsensor=9)
```

For dev, **keep ethernet plugged in** while you toggle hotspot on — sync and SSH continue to work over wired even when wlan0 is in AP mode and has no internet.


If you want the Pi to boot into hotspot mode automatically, add a systemd unit like this on the Pi:

```ini
# /etc/systemd/system/whisker-hotspot.service
[Unit]
Description=Enable whisker hotspot at boot
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/bash /home/pi/whisker_sensor/rpi/scripts/net_mode.sh hotspot
ExecStop=/bin/bash /home/pi/whisker_sensor/rpi/scripts/net_mode.sh client

[Install]
WantedBy=multi-user.target
```

Then enable it:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now whisker-hotspot.service
sudo systemctl status whisker-hotspot.service
```

To stop using boot-time hotspot mode and go back to normal wifi-client behavior:

```bash
sudo systemctl disable --now whisker-hotspot.service
sudo ./rpi/scripts/net_mode.sh client
./rpi/scripts/net_mode.sh status
```

`disable --now` stops the service immediately and prevents it from being started again on the next boot. The explicit `client` call brings the AP down right away so NetworkManager can reconnect `wlan0` to a known wifi network.

### Expected range (Pi 4B built-in antenna)

Outdoors, line-of-sight, no obstructions:

- **2.4 GHz** (`band bg`, what the setup script uses): 50–100 m practical, up to ~150 m in ideal conditions.
- **5 GHz** (`band a`, edit the profile if you want it): 30–50 m, faster throughput close in.

Add walls or bodies and these drop fast (often to 10–20 m through one wall). A USB dongle with an external antenna can easily double or triple range.

## Phase 3 — Pi-side parsing, recording, multi-client (deferred)

Promote `daq_bridge.py` to a `daq_server.py` with a parsed source/server/recorder split: Pi owns frame parsing, multi-client broadcast, and `.dat` recording. The on-the-wire protocol will follow [gantry_control/python/connection_manager.py](../gantry_control/python/connection_manager.py) style — big-endian, length-prefixed messages, single-byte command headers — with the MATLAB-side client mirroring [gantry_control/matlab/NetworkClient.m](../gantry_control/matlab/NetworkClient.m) (raw `java.net.Socket` + `TcpNoDelay`).

Worth doing when **any** of these is true:

- A client crash mid-recording costs data (Pi-side recording survives client disconnects).
- Two consumers need the same stream simultaneously (live viz + a separate logger / classifier).
- A second data source (different sensor, mock generator) needs to join the same protocol.

Until then, Phase 1's tee is enough.
