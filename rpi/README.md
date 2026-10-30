# Raspberry Pi DAQ

The Pi sits between the Arduino (USB serial) and one or more PC clients (TCP). Conceptually it's a `tee` for the Arduino's byte stream: data flows out to whoever's listening, and the Pi can grow extra capabilities (parsing, recording, multi-client) on a separate branch later without touching the data path.

## BlueROV2: whisker array + IMU server (`daq_server.py`)

On the BlueROV2 the "Pi" is the vehicle's own BlueOS computer (Raspberry Pi 4 + Navigator, 192.168.2.2 on the tether). The ESP32 whisker DAQ ([`msd/esp32/`](../msd/esp32/)) plugs into it by USB. [`daq_server.py`](daq_server.py) replaces both `daq_bridge.py` and BlueOS's UDP serial bridge (bridget) for this setup:

```
ESP32 (USB, binary frames, 921600) ──┐
                                     ├─► stamped on the Pi's monotonic clock ─┬─► ~/whisker_rec/*.jsonl  (on the Pi)
ArduSub (MAVLink, UDP 127.0.0.1:14670)┘                                       └─► TCP :5600, JSON lines ─► wavi_py on the laptop
```

- Both sources are timestamped by the same clock on the Pi, and each also carries its own sample clock (ESP32 `micros()`, autopilot `time_boot_ms`). [`whisker_link/convert.py`](../whisker_link/convert.py) uses both to put whiskers and IMU on one time base to well under a millisecond (checked end to end with the simulators: an injected 50 ms lag is recovered as 50 ± 1 ms).
- Requested MAVLink streams (re-requested every 5 s, since a GCS can override rates): `RAW_IMU` at 100 Hz, `ATTITUDE`/`ATTITUDE_QUATERNION` 50 Hz, `SCALED_PRESSURE2` (depth), `SERVO_OUTPUT_RAW` (thrusters) and `RC_CHANNELS` (pilot inputs) 20 Hz, `LOCAL_POSITION_NED` 10 Hz. Everything else the router delivers is recorded too. Change rates with `--mav-rates RAW_IMU=200,SERVO_OUTPUT_RAW=50`; the measured rates are in the status line, so check what ArduSub actually delivers before relying on a number.
- Recording format = stream format ([`whisker_link/wire.py`](../whisker_link/wire.py)): one JSON object per line, ~330 MB/hour. Convert with `python -m whisker_link.convert file.jsonl --mat`.

### One-time vehicle setup

1. **Flash the new firmware** ([`msd/esp32/README.md`](../msd/esp32/README.md)) over USB from the laptop.
2. **Delete the bridget serial bridge** — it holds `/dev/ttyUSB0` open, and the server can't share it: `curl -X DELETE http://192.168.2.2/bridget/v1.0/bridges -H 'Content-Type: application/json' -d '{"serial_path":"/dev/ttyUSB0","baud":115200,"ip":"192.168.2.1","udp_target_port":27000,"udp_listen_port":27001}'`, then check `curl -s http://192.168.2.2/bridget/v1.0/bridges` prints `[]`.
3. **Add a MAVLink endpoint** — BlueOS web UI → enable Pirate mode → MAVLink Endpoints → new endpoint: type **UDP Client**, IP `127.0.0.1`, port `14670`, name `whisker`. Don't use 14660: BlueOS reserves it for its protected "Ping360 Heading" UDP Server endpoint, and daq_server fails with `Address already in use`.
4. **Python deps on the Pi host (offline).** `ssh pi@192.168.2.2` (BlueOS default password `raspberry` — change it). The Pi is Raspbian 11, armv7l (32-bit), Python 3.9.2, with no pip and usually no internet. Debian also strips `ensurepip` and `distutils`, so pip can't run there at all, not even from its wheel. Instead, run [`scripts/fetch_pi_wheels.sh`](scripts/fetch_pi_wheels.sh) on any machine with internet (armv7l wheels come from piwheels), copy the `pi_wheels` folder to the Pi, and unzip the wheels straight into a venv:
   ```bash
   python3 -m venv --without-pip ~/whisker_venv
   SITE=$(~/whisker_venv/bin/python -c "import sysconfig; print(sysconfig.get_paths()['purelib'])")
   for w in ~/pi_wheels/*.whl; do ~/whisker_venv/bin/python -m zipfile -e "$w" "$SITE"; done
   ~/whisker_venv/bin/python -c "from pymavlink import mavutil; import serial; print('ok')"
   ```
5. **Copy the code** from the laptop: `.\rpi\scripts\sync.ps1 -PiHost pi@192.168.2.2` (pushes `rpi/` and `whisker_link/` to `~/whisker_sensor`).
6. **Install the service** so the server starts at boot: on the Pi, `sudo bash ~/whisker_sensor/rpi/scripts/install_service.sh`. It installs [`scripts/whisker-daq.service`](scripts/whisker-daq.service) as `/etc/systemd/system/whisker-daq.service` (runs as `pi`, with the `dialout` group for the serial port, restarts 3 s after any exit, forever), writes `~/whisker_sensor.env` with the server arguments if it doesn't exist yet, enables and (re)starts it, and prints its status. It warns if a bridget bridge still uses the serial port (step 2), if a daq_server started by hand is still running, or if the venv or the code is missing. Running it again is safe (it keeps your env file); `--uninstall` stops and removes the service (env file and recordings stay).

### Every session: it starts at boot

The `whisker-daq` service runs `~/whisker_venv/bin/python -u rpi/daq_server.py $DAQ_ARGS` in `~/whisker_sensor`, with `DAQ_ARGS` from `~/whisker_sensor.env`:

```bash
DAQ_ARGS=--serial /dev/ttyUSB0 --mode wide --selects 0123457
```

The default arguments keep the ESP32 in wide mode (250 Hz, OSR 128) with selects `0123457` (12 live whiskers; select 6 is dead). Firmware from 2026-10-28 on boots that way anyway; earlier firmware boots in quiet 62.5 Hz with all 8 selects, so keep `--mode` / `--selects` in the env file while any ESP32 may run older firmware (they are re-sent before every `B`, so they also survive reboots). Without them the server leaves the ESP32 as it is and reports what its INFO says (`esp32.cfg` in the status).

Edit that line to change the arguments (no quotes needed; don't add `--record`, recording is started from wavi_py or a command), then restart. It doesn't matter whether the ESP32 is plugged in or BlueOS's MAVLink router is up yet: the server waits for the serial port (retrying every 2 s) and for the MAVLink endpoint, and clients can connect meanwhile.

```bash
systemctl status whisker-daq            # running? last log lines
journalctl -u whisker-daq -f            # follow the log (-b: since boot, -n 200: last 200 lines)
sudo systemctl restart whisker-daq      # after a code sync or an env-file edit
sudo systemctl stop whisker-daq         # e.g. to run the server by hand; `start` brings it back
sudo systemctl disable whisker-daq      # don't start at boot any more (enable: again)
```

After a sync the service still runs the old code: restart it, or sync with `./rpi/scripts/sync.sh pi@192.168.2.2 --restart` / `.\rpi\scripts\sync.ps1 -PiHost pi@192.168.2.2 -Restart` (asks for the Pi's sudo password). To run the server by hand instead (e.g. with other arguments for a test), stop the service first, as both need TCP 5600 and the serial port: `sudo systemctl stop whisker-daq && cd ~/whisker_sensor && ~/whisker_venv/bin/python -u rpi/daq_server.py --serial /dev/ttyUSB0`.

Every 5 s it logs e.g. `whisker 82.4 Hz | RAW_IMU 100, ATTITUDE 50, ... | clients 1 | rec off | errs: gap 0 link-crc 0 junk 0 reopen 0 adc-crc 0 adc-recov 0`. The `errs` are running totals: `gap` = scans lost on USB, `link-crc` = corrupted frames, `junk` = bytes that weren't frames (boot text after an ESP32 reset is normal; a count that climbs while whisker is 0 Hz means bytes arrive but don't decode), `reopen` = times the port was reopened because nothing decoded for 10 s, `adc-crc` = ADC SPI reads that failed CRC and were re-read (data stays valid; a climbing count means marginal SPI wiring or ground), `adc-recov` = ADC halts recovered. wavi_py shows the same counters in its status line. If whisker is 0 Hz, the ESP32 isn't streaming: the server sends `B` every 2 s on its own; if that doesn't help, press **Reset ESP32** in wavi_py (next to the scan mode; it asks first, since streaming stops for a few seconds and a running recording gets a gap) or send `{"cmd":"esp32_reset"}`. Either pulses RTS (= pressing EN); the ack says whether it worked (`ok`, or `error` with the reason), and a successful reset puts a `mark` "ESP32 reset (EN pulse) by client …" in the stream and the recording. It doesn't power-cycle the whisker PCB, so it can't clear an ADC lock-up. If the IMU entries are missing, the MAVLink endpoint (step 3) isn't set up.

**Self-healing and a stuck server.** If the port is open but nothing decodes for 10 s, the server closes and reopens it (logged as `nothing from /dev/ttyUSB0 for 10 s … reopening the port`). A reopen doesn't reset the ESP32. Serial writes time out after 1 s instead of blocking. If the ESP32 or MAVLink thread stops looping, or dies, for `--stall-exit` seconds (default 20), the server logs `CRITICAL … thread stuck …`, dumps every thread's stack into the journal, closes any recording and exits, so systemd restarts it within 3 s. To see where a running server is without stopping it: `sudo kill -USR1 $(systemctl show -p MainPID --value whisker-daq)`, then `journalctl -u whisker-daq -n 80`. On 2026-10-30, one server process stopped receiving at 14:53, about 15 s after ArduSub came up. An EN reset and a USB replug didn't help, but a fresh process streamed right away. If whisker stays at 0 Hz after a reset, `sudo systemctl restart whisker-daq` before touching the hardware.

**ADC fault alarm.** The ADS131M04 on the whisker PCB (vehicle 5 V) can lock up so that only a PCB power cycle clears it, which on the ROV means a vehicle power cycle. The server runs the fault model in `whisker_link/esp32_fault.py` on the ESP32's frames. A **fault** is an `[halt #N] UNRECOVERED`, CLOCK config not accepted, ADC not answering, a locked-mode halt storm with no scans, no scans for 5 s while the port is open, or the serial port gone for 5 s. With firmware that sends the 1 Hz heartbeat (BEAT frame, 2026-10-23 brief 6) it also tells "ADC dead" from "ESP32 hung": beats flagging the ADC down or with no DRDY edges are an ADC fault, and nothing at all (no beat, scan or text) for 5 s is `esp32_hung` (reset the ESP32). The latest beat is in the status as `esp32.beat`. It is latched and clears only after 10 s of clean streaming. A **warning** is a recovered halt, an ADC reset, a reader-watchdog or other unexpected ESP32 reboot, an `[idf]` error, or more than 1 % of scans flagged SYNC/EMPTY. The state is in every 1 Hz status record as `esp32.fault` = `{level: ok|warn|fault, code, msg, action, detail, since (unix), count, warns, history}` (for Cockpit later). Each fault onset, clear and warning is logged (ERROR / WARNING, and the active fault again with every status log line) and published as a `mark` event with a `fault` field, so it lands in the recording and reaches the clients. A recording started during a fault begins with an "at record start" mark. Test it without hardware: `kill -USR2 <fake_esp32 pid>` locks the fake ADC up, a second USR2 is the PCB power cycle, and `kill -STOP` is a hung ESP32 (`rpi/tests/test_fault_alarm.py`).

On the laptop:

```bash
python -m wavi_py --transport server --host 192.168.2.2 --scale 0.2
```

**Rec** in the GUI starts a recording on the Pi *and* writes the same stream to a local `.jsonl` (plus the usual whisker-only `.dat`) in `~/wavi_data`. **IMU** opens the accel / gyro / attitude view. Recording without the GUI: start the server with `--record --tag run1`, or send commands over the socket (`{"cmd":"rec_start","tag":"run1"}`, `{"cmd":"rec_stop"}`, `{"cmd":"mark","text":"tap W3"}`).

### Testing without the vehicle

[`sim/fake_esp32.py`](sim/fake_esp32.py) (pty) and [`sim/fake_autopilot.py`](sim/fake_autopilot.py) (MAVLink UDP) stand in for the hardware; both put the same sine motion on a shared wall clock so alignment can be checked:

```bash
python rpi/sim/fake_esp32.py --link /tmp/wtty &      # boots like the firmware: wide, 0123457
                                                      # (--boot-mode 1 --boot-selects "" = older firmware)
python rpi/sim/fake_autopilot.py --lag-ms 50 &
python rpi/daq_server.py --serial /tmp/wtty --rec-dir /tmp/rec
python -m wavi_py --transport server --host 127.0.0.1
```

The fake ESP32 is a pty, which has no RTS line: `esp32_reset` answers `ok: false` with `RTS pulse failed ...` there (the Sim transport in wavi_py does reboot on it). [`tests/test_service.py`](tests/test_service.py) checks the boot-time behaviour (server up before the ESP32 port exists and while the MAVLink port is taken, both picked up later, clean exit on SIGTERM) and the reset's ack; `DAQ_SERVER_PY="env PYTHONPATH=<site> /usr/bin/python3"` runs the server under the Pi's Python 3.9.

## Phase 1 — transparent byte pump ✅ done (multi-client)

[`daq_bridge.py`](daq_bridge.py) opens the Arduino serial port once and broadcasts the byte stream verbatim to every connected TCP client. No parsing, no recording on the Pi. Each client also gets its own back-channel — bytes from any client (`N=<n>\n`, `S`, `E`) are forwarded to the Arduino through a single serialized write lock.

Multi-client notes:

- The Arduino emits one stream; every client gets the same bytes at the same time. A single background fan-out thread reads from serial and pushes each chunk into per-client send queues.
- Each client has a bounded send queue (default 1024 chunks, ~16 s of buffering at default rates). If a client falls so far behind that its queue fills, it's disconnected; other clients are unaffected.
- The Arduino's `loop()` only acts on `S` / `E` bytes and ignores everything else, so two clients can race on the record-indicator pin. Either coordinate at the application level or only let one client send control bytes.
- A new client joining mid-stream sees the Arduino bytes from wherever the byte stream is *right now*; the client's marker-search alignment (in `wavi.m` / `wavi_py`) finds the next frame boundary on its own.

The MATLAB and Python clients ([wavi/wavi.m](../wavi/wavi.m) and [wavi_py/](../wavi_py/)) handle framing and record `.dat` files locally exactly as they do over a direct serial link.

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

## Phase 3 — Pi-side parsing and recording (deferred)

Multi-client landed as part of Phase 1's transparent fan-out (see above), so the remaining Phase 3 work is Pi-side parsing + recording: promote `daq_bridge.py` to a `daq_server.py` that parses Arduino frames, owns `.dat` recording, and exposes a framed protocol. The wire format would follow [gantry_control/python/connection_manager.py](../gantry_control/python/connection_manager.py) style — big-endian, length-prefixed messages, single-byte command headers — with the MATLAB-side client mirroring [gantry_control/matlab/NetworkClient.m](../gantry_control/matlab/NetworkClient.m) (raw `java.net.Socket` + `TcpNoDelay`).

Worth doing when **either** of these is true:

- A client crash mid-recording costs data (Pi-side recording survives client disconnects).
- A second data source (different sensor, mock generator) needs to join the same protocol with consistent framing.

Until then, Phase 1's tee is enough — multiple clients already share the raw stream, and any of them can record locally on their own machine.
