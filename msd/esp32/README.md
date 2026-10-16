# ESP32 firmware — whisker array DAQ

`whisker_daq/whisker_daq.ino` reads the 32-channel whisker PCB (ADS131M04 + 2×CD4097, 16 whiskers × 2 strain gauges) and streams it to the host. It is a rewrite of `whisker_reader.ino` (Rev 2026-08-04-c, MSD team): the ADC driver, mux mapping and halt recovery are unchanged; the host link is new.

## What changed vs `whisker_reader.ino`

| | whisker_reader.ino | whisker_daq.ino |
|---|---|---|
| Baud | 115200 (caps the scan rate at ~50/s) | 921600 (scan rate ~80/s, set by the ADC) |
| Stream | ASCII `$,scan,railmask,mV×32` | binary frames with CRC (`B` command) |
| Values | mV deviation from the ESP32's boot baseline | raw 24-bit ADC codes (zeroing is host side, nothing lost) |
| Timestamp | none | `micros()` at the start of every scan |
| Log text in stream mode | mixed into the data lines | framed as TEXT, never corrupts data |

The text modes (grid, table, profile, calibrate, noise, map) are kept for bench work in a Serial Monitor at 921600 baud. The old `whisker_plot.py` / `whisker_scope.py` do not understand the binary stream; use `wavi_py --transport server` with `rpi/daq_server.py` instead.

## Frame format

Defined once in [`whisker_link/esp32_frame.py`](../../whisker_link/esp32_frame.py) (keep the two in sync). Every frame is `A5 5A | type | len | payload | crc16-LE`, CRC-16-CCITT over type, len and payload. SCAN payload: `seq u32, t_us u32, dur_us u16, railmask u32, flags u8, nch u8, nch × i24` in OUT order 1..32, which is W1_A, W1_B, W2_A, … W16_B.

Channels within one scan are not simultaneous: the four ADC channels of one mux select are, but the 8 selects follow each other ~1.5 ms apart (~12.3 ms per scan). `t_us` is the scan start.

## Signal chain and units

Per gauge (32 per board): one strain gauge in a **350 Ω quarter bridge** (one active gauge, three fixed resistors), excited at **3.3 V** from the PCB's on-board regulator (the PCB is fed 5 V). The bridge output goes to an **INA351ABSIDDFR** instrumentation amplifier, whose gain is set by its GS pin (pin 1): **low = 10, high or unconnected = 20**. Which one this board uses has not been verified yet. Its output is `G × V_bridge + V_REF` (REF is pin 6), so the REF connection sets the zero offset seen by the ADC. The older firmware comments said "gain ~141", which this part can't produce, so treat that number as wrong. The amplifier output goes through the CD4097 mux to the **ADS131M04**, which runs at PGA gain 1 with its internal 1.2 V reference.

Units: the stream carries raw 24-bit codes. `mV = code × 1200 / 2^23` (`MV_PER_CODE` in `whisker_link/esp32_frame.py`) is the voltage at the ADC input, range ±1200 mV. A channel with nothing plugged in rails at +1200 mV.

Strain (quarter bridge, small strain, after subtracting the resting offset V0): `ε = 4 · V_adc / (G · V_ex · GF)`. With V_ex = 3.3 V and GF = 2, that is **30.3 µε per mV at G = 20** and **60.6 µε per mV at G = 10**. Measure V_ex on the board once. For physical units, an end-to-end calibration (known tip load or deflection) is more reliable than these nominal values.

## Build and flash

Arduino IDE 2.x with the **esp32 by Espressif** core 3.x; board **ESP32 Dev Module**. Open `whisker_daq/whisker_daq.ino`, upload over USB. Compile check from the command line:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 msd/esp32/whisker_daq
```

**Over the tether:** the ESP32 is plugged into the BlueOS Pi, so flash it from there with [`rpi/scripts/flash_esp32.sh`](../../rpi/scripts/flash_esp32.sh) (esptool runs on the Pi, where DTR/RTS auto-reset works). Build the image with `arduino-cli compile --fqbn esp32:esp32:esp32 --output-dir fw msd/esp32/whisker_daq` and copy `fw/whisker_daq.ino.merged.bin` to `~/esp32_flash/` on the Pi; the script's header has the one-time esptool install. Or unplug the ESP32 and use the Arduino IDE on the laptop.

## Pinout

See the header of the `.ino` and the MSD guide [`legacy/Guide.md`](legacy/Guide.md) for P1/P2 wiring and known channel notes. `legacy/` holds the MSD team's last firmware and host scripts as received on 2026-10-05 (the UDP/bridget setup), for reference.
