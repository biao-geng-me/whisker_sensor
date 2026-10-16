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

From the MSD Altium sources (`/shared/rc/whiskers/MSD_P25937/P25937_Whisker_Final.zip`, sheets Whisker / Processing / Amplifier_Reference / MAIN / Power), per channel (32 per board):

- **Bridge (Whisker sheet), quarter bridge at 3.3 V:** the external gauge runs from 3V3 to `StrainPOS`, with 352 Ω from `StrainPOS` to GND. The reference half is 123 Ω + 123 Ω from 3V3 to GND, giving `StrainNEG` ≈ 1.65 V. That is about 18 mA per channel, ~0.6 A for 32 channels. 3V3 comes from a TPSM84203 buck module fed 5 V through the USB-A jack J1.
- **Amplifier (Processing sheet):** INA351ABS, SOT-23-8 (DDF) package, with pinout GS 1, IN− 2, IN+ 3, V− 4, REF 5, OUT 6, V+ 7, SHDN 8. **GS (pin 1) is tied to GND, so G = 10** (low = 10, high or unconnected = 20). SHDN is unconnected, so the amp is enabled. OUT drives the mux through 1 kΩ / 0.1 µF (corner ≈ 1.6 kHz). The older firmware comments said "gain ~141", which this part can't produce.
- **ADC (MAIN):** ADS131M04 at PGA gain 1 with its internal 1.2 V reference. AINxP comes from the mux COM lines; **all AINxN are tied to GND**.

**Open issue, REF offset:** in the schematic, every INA351 REF (pin 5) goes to `AMP_REF_3V3`, which is a 100 k/100 k divider from 3V3, so ≈ 1.65 V. With AINxN at GND, every connected channel would then read 1.65 V + signal, which is beyond the ±1.2 V full scale, so it would rail. The live board doesn't: W4_B rests at about +26 mV. So the built board probably differs from the schematic, e.g. the divider is reworked and REF ≈ 0 V. If REF is at GND, the single-supply INA output can't go below ≈ 0 V, so **bridge outputs below zero clip**, and only one bending direction is measured from a near-balanced rest. To check, measure REF (pin 5 of any INA351, or C4 on the reference sheet) and one INA output against GND with a multimeter.

Units: the stream carries raw 24-bit codes. `mV = code × 1200 / 2^23` (`MV_PER_CODE` in `whisker_link/esp32_frame.py`) is the voltage at the ADC input, range ±1200 mV. A channel with nothing plugged in rails at +1200 mV.

Strain: the gauge sits on the top arm, so tension (gauge R up) lowers `StrainPOS` and the reading. For a quarter bridge with small strain, after subtracting the resting offset V0, `ε ≈ −4 · V_adc / (G · V_ex · GF)`, assuming IN+ = StrainPOS. With G = 10, V_ex = 3.3 V and GF = 2, that is **−60.6 µε per mV**. For physical units, an end-to-end calibration (known tip load or deflection) is still more reliable than these nominal values.

## Build and flash

Arduino IDE 2.x with the **esp32 by Espressif** core 3.x; board **ESP32 Dev Module**. Open `whisker_daq/whisker_daq.ino`, upload over USB. Compile check from the command line:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 msd/esp32/whisker_daq
```

**Over the tether:** the ESP32 is plugged into the BlueOS Pi, so flash it from there with [`rpi/scripts/flash_esp32.sh`](../../rpi/scripts/flash_esp32.sh) (esptool runs on the Pi, where DTR/RTS auto-reset works). Build the image with `arduino-cli compile --fqbn esp32:esp32:esp32 --output-dir fw msd/esp32/whisker_daq` and copy `fw/whisker_daq.ino.merged.bin` to `~/esp32_flash/` on the Pi; the script's header has the one-time esptool install. Or unplug the ESP32 and use the Arduino IDE on the laptop.

## Pinout

See the header of the `.ino` and the MSD guide [`legacy/Guide.md`](legacy/Guide.md) for P1/P2 wiring and known channel notes. `legacy/` holds the MSD team's last firmware and host scripts as received on 2026-10-05 (the UDP/bridget setup), for reference.
