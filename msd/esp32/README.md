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

SCAN `flags`: bit0 halt recovered, bit1 CRC re-read, bit2 stale read dropped, bit3 ADC reset detected and re-initialised (`adc_reinit`). INFO (sent on `B`) carries, from `PROTO_VERSION` 2, the active `osr`, `settle_us` and `discard_samples` next to `clkin_hz` and `fdata_sps`; `daq_server.py` and the wavi_py esp32 transport copy them into every 1 Hz status record as `esp32.cfg`, so each jsonl log says what config it was taken at. Version-1 INFO frames still decode (the new fields are `None`).

## ADC reset detection (2026-10-22)

When the whisker board is powered after the ESP32 (e.g. from the BlueROV 5 V rail), or the ADC browns out, the ADS131M04 comes back at its power-on defaults (OSR 1024). Frames and CRCs stay valid, so the old firmware never noticed and the scan ran at exactly 12.2 Hz (8 selects × 4 conversions × 2.56 ms) until the ESP32 was reset. Now `initADC()` writes MODE = 0x0110, which clears the sticky RESET flag (SBAS890D Tables 8-15/8-16: STATUS and MODE bit 10). Every frame read checks STATUS bit 10; if it is set again, the ADC really reset, so the firmware re-runs the boot init, restarts the sweep, sets SCAN flag bit3 and logs `[adc reset #n]`. Re-inits are limited to one per second, so a MODE write that doesn't stick can't cause a storm. A periodic CLOCK readback was considered and left out: the RESET bit is checked on every conversion (~2500/s) at no cost, while a readback needs a command frame between scans that disturbs the DRDY/FIFO accounting.

Also fixed: the RESET command was sent as a single 3-byte word, but the ADC latches RESET only at the end of a complete 6-word frame ("Terminating the frame early causes the RESET command to be ignored", SBAS890D 8.4.1.3), so the init's reset was probably a no-op. It now clocks a full 18-byte frame. The registers the init writes end up the same either way.

## Raw burst capture and spectrum (finding the true interference frequencies)

The scan samples each gauge at ~76 Hz, so the ~0.2 mV common tone at 6–7 Hz and the ROV-supply tones at 16.25 and 27.5 Hz are probably aliases of faster sources. The `r` command holds one mux select and records every conversion of the 4 ADC channels into ESP32 RAM, with the DRDY edge time, then dumps it as text:

```
r<sel> [n] [osr] [clkin_hz] [spi_hz]      then ';' or newline, e.g.  r2 8192 1024;
```

Defaults: n 8192 (capped by free RAM, up to 16384), OSR 128, CLKIN 0.8 MHz, SPI 600 kHz. OSR is 128…16384 (powers of 2), and fDATA = CLKIN / 2 / OSR. CLKIN can be raised for the burst only, up to 8 MHz. It must be 80 MHz / integer, so 8.192 MHz isn't reachable; the ADC allows 0.3–8.4 MHz in high-resolution mode. Each conversion is one 18-byte SPI read (~260 µs at 600 kHz), so keep fDATA below ~3.5 kSPS or raise `spi_hz`; missed conversions are flagged as overruns. Afterwards CLKIN, SPI and the OSR-128 config are restored with the boot init, and normal operation resumes (binary mode drops to the grid view; `daq_server.py` re-sends `B` by itself). The dump format is in the comment above `burst_capture()` in the `.ino`; it ends with a line count and a CRC over the data lines.

Host side, [`burst_spectrum.py`](burst_spectrum.py) (numpy, scipy, pyserial; matplotlib for the plot). Close wavi_py / daq_server first, since they hold the port.

```bash
# laptop, ESP32 on USB: 3 bursts, saves bench_sel2.{txt,npz,png,html}
python msd/esp32/burst_spectrum.py capture --port /dev/ttyUSB0 --sel 2 --out bench_sel2
# choose the bursts: OSR alone (0.8 MHz) or CLKIN:OSR
python msd/esp32/burst_spectrum.py capture --port COM5 --sel 2 --burst 128 1000000:128 8000000:2048
# offline, e.g. on the cluster
python msd/esp32/burst_spectrum.py analyze bench_sel2.npz --out bench_sel2
```

The report gives Welch amplitude spectra (peak mV at the ADC input, 1.4305e-4 mV/code) per channel and burst, the peaks, the peaks common to ≥3 channels, and where each peak would alias in the scan (`--scan-hz`, default 76.2). It also lists the true-frequency candidates up to `--f-max` that explain the peaks of every burst. Bursts that differ only in OSR can't resolve aliases, because the rates nest (3125 = 4 × 781.25) and any frequency that fits the fastest burst fits them all. That is why the default set mixes CLKINs: 0.8 MHz/OSR 128 (3125 SPS), 1 MHz/OSR 128 (3906.25 SPS) and 8 MHz/OSR 2048 (1953.125 SPS). Tests on synthetic dumps: `python -m pytest msd/esp32/test_burst_spectrum.py`.

## Fast-scan prep: SPI sweep, fine settle diagnostic, active selects (2026-10-22)

The planned fast scan (CLKIN 8 MHz, OSR 128/256, several conversions averaged per select, low-pass and decimation to ~80 Hz on the ESP32) needs three numbers first: how fast the SPI link runs cleanly, how long a select change really takes at that rate, and the select list. These commands measure them; the fast scan itself isn't built yet. `D` and `p` change CLKIN, SPI clock and OSR for the measurement and restore the normal config (0.8 MHz, OSR 128, 600 kHz) afterwards with the same init as boot. The host tool is [`adc_diag.py`](adc_diag.py) (numpy, scipy, pyserial; matplotlib only for the plot; all files UTF-8). Close wavi_py / daq_server first, since they hold the port. In the commands below, replace `COM5` with your port.

**1. SPI speed sweep** (firmware `p[clkin] [osr] [secs] [spi_hz ...];`, default 8 MHz, OSR 256, 2 s per step, 0.6/1/2/3/4/5/6/8 MHz). Per SPI clock it does a full ADC init, then reads every conversion and reports frames vs expected, CRC errors, stale reads, overruns (more than one DRDY edge pending, so conversions were missed), edgeless claims (a DRDY pulse lost because a conversion finished during a read), whether the CLOCK register read back correctly, and the measured read time (CS low to high) and latency (DRDY edge to end of read). The latency must stay below the conversion period: 64 µs at OSR 256, 32 µs at OSR 128. An 18-byte frame is 144 SCLK, i.e. 18 µs at 8 MHz before any overhead.

```powershell
cd C:\path\to\whisker_sensor_repo\msd\esp32
python adc_diag.py spi --port COM5
python adc_diag.py spi --port COM5 --osr 128 --spi 4000000 5000000 6000000 8000000 10000000 13333333
```

It prints a table, the fastest clean SPI clock with every slower step clean too, and one step below it as the margin pick. It saves `spi_sweep_<time>.txt` / `.csv`. Above 8 MHz, the ADC allows up to 25 MHz (SBAS890D 6.6), so the jumper wiring is the limit. The firmware accepts up to 20 MHz, and the ESP32 rounds to 80 MHz / n (`spi_actual`).

**2. Fine mux settle diagnostic** (firmware `D<from> <to> [clkin] [osr] [spi] [reps] [window_us] [dither] [dwell_us];`, defaults 8 MHz, OSR 256, SPI 4 MHz, 64 reps, 2000 µs, dither 4, 3000 µs). Per rep: dwell on `<from>` reading every conversion, wait a fraction (rep mod dither)/dither of a conversion period, so that over the reps the switch lands at different phases, then switch to `<to>` and read every conversion for the window. It dumps every conversion (`rep t_us flags c0 c1 c2 c3`, `t_us` = DRDY edge minus the switch) with a CRC. The expected limit is charge sharing, not the mux: the COM node (~30 pF) dumps the previous channel's voltage into the new channel's 1 k / 0.1 µF filter, a kick of ~3e-4 × the voltage difference that decays with τ ≈ 100 µs. So switch from a select whose channels are railed (nothing plugged in) to one with gauges. With the planned 12 whiskers, select 6 carries only uninstalled slots (OUT27/19/6/16 = W14_A, W10_A, W3_B, W8_B), and select 3 only installed ones (OUT26/24/1/9 = W13_B, W12_B, W1_A, W5_A).

```powershell
python adc_diag.py settle --port COM5 --from 6 --to 3 --spi <best safe SPI from step 1> --out settle_6to3
python adc_diag.py settle --port COM5 --from 6 --to 3 --osr 128 --spi <best safe SPI> --out settle_6to3_osr128
python adc_diag.py analyze settle_6to3.npz          # offline, e.g. on the cluster
```

The analysis compares each channel with the ideal response, i.e. what an instantaneous analog step gives through the sinc3 filter of the same OSR and CLKIN (computed). It fits the DRDY/ISR latency on the channel with the biggest step. It then fits the leftover analog tail as a sinc3-filtered A·exp(−t/τ) and reports A, τ, A/(pre − final), A·τ, and the time after the switch when the tail drops below 0.01 mV, both from the fit and from the data. It writes `<out>.txt` (raw), `.npz`, `.html` and, with matplotlib, `.png`. Reps that report CRC, stale, overrun or edgeless are dropped from the fit, and their count is listed.

**3. Active selects** (firmware `S<digits>;`, `S;` = all 8). The normal scan then visits only those selects, so with `S0123457;` a scan takes 7/8 of the time. Outputs of a skipped select hold +full scale, so they are railed in the SCAN railmask and show as "open" in wavi_py and old decoders. INFO (`PROTO_VERSION` 3) carries `sel_mask`, and the status record shows `esp32.cfg.selects`, e.g. `"0123457"`. wavi_py adds `| selects 0123457` to its status line. The mask is kept in RAM only, so the ESP32 is back at all 8 after a reboot.

```powershell
python adc_diag.py selects --port COM5 0123457      # 'all' to undo
```

On the ROV, `rpi/daq_server.py --selects 0123457` sends `S0123457;` before every `B`, so the list survives ESP32 reboots. In a serial monitor, type `S0123457;` directly. In a PowerShell one-liner that writes to the port, quote the string, because `;` separates PowerShell statements.

## Mux settling and fresh conversions (bench, 2026-10-20)

Before commit 3c7e881, `waitDRDY()` tested the DRDY level, so a scan could return a conversion that had already been read (or the older slot of the ADC's 2-deep FIFO). That stale-sample bug, not the hardware, caused the spikes and the "~2 ms/select analog settling floor (~52 Hz for 32 ch)" in the MSD handoff (`/shared/rc/whiskers/MSD_P25937/PROJECT_HANDOFF.md` §3.3). The firmware now counts DRDY falling edges in an ISR and checks STATUS DRDY[3:0] = 1111 on every frame; a read that fails is dropped and counted (SCAN flag bit2 `FLAG_STALE`, `adc-stale` in the status line). The handoff's "SCLK must stay below CLKIN" rule is not in the datasheet either.

Bench results with a 352 Ω gauge on W2_A (OUT3, ADC CH2, select 2), boot reporting DRDY edges/200 ms = 1246, CLOCK readback 0x0F02 and Device ID 0x2405:

- `d2` settling curve, step −1107 mV from a railed channel: conversions k = 0, 1 after the switch still hold the old value; k = 2 (~700 µs) has 41 % of the step left, k = 3 (~1015 µs) 4 mV, k = 4 (~1330 µs) 0.03 mV, within the 0.06 mV noise, with no slow tail out to 4.9 ms. Settling is set by the ADS131M04 sinc3 latency (~4 conversions × 320 µs at CLKIN 0.8 MHz, OSR 128), not by the analog chain (at 320 µs resolution). The firmware keeps a conversion ending ≥ 250 + 4 × 320 = 1530 µs after the switch, about one conversion of margin.
- Streaming through the wavi_py esp32 transport: 76.2 Hz, steady 13 ms period, no gaps. W2_A first-difference noise 0.063 mV (was 3.07 mV) and 0 spikes in 10 s (was 36, up to 145 mV). The floating channels W3_B and W8_B lost their spikes too. The stale counter rises by about 1 per several hundred scans in normal scanning; the `d` diagnostic produces more, which is harmless.

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
