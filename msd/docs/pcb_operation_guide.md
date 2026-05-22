# Seal Whisker 32-Channel PCB — Operation Guide

**Subject:** Bring-up, daily operation, and shutdown of the 16-whisker / 32-channel logger PCB connected to a Raspberry Pi 4B.
**Author:** Claude (Anthropic, model Opus 4.7), assisting per chat session with the project owner
**Date:** 2026-05-22

This guide describes how to physically and procedurally operate the logger hardware. It does not cover bench-level board repair. Treat the bring-up sequence as authoritative — most failures the project has seen are sequencing or wiring issues that this procedure rules out before they happen.

---

## 1. Hardware overview

| Component | Role |
|-----------|------|
| Raspberry Pi 4B | Host. Drives ADC master clock, SPI, MUX address lines. Runs the Python logger. |
| ADS131M04 | 4-channel, 24-bit simultaneously-sampling delta-sigma ADC. |
| 2 × CD4097B | Differential 8-channel analog MUX. Together they multiplex 32 sensor lines into 4 ADC inputs at 8 MUX positions. |
| 16 × Wheatstone bridges (350 Ω, strain-gauge) | One bridge per whisker, two channels per whisker → 32 channels total. |

All chips on the analog board are powered at **VDD = 3.3 V** (confirmed by multimeter). This matches the Pi's GPIO logic level so no level shifters are required.

---

## 2. Wiring summary

All numbers below are BCM GPIO numbering (the standard convention used by `pigpio`, `RPi.GPIO`, and `spidev`). Physical pin numbers are also given.

### P1 connector — ADC

| Pi GPIO | Pi physical pin | Signal | ADC pin | Notes |
|---------|----------------|--------|---------|-------|
| 4       | 7              | CLKIN  | CLKIN   | Hardware clock output (GPCLK0), driven by `pigpio.hardware_clock` at 8.192 MHz. **Must be GPIO 4** — no other free pin on the header can generate this clock from `pigpio` without conflicts. |
| 5       | 29             | DRDY   | DRDY    | Active-low input. Internal pull-up enabled. |
| 9       | 21             | MISO   | DOUT    | SPI0 MISO. **Forced by hardware.** |
| 10      | 19             | MOSI   | DIN     | SPI0 MOSI. **Forced by hardware.** |
| 11      | 23             | SCLK   | SCLK    | SPI0 SCLK. **Forced by hardware.** |
| 27      | 13             | CS     | CS      | Manual GPIO (`spi.no_cs = True`). Any free GPIO works; 27 is the project convention. |

### P2 connector — MUX address lines

Both CD4097B chips on the board receive **the same address** from the same Pi GPIO writes (see `set_mux` in the logger). The two sets of select pins go to the two different chips.

| Pi GPIO | Pi physical pin | Signal | Bit | Notes |
|---------|----------------|--------|-----|-------|
| 17 | 11 | SELA_1 | bit 0 | MUX #1 A pin |
| 22 | 15 | SELB_1 | bit 1 | MUX #1 B pin |
| 23 | 16 | SELC_1 | bit 2 | MUX #1 C pin |
| 24 | 18 | SELA_2 | bit 0 | MUX #2 A pin |
| 25 | 22 | SELB_2 | bit 1 | MUX #2 B pin |
| 26 | 37 | SELC_2 | bit 2 | MUX #2 C pin |

### Power and ground

- Analog board VDD (3.3 V) — see Section 3 for source options.
- Analog board GND — must be tied to Pi GND at one point (single-point ground at the ADC is best).

---

## 3. Power options

The analog board can be powered in three ways, ordered from cleanest to dirtiest:

1. **Best — separate USB charger / bench supply.** Power the analog board off its own 5 V brick (into an on-board 3.3 V LDO) or off a clean 3.3 V bench supply. Only **GND** is shared with the Pi. This is the option to use whenever measurement quality matters.
2. **Acceptable — Pi USB-A port → analog board's 5 V input → on-board LDO.** The Pi's USB-A ports deliver 5 V at up to ~1.2 A total across all four ports. The 5 V they pass is the same rail as the Pi itself, so digital noise from the SoC, HDMI, Ethernet, and SD I/O rides on it. The on-board LDO's PSRR helps but does not fully clean it.
3. **Not recommended — Pi 3.3 V GPIO header pin direct to bridges.** The 3.3 V GPIO rail on the Pi is regulator-limited to ~500–800 mA total, and at 32 × 350 Ω bridges drawing ~300 mA at 3.3 V, you would consume most of that budget. The bridges would also see Pi digital noise directly.

If using option 2 or 3, expect noise on the readings and consider lowering bridge excitation to ~1.25 V using a precision reference (see *Future improvements* in Section 9).

---

## 4. Power-on sequence

Always assume that GPIO wires are vulnerable to slipped probes and that CMOS chips can latch up if signals are driven into an unpowered rail.

1. With **everything powered off**, plug the GPIO wires between the Pi 40-pin header and the PCB.
2. Double-check the pinout one more time against Section 2 — especially GND.
3. Power on **the analog board first** so its V_DD is established before the Pi's GPIOs start driving anything.
4. Power on **the Pi second**. The Pi's GPIOs come up as inputs during boot, so they will not inject high logic into the chips before the Pi software runs.
5. Wait until the Pi has finished booting and you can SSH in (~30 s).
6. On the Pi, verify the pigpio daemon is running:
   ```bash
   pgrep pigpiod || sudo systemctl start pigpiod
   ```
   (Or `sudo pigpiod` if not configured as a service.)
7. Verify SPI is enabled:
   ```bash
   ls /dev/spidev0.0
   ```
   Should print the device path with no error. If it errors, enable SPI with `sudo raspi-config` → Interface Options → SPI → Enable, then reboot.

If you do this sequence in reverse — Pi on first, analog board off, GPIOs driving high into an unpowered chip — you risk CMOS latch-up on the ADC. The chip will survive most occasional mistakes thanks to its protection diodes, but the failure mode is real and is documented in §6.1 of the ADS131M04 datasheet.

---

## 5. Running the logger

```bash
cd ~/.../seal_whisker_32ch_logger
python3 logger_32ch_dat_fixed.py
```

The logger writes timestamped `.dat` files into `data_32ch/`:
- `whisker16_32ch_voltage_YYYYMMDD_HHMMSS.dat` — converted volts.
- `whisker16_32ch_counts_YYYYMMDD_HHMMSS.dat` — raw ADC counts.

Stop with Ctrl+C. The script flushes and closes files, releases SPI and GPIO, and stops the master clock.

The current target frame rate is set by `TARGET_HZ` in [config_32ch.py](../config_32ch.py). At 32 channels through 8 MUX positions with the current 2 ms MUX settle, the physical floor is roughly 45 Hz. To reach higher rates, see Section 9.

---

## 6. Bring-up in stages (recommended for any change to the board)

Whenever you modify the PCB, swap a chip, or rewire something, bring the system up in stages instead of running the full logger immediately:

1. **Stage 1 — Pi only, analog board disconnected.** Verify the Pi boots and `pigpiod` runs. Run a quick script that toggles GPIO 27 (CS) and observes it on a scope or LED.
2. **Stage 2 — Analog board powered, ADC + SPI only, MUX address lines left at GND.** Wire up only CLKIN, DRDY, SCLK, MISO, MOSI, CS, and GND. Force the ADC's analog inputs to AGND (short to ground on the board). The logger should produce stable counts near zero on all four channels. This proves SPI + register configuration in isolation.
3. **Stage 3 — One bridge, MUX held at position 0.** Connect one Wheatstone bridge to the ADC via MUX position 0. The logger should show a stable bridge reading that changes when you mechanically deflect the whisker. This proves the analog signal path for one channel.
4. **Stage 4 — Full system.** Connect the MUX address GPIOs and confirm that walking through MUX positions selects different bridges.

If any stage fails, fix it before moving to the next. Most "this whole system is broken" symptoms are actually a single fault at one of these stages that you can isolate in minutes.

---

## 7. Power-off sequence

1. **Stop the logger** (Ctrl+C). Wait for it to print the final summary line. This flushes the `.dat` files.
2. **Cleanly shut down the Pi:**
   ```bash
   sudo poweroff
   ```
   Wait until the green ACT LED stops blinking (newer firmware does a final ~10-blink burst, then dark). The red PWR LED stays on — it just indicates 5 V is applied, not that the Pi is running.
3. Unplug the Pi's USB-C power.
4. Power off the analog board.
5. Now safe to unplug GPIO wires.

Reversing this order risks (a) corrupting the SD card if a write was in progress and (b) latch-up on the ADC if the analog board is killed while the Pi is still driving signals.

---

## 8. Troubleshooting

### "Could not connect to pigpio daemon"
The pigpio daemon is not running. Start it: `sudo pigpiod` (or `sudo systemctl start pigpiod`).

### "No such file or directory: /dev/spidev0.0"
SPI is disabled in the kernel config. Enable with `sudo raspi-config` → Interface Options → SPI → Enable, then reboot.

### "DRDY did not assert after RESET"
- The ADC is not getting CLKIN. Check that GPIO 4 is putting out 8.192 MHz on the scope. If absent: pigpio daemon not running, or GPIO 4 conflict with 1-Wire (`dtoverlay=w1-gpio` in `/boot/firmware/config.txt`).
- The ADC is not powered. Measure V_DD at the chip.
- The DRDY wire is not connected, or is mis-wired. Measure continuity from Pi physical pin 29 to ADC DRDY pin.

### Readings stuck at zero on all channels
- The buggy CLOCK register write (Bug 1 in the [diagnosis document](diagnosis_logger_32ch.md)) is disabling all four ADC channels. Apply that fix.
- Or all four ADC inputs are at AGND (intentional, see Stage 2 above).

### Readings drift / are noisy
- Bridge excitation is sharing a noisy supply. Move to option 1 in Section 3.
- Self-heating in the strain gauges (current too high). Lower excitation voltage.
- Long wires picking up EMI. Twist pairs; consider shielded cable; keep away from HDMI / power cables.

### "Reinitialising ADC/clock/SPI path" every few seconds
Caused by the periodic-reinit workaround (`REINIT_EVERY_S = 8.0` in config). This was added to work around the symptoms of the bugs documented in [diagnosis_logger_32ch.md](diagnosis_logger_32ch.md). After applying the register-write fixes, you should be able to raise `REINIT_EVERY_S` substantially (or remove the reinit logic).

### Channel readings look like they belong to a different sensor
- MUX address lines mis-wired. Verify per Section 2.
- MUX VDD = 5 V (would cause level-compatibility issues with 3.3 V GPIO). Confirm VDD = 3.3 V at the CD4097B chips.
- MUX INHIBIT pin floating. Should be tied to GND on the board. If floating, the MUX may randomly turn all channels off.

---

## 9. Future improvements (not required for today's operation)

Documented here because they came up during diagnosis and will matter if higher sample rates or better signal quality are needed.

- **Apply the register-write fixes** from [diagnosis_logger_32ch.md](diagnosis_logger_32ch.md). Most impactful single change.
- **Drop bridge excitation to ~1.25 V** using a precision reference (e.g. REF3012). Cuts current draw from ~300 mA to ~115 mA and reduces strain-gauge self-heating by 7×.
- **Buffer each bridge output with a low-offset op-amp** (e.g. OPA2333) before the MUX. Drops the source impedance feeding the MUX from ~100 Ω + filter network to <1 Ω, which lets you shrink the MUX settle time from 2 ms toward microseconds.
- **Replace `time.sleep` MUX delays with DRDY-driven settling** once the analog network is fast enough. The ADC's own throwaway conversions become your settle clock.
- **Increase OSR** if noise is a problem (slower but lower noise; lower OSR is faster but noisier).
- **Use `pigpio.callback()` on DRDY** instead of polling, to remove the OS jitter on the read-timing loop.

---

## 10. Reference documents

- [ADS131M04 reference (this project)](adc_ads131m04_reference.md) — distilled chip facts that matter for this PCB.
- [CD4097B reference (this project)](mux_cd4097b_reference.md) — distilled chip facts that matter for this PCB.
- [ADS131M04 logger code diagnosis](diagnosis_logger_32ch.md) — bug list and fixes for the current Python logger.
- TI datasheets:
  - ADS131M04 (SBAS890D): <https://www.ti.com/lit/ds/symlink/ads131m04.pdf>
  - CD4097B (SCHS052D): <https://www.ti.com/lit/ds/symlink/cd4097b.pdf>
- Raspberry Pi 4B GPIO pinout: <https://pinout.xyz/>
