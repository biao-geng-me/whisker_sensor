# BlueROV2 whisker array — vehicle bring-up and first findings (2026-10-15/16)

Phase 1 of the BlueROV2 integration is to record the 32-channel whisker array and the ArduSub IMU on one time base. The goal is to test whether the array carries IMU information (black-box map: whisker array → IMU outputs). Whisker → vehicle motion (velocity) is a later step; there is no DVL yet. This note records what was brought up on the real vehicle, what the first recording showed, and what is still open. How-to details live in [`rpi/README.md`](../../rpi/README.md) (server, endpoint, flashing) and [`msd/esp32/README.md`](../esp32/README.md) (firmware, frame format, signal chain).

## Data path (working on the vehicle)

ESP32 (`whisker_daq` firmware, CP2102 USB-UART at `/dev/ttyUSB0`, 921600 baud, binary frames with ESP32 `micros()` timestamps) → BlueOS Pi (`rpi/daq_server.py`, which also receives ArduSub MAVLink on UDP 14670) → recording on the Pi (`~/whisker_rec/*.jsonl`) and TCP :5600 → `wavi_py --transport server` on the laptop, which keeps a local `.jsonl` + `.dat` copy. `python -m whisker_link.convert file.jsonl --mat` puts every source on the Pi's clock.

Bring-up gotchas, all fixed or documented:

- The BlueOS bridget serial bridge holds `/dev/ttyUSB0`; delete it first.
- UDP 14660 is a protected BlueOS endpoint ("Ping360 Heading"), so daq_server uses 14670 with a BlueOS **UDP Client** endpoint to `127.0.0.1:14670`.
- Flashing over the tether works with esptool on the Pi (`rpi/scripts/flash_esp32.sh`). DTR/RTS auto-reset works through the CP2102, so no BOOT button is needed.
- The Pi copy of the code is rsynced (`rpi/scripts/sync.sh pi@192.168.2.2`), not git-pulled; the Pi has no internet.
- Starting Cockpit used to drop the wavi_py client: a 0.5 s socket timeout cut the server's sends whenever the laptop stalled. That is fixed, and wavi_py now notices a lost link and says so.

## First recording (60 s, 2026-10-16 11:41, vehicle on the bench)

- **Rates:** whisker 82.35 Hz after raising the ADC SPI clock from 200 to 600 kHz. At 200 kHz an 18-byte ADC read took 720 µs, longer than the 320 µs sample period, which capped the rate at ~33 Hz. RAW_IMU arrives at exactly 100 Hz, ATTITUDE at 50 Hz. Zero scan gaps.
- **The two copies agree:** the Pi copy and the laptop copy hold the same events except a couple at the start and stop edges.
- **Clocks:** the ESP32 clock runs ~+1030 ppm against the Pi, far outside a normal crystal; the MAVLink sources are within 30 ppm. `convert`'s per-source clock fit removes it. USB delivery adds 11.5 ms median (p99 19 ms) of latency, also removed by the fit. wavi_py measures the whisker rate from the ESP32 timestamps rather than from arrival times, which are bursty.
- **ADC CRC retries:** 22 in 60 s at 600 kHz, spread evenly, and none of them corrupts data (each failed read is re-read). The PCB–ESP32 ground was a loose jumper wire. Fix the ground and recheck the `adc-crc` count.
- **Signals:** only W4_B had a gauge connected. Hand bending shows as held shifts of 10–15 mV at the ADC (≈ 600–900 µε at gain 10) on a 26 mV rest, with ≈ 0.5 mV sample noise. W3_B and W8_B are not railed but noisy (≈ 550 mV sample-to-sample); they look like floating inputs with a hardware fault.
- **7.5 Hz interference:** W4_B dips by about −7 mV and then −4 mV every 11 scans (134 ms), with harmonics at 15 and 22.5 Hz. The dip is partly common to W3_B and W8_B, so it is board supply or ground (or REF, see below), not the gauge.
- **IMU:** `SCALED_IMU2` has all-zero accel/gyro on this vehicle (magnetometer only), so it is no longer requested. The vehicle sat inverted on the bench (roll ≈ ±180°, z-accel ≈ +1 g). `RC_CHANNELS` is now requested at 20 Hz for pilot inputs.
- **Size:** ≈ 8.5 MB/min as JSON lines (≈ 0.5 GB/h); gzip cuts it about 7×.

## PCB findings from the MSD design files

Source: `/shared/rc/whiskers/MSD_P25937/P25937_Whisker_Final.zip` (Altium). The `.SchDoc` sheets were read with a small parser, kept at `/scratch/bigeme/whisker_dev/ana/schdoc.py` (needs `olefile`). The Gerbers in `REV2_GERBER.zip` carry no netlist.

- **Amplifier gain is 10.** Every INA351ABS (SOT-23-8: GS 1, IN− 2, IN+ 3, V− 4, REF 5, OUT 6, V+ 7, SHDN 8) has GS tied to GND. The "gain ~141" in the inherited firmware was wrong.
- **Quarter bridge at 3.3 V:** the gauge runs from 3V3 to StrainPOS, with 352 Ω from StrainPOS to GND. The reference half is 123 Ω + 123 Ω (≈ 1.65 V). Tension lowers the reading. Strain ≈ −60.6 µε per mV at the ADC (GF 2, V_ex 3.3 V), after subtracting the resting offset.
- **Supply:** 5 V enters through the USB-A jack and a TPSM84203 buck module makes 3.3 V. The bridges draw ≈ 18 mA per channel, ≈ 0.6 A in total.
- **Resolved — REF offset:** the schematic ties every INA351 REF to a 100 k/100 k divider (≈ 1.65 V, net `AMP_REF_3V3`) while all ADC AINxN are at GND, which would rail every channel. Fix on the board: R8 replaced with 10 k (R7 measured ≈ 46 k), REF now 0.29 V, so negative bridge outputs no longer clip at 0 V. The measured REF does not match the divider calculation (an extra ~8–13 k load is unexplained); that discrepancy is parked.

## Phase-1 study design (discussed)

- **Targets:** choose ones the whiskers can in principle see: linear acceleration, angular rates, roll and pitch. Absolute yaw is not observable from the whiskers.
- **Inputs:** whisker channels only. Pilot and thruster signals (`RC_CHANNELS`, `SERVO_OUTPUT_RAW`) are for analysis and labeling, not model inputs, or the model can bypass the whiskers.
- **Thruster-wash control:** run the thrusters with the vehicle held still. A whisker response there is wash, not motion.
- **Drift:** quarter bridges have no temperature compensation. Record a rest baseline at the start and end of every run, and include water temperature (`SCALED_PRESSURE2`). Split train and test by session.
- **Amount:** about 6–10 sessions × 15–20 min (structured steps per axis at 2–3 levels, holds, free piloting), roughly 2–3 h in total.
- **Time alignment first:** tap test on a connected whisker together with the frame, to measure whisker↔IMU alignment. Not done yet.

## Next steps

1. ~~Meter the REF/AINxN question above~~ — done: R8 replaced, REF 0.29 V.
2. ~~Firm up the PCB–ESP32 ground~~ — done: one GND wire bundled with P1 (adc_crc 0 on the ROV, 2026-10-28).
3. Tap test with a connected whisker for time alignment.
4. Optional tooling: gzip recordings on the fly, a Mark button in wavi_py (the server already has a `mark` command). The daq_server systemd service (`whisker-daq`) is done.
