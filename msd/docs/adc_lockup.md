# ADS131M04 lock-up: evidence so far (parked 2026-10-26)

Status: parked. It has been seen once, the trigger is unknown, and this board has no fix. The firmware and host now detect and report it. Reopen this if it happens again, and add every new occurrence to the log below.

## Symptom

The ADC stops converting, and no software recovery brings it back. The firmware's line is `[halt #N] UNRECOVERED  ID=0x2405 CLOCKrb=0x00FF crcbad=0  -> power-cycle the PCB with the ESP32 already running`. The device ID reads back correctly and there are no SPI CRC errors, so the SPI link works and the ADC answers register reads. But the CLOCK register reads 0x00FF instead of the written value (e.g. 0x0F0A for quiet). Its channel-enable bits read 0, and the write doesn't take. The firmware's recovery ladder (soft re-init; then SPI flush + re-init; then CLKIN restart + re-init) doesn't clear it. Only removing the PCB's power does.

## Occurrence log

| # | Date | Firmware | Context | What was seen | Cleared by |
|---|---|---|---|---|---|
| 1 | 2026-10-23 ~18:20 | 6aeb4ef (dual-core, round 5 sequence) | Bench, PCB on a bench supply, ESP32 on laptop USB. It came after a run of "unplug/replug the ESP32's USB while the PCB stays powered" fresh-boot steps; which replug triggered it is unknown. | `[halt #11] UNRECOVERED  ID=0x2405 CLOCKrb=0x00FF crcbad=0` on the console. It came from the legacy/text-mode scan path (the locked reader of that firmware never declared UNRECOVERED), so the ADC was probably stuck before streaming started. No capture holds it; the next run, `quiet_c0.bin`, was clean after a PCB power cycle. | PCB supply off/on with the ESP32 running |

The ROV bring-up on 2026-10-15/16 (msd/docs/bluerov_bringup_2026-10.md) powered the PCB from the vehicle 5 V and the ESP32 from the Pi USB, and streamed with zero scan gaps; no lock-up is recorded there. That was firmware before the halt/UNRECOVERED reporting, so a lock-up would have shown only as missing scans.

## What is known

- **No hardware reset on this board.** The schematic (MAIN.SchDoc in /shared/rc/whiskers/MSD_P25937/, traced with /scratch/bigeme/whisker_dev/sch/net.py) shows U4 (ADS131M04, TSSOP-20) pin 11 SYNC/RESET tied straight to the 3V3 net. AVDD (pin 1) and DVDD (pin 20) share that 3V3 with the bridge excitation. CS → PIN27, CLKIN (pin 17) → PIN07, DRDY (pin 13) → PIN29. The headers carry no reset line, and GPIO17 (`PIN_RESET` in daq_common.h, `HAVE_HW_RESET 0`) connects to nothing.
- **The datasheet sets no power-order requirement.** The ADS131M04 datasheet (SBAS890D §8.4) says the power-on reset completes within 250 µs and the ADC converts whenever CLKIN toggles. "Power the PCB after the ESP32" is the MSD team's empirical workaround, not a documented requirement.
- **It is not just a missing clock.** In bench round 7 (2026-10-26, firmware 2e9d8dc, /scratch/bigeme/whisker_dev/bench_1023/r7/r7_clkin_pull.bin), pulling the CLKIN jumper while streaming quiet stopped all conversions (0 DRDY/s, 5 halts, then UNRECOVERED and retries every 5 s). But CLOCK still read back 0x0F0A, the configured value, and the ADC recovered by itself at the first retry after the jumper went back. The real lock-up reads 0x00FF and needs a power cycle, so the ADC's register/digital state is corrupted, not merely unclocked.
- **A software RESET command doesn't help** in the observed state (the ADC answers reads but ignores the CLOCK write).

## Hypotheses (none confirmed)

1. Back-powering through the logic lines. When the ESP32 reboots or is unplugged while the PCB is powered, the SPI, CS and CLKIN lines float or go low and the ADC sees its clock stop. The reverse case, an unpowered ADC with driven inputs, injects current through its ESD diodes.
2. A brief dip or brown-out on 3V3: on the ROV, the 5 V rail under thruster load.
3. A latch-up from a transient on the shared 3V3.

## Tests to run when it is reopened

1. **Power-order trials,** about 5 boots each. After each boot, check the banner's `CLOCK readback = … (config ACCEPTED)` and the beats' `boot_clock_rb` / BOOT_REC_FAIL flag. The orders: PCB first, then the ESP32; ESP32 first, then the PCB; both on one switch (closest to the ROV); and replugging the ESP32 USB with the PCB on (closest to a mid-dive Pi/ESP32 reboot).
2. **Reset-pin test during a real lock-up.** Briefly short U4 pin 11 to GND and release it, then see whether the next retry gives `[adc ok]`. If it does: lift pin 11 (or cut its trace), add a 10 kΩ pull-up to 3V3, wire it to ESP32 GPIO17, and set `HAVE_HW_RESET 1`. The firmware's last recovery step then pulses it. If it doesn't, the fix is an ESP32-controlled load switch on the PCB 5 V feed (~0.6 A).
3. **Supply dip:** briefly sag the bench supply while streaming to see whether a dip reproduces it.

## Detection and reporting (in place)

- **Firmware 2e9d8dc:** all modes declare `[halt #N] UNRECOVERED` after 5 failed recoveries, then retry every 5 s. A 1 Hz BEAT heartbeat carries DRDY edges per second, halt/unrecovered counts, CLOCK readback, reset reason, boot CLOCK readback, and flags ADC_DOWN / BOOT_RECOVERY / BOOT_REC_FAIL. Details are in msd/esp32/README.md ("Fault reporting").
- **Host:** whisker_link/esp32_fault.py (shared by rpi/daq_server.py and wavi_py) raises a latched `adc_unrecovered` / `adc_not_converting` fault. wavi_py shows a red banner and a Fault log, and writes `<recording>.faults.txt`. daq_server reports `esp32.fault` and `esp32.beat` in its 1 Hz status.
- **Simulation:** host_sim `--adc-dead T[:T2]`; fake_esp32 SIGUSR2 / `--adc-dead`; and the wavi Sim panel's "ADC lock-up" / "Power-cycle PCB" buttons.

## Respin

msd/docs/daq_respin_spec.html requires SYNC/RESET wired to an ESP32 GPIO (with a pull-up) on the next board.
