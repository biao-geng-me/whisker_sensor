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

Other frames: TEXT (framed log lines), INFO (on `B`), HEALTH (1 Hz while a locked mode streams) and BEAT (the 1 Hz heartbeat in every mode, see "Fault reporting" below). Decoders skip frame types they don't know, and `esp32_frame.py` decodes INFO / HEALTH / BEAT payloads longer than it knows as far as it knows them, so appending fields doesn't break it.

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

**2. Fine mux settle diagnostic** (firmware `D<from> <to> [clkin] [osr] [spi] [reps] [window_us] [dither] [dwell_us] [jitter_us];`, defaults 8 MHz, OSR 256, SPI 4 MHz, 64 reps, 2000 µs, dither 4, 3000 µs, 1000 µs). Per rep: dwell on `<from>` for `dwell_us` plus a random 0…`jitter_us` (with a fixed spacing, the board's 1 kHz line, which divides 3000 µs exactly, added up coherently across reps and showed as ±0.05–0.12 mV bumps after 1 ms), reading every conversion, wait a fraction (rep mod dither)/dither of a conversion period, so that over the reps the switch lands at different phases, then switch to `<to>` and read every conversion for the window. It dumps every conversion (`rep t_us flags c0 c1 c2 c3`, `t_us` = DRDY edge minus the switch) with a CRC. The expected limit is charge sharing, not the mux: the COM node (~30 pF) dumps the previous channel's voltage into the new channel's 1 k / 0.1 µF filter, a kick of ~3e-4 × the voltage difference that decays with τ ≈ 100 µs. So switch from a select whose channels are railed (nothing plugged in) to one with gauges. With the planned 12 whiskers, select 6 carries only uninstalled slots (OUT27/19/6/16 = W14_A, W10_A, W3_B, W8_B), and select 3 only installed ones (OUT26/24/1/9 = W13_B, W12_B, W1_A, W5_A).

```powershell
python adc_diag.py settle --port COM5 --from 6 --to 3 --spi <best safe SPI from step 1> --out settle_6to3
python adc_diag.py settle --port COM5 --from 6 --to 3 --osr 128 --spi <best safe SPI> --out settle_6to3_osr128
python adc_diag.py analyze settle_6to3.npz          # offline, e.g. on the cluster
```

The analysis fits each channel that has a step with an analog model, seen through the sinc3 of the same OSR and CLKIN (computed): a delay d, a fast first-order response carrying the whole step (τ_a), and a slow tail A2·exp(−u/τ2) with |A2| bounded to 5 % of the step. The weights include DRDY timestamp jitter (`--sigma-t-us`, default 2 µs), which dominates the error while the filtered signal moves by mV/µs. It prints d, τ_a, A2, A2/step and τ2. It gives the analog time after the switch when the slow tail and the fast part fall below 3e-4 and 1e-4 of the step, and when a conversion must end for its whole sinc3 window to come after that. It also prints a residual table by time window (150–250, 250–350, 350–500, 500–700, 700–1000, 1000–1400, 1400–2000 µs): data minus the fast-only model, mean ± sem, beside the slow-tail fit. It writes `<out>.txt` (raw), `.npz`, `.html` and, with matplotlib, `.png`. Conversions flagged CRC, stale, overrun or edgeless are dropped, and their count is listed.

Bench (2026-10-22, CH2 only, steps ~±900–960 mV, captures before the rep jitter): d ≈ 14–26 µs effective delay (τ_a not resolved from d at 64 µs / 32 µs per conversion), slow tail A2/step ≈ 1–6e-3 with τ2 ≈ 70–90 µs (140 µs on the noisier 0→2 OSR 256 run). The slow tail is below 1e-4 of the step 270–360 µs after the switch, so a conversion should end ≳ 460–550 µs after it at OSR 256 (3 × 64 µs window) or ≳ 410 µs at OSR 128.

**3. Active selects** (firmware `S<digits>;`, `S;` = all 8). The normal scan then visits only those selects, so with `S0123457;` a scan takes 7/8 of the time. Outputs of a skipped select hold +full scale, so they are railed in the SCAN railmask and show as "open" in wavi_py and old decoders. INFO (`PROTO_VERSION` 3) carries `sel_mask`, and the status record shows `esp32.cfg.selects`, e.g. `"0123457"`. wavi_py adds `| selects 0123457` to its status line. The mask is kept in RAM only, so the ESP32 is back at all 8 after a reboot.

```powershell
python adc_diag.py selects --port COM5 0123457      # 'all' to undo
```

On the ROV, `rpi/daq_server.py --selects 0123457` sends `S0123457;` before every `B`, so the list survives ESP32 reboots. In a serial monitor, type `S0123457;` directly. In a PowerShell one-liner that writes to the port, quote the string, because `;` separates PowerShell statements.

## Locked scan modes: quiet 62.5 Hz, wide 250 Hz (2026-10-22)

Most of the remaining noise is a comb at exact multiples of 62.5 Hz (16 ms, ESP32/USB activity), plus a small 845 Hz line. CLKIN comes from the ESP32 crystal, so a scan that is exactly N conversions of the free-running ADC, with N × T a multiple of 16 ms, folds the whole comb onto 0 Hz, a constant offset per channel. The simulation (`/scratch/bigeme/whisker_dev/sim/sim.py`) gives ~0.012 mV for the quiet mode against ~0.055 mV now. Since 2026-10-23 quiet (`M1`, OSR 512) is the boot default. The legacy ~76 Hz scan stays available as `M0;`.

| mode | command | CLKIN | OSR (T) | SPI | N per scan | scan | conversions averaged per select (7 selects) |
|---|---|---|---|---|---|---|---|
| legacy | `M0;` | 0.8 MHz | 128 (320 µs) | 600 kHz | – | ~13 ms, ~76 Hz | 1 |
| quiet (boot default; OSR 512 since 2026-10-23) | `M1;` | 8 MHz | 512 (128 µs) | 8 MHz | 125 | 16 ms, 62.5 Hz | 13–14 (drop 4) |
| quiet, OSR 256 | `M1 200 256;` | 8 MHz | 256 (64 µs) | 8 MHz | 250 | 16 ms, 62.5 Hz | 29–30 (drop 6) |
| quiet, OSR 256, 125 Hz (bench comparison) | `M1 200 256 8000;` | 8 MHz | 256 (64 µs) | 8 MHz | 125 | 8 ms, 125 Hz | 11–12 (drop 6) |
| wide | `M2;` | 8 MHz | 128 (32 µs) | 12 MHz (11.43 actual) | 125 | 4 ms, 250 Hz | 8–9 (drop 9) |

The command is `M<mode> [settle_us] [osr] [period_us] [spi_hz];`, where 0 or a missing field means the mode's default. Before 2026-10-23 the fourth field was `spi_hz`. A quiet period must be a multiple of 8000 µs (the comb sits at multiples of 125 Hz, so 8 ms folds it too) and a whole number of conversions, so 8000 µs needs OSR ≤ 256 and `M1 200 512 8000;` is refused. Wide mode is fixed at 4000 µs.

How it works:
- **Slots.** Each scan is N conversions split over the active selects: slots get floor or ceil of N/nsel, and the total is exactly N. Scan k starts on conversion k·N.
- **Mux switching.** The mux switches right after the DRDY that ends the previous slot, by direct GPIO register writes. `digitalWrite` × 6 took ~10 µs.
- **Averaging.** A slot's reading is the plain mean of the conversions whose sinc3 window (3 T) starts at least `settle_us` (default 200 µs, plus 10 µs allowed for the switch latency) after the switch. The same positions are dropped in every slot of every scan.
- **Crosstalk.** With the measured tail (τ 70–90 µs, ~0.0136 × the step at the switch), crosstalk comes out at ~2e-5 (quiet) to ~1e-4 (wide) of the step.

Every conversion is still read. Its index on the grid comes from the DRDY edge time (ISR `micros()`, same crystal), so a lost DRDY pulse or a late read can't shift the scan.

The output is the usual 32-channel SCAN frame with the averaged codes:
- `t_us` is the grid time of the switch to slot 0, so scans are exactly 16000 or 4000 µs apart.
- `seq` counts grid scans, so a skipped scan shows as a seq gap.
- Skipped selects read railed, as before.

INFO (`PROTO_VERSION` 4) carries the mode, CLKIN, OSR, SPI, N, settle, discard count, selects and conversions averaged per select. `daq_server.py` and the wavi_py esp32 transport put these in the status record as `esp32.cfg`, e.g. `mode_name`, `scan_hz` and `avg_per_sel`.

Health:
- **Missed conversions.** If conversions are missed (a late read, an interrupt hog), the affected slot is not used and the scan gets SCAN flag bit4 `SYNC`. A slot left without usable conversions keeps its previous value and sets bit5 `EMPTY`. After a pause longer than one scan (text mode, a command), the reader drops the FIFO and restarts at the next scan boundary of the same grid.
- **ADC faults.** An ADC reset (STATUS RESET) or a 50 ms DRDY timeout re-initialises the locked mode.
- **HEALTH frame.** In the locked modes the firmware sends a HEALTH frame (type 4) every ~1 s with:
  - busy % (CPU time in the reader and sender, without the ISR)
  - DRDY-edge-to-end-of-read latency, average and max
  - SPI read time, average and max
  - TX write max
  - conversions, overruns, missed conversions, edgeless reads (DRDY pulse blocked by a read), CRC errors, stale reads, late switches, TX drops, empty slots, resyncs, re-inits and recoveries
- **Where it shows.** The status record carries it as `esp32.health`. The daq_server log and the wavi_py status line show busy %, max latency and overruns.

CPU: see "Reader core: C0 / C1" below. In C1 (the boot default) the reader streams on core 1 and core 0 sends; in C0 (fallback) the reader also sends. Frames leave through a TX queue, written in small chunks only when the next DRDY is ≥ 6 µs away, the next conversion doesn't end a slot, and the UART FIFO is nearly empty.

What to watch on the bench:
- **Latency in wide mode.** `lat max` must stay below the 32 µs conversion period. An 18-byte read at 11.4 MHz is ~13 µs of clocking plus the driver and ISR overhead.
- **Errors.** `overrun`, `missed`, `late_sw` and `empty` should stay at 0. A few `edgeless` per second mean reads are finishing just as the next conversion lands. That's harmless, but it means the read path is close to the limit.
- **CPU and TX.** `busy` is the CPU share. `tx_drop` > 0 means frames didn't get idle time to leave.
- **Flags and gaps.** SCAN flags `sync` and seq gaps in the `stream` summary.

```powershell
cd C:\Users\Biao\Projects\whisker_sensor\msd\esp32
# wide: set selects + mode, stream 20 s, print HEALTH each second and a summary
python adc_diag.py stream --port COM4 --selects 0123457 --mode wide --secs 20 --out wide.bin
# quiet (OSR 512), the OSR 256 variant, and OSR 256 at 125 Hz
python adc_diag.py stream --port COM4 --selects 0123457 --mode quiet --secs 20 --out quiet512.bin
python adc_diag.py stream --port COM4 --mode quiet --osr 256 --secs 20 --out quiet256.bin
python adc_diag.py stream --port COM4 --mode quiet --osr 256 --period-us 8000 --secs 20 --out quiet125.bin
# leave a mode set, then look at it in wavi_py (the ESP32 keeps it until a reboot)
python adc_diag.py mode --port COM4 wide
cd ..\..
python -m wavi_py --transport esp32 --serial-port COM4
# back to the legacy scan
python msd\esp32\adc_diag.py mode --port COM4 legacy
```

On the ROV, `rpi/daq_server.py --selects 0123457 --mode wide` (or `--mode quiet`, `--mode quiet,200,256`, `--mode quiet,200,256,8000`) sends `S…;M…;B` on every stream start. In a serial monitor, type `M2;` etc. directly.

Host simulation: [`host_sim/`](host_sim/) compiles the real sketch (arduino-cli `--preprocess` output of the `.ino`, plus the sketch's `.cpp` files in the same unit) against a mock Arduino layer and a model of the ADS131M04. The model covers the conversion grid, the 2-deep FIFO, DRDY edges and blocked pulses, the narrow DRDY pulse at the end of a read that leaves data in the FIFO (`--no-pulse` turns it off), the mux tail, and SPI frames with CRC. `python -m pytest msd/esp32/host_sim` (needs arduino-cli and a g++, e.g. `CXX=~/app/miniconda/envs/mpi/bin/g++` on the cluster) checks:
- both modes: exact scan spacing, averages and no flags
- a 300 µs stall: one SYNC-flagged scan, grid kept
- an overrun in quiet OSR 256, OSR 512 and wide at several phases: at most one flagged scan, and `resync` doesn't keep counting afterwards (the 2026-10-23 bug)
- an ADC reset
- switching back to legacy

- quiet OSR 256/512/125 Hz with `--board-costs`: no overrun (the UART pacing fix)
- C1: wide with board costs at scale 1.0 and 1.5 (free and windowed TX), quiet windowed, a stall on each core, a full queue (HEALTH still sent), an ADC reset, S/M/text/legacy command spam, C0 ↔ C1 switching
- C0 after C1 without a reboot: no stale/edgeless reads (bench r3)
- ESP-IDF log lines while streaming (C0, C1, windowed) arrive framed and rate-limited; every run checks there are no task-watchdog errors, no raw log lines and (where it streams only) no bytes outside frames
- `test_xcore_tsan.py`: the queue and handshake of `lk_xcore.h` with two real threads under ThreadSanitizer

The sim runs each FreeRTOS task as a thread, one at a time, on two virtual cores. Each core has a clock; the earliest core runs next, and device events and ISRs are processed in time order (see the top of `sim.cpp`). `--stall T:US[:CORE]` stalls one core (default 1, the streaming reader's). `--idf-log T:TEXT` makes an ESP-IDF component log a line. A fake task watchdog counts resets from unsubscribed tasks. `--board-costs` sets the Serial and UART-ISR costs calibrated to the bench HEALTH data: tx max 57–59 µs, lat max ~73 µs at OSR 256, plus an 8 µs tick (it reproduces bench r3). Other CPU costs are guesses, so the busy % and latency the sim reports are only indicative. The bench HEALTH numbers are the real ones.

## Reader core: C0 / C1 (2026-10-23, bring-up)

While a locked mode streams, the reader runs as its own task on core 1, next to the DRDY ISR. `C1;` (the boot default since round 5) has it hand the records to core 0, which sends them. `C0;` (runtime fallback) has it send its own frames through the paced TX queue, which is the single-core firmware's execution path. Wide mode needs C1: on one core, Serial writes of up to ~80 µs can't fit in T = 32 µs. `C1 <us>;` additionally holds each scan frame until `<us>` after its scan start (windowed TX, 100 µs up to the scan period), so serial activity stays at a fixed phase of the scan. The setting is RAM only. C1 became the default after round 5: quiet noise was 16/16 µV against 35/32 µV in C0. Windowed TX (18/19 µV) gave no gain over free TX.

Layout:
- **Core 1.** The reader task (priority 23, `lk_reader.cpp`) and the DRDY ISR (`setup()` attaches it on core 1).
- **Core 0.** The host task, which runs everything that used to be `loop()` except streaming a locked mode: commands, text modes, the legacy scan, and the sending in C1. `Serial.begin` runs there, so the UART ISR is on core 0. Core 0's idle task is taken off the task watchdog, because long diagnostics (`p`, `D`, `r`) run there without yielding, as they did on the unwatched core 1 before.
- **Handover.** In C1, core 1 hands records (scan, HEALTH, message) to core 0 through a single-producer/single-consumer queue (`lk_xcore.h`, 32 slots). The reader makes no FreeRTOS calls while it streams; core 0 polls the queue every 1 ms tick. With windowed TX, core 0 is awake and polling when the next record is due. The reader's hot path (`lkStep`, `lkCloseScan`, `adsReadFast`, `lkHealthReset`) is in IRAM.
- **Ownership.** Every command first parks the reader and takes it back to the host core (the park/run handshake in `lk_xcore.h`, with generation numbers). Streaming hands it to core 1 again. While the reader runs, it alone touches SPI, CS, the mux and CLKIN. The host holds the Arduino SPI transaction throughout, so the reader's transfers take the no-lock path and it never changes the SPI clock.
- **Watchdog.** The reader task is not on the ESP-IDF task watchdog. The host watches its heartbeat (`g_lkBeat`) and restarts the ESP32 if it stops for 3 s while it should run.
- **ESP-IDF logs.** These would otherwise go raw into UART0, past the Serial driver. A `vprintf` hook captures them, and they go out as framed TEXT `[idf] ...`, at most 10 lines/s with a drop count. `Serial.setDebugOutput(false)` turns off ROM/Arduino debug prints. A crash dump still prints raw.

HEALTH version 2 appends the following fields to the version-1 frame, in a format a version-1 decoder can still read:
- `core`
- `q_hwm`: the queue's high-water mark
- `tx_win_us`
- `parks` and `park_timeouts`: cumulative since boot
- `host_tx_max_us`: the longest frame write on core 0
- `det_max_us`: DRDY edge to the start of the read
- `tx_late`: windowed frames sent > 200 µs late

`adc_diag.py stream` prints them and a totals line.

Bench round 3 (2026-10-23, firmware 158c314, captures in `/scratch/bigeme/whisker_dev/bench_1023/r3/`) and the fixes:
- **Raw log lines.** `E (…) task_wdt: esp_task_wdt_reset(707): task not found` lines appeared in the stream. The reader task called `esp_task_wdt_reset()` at every scan end without being subscribed (`esp_task_wdt_add`'s return value wasn't checked, so why it wasn't subscribed is unknown). Each failure printed a ~60-byte line synchronously from core 1, ~0.65 ms of UART time. In C1 wide that starved the reader (overruns, extra scan ends, more resets), saturating at the line rate of ~1 line/ms, which is why C1 gave no usable output. The C0 captures start with the tail of that backlog (timestamps 292 s and 423 s after boot; no reboot). Fix: no task watchdog in the reader; the log hook above.
- **C0 stale reads.** C0 had one stale + edgeless + resync per LCM(1 ms tick, T): 63/s at OSR 512, 126/s at OSR 256. 158c314 ran C0 on core 0 while the DRDY ISR sat on core 1. When core 1's FreeRTOS tick (~8 µs) delayed the ISR, the reader saw DRDY low with no edge counted after its 3 µs wait, claimed the conversion edgeless, and the late edge then gave a resync and a stale re-read. Fix: C0 streams on the reader task on core 1 again. When the host core reads (text modes, baseline), the edgeless wait is 15 µs. host_sim reproduces it: ISRs of one core don't nest, the cores' ticks aren't aligned, and `--board-costs` has an 8 µs tick. The previous firmware gives exactly 63 per HEALTH frame there; the fix gives 0.

Bench round 4 (2026-10-23, firmware 6d90c76, `/scratch/bigeme/whisker_dev/bench_1023/r4/`): no locked run ever streamed. Every run gave one INFO, then `[reader watchdog]` and a reboot, after which the board showed the text grid. The reader was fine; the watchdog was wrong. Its "last progress" time started at 0 (boot) instead of at the RUN request, and it is only checked while streaming. A first `B` more than 3 s after boot therefore tripped it on the first check, before the reader had run one step. After a software restart, the stress loop's next re-send came < 3 s after boot, so that path happened to work. host_sim missed it because every test started streaming at ~1.3 s uptime; the tests now send `adc_diag`'s exact bytes at 5 s, and 6d90c76 fails there exactly as on the board.

Fixes in this round:
- **Watchdog.** It is armed per RUN request. Before restarting it sends a framed TEXT with the reader state: handshake request/state, flags, conversions, time since the last step, DRDY edges, heartbeat. A note survives the restart (RTC memory) and is printed after the boot banner as `previous reset: ...`, together with `reset reason N`.
- **Task creation.** Creating either task is checked: a missing reader task is reported, and locked modes then stream from the host core. A missing host task runs in setup's task.
- **HEALTH and commands.** Each command parks the reader. The host now sends the HEALTH window streamed so far when it takes the reader back, so HEALTH keeps coming when commands arrive more often than once a second (the r4 stress run had none).
- **Seq numbering.** In the r4 stress run each re-send of `M2;` re-initialised the ADC and started a new conversion grid. `seq` counts scans of the current grid, so a re-init shows as a `t_us` jump (~67 ms there), not as a seq gap.
- **Mux-switch allowance.** `LK_LAT_ALLOW_US` is now 20 µs (was 10). In wide mode, core 1's ~8–12 µs tick hits the same conversion positions every 4 ms scan. If that lands on a slot end, every scan's switch was "late". With settle 200 the discard counts don't change (wide 9, OSR 256 6, OSR 512 4). host_sim sweeps the tick phase (`--tick-phase`).
- **Scan flags.** They no longer carry over into the first streamed scan.
- **adc_diag stream.** It now fails loudly: it stops with an error if no SCAN arrives within 2 s of B, and exits non-zero (3) if a reboot banner, `[reader watchdog]`, a crash line or a park timeout appears.

Bench round 5 (2026-10-23, firmware 6aeb4ef, `r5/`): everything streamed, with link CRC 0, tx_drop 0 and HEALTH during stress. Findings and fixes:
- **Wide C1: one overrun per second.** 10 min gave overrun 601, lat max 47–54 µs. Every flagged scan was the first scan after a HEALTH frame (597 of 600, spaced exactly 1.004 s, the HEALTH window). In the stress run HEALTH came from core 0 (the partial window at each park), and there were no such overruns. So the cause was the reader's own once-a-second HEALTH work right after a scan end. That work did a ~100 B copy, a queued record and a second `xTaskNotifyGive` right after the scan record's (the only kernel call there, cross-core while core 0 was waking up). Fix: the reader makes no kernel calls at all while streaming, and its hot path is in IRAM. host_sim checks that the streaming reader makes 0 FreeRTOS calls. It does not model kernel-lock contention, so the overrun itself is not reproduced there.
- **C0 lines at multiples of 3.906 Hz.** These are a 16-scan periodic pattern: folding by scan index mod 16 gives ±60 µV on W2_A/W2_B, and removing it leaves 16 µV, the C1 level. It is in every r5 C0 run and in none before: r2 (69094c2) and r3 (158c314, which already had the UART pacing) are flat. The only 16-scan-periodic change was this round's C0 loop. It called `lkService(2000)`, which returned every 16 conversions (2048 µs), and LCM(2048 µs, 16 ms) = 256 ms = 16 scans. The C0 loop now comes back after every conversion (period T, locked to the scan). The sim's TX-coupling model (`--tx-couple`) can't show this re-entry effect, so the fix is based on the bench correlation and the arithmetic.
- **The silent missing frame** (`quiet_c0_ab`, seq gap 1). The firmware sent it. On the link, the frame's two sync bytes were replaced by 64 × 0xFF (one USB packet), so the decoder skipped its 116 bytes as loose bytes (`n_junk` 180; CRC failures only count false syncs). `adc_diag stream` now warns about bytes outside frames. The audit of the firmware's drop paths found one uncounted case: a C0 scan closed but not yet packed (`pendingSend`) was discarded when a command parked the reader or the reader re-armed. It is now sent (or counted as `tx_drop` when it can't be). The others were already counted: queue full, the previous scan never got idle time, TX queue full, and the C1 queue full. HEALTH is never dropped. Seq skips from re-arming on the same grid are counted as `resync`.

Bench round 6 (2026-10-23, firmware 9e57703, `r6/`, analysis `ana2.py`): the whole sequence below passed. Overrun, missed, empty, stale, edgeless, crc, tx_drop and park_timeouts were 0 in every run. Wide C1 for 10 min: 149945 scans, 0 flagged, lat max 45 µs, det max 27 µs, busy 59 %. Stress: 0 flagged, HEALTH 59 of 60 s. The C0 3.906 Hz lines are gone; with 4 live gauges the floor above 2 Hz is ~2 µV/√Hz in C0 and C1, and the std differences between runs are sub-Hz wander of the real sensors. What is left is on the link: the decoder skipped 302–2560 bytes outside frames per run, and the 10 min run had one seq gap (one 8000 µs step) with no firmware counter, the USB-side corruption seen in r5.

Board test sequence (round 6). Power-cycle (or press EN) where it says so, and wait > 3 s after boot. A step is only valid if `adc_diag` exits 0 with no `ERROR:` lines:

```powershell
cd C:\Users\Biao\Projects\whisker_sensor\msd\esp32
# flash (from the shared folder copy, or build: arduino-cli compile --fqbn esp32:esp32:esp32 --upload -p COM4 whisker_daq)
python -m esptool --chip esp32 --port COM4 --baud 921600 write_flash 0x0 whisker_daq_<commit>_quietboot_c1.merged.bin
# 1. power-cycle first: quiet, boot default (C1), 30 s
python adc_diag.py stream --port COM4 --selects 0123457 --secs 30 --out r6_quiet_default.bin
# 2. no power-cycle needed: wide C1, 20 s
python adc_diag.py stream --port COM4 --selects 0123457 --core 1 --mode wide --secs 20 --out r6_wide_c1.bin
# 3. power-cycle first: wide C1, 10 min
python adc_diag.py stream --port COM4 --selects 0123457 --core 1 --mode wide --secs 600 --out r6_wide_c1_10min.bin
# 4. no power-cycle needed: quiet C0, 30 s
python adc_diag.py stream --port COM4 --selects 0123457 --core 0 --mode quiet --secs 30 --out r6_quiet_c0.bin
# 5. power-cycle first: stress, wide C1, re-send every second, 60 s
python adc_diag.py stream --port COM4 --selects 0123457 --core 1 --mode wide --secs 60 --resend-every 1 --out r6_stress_c1.bin
```

## Fault reporting: UNRECOVERED in the locked modes, heartbeat, boot fields (2026-10-23)

The host's ADC fault alarm ([`whisker_link/esp32_fault.py`](../../whisker_link/esp32_fault.py), used by `rpi/daq_server.py` and wavi_py) needs three things the firmware didn't give: an unrecovered verdict in the locked modes, a sign of life when nothing streams, and the boot result in a form a program can read.

**Locked modes declare an unrecovered ADC.** Before, a dead ADC in quiet or wide mode printed `[halt] no conversion for 50 ms ...` about 10 times a second forever. Now a halt is "no usable conversion for 50 ms": a read with a CRC error or stale STATUS doesn't count, so DRDY stuck low is caught too. Each halt does a CLKIN restart and a full re-init (up to ~0.4 s, as `initADC` waits 300 ms for a DRDY). After 5 halts in a row with no usable conversion in between (`LK_UNREC_N`), the reader sends the legacy scan's line, same text from the same formatter: `[halt #N] UNRECOVERED  ID=0x.... CLOCKrb=0x.... crcbad=N  -> power-cycle the PCB with the ESP32 already running`. It then retries every 5 s (`LK_UNREC_RETRY_MS`), each failed retry repeating the line. Commands don't postpone the retries. When conversions come back (a PCB power cycle; the ADC then reports a reset and is re-initialised) it logs `[adc ok] <mode> mode: converting again after N halt recoveries`. In C0, the reader now also sends while the ADC is silent (no read for 2 conversion periods, at least 2 ms; there is no read to protect then), so its halt lines no longer pile up unsent, and `lkService` returns at its budget then, so the reader still answers park requests and its watchdog. A park now waits up to 1 s (a recovery holds the reader ~0.4 s). The legacy scan is unchanged: UNRECOVERED after each failed 3-level recovery, ~1.4 s apart. At boot, a dead ADC gives up on the baseline after 2 failed scans instead of 16, so the boot takes ~10 s, not ~50 s.

**Heartbeat (BEAT frame, type 0x05).** About once a second in every mode: locked and legacy scans, binary and text modes, and while nothing streams. Payload (38 B): `version u8 = 1, uptime_ms u32, mode u8, core u8 (the C setting), host_mode u8 (ASCII 'B' or the text mode), flags u8, window_ms u16, drdy_edges u32, halts u32, unrecovered u32, adc_reinits u32, clock_rb u16, device_id u16, scans u32, reset_reason u8, boot_clock_rb u16`. `drdy_edges` counts DRDY falling edges in the window (the ISR runs in every mode), so the ADC converts if and only if it is > 0. Flags: 0x01 ADC_DOWN (UNRECOVERED declared, no usable conversion since), 0x02 NO_READER (the reader task wasn't created), 0x04 PREV_WD (the previous reset was the reader watchdog), 0x08 BOOT_CLOCK_OK, 0x10 BOOT_RECOVERY (the ADC was stuck at boot), 0x20 BOOT_REC_FAIL. In text modes the same fields come as one line, `#BEAT v1 up_ms=.. mode=.. core=.. hmode=g flags=0x.. window_ms=.. edges=.. halts=.. unrec=.. reinit=.. clock_rb=0x.... id=0x.... scans=.. reset=.. boot_clock_rb=0x....`, which `esp32_frame.Decoder` also turns into a `Beat`. Beats start right after the boot banner. They are sent by whoever sends at the moment: core 0 (after a scan frame in C1, so windowed TX keeps its window), or the reader's idle time in C0. They are never sent inside a command or a diagnostic dump (`r`, `D`, `p`), so a long command can delay one (an `M` with a dead ADC: ~0.9 s). Old decoders skip the unknown frame type.

**Boot fields.** The reset reason (`esp_reset_reason()`), the boot CLOCK readback and the boot recovery result are in every beat, so a host that connects after the boot still gets them; the banner lines (`reset reason N`, `CLOCK readback = ...`) are unchanged. INFO stays at version 4: older decoders match the INFO length exactly and would have dropped a longer INFO entirely. The reader watchdog's note from before a restart is printed raw in the banner as before, flagged in the beats (PREV_WD), and sent once as framed TEXT `previous reset: [reader watchdog] ...` at the first `B`. `adc_diag stream` treats that line as a failure, as it does the raw one: the run follows a watchdog restart.

**Host side.** The fault model takes the beats (`FaultMonitor.on_beat`). A beat with ADC_DOWN raises `adc_unrecovered`, and two beats in a row with no DRDY edge raise `adc_not_converting` ("ESP32 alive"), even if the text lines were lost. A firmware that has sent beats and then sends nothing at all (no beat, scan or text) for 5 s while the port is open is `esp32_hung` (action: reset the ESP32, `esp32_reset` / EN). Without beats (older firmware) that stays the old `no_scans`. A beat whose uptime went back is a reboot (a warning with its reset reason) when the banner wasn't seen. daq_server and the wavi ESP32 transport put the latest beat in the status as `esp32.beat` (with `conv_hz`, `converting`, `reset_name`), and daq_server also reports the decoder's `junk` bytes. The fake ESP32 sends beats (`--no-beat` leaves them out) and mimics the new locked-mode halt sequence; `kill -STOP` on it is a hung ESP32.

**Tests.** host_sim models a locked-up ADC (`--adc-dead T[:T2]`: SPI and ID answer, no conversions or DRDY, the CLOCK write doesn't stick and reads 0x00FF, RESET doesn't help; at T2 the PCB is power-cycled) and a reader-watchdog note in RTC memory (`--prev-wd TEXT`). The tests check, in quiet C1, quiet C0 and wide: exactly 5 halt lines, then UNRECOVERED in the legacy format, the next one 5 s later, beats every ≤ 1.35 s with no DRDY and ADC_DOWN after the verdict, recovery at the power cycle, and the fault model's verdict on the decoded stream. Also: the legacy scan, a dead ADC from power-up, daq_server-style re-sends every 2 s (no park timeout), healthy beats in every mode, and the watchdog note sent once. `adc_diag beat` checks the heartbeat on a board: 6 s in a text mode, 6 s in binary mode, with the boot fields.

What the board can't show on demand is a real lock-up. On a healthy board, `adc_diag beat` checks the heartbeat and the boot fields, and the stream runs check that the beat doesn't disturb the locked timing. If CLKIN (P1 pin 6 → GPIO16) is a jumper on the bench, pulling it while streaming stops the ADC (it has no oscillator of its own). That gives the halt → UNRECOVERED → back-off sequence, and putting it back gives `[adc ok]`. Whether the CLOCK write then sticks without CLKIN is not known, so CLOCKrb may read the configured value.

## First board run of the locked modes (bench, 2026-10-23)

Captures and analysis are in `/scratch/bigeme/whisker_dev/bench_1023/`. Quiet timing was clean: 62.500 Hz, every step exactly 16000 µs, no seq gaps. Noise on W2_A/W2_B was 0.025/0.027 mV at OSR 256 and 0.019/0.016 mV at OSR 512, against 0.065 mV in the legacy scan. OSR 512 matches the floor emulated from a raw burst (~1 µV/√Hz input noise seen for ~1.9 ms of every 16 ms), so it is now the quiet default. A comb burst put the interference at multiples of 125 Hz, locked to the ESP32 clock, so 8 ms periods fold it too. Wide failed on one core (overrun ~15 %, latency max 113 µs against T = 32 µs). A dual-core reader is in design.

**Resync bug, fixed.** In quiet OSR 256, one startup overrun made `resync` count every conversion for the rest of the run. Mechanism, reproduced in `host_sim`:
- A stall longer than one T leaves two conversions in the ADC FIFO. The reader labels its read with the newer edge's index but gets the older entry.
- That read ends with the datasheet's narrow DRDY pulse (data still in the FIFO), and the ISR counts it as a new edge. Its time maps to the index just read.
- The old code forced such an edge to `lastK + 1`. From then on every index was one ahead of the edge grid, every edge took the glitch path, and the offset never cleared.
- The reader now trusts the edge time (`k` stays as the grid says). The slot is tainted and the scan flagged SYNC, and `resync` stops at the glitch. An edge that lands in the scan already closed is read and dropped.

Noise check: with the offset, every mux switch and every averaged conversion moved one T earlier together. The conversions used relative to each switch were the same, so the offset can't change the noise. The "12.5 Hz" lines are at 12.57 Hz (quiet OSR 256) and 12.72 Hz (OSR 512) in the 20 s captures. They are not locked to the scan, so the deterministic offset didn't make them either. Removing them lowers the OSR 256 std by only ~7 %. The remaining OSR 256 excess is broadband, with an ASD floor 2.1–2.45 µV/√Hz against 1.9–1.95 at OSR 512, and is still unexplained. The next OSR 256 run, without the offset, will tell.

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

Arduino IDE 2.x with the **esp32 by Espressif** core 3.x; board **ESP32 Dev Module**. Open `whisker_daq/whisker_daq.ino`, upload over USB. The sketch is several files in `whisker_daq/` (`whisker_daq.ino`, `daq_common.h`, `lk_reader.h/.cpp`); the IDE opens them as tabs and builds them together, so keep the folder intact. Compile check from the command line:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 msd/esp32/whisker_daq
```

**Over the tether:** the ESP32 is plugged into the BlueOS Pi, so flash it from there with [`rpi/scripts/flash_esp32.sh`](../../rpi/scripts/flash_esp32.sh) (esptool runs on the Pi, where DTR/RTS auto-reset works). Build the image with `arduino-cli compile --fqbn esp32:esp32:esp32 --output-dir fw msd/esp32/whisker_daq` and copy `fw/whisker_daq.ino.merged.bin` to `~/esp32_flash/` on the Pi; the script's header has the one-time esptool install. Or unplug the ESP32 and use the Arduino IDE on the laptop.

## Pinout

See the header of the `.ino` and the MSD guide [`legacy/Guide.md`](legacy/Guide.md) for P1/P2 wiring and known channel notes. `legacy/` holds the MSD team's last firmware and host scripts as received on 2026-10-05 (the UDP/bridget setup), for reference.
