# ADS131M04 — Project Reference

**Subject:** Texas Instruments ADS131M04 4-channel 24-bit delta-sigma ADC, as used in the 16-whisker / 32-channel logger PCB.
**Author:** Claude (Anthropic, model Opus 4.7), assisting per chat session with the project owner
**Date:** 2026-05-22

This document distills the facts from the ADS131M04 datasheet (TI **SBAS890D**, March 2019, revised May 2021) that matter for **this** project. It is not a substitute for the datasheet — for anything not covered here, go to the source: <https://www.ti.com/lit/ds/symlink/ads131m04.pdf>.

---

## 1. Chip at a glance

- 4 channels, 24-bit, simultaneously sampling delta-sigma ADC.
- Programmable data rate up to 64 kSPS (Turbo Mode).
- Programmable per-channel PGA gain 1 / 2 / 4 / 8 / 16 / 32 / 64 / 128.
- 1.2 V internal voltage reference (no buffered REFOUT pin; external reference not supported on this part).
- SPI interface (mode 1: CPOL = 0, CPHA = 1).
- Single supply 2.7–3.6 V (AVDD = DVDD = same 3.3 V rail on this PCB).

---

## 2. Signals used on this PCB

| Pin name | Direction | Active | Wired to Pi | Description |
|---|---|---|---|---|
| CLKIN     | input  | n/a       | GPIO 4 (GPCLK0) | Master clock input. Driven at 8.192 MHz by `pigpio.hardware_clock`. |
| SCLK      | input  | n/a       | GPIO 11 (SPI0 SCLK) | SPI clock. ≤ 25 MHz; project uses 4 MHz. |
| DIN       | input  | n/a       | GPIO 10 (SPI0 MOSI) | SPI MOSI. Latched on falling SCLK edge. |
| DOUT      | output | n/a       | GPIO 9 (SPI0 MISO) | SPI MISO. Transitions on rising SCLK. Hi-Z when CS is high. |
| CS        | input  | low       | GPIO 27 (manual GPIO) | Chip select. Must be held low for the duration of a frame. |
| DRDY      | output | low       | GPIO 5 | Data-ready signal. Asserts when a new conversion result is available. |
| SYNC/RESET | input  | low pulse | (not wired) | Hardware sync / reset. The project uses the SPI RESET command instead. |
| AVDD, DVDD | power | —         | 3.3 V on this PCB | Both supplies tied together at 3.3 V. |
| AGND, DGND | power | —         | shared GND on this PCB | Tied together at the chip on a single-point star ground. |

---

## 3. Supply, range, and absolute maximum

From §6.1 and §6.3 of the datasheet:

| Parameter | Value |
|---|---|
| Supply voltage (AVDD = DVDD) | 2.7 V min – 3.6 V max |
| Reference voltage (internal) | 1.2 V |
| Full-scale differential input | ±FSR where FSR = 1.2 V / Gain |
| Any analog input pin | AVSS − 0.3 V to AVDD + 0.3 V (absolute max) |
| Any digital input pin | DVSS − 0.3 V to DVDD + 0.3 V (absolute max) |
| CLKIN frequency | typical 8.192 MHz; spec range allows higher |

**Important:** Never drive a digital input (CS, SCLK, DIN, SYNC/RESET) above DVDD + 0.3 V. This is the classic *latch-up* failure mode and is why the power-on sequence (analog board first, Pi second) matters.

---

## 4. SPI interface

### 4.1 Mode

CPOL = 0, CPHA = 1 → **SPI mode 1**. In spidev terms this is `spi.mode = 0b01`.

### 4.2 Frame structure

Communication is in frames. Each frame is a sequence of *words*. Word length is programmable via the MODE register's WLENGTH[1:0] field: **16, 24, or 32 bits**. **The project uses the default 24-bit word**, giving 18-byte frames.

A typical data-collection frame (Figure 8-18) looks like this on DOUT (MISO):

| Word index | Bytes (24-bit mode) | Content |
|---|---|---|
| 0 | 0..2  | Response to the *previous* frame's command (or STATUS register for NULL command). |
| 1 | 3..5  | Channel 0 conversion data (24-bit two's complement). |
| 2 | 6..8  | Channel 1 conversion data. |
| 3 | 9..11 | Channel 2 conversion data. |
| 4 | 12..14 | Channel 3 conversion data. |
| 5 | 15..17 | CRC of the output frame. |

On DIN (MOSI) the host sends a command word in the first slot, optional input CRC, and zero-padding for the rest.

Important corollary for this project: if you ever change WLENGTH to 16 bits, the frame becomes 12 bytes, not 18 — and the read loop in the Python logger must change accordingly. The default of 24 is what the current code assumes.

### 4.3 Commands

From Table 8-11. Each command is a 16-bit word sent in the first half-word of a frame.

| Command | Opcode (hex) | Purpose |
|---|---|---|
| NULL    | 0x0000 | No-op. Response is STATUS register. Used to clock data out. |
| RESET   | 0x0011 | Reset to register defaults. Reply word is `0xFF24` if accepted. |
| STANDBY | 0x0022 | Enter low-power standby. |
| WAKEUP  | 0x0033 | Exit standby. |
| LOCK    | 0x0555 | Lock the interface — only NULL, UNLOCK, RREG are then accepted. |
| UNLOCK  | **0x0666** | Unlock the interface. (The project's `CMD_UNLOCK = 0x0655` is **wrong** — see [diagnosis](diagnosis_logger_32ch.md).) |
| RREG    | `101a aaaa annn nnnn` → base 0xA000 | Read N+1 registers starting at address `a aaaa a`. |
| WREG    | `011a aaaa annn nnnn` → base 0x6000 | Write N+1 registers starting at address `a aaaa a`. |

The address field is **6 bits** (mask `0x3F`), not 5. The count field `nnn nnnn` is 7 bits and encodes (number of registers − 1).

### 4.4 CRC

Optional input CRC (controlled by RX_CRC_EN in MODE) and optional register-map CRC. Output CRC is always present in the data frame, but reading it is optional. The current project bypasses both — RX_CRC_EN remains 0 (the default), and the output CRC bytes are simply ignored.

---

## 5. Registers used in this project

Full register map is Table 8-12. Below are just the ones the logger touches.

### 5.1 ID — address 0x00, read-only

Reset value `0x24xx`. The high nibble of the upper byte identifies the part (`0x2`) and the channel count (`0x4`). Use this register to verify SPI is working.

### 5.2 STATUS — address 0x01, read-only

Returned in response to NULL commands. Bits include:
- 15 LOCK — interface lock state.
- 10 RESET — sticky "reset has occurred" flag.
- 9..8 WLENGTH — current word-length setting.
- 3..0 DRDY3..DRDY0 — per-channel data-ready flags.

### 5.3 MODE — address 0x02, R/W, reset 0x0510

| Bits | Field | Notes for this project |
|---|---|---|
| 13 | REG_CRC_EN | 0 (default). |
| 12 | RX_CRC_EN | 0 (default — no input CRC required). |
| 11 | CRC_TYPE | 0 = 16-bit CCITT. |
| 10 | RESET | Sticky reset flag; write 0 to clear. |
| 9..8 | WLENGTH | **Must stay at 01b (24-bit) for this project's frame layout.** |
| 4 | TIMEOUT | 1 = SPI timeout enabled (recommended). |
| 3..2 | DRDY_SEL | DRDY source. 00 = most-lagging channel (default). |
| 1 | DRDY_HiZ | 0 = push-pull DRDY output. |
| 0 | DRDY_FMT | 0 = level (recommended for polling). |

Project recommendation: leave at reset default `0x0510`, or set explicitly to `0x0510`. Do not write `0x0000` (that's Bug 2 in the [diagnosis](diagnosis_logger_32ch.md)).

### 5.4 CLOCK — address 0x03, R/W, reset 0x0F0E

| Bits | Field | Notes for this project |
|---|---|---|
| 11..8 | CH3_EN..CH0_EN | **All four = 1 for this project (1111b).** |
| 5 | TBM | 0 unless you want 64 kSPS turbo mode. |
| 4..2 | OSR[2:0] | See table below. |
| 1..0 | PWR[1:0] | 10b = high-resolution (default). |

OSR encoding and resulting data rate at CLKIN = 8.192 MHz (fMOD = 4.096 MHz, fDATA = fMOD / OSR):

| OSR[2:0] | Decimation | fDATA |
|---|---|---|
| 000 | 128   | 32 kSPS |
| 001 | 256   | 16 kSPS |
| 010 | 512   | 8 kSPS |
| **011** | **1024** | **4 kSPS (project default)** |
| 100 | 2048  | 2 kSPS |
| 101 | 4096  | 1 kSPS |
| 110 | 8192  | 500 SPS |
| 111 | 16256 | ~252 SPS |

Turbo Mode (TBM = 1) forces OSR = 64 and fDATA = 64 kSPS regardless of OSR[2:0].

**Project recommendation:** write `0x0F0E` (which is the post-RESET default), or simply skip the write.

### 5.5 GAIN — address 0x04, R/W, reset 0x0000

3-bit PGA-gain field per channel. Encoding: 000=1, 001=2, 010=4, 011=8, 100=16, 101=32, 110=64, 111=128. The project uses gain = 1 on all channels (write `0x0000`).

---

## 6. ADC data interpretation

Each channel returns a 24-bit two's-complement code. From §8.5.1.9:

```
LSB = 2 × FSR / 2^24  =  FSR / 2^23

where FSR = V_REF / Gain  =  1.2 V / 1  =  1.2 V (in this project)
```

Conversion to volts:
```
V_input = (code / 2^23) × (V_REF / Gain)
        = code × 1.2 / 2^23     (this project)
```

Ideal codes:
| Differential input | Output code |
|---|---|
| ≥ +FSR | 0x7FFFFF (positive saturation) |
| +FSR / 2^23 | 0x000001 |
| 0 | 0x000000 |
| −FSR / 2^23 | 0xFFFFFF |
| ≤ −FSR | 0x800000 (negative saturation) |

---

## 7. DRDY behavior

From §8.5.1.5:

- DRDY is active **low**.
- New data are indicated by DRDY going high briefly (or by staying low, depending on DRDY_FMT in MODE).
- DRDY is blocked while new conversions complete during a read, so consistent DRDY behavior requires reading the data each conversion period.
- After a MUX switch the previously-asserted DRDY may correspond to a conversion of the *old* channel. The project handles this in `wait_drdy_fresh()` by first waiting for DRDY high, then for the next falling edge.

The ADS131M04 has a 2-deep FIFO for output data. If the host does not read for more than one conversion period and then resumes, the device requires either a SYNC/RESET pulse or two consecutive frames to re-align — see §8.5.1.9.1.

---

## 8. SINC filter and settling after a MUX switch

The ADS131M04 uses a SINC³ digital filter (or SINC³ + SINC¹ in some OSR settings). The filter has **three** conversion cycles of memory. After a step change at the input (e.g. switching the MUX to a new channel), the first three conversions are a weighted average of old and new values. By the fourth conversion the filter is fully settled to the new value.

This is **independent** of the analog settling time at the MUX/ADC interface, which depends on R_ON, source impedance, and line capacitance and is in the millisecond range for this PCB (see the [PCB operation guide](pcb_operation_guide.md), Section 9, and the [CD4097B reference](mux_cd4097b_reference.md)).

The project handles this with `THROWAWAY_CONVERSIONS = 2` (discard 2 conversions, keep the third) in [config_32ch.py](../config_32ch.py).

---

## 9. Reset behavior

The RESET command (`0x0011`) returns all registers to defaults:
- CLOCK = 0x0F0E (all 4 channels enabled, OSR = 1024 → 4 kSPS, high-resolution).
- MODE  = 0x0510 (24-bit words, TIMEOUT enabled).
- GAIN  = 0x0000 (all gains = 1).
- Interface state = UNLOCKED.

After RESET the host must wait t_REGACQ (a few µs to ms — the datasheet specifies; the project's 5 ms is conservative) before issuing further SPI commands.

For this project, the defaults are exactly what is wanted. The cleanest initialization sequence is:

1. RESET.
2. Wait 5 ms (or wait for DRDY to assert).
3. Optionally: write only the registers you actually want to change from defaults. **Do not** write 0 into MODE (Bug 2). Do not write 0x007A into CLOCK (Bug 1).
4. Done. The chip is now sampling on all 4 channels at 4 kSPS with gain = 1, in 24-bit-word frame mode.

---

## 10. Sources

- TI ADS131M04 datasheet, SBAS890D, May 2021: <https://www.ti.com/lit/ds/symlink/ads131m04.pdf>
- TI product page: <https://www.ti.com/product/ADS131M04>
