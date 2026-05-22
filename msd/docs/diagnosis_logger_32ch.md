# ADS131M04 Logger Code Diagnosis

**Subject file:** [logger_32ch_dat_fixed.py](../logger_32ch_dat_fixed.py) (and matching [config_32ch.py](../config_32ch.py))
**Reviewed against:** ADS131M04 datasheet, TI document **SBAS890D** (March 2019 – Revised May 2021)
**Author:** Claude (Anthropic, model Opus 4.7), assisting per chat session with the project owner
**Date:** 2026-05-22

---

## Summary

The code's overall architecture (pigpio for CLKIN, spidev for SPI, manual CS via GPIO 27, MUX walk + throwaway conversions, periodic reinitialisation) is sound. However, the ADC initialisation contains several bugs where the bit-layouts written to the chip do not match the datasheet's register definitions. The most consequential are the CLOCK and MODE register writes, which currently *disable all four ADC channels*, enable Turbo Mode, and switch the device to 16-bit word framing while the SPI read code continues to assume 24-bit words. The "instability after 18–25 s" noted in the code comments is plausibly a consequence of these misconfigurations rather than a true intermittent hardware fault.

The bugs are ordered below by impact.

---

## Bug 1 — CLOCK register write disables all channels and enables Turbo Mode

**Location:** [logger_32ch_dat_fixed.py:308-310](../logger_32ch_dat_fixed.py#L308-L310)

**Datasheet reference:** §8.6.4 Table 8-17 (CLOCK register, address 0x03, reset value 0x0F0E).

CLOCK register bit layout:

| Bits  | Field                |
|-------|----------------------|
| 15..12 | RESERVED (write 0000) |
| 11..8  | CH3_EN, CH2_EN, CH1_EN, CH0_EN |
| 7..6   | RESERVED (write 00)  |
| 5      | TBM (turbo mode; 1 forces OSR=64, 64 kSPS) |
| 4..2   | OSR[2:0] (000=128 … 011=1024 default … 111=16256) |
| 1..0   | PWR[1:0] (10=high-resolution default) |

**Code:**
```python
clock_val = (0b1111 << 3) | ADC_OSR_SETTING   # ADC_OSR_SETTING = 2  =>  0x007A
self.write_register(REG_CLOCK, clock_val)
```

`0x007A = 0000 0000 0111 1010` decodes as:
- CH3..CH0_EN = **0000** — all four channels **disabled**.
- bit 6 RESERVED = 1 (datasheet says "always write 00").
- TBM = **1** — turbo mode, OSR forced to 64 (fDATA = 64 kSPS regardless of OSR field).
- OSR[2:0] = 110 (not in effect because TBM=1).
- PWR[1:0] = 10 (high-resolution, OK).

The channel-enable nibble belongs at bits **[11:8]**, and the OSR field at bits **[4:2]**. The code shifts both into the wrong positions.

**Fix.** For "all four channels enabled, OSR=1024 → 4000 SPS at 8.192 MHz CLKIN, high-resolution":
```python
# 0x0F0E happens to be the post-RESET default; either skip the write or be explicit.
CH_EN_MASK = 0b1111 << 8
OSR_FIELD  = 0b011 << 2     # 1024 -> 4000 SPS @ 8.192 MHz CLKIN
PWR_HR     = 0b10
clock_val  = CH_EN_MASK | OSR_FIELD | PWR_HR     # 0x0F0E
self.write_register(REG_CLOCK, clock_val)
```

---

## Bug 2 — MODE register write switches the device to 16-bit word framing

**Location:** [logger_32ch_dat_fixed.py:313](../logger_32ch_dat_fixed.py#L313)

**Datasheet reference:** §8.6.3 Table 8-16 (MODE register, address 0x02, reset value 0x0510).

MODE register reset value `0x0510` includes:
- WLENGTH[1:0] = 01b → **24-bit words** (default, what the code's 18-byte frame depends on).
- TIMEOUT = 1 (SPI watchdog enabled; resets stuck SPI state).
- RESET = 1 (sticky reset-status flag, normal after reset).

**Code:**
```python
self.write_register(REG_MODE, 0x0000)
```

Writing `0x0000` sets WLENGTH=00 → **16-bit words**, meaning the device now expects 6 × 2 B = **12-byte frames**, while the read loop continues to push 18 bytes per transfer ([logger_32ch_dat_fixed.py:353](../logger_32ch_dat_fixed.py#L353)). This is exactly the kind of mismatch that produces "works briefly, then DRDY stops behaving" symptoms. The write also disables the SPI TIMEOUT recovery.

**Fix.** Either skip the MODE write entirely (RESET already leaves it at 0x0510), or set it explicitly while preserving WLENGTH=24:
```python
self.write_register(REG_MODE, 0x0510)
```

---

## Bug 3 — `CMD_UNLOCK = 0x0655` is wrong; should be `0x0666`

**Location:** [logger_32ch_dat_fixed.py:50](../logger_32ch_dat_fixed.py#L50)

**Datasheet reference:** §8.5.1.10.6 / Table 8-11.

The UNLOCK command is `0000 0110 0110 0110` = **0x0666**. The code sends `0x0655`, which is an invalid command and is ignored by the device (it returns the STATUS register, exactly like NULL).

This is currently harmless because the device is left UNLOCKED by the preceding RESET, so subsequent register writes go through anyway. However, if any future code path issues a LOCK and then expects UNLOCK to work, the device will remain locked and writes will be silently rejected.

**Fix.**
```python
CMD_UNLOCK = 0x0666
```

---

## Bug 4 — RREG opcode `0x2000` should be `0xA000`

**Location:** [logger_32ch_dat_fixed.py:221](../logger_32ch_dat_fixed.py#L221) (inside `read_register`)

**Datasheet reference:** §8.5.1.10.7 / Table 8-11.

RREG command word format is `101a aaaa annn nnnn`. The top three bits must be **101**, giving a base opcode of **0xA000**. The code uses `0x2000` (top bits `001`), which the device treats as an invalid command and replies to with STATUS.

Consequence: `read_register(REG_ID)` is not actually reading the ID register — it reads STATUS. On a freshly-reset device that yields `0x0500` (RESET=1, WLENGTH=01), which is neither 0 nor 0xFFFF, so the warning branch in `init_adc` is bypassed and the logged "device ID" is misleading. The init code already tolerates ID-read failure, so the impact is cosmetic — but the read does not do what its name says.

**Fix.**
```python
cmd = 0xA000 | ((reg & 0x3F) << 7)
```

Note the address mask change in the next item.

---

## Bug 5 — Register-address mask `& 0x1F` should be `& 0x3F`

**Location:** [logger_32ch_dat_fixed.py:221](../logger_32ch_dat_fixed.py#L221) and [:234](../logger_32ch_dat_fixed.py#L234)

The RREG/WREG address field is `a aaaa a` — **6 bits**, occupying command bits [12:7]. The code uses `& 0x1F` (5 bits). This is latent: every register currently touched (0x00–0x04) is within the 5-bit subset, so the bug has no current effect. It would surface the first time the channel CFG, OCAL, GCAL, or REGMAP_CRC registers (0x09..0x3E) are accessed.

**Fix.**
```python
cmd = 0xA000 | ((reg & 0x3F) << 7)   # RREG
cmd = 0x6000 | ((reg & 0x3F) << 7)   # WREG
```

---

## Bug 6 — `ADC_OSR_SETTING = 2` does not give 4000 SPS

**Location:** [logger_32ch_dat_fixed.py:60](../logger_32ch_dat_fixed.py#L60)

**Datasheet reference:** §8.6.4 Table 8-17, OSR[2:0] field encoding.

OSR field encoding (binary): `000=128, 001=256, 010=512, 011=1024, …`. With CLKIN = 8.192 MHz the modulator runs at fMOD = 4.096 MHz, and fDATA = fMOD / OSR:

| OSR field | Decimation | fDATA |
|-----------|------------|-------|
| 010 (=2)  | **512**    | **8000 SPS** |
| 011 (=3)  | 1024       | **4000 SPS** (what the comment claims) |

So `ADC_OSR_SETTING = 2` does not produce the documented data rate. To get 4 kSPS the value should be `3`. (Moot until Bug 1 is also fixed, because the current CLOCK write places the OSR field in the wrong bits.)

---

## What is correct (verified against the datasheet)

The following items were checked and are not bugs:

- **SPI mode.** `SPI_MODE = 0b01` matches the datasheet's CPOL=0, CPHA=1 requirement (§8.5.1).
- **Frame size.** 18 bytes = 6 words × 3 B in 24-bit-word mode (Figure 8-18).
- **Frame layout.** `conv24(raw[3:6])`, `raw[6:9]`, `raw[9:12]`, `raw[12:15]` correctly extract channels 0..3 from positions 1..4 of the frame; bytes 0..2 are the STATUS word, bytes 15..17 are CRC. Matches Figure 8-18.
- **Two's complement conversion.** `conv24` correctly sign-extends a 24-bit two's-complement number.
- **Voltage scaling.** `count * (VREF/Gain) / 2^23` matches §8.5.1.9 LSB = 2·FSR/2²⁴ with FSR = 1.2 V / Gain.
- **Command opcodes:** NULL `0x0000`, RESET `0x0011`, STANDBY `0x0022`, WAKEUP `0x0033`, LOCK `0x0555` all match Table 8-11. (UNLOCK is wrong — Bug 3.)
- **WREG base opcode** `0x6000` matches the `011a aaaa annn nnnn` format.
- **Register addresses** ID 0x00, MODE 0x02, CLOCK 0x03, GAIN1 0x04 match Table 8-12.
- **CLKIN frequency.** 8.192 MHz is in the typical operating range used throughout the datasheet (e.g. §1 "244-ns resolution, 8.192-MHz fCLKIN").
- **CS active-low, DRDY active-low.** Both match §8.5.1.1 and §8.5.1.5.

---

## Why does the code appear to run at all?

Despite Bugs 1 and 2, the code visibly produces data and runs for ~20 seconds before degrading. The most likely explanation:

1. The RESET command (`0x0011`) is correct and *does* execute. After RESET, the chip's register defaults are themselves a workable configuration: CLOCK = 0x0F0E (all 4 channels enabled, OSR = 1024 → 4000 SPS at 8.192 MHz, high-resolution), MODE = 0x0510 (24-bit words, TIMEOUT enabled).
2. The buggy UNLOCK (Bug 3) is silently ignored — but the interface is *already* unlocked at this point, so the subsequent WREG calls go through.
3. The CLOCK write (Bug 1) and MODE write (Bug 2) then begin to corrupt the device's state. Depending on exact timing and on whether the device latches partial frames before its WLENGTH changes mid-stream, what comes out next can range from "looks like data" to "DRDY misbehaves." The periodic `REINIT_EVERY_S = 8.0` in [config_32ch.py:60](../config_32ch.py#L60) papers over this by re-issuing RESET, restoring defaults, and starting the cycle again.

This is consistent with the symptoms documented in the code's own comments.

---

## Recommended minimum patch set

1. Apply Bug 1 fix (CLOCK = 0x0F0E or skip).
2. Apply Bug 2 fix (MODE = 0x0510 or skip).
3. Apply Bug 3 fix (`CMD_UNLOCK = 0x0666`).
4. Apply Bug 4 + Bug 5 fix (RREG opcode and 6-bit address mask) — only needed if register reads are ever relied upon. Currently optional but worth doing for hygiene.
5. Bug 6 (OSR setting comment / value) becomes moot once Bug 1 is fixed; revisit if you decide to change the data rate.

After this patch set, expect the "needs reinit every 8 s" behaviour to either disappear or change character. If it does, the periodic-reinit workaround can be loosened or removed.

---

## Sources

- ADS131M04 datasheet (TI SBAS890D, March 2019, Revised May 2021): <https://www.ti.com/lit/ds/symlink/ads131m04.pdf>
- TI product page: <https://www.ti.com/product/ADS131M04>
