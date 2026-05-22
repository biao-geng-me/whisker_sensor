# ADS131M04 Logger Code Diagnosis

**Author:** GitHub Copilot (Claude Sonnet 4.6)  
**Revised by:** GitHub Copilot (GPT-5.4)  
**Date:** 22 May 2026  
**Files reviewed:**
- `seal_whisker_32ch_logger/logger_32ch_dat.py`
- `seal_whisker_32ch_logger/logger_32ch_dat_fixed.py`
- `seal_whisker_32ch_logger/config_32ch.py`

**Reference sources used for diagnosis:**
- TI ADS131M04 Datasheet Rev. D (`https://www.ti.com/lit/gpn/ads131m04`)
- LucasEtchezuri/Arduino-ADS131M04 (GitHub)
- icl-rocketry/ADS131M04-Lib (GitHub)
- SainsburyWellcomeCentre/ads131m04_rpi (GitHub, Raspberry Pi C driver)

---

## 1. Difference Between the Two Files

The only functional difference between `logger_32ch_dat.py` and `logger_32ch_dat_fixed.py` is in the `init_adc()` method.

### `logger_32ch_dat.py` (skips RESET):
```python
# Do NOT reset the ADC here. On this PCB, CMD_RESET can make DRDY stay high.
# Instead, wake/unlock/configure the ADC and begin reading frames.
self.send_command(CMD_WAKEUP)
time.sleep(0.02)

self.send_command(CMD_UNLOCK)
time.sleep(0.02)
```

### `logger_32ch_dat_fixed.py` (performs RESET):
```python
self.send_command(CMD_RESET)
time.sleep(0.005)

if not self.wait_drdy(timeout_s=0.5):
    drdy = GPIO.input(cfg.GPIO_DRDY)
    raise RuntimeError(f"DRDY did not assert after RESET. Current DRDY={drdy}")

self.send_command(CMD_UNLOCK)
```

All other code — SPI framing, data acquisition, file output, MUX handling, reinit logic — is **identical** in both files.

The "fixed" label refers to the addition of `CMD_RESET` and a hard DRDY check. However, as described in Section 3 below, both files share deeper bugs in the register configuration that are more consequential.

---

## 2. What Is Correct

The following aspects of both files were verified correct against the ADS131M04 datasheet.

| Area | Detail | Status |
|------|--------|--------|
| SPI Mode | `SPI_MODE = 0b01` (CPOL=0, CPHA=1) | ✅ Correct |
| SPI frame size | `FRAME_BYTES = 18` (6 words × 3 bytes in 24-bit mode) | ✅ Correct |
| Command values | RESET=0x0011, WAKEUP=0x0033, LOCK=0x0555, UNLOCK=0x0655, NULL=0x0000 | ✅ Correct |
| WRITE_REG opcode | `0x6000 \| (reg << 7)` — matches WREG format `011aaaaa annnnnnn` | ✅ Correct |
| Register addresses | REG_ID=0x00, REG_MODE=0x02, REG_CLOCK=0x03, REG_GAIN1=0x04 | ✅ Correct |
| Channel data extraction | CH0=[3:5], CH1=[6:8], CH2=[9:11], CH3=[12:14] within the 18-byte frame | ✅ Correct |
| 24-bit sign extension (`conv24`) | Two's-complement conversion via `v -= 1 << 24` when bit 23 is set | ✅ Correct |
| GAIN write = 0x0000 | Encodes PGA ×1 on all 4 channels (log₂(1) = 0 for each channel field) | ✅ Correct |
| MODE write = 0x0000 | Selects 24-bit word length, no CRC — matches intended default | ✅ Correct |
| DRDY polarity | Active-low; waiting for `GPIO.input() == 0` | ✅ Correct |
| CS handling | `spi.no_cs = True` with manual GPIO CS — prevents kernel mid-frame CS toggle | ✅ Correct |
| MUX fresh-edge logic | `wait_drdy_fresh()` correctly flushes stale assertions before reading | ✅ Correct |
| Voltage conversion formula | `(count / FULL_SCALE_COUNTS) × (VREF / PGA_GAIN)` | ✅ Correct |

**GitHub Copilot (GPT-5.4) correction:** The `MODE write = 0x0000` row is not correct. Per the datasheet, `0x0000` selects 16-bit words because `WLENGTH[1:0] = 00b`. The 24-bit reset default is part of `MODE = 0x0510`, not `0x0000`.

---

## 3. Bugs Found

### Bug 1 — CLOCK Register: Wrong Bit Positions (Critical, both files)

**Location:** `init_adc()` in both files, lines ~307–309.

**Code as written:**
```python
ADC_OSR_SETTING = 2
# ...
clock_val = (0b1111 << 3) | ADC_OSR_SETTING   # = 0x007A
self.write_register(REG_CLOCK, clock_val)
```

**What `0x007A` does to the CLOCK register (0x03):**

The CLOCK register bit layout per the ADS131M04 datasheet:

```
Bit:  15  14  13  12 | 11  10   9   8 |  7   6   5 |  4   3   2 |  1   0
      [reserved=1111] [CH3 CH2 CH1 CH0]  [reserved]  [OSR2 OSR1 OSR0]  [PWR1 PWR0]
```

**GitHub Copilot (GPT-5.4) correction:** The datasheet field map is actually `[15:12] reserved = 0000`, `[11:8] channel enables`, `[7:6] reserved`, `[5] TBM`, `[4:2] OSR`, and `[1:0] PWR`. The original analysis correctly identified the misplaced channel-enable bits, but it omitted that `0x007A` also sets `TBM = 1`.

Writing `0x007A = 0b0000_0000_0111_1010`:

| CLOCK bits | Field | Value written | Effect |
|------------|-------|--------------|--------|
| [11:8] | CH3EN–CH0EN | `0000` | **All 4 channels disabled** |
| [6:3] | (reserved) | `1111` | Written to undefined/reserved bits |
| [4:2] | OSR[2:0] | `110` = 6 | OSR = 8192 → **~1000 SPS** (not 4000) |
| [1:0] | PWR[1:0] | `10` = 2 | Very Low Power mode |

**GitHub Copilot (GPT-5.4) correction:** With the actual datasheet bit assignments, `0x007A` decodes as: channels disabled, reserved bits written non-default, `TBM = 1`, `OSR[2:0] = 110` but ignored because turbo mode is active, and `PWR = 10` which is High Resolution, not Very Low Power.

**What the code intended:**

The comment states "OSR setting 2 = 4000 SPS when CLKIN = 8.192 MHz". To achieve 4000 SPS, OSR must be 2048, which requires OSR[2:0] = `100` (decimal 4) placed at bits [4:2]. The channel enables must be at bits [11:8].

**Correct code:**
```python
# OSR[2:0] = 0b100 = 4 for 2048x oversampling = 4000 SPS at 8.192 MHz
# Channel enables at bits [11:8]; OSR field at bits [4:2]; PWR at bits [1:0]
clock_val = (0b1111 << 8) | (4 << 2) | 0b10   # = 0x0F12
self.write_register(REG_CLOCK, clock_val)
```

Confirmed by the SainsburyWellcomeCentre Raspberry Pi C driver, which explicitly states in comments:
> *"OSR field is at bit positions [4:2], so shift left by 2"*
> and channel enables use bit-shifts 8–11.

**Why the system still produces data despite this bug:**

- In `logger_32ch_dat.py` (no RESET): the ADC powers on with CLOCK default value `0xFF04` (all channels enabled, OSR=256, 32 kSPS). If the CMD_UNLOCK/write sequence does not succeed on this PCB hardware, the ADC continues running on its power-on defaults.
- In `logger_32ch_dat_fixed.py` (with RESET): after RESET, CLOCK returns to `0xFF04`. If the write of `0x007A` then takes effect, all channels would be disabled and DRDY would stop pulsing entirely — which explains why this file is less stable and can cause the DRDY timeout / `RuntimeError` that the "fixed" label was intended to address.

**GitHub Copilot (GPT-5.4) correction:** The datasheet reset default for CLOCK is `0x0F0E`, not `0xFF04`. The higher-level conclusion is still directionally reasonable: if the bad write fails to latch, the ADC can keep running on defaults; if it does latch, the ADC configuration becomes unusable for the intended capture path. But the original default value cited here is not correct.

**Root cause of discrepancy between the two files:** The instability in `logger_32ch_dat_fixed.py` is likely caused by this CLOCK register bug being triggered after a clean RESET, not by CMD_RESET itself being harmful.

**GitHub Copilot (GPT-5.4) correction:** This is a plausible hardware hypothesis, but it is stronger than what the current evidence proves. What is proven from the datasheet/code comparison is that the code writes an invalid CLOCK value after initialization. A readback-based hardware check is still needed to confirm whether that bad write is the direct cause of the observed DRDY behavior on this board.

---

### Bug 2 — READ_REG Opcode Incorrect (Minor, both files)

**Location:** `read_register()` in both files.

**Code as written:**
```python
cmd = 0x2000 | ((reg & 0x1F) << 7)
```

**Correct opcode:**
```python
cmd = 0xA000 | ((reg & 0x1F) << 7)
```

**Explanation:**  
The ADS131M04 RREG command format is `101a aaaa annn nnnn`, where the top 3 bits are `101` = `0b101` in positions [15:13]. This gives a base of `0b1010_0000_0000_0000` = `0xA000`.

The code uses `0x2000 = 0b0010_0000_0000_0000`, which has only bit 13 set. This is not a valid ADS131M04 command and the ADC will likely respond with a NULL/STATUS frame.

**GitHub Copilot (GPT-5.4) correction:** There is a second bug in the same function: after the second SPI transaction, the code reads the value from the first channel-data word position instead of the first output word. For a single-register read, the datasheet places the register contents in the first output word of the following frame.

**Impact:**  
`read_register()` is called once during `init_adc()` to read the device ID (`REG_ID`). Because the command is malformed, the returned ID will be 0x0000 or 0xFFFF. The code already handles this case gracefully:

```python
if dev_id == 0 or dev_id == 0xFFFF:
    log.warning("ADS131M04 ID read returned %s; continuing...", hex(dev_id))
```

Since operation continues regardless, this bug does not affect data collection. However, the ID check becomes entirely useless as a hardware diagnostic tool — a disconnected or dead ADC would be indistinguishable from a connected one at startup.

**GitHub Copilot (GPT-5.4) correction:** There is also a separate critical bug missing from the original diagnosis: `self.write_register(REG_MODE, 0x0000)` switches the ADC into 16-bit word mode while the rest of the logger continues using 24-bit framing and 18-byte reads. That issue belongs alongside the CLOCK bug as a major configuration error.

---

## 4. Summary Table

| # | Bug | Severity | Location | Affects both files? |
|---|-----|----------|----------|-------------------|
| 1 | CLOCK register: channel enables at wrong bit position (`<< 3` instead of `<< 8`) | **Critical** | `init_adc()` | Yes |
| 1 | CLOCK register: OSR value not shifted to bits [4:2] → wrong sample rate and may land in PWR field | **Critical** | `init_adc()` | Yes |
| 2 | READ_REG opcode `0x2000` instead of `0xA000` | Low | `read_register()` | Yes |

**GitHub Copilot (GPT-5.4) correction:** The summary table is missing one of the major issues. A corrected summary would also include: `MODE register value 0x0000 forces 16-bit mode while the host still uses 24-bit framing` with critical severity in `init_adc()`.

---

## 5. Recommended Fixes

### Fix for Bug 1 — CLOCK register

Replace in both files:

```python
ADC_OSR_SETTING = 2
# ...
clock_val = (0b1111 << 3) | ADC_OSR_SETTING
self.write_register(REG_CLOCK, clock_val)
```

With:

```python
# OSR[2:0] = 4 (binary 100) gives 2048x oversampling = 4000 SPS at 8.192 MHz
# Channel enables are bits [11:8]; OSR is bits [4:2]; PWR is bits [1:0]
OSR_BITS = 4        # = 0b100, for 2048x oversampling (4000 SPS at 8.192 MHz)
PWR_HIRES = 0b10    # High-resolution power mode
clock_val = (0b1111 << 8) | (OSR_BITS << 2) | PWR_HIRES   # = 0x0F12
self.write_register(REG_CLOCK, clock_val)
```

Also remove or update the `ADC_OSR_SETTING = 2` constant at the top of the file, and update its log message:

```python
log.info("ADC initialized: OSR=2048 (4000 SPS at 8.192 MHz CLKIN)")
```

### Fix for Bug 2 — READ_REG opcode

Replace in both files:

```python
cmd = 0x2000 | ((reg & 0x1F) << 7)
```

With:

```python
cmd = 0xA000 | ((reg & 0x1F) << 7)
```

**GitHub Copilot (GPT-5.4) correction:** A complete fix section should also add:

```python
self.write_register(REG_MODE, 0x0110)
```

or remove the MODE write entirely if the code intends to keep the ADC at its reset-default 24-bit mode. Also, after fixing the RREG opcode, the returned register value should be parsed from the first output word of the next frame, not the first channel-data slot.

---

## 6. Additional Observation — logger_32ch_dat_fixed.py and CMD_RESET

The comment in `logger_32ch_dat.py` states:
> *"On this PCB, CMD_RESET can make DRDY stay high."*

Based on this diagnosis, the most likely root cause is **Bug 1**: after CMD_RESET restores the CLOCK register to default (all channels enabled), the subsequent write of `clock_val = 0x007A` disables all channels, causing DRDY to stop asserting permanently. The symptom looks like CMD_RESET broke DRDY, but the actual cause is the register write that follows.

Once Bug 1 is corrected (writing `0x0F12` instead of `0x007A`), CMD_RESET should be safe to use. The `logger_32ch_dat_fixed.py` approach (RESET + DRDY check) is architecturally the more correct initialization path.

**GitHub Copilot (GPT-5.4) correction:** This conclusion should include both post-reset configuration problems, not just CLOCK. Once the CLOCK write and the MODE write are both corrected, the reset-based initialization path is the more defensible approach because it starts from a known register state.

---

*End of document.*
