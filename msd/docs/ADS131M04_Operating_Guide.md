# ADS131M04 Operating Guide

**Author:** GitHub Copilot (Claude Sonnet 4.6)  
**Revised by:** GitHub Copilot (GPT-5.4)  
**Date:** 22 May 2026  
**Based on:** TI ADS131M04 Datasheet (Rev. D), cross-referenced against multiple open-source driver implementations including LucasEtchezuri/Arduino-ADS131M04, icl-rocketry/ADS131M04-Lib, and SainsburyWellcomeCentre/ads131m04_rpi.

---

## 1. Overview

The ADS131M04 is a Texas Instruments 4-channel, 24-bit, simultaneous-sampling delta-sigma ADC with an SPI interface. Key characteristics:

- **Resolution:** 24-bit
- **Channels:** 4, sampled simultaneously
- **Max sample rate:** 64 kSPS (at CLKIN = 8.192 MHz, OSR = 128)
- **SPI Mode:** Mode 1 (CPOL = 0, CPHA = 1)
- **Word length (default):** 24-bit
- **Reference voltage:** Internal 1.2 V
- **PGA gain range:** ×1 to ×128
- **Supply voltage:** 2.7 V – 3.6 V analog, 1.65 V – 3.6 V digital
- **DRDY pin:** Active-low signal indicating a new conversion result is ready

---

## 2. Hardware Interface

### 2.1 SPI Configuration

| Parameter | Value |
|-----------|-------|
| Mode | Mode 1 (CPOL=0, CPHA=1) — data sampled on rising edge |
| Max clock | Up to 25 MHz (practical: 1–8 MHz for Raspberry Pi) |
| CS | Active-low, manually driven (use `no_cs = True` in spidev) |
| Byte order | MSB first |

### 2.2 CLKIN

The ADC requires an external master clock on the CLKIN pin. **All sample rates are derived from this clock.** The standard value is:

```
CLKIN = 8.192 MHz
```

On a Raspberry Pi, this is conveniently generated using `pigpio.hardware_clock()` on GPIO4.

### 2.3 DRDY

DRDY is an **active-low** output from the ADC. It pulses low each time a new conversion result is available across all 4 channels simultaneously. The host must wait for DRDY to go low before reading a new data frame.

---

## 3. SPI Frame Format (24-bit Word Mode)

The ADS131M04 communicates in **frames** of fixed-length 24-bit words. In the default 24-bit word length mode:

- Each word = 3 bytes (16-bit data + 8-bit padding/zero)
- A complete data frame = **6 words = 18 bytes**:

| Word | Bytes | Content |
|------|-------|---------|
| 0 | [0–2] | STATUS word (16 meaningful bits + 8 padding) |
| 1 | [3–5] | Channel 0 (24-bit signed ADC value) |
| 2 | [6–8] | Channel 1 (24-bit signed ADC value) |
| 3 | [9–11] | Channel 2 (24-bit signed ADC value) |
| 4 | [12–14] | Channel 3 (24-bit signed ADC value) |
| 5 | [15–17] | CRC word (or zeros if CRC disabled) |

To read data, send 18 null bytes over SPI while DRDY is low; the ADC clocks out the above frame.

**GitHub Copilot (GPT-5.4) correction:** The datasheet describes the sixth 24-bit word as the output CRC word in a normal full frame. If CRC checking is disabled, the host may choose to ignore that word, but the frame structure is still a six-word frame with a CRC slot.

---

## 4. Command Words

Commands are sent as the first word in a frame (16 meaningful bits + 8 zero padding = 3 bytes). All other words in the frame are sent as zeros.

| Command | 16-bit Value | Purpose |
|---------|-------------|---------|
| NULL | `0x0000` | No operation; used to clock out a data frame |
| RESET | `0x0011` | Software reset; reloads all registers to default |
| STANDBY | `0x0022` | Low-power standby mode |
| WAKEUP | `0x0033` | Exit standby, resume conversions |
| LOCK | `0x0555` | Lock register writes |
| UNLOCK | `0x0655` | Unlock register writes |
| RREG | `0xA000 \| (addr << 7)` | Read register at `addr` |
| WREG | `0x6000 \| (addr << 7)` | Write register at `addr` |

### Sending a command over SPI (Python example):
```python
cmd = 0x0033  # WAKEUP
payload = [(cmd >> 8) & 0xFF, cmd & 0xFF, 0x00] + [0x00] * 15  # 18 bytes total
spi.xfer2(payload)
```

---

## 5. Register Map

### 5.1 Key Registers

| Address | Name | Purpose |
|---------|------|---------|
| 0x00 | ID | Device ID (read-only) |
| 0x01 | STATUS | Conversion status flags |
| 0x02 | MODE | Word length, CRC, timeout settings |
| 0x03 | CLOCK | Channel enables, OSR, power mode |
| 0x04 | GAIN1 | PGA gain for all 4 channels |
| 0x06 | CFG | Global chop, delay settings |

### 5.2 CLOCK Register (0x03) — Most Critical

Default value after RESET: **0xFF04**

**GitHub Copilot (GPT-5.4) correction:** Per TI ADS131M04 Rev. D, the CLOCK register reset value is `0x0F0E`, not `0xFF04`. The reserved upper nibble reads `0000`, channels are enabled by default, `TBM = 0`, `OSR[2:0] = 011`, and `PWR[1:0] = 10`.

| Bits | Field | Description |
|------|-------|-------------|
| [15:12] | Reserved | Set to 1111 (default) |
| [11] | CH3EN | Channel 3 enable (1 = on) |
| [10] | CH2EN | Channel 2 enable (1 = on) |
| [9] | CH1EN | Channel 1 enable (1 = on) |
| [8] | CH0EN | Channel 0 enable (1 = on) |
| [7:5] | Reserved | Set to 000 |
| [4:2] | OSR[2:0] | Oversampling ratio selection |
| [1:0] | PWR[1:0] | Power mode |

**GitHub Copilot (GPT-5.4) correction:** The field map is slightly different in the datasheet: `[15:12]` is reserved and reads `0000`; `[7:6]` are reserved; bit `[5]` is `TBM`; `[4:2]` is `OSR[2:0]`; and `[1:0]` is `PWR[1:0]`.

#### OSR Field (bits [4:2]) — sample rate at CLKIN = 8.192 MHz

| OSR[2:0] | OSR Multiplier | fDATA |
|----------|---------------|-------|
| 000 | 128 | 64,000 SPS |
| 001 | 256 | 32,000 SPS |
| 010 | 512 | 16,000 SPS |
| 011 | 1024 | 8,000 SPS |
| 100 | 2048 | **4,000 SPS** |
| 101 | 4096 | 2,000 SPS |
| 110 | 8192 | 1,000 SPS |
| 111 | 16384 | 500 SPS |

#### PWR Field (bits [1:0])

| PWR[1:0] | Mode |
|----------|------|
| 00 | High Resolution (default) |
| 01 | Low Power |
| 10 | Very Low Power |

**GitHub Copilot (GPT-5.4) correction:** The datasheet defines `PWR[1:0]` as `00 = Very Low Power`, `01 = Low Power`, and `10` or `11 = High Resolution`. The reset default is `10`, so the default mode is High Resolution, not `00`.

#### Computing the CLOCK register value correctly

```python
# Enable all 4 channels, OSR=2048 (4000 SPS at 8.192 MHz), high-resolution power mode:
CH_EN_ALL  = 0b1111 << 8   # = 0x0F00  — bits [11:8]
OSR_4000   = 4    << 2     # = 0x0010  — OSR bit pattern 100 (2048x), shifted to bits [4:2]
PWR_HIRES  = 0b10          # = 0x0002  — bits [1:0]

clock_val  = CH_EN_ALL | OSR_4000 | PWR_HIRES   # = 0x0F12
```

**Common mistake:** Using `(0b1111 << 3)` places the channel enable bits at [6:3] instead of [11:8], leaving all channels disabled. Using the raw OSR index without a left-shift-by-2 places the value into the PWR field instead of the OSR field.

**GitHub Copilot (GPT-5.4) correction:** `0x007A` is worse than just a shifted channel-enable mistake. With the actual datasheet bit map, it also sets `TBM = 1`, which forces turbo mode and overrides the normal `OSR[2:0]` field.

### 5.3 GAIN Register (0x04)

| Bits | Field | Channel |
|------|-------|---------|
| [14:12] | PGAGAIN3 | Channel 3 |
| [10:8] | PGAGAIN2 | Channel 2 |
| [6:4] | PGAGAIN1 | Channel 1 |
| [2:0] | PGAGAIN0 | Channel 0 |

Each field encodes log₂(gain): 0 = ×1, 1 = ×2, 2 = ×4, ..., 7 = ×128.

For all channels at unity gain: write `0x0000`.

### 5.4 MODE Register (0x02)

For default operation (24-bit words, no CRC): write `0x0000`.

**GitHub Copilot (GPT-5.4) correction:** This statement is not correct per the datasheet. MODE reset default is `0x0510`, with `WLENGTH[1:0] = 01b` for 24-bit words and `TIMEOUT = 1b`. Writing `0x0000` sets `WLENGTH = 00b`, which selects 16-bit word mode, and also disables the SPI timeout. For a 24-bit design, either leave MODE at its reset default or explicitly write a value such as `0x0110` if you need to clear the RESET flag while keeping 24-bit words and timeout enabled.

---

## 6. Register Read/Write Protocol

### 6.1 Writing a Register

The WREG command occupies the command word slot. The register value fills the next word. Remaining words are padded with zeros.

```
Frame sent (18 bytes, 24-bit word mode):
  Word 0: WREG command  = 0x6000 | (addr << 7)   [3 bytes]
  Word 1: register data = <value>                  [3 bytes]
  Words 2–5: 0x000000 each                         [12 bytes]
```

**Must unlock before writing, and lock after:**
```python
send_command(CMD_UNLOCK)
write_register(REG_CLOCK, 0x0F12)
send_command(CMD_LOCK)
```

### 6.2 Reading a Register

Reading requires two SPI transactions:

1. **Transaction 1:** Send RREG command → ADC echoes STATUS
2. **Transaction 2:** Send NULL → ADC returns register data in the STATUS word position of the response

```python
# Correct READ_REG base opcode is 0xA000 (bits [15:11] = 10100)
cmd = 0xA000 | ((reg & 0x1F) << 7)

# Transaction 1: send command
spi_transfer([(cmd >> 8) & 0xFF, cmd & 0xFF, 0x00] + [0x00] * 15)

# Transaction 2: send NULLs, register data arrives in bytes [0:2] of the response
response = spi_transfer([0x00] * 18)
reg_value = (response[0] << 8) | response[1]
```

---

## 7. Initialization Sequence

The correct startup sequence is:

```
1. Start CLKIN (e.g., 8.192 MHz on GPIO4 via pigpio)
2. Wait ≥ 1 ms for power stabilization
3. Configure GPIO: CS high, DRDY as input with pull-up
4. Open SPI (Mode 1, chosen speed)
5. Send 3× CMD_NULL to synchronize SPI framing
6. Send CMD_RESET
7. Wait for DRDY to assert low (≤ 500 ms timeout) — confirms ADC is alive
8. Send CMD_UNLOCK
9. Write REG_CLOCK  = 0x0F12  (all channels, OSR=2048, high-res)
10. Write REG_MODE  = 0x0000  (24-bit words, no CRC)
11. Write REG_GAIN1 = 0x0000  (all channels PGA ×1)
12. Send CMD_LOCK
13. Begin reading data frames
```

**GitHub Copilot (GPT-5.4) correction:** Step 10 should not say `REG_MODE = 0x0000 (24-bit words, no CRC)`. `0x0000` selects 16-bit words. A safer sequence is to leave MODE at reset default, or if MODE must be written, use a 24-bit-compatible value such as `0x0110`.

After `CMD_RESET`, DRDY should pulse low within a few milliseconds. If it does not, it indicates a hardware fault (clock not present, power issue, or PCB wiring problem).

---

## 8. Reading Conversion Data

### 8.1 Standard read loop

```python
def wait_drdy(gpio_drdy, timeout_s=0.2):
    deadline = time.monotonic() + timeout_s
    while GPIO.input(gpio_drdy):           # DRDY is active-low
        if time.monotonic() > deadline:
            return False
        time.sleep(50e-6)
    return True

def read_frame(spi, gpio_cs, gpio_drdy):
    if not wait_drdy(gpio_drdy):
        return None                        # timeout

    GPIO.output(gpio_cs, GPIO.LOW)
    time.sleep(10e-6)
    raw = spi.xfer2([0x00] * 18)          # 18 bytes = full frame
    time.sleep(10e-6)
    GPIO.output(gpio_cs, GPIO.HIGH)

    def s24(b0, b1, b2):                   # bytes to signed 24-bit int
        v = (b0 << 16) | (b1 << 8) | b2
        return v - (1 << 24) if v & 0x800000 else v

    return [
        s24(raw[3],  raw[4],  raw[5]),     # CH0
        s24(raw[6],  raw[7],  raw[8]),     # CH1
        s24(raw[9],  raw[10], raw[11]),    # CH2
        s24(raw[12], raw[13], raw[14]),    # CH3
    ]
```

### 8.2 Converting counts to voltage

```
V_input = (count / FULL_SCALE_COUNTS) × (VREF / PGA_GAIN)

Where:
  FULL_SCALE_COUNTS = 2^23 - 1 = 8,388,607
  VREF = 1.2 V (internal reference)
  PGA_GAIN = configured gain (1 for ×1)
```

**GitHub Copilot (GPT-5.4) correction:** The datasheet ideal LSB equation uses `2^23` in the denominator. In other words, one common conversion form is `count / 2^23 * (VREF / gain)`. Using `2^23 - 1` is a common practical approximation near full scale, but it is not the exact denominator used in the datasheet formula.

---

## 9. DRDY Handling with Multiplexer Switching

When an analog multiplexer (MUX) is placed in front of the ADC inputs (e.g., to expand beyond 4 channels), switching the MUX introduces a hazard: the ADC may already have DRDY asserted (low) from a conversion that started on the **previous** channel. Reading immediately would return stale data.

### Correct MUX-switch read sequence:

```python
def wait_drdy_fresh(gpio_drdy, timeout_s=0.2):
    """Wait for a guaranteed new conversion after a MUX switch."""
    deadline = time.monotonic() + timeout_s

    # Step 1: wait for DRDY to go high (flush the stale assertion)
    while not GPIO.input(gpio_drdy):
        if time.monotonic() > deadline:
            return False
        time.sleep(50e-6)

    # Step 2: wait for the next falling edge (fresh conversion on new channel)
    while GPIO.input(gpio_drdy):
        if time.monotonic() > deadline:
            return False
        time.sleep(50e-6)

    return True
```

Additionally, discard at least 1–2 throwaway conversions after a MUX switch to allow the analog input to fully settle, especially with high source impedance or long wiring.

---

## 10. Periodic Reinitialization

In long-running sessions, the ADS131M04 SPI framing can drift (particularly on Raspberry Pi with spidev), causing DRDY timeouts. Best practice:

- Reinitialize the clock, SPI, and ADC registers periodically (e.g., every 8–30 seconds)
- On ≥ 3 consecutive DRDY timeouts, trigger an immediate reinitialization
- Reinitialization sequence: close SPI → stop CLKIN → wait 200 ms → restart CLKIN → wait 100 ms → open SPI → run full init sequence from step 5 above

---

## 11. Common Pitfalls

| Pitfall | Consequence | Fix |
|---------|-------------|-----|
| Using `(0b1111 << 3)` for channel enables | All channels disabled (bits at wrong position) | Use `(0b1111 << 8)` |
| Using raw OSR index without `<< 2` | OSR value lands in PWR field | Use `osr_bits << 2` |
| READ_REG opcode 0x2000 instead of 0xA000 | Register reads return garbage | Use `0xA000 \| (addr << 7)` |
| Not waiting for DRDY after CMD_RESET | Reading before ADC is ready | Poll DRDY low with timeout |
| Reading immediately after MUX switch | Stale data from previous channel | Use `wait_drdy_fresh()` |
| SPI Mode 0 instead of Mode 1 | Misaligned data / all zeros | Set SPI mode to 1 (CPOL=0, CPHA=1) |
| CS managed by kernel SPI driver | CS deasserted mid-frame | Set `no_cs = True`, drive CS manually |
| Skipping CMD_UNLOCK before writes | Register writes silently ignored | Always UNLOCK before writing, LOCK after |

**GitHub Copilot (GPT-5.4) correction:** Two additional pitfalls belong in this list: writing `MODE = 0x0000` in a 24-bit design switches the ADC to 16-bit word mode, and parsing a single-register readback from bytes `[3:5]` instead of the first output word returns the wrong value.

---

## 12. Quick Reference — Correct CLOCK Register Values

| Configuration | CLOCK value |
|---------------|-------------|
| All 4 ch, 64 kSPS, high-res | `0x0F02` |
| All 4 ch, 32 kSPS, high-res | `0x0F06` |
| All 4 ch, 16 kSPS, high-res | `0x0F0A` |
| All 4 ch, 8 kSPS, high-res  | `0x0F0E` |
| All 4 ch, **4 kSPS**, high-res | **`0x0F12`** |
| All 4 ch, 2 kSPS, high-res  | `0x0F16` |
| All 4 ch, 1 kSPS, high-res  | `0x0F1A` |

Formula: `0x0F00 | (osr_bits << 2) | pwr_bits`  
where `osr_bits` is the 3-bit OSR field value from the table in Section 5.2.

---

*End of document.*
