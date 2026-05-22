# CD4097B — Project Reference

**Subject:** Texas Instruments CD4097B differential 8-channel CMOS analog multiplexer, as used in the 16-whisker / 32-channel logger PCB.
**Author:** Claude (Anthropic, model Opus 4.7), assisting per chat session with the project owner
**Date:** 2026-05-22

This document distills the facts from the CD4067B/CD4097B datasheet (TI **SCHS052D**, June 2003, revised August 2024) that matter for **this** project. It is not a substitute for the datasheet — for anything not covered here, go to the source: <https://www.ti.com/lit/ds/symlink/cd4097b.pdf>.

---

## 1. Chip at a glance

- Differential 8-channel CMOS analog multiplexer.
- Two independent banks of 8 switches (X bank and Y bank) sharing common address pins A, B, C.
- Each bank has its own COMMON pin: COMMON X (pin 1) and COMMON Y (pin 17).
- 3-bit binary address (A, B, C) selects channel pair `n` (both `nX` and `nY` simultaneously).
- INHIBIT pin (active-high) turns *all* channels off.
- Bidirectional — usable as either a 1-to-8 mux or an 8-to-1 demux.
- Operating supply 3–18 V.

For **this PCB**, two CD4097B chips are used. Each chip contributes two of the ADC's four input lines (via its X and Y common pins), so the two chips together route 4 × 8 = 32 sensor lines into the 4 ADC channels.

---

## 2. Pinout (24-pin DIP / SOIC / TSSOP)

| Pin | Name | Function |
|-----|------|----------|
| 1   | COMMON X OUT/IN | X-bank common |
| 2   | 7X | X-bank channel 7 |
| 3   | 6X | |
| 4   | 5X | |
| 5   | 4X | |
| 6   | 0X | (note: pinout ordering on the datasheet diagram, double-check on the PCB) |
| 7   | 1X | |
| 8   | 2X | |
| 9   | 3X | |
| 10  | A | Address input, **bit 0 (LSB)** |
| 11  | B | Address input, **bit 1** |
| 12  | V_SS | Negative supply / GND in single-supply use |
| 13  | INHIBIT | Active-high; tie to GND in this project |
| 14  | C | Address input, **bit 2 (MSB)** |
| 15..22 | Y-bank channels 0Y..7Y | |
| 17  | COMMON Y OUT/IN | Y-bank common |
| 23  | (Y channel) | |
| 24  | V_DD | Positive supply, 3.3 V in this project |

(Refer to Figure 4-1 of the datasheet for the authoritative pin ordering — pin numbering of individual channels varies and is documented there.)

---

## 3. Truth table

From Table 4-2 of the datasheet:

| A | B | C | INH | Selected channel pair |
|---|---|---|-----|----|
| X | X | X | 1   | None (all switches off) |
| 0 | 0 | 0 | 0   | 0X, 0Y |
| 1 | 0 | 0 | 0   | 1X, 1Y |
| 0 | 1 | 0 | 0   | 2X, 2Y |
| 1 | 1 | 0 | 0   | 3X, 3Y |
| 0 | 0 | 1 | 0   | 4X, 4Y |
| 1 | 0 | 1 | 0   | 5X, 5Y |
| 0 | 1 | 1 | 0   | 6X, 6Y |
| 1 | 1 | 1 | 0   | 7X, 7Y |

**Channel index = A + 2·B + 4·C.** A is the LSB, C is the MSB.

The project's `set_mux()` correctly implements this:
```python
for i, pin in enumerate(cfg.MUX1_PINS):   # [SELA, SELB, SELC]
    GPIO.output(pin, (addr >> i) & 0x01)
```
`i=0 → SELA ← bit 0`, `i=1 → SELB ← bit 1`, `i=2 → SELC ← bit 2`. Matches the truth table.

---

## 4. Electrical characteristics relevant at V_DD = 3.3 V

The datasheet's electrical characteristics tables list values at V_DD = 5 V, 10 V, and 15 V. At V_DD = 3.3 V (this project), values are extrapolated downward from the 5 V column.

### 4.1 Logic-input thresholds

V_IHC (min "1" level) scales roughly as 0.7 × V_DD:

| V_DD | V_IHC (min) | V_ILC (max) |
|---|---|---|
| 5 V | 3.5 V | 1.0 V |
| 10 V | 7.0 V | 1.0 V |
| 15 V | 11.0 V | 1.0 V |
| **3.3 V (this project)** | **~2.3 V** | ~1.0 V |

At V_DD = 3.3 V, the Pi's 3.3 V GPIO output (V_OH ≈ 3.2 V) comfortably exceeds V_IHC. **No level shifter is needed.** This is the critical reason the project specifies V_DD = 3.3 V; if V_DD were 5 V, the Pi's 3.3 V high would fall below V_IHC = 3.5 V and address-pin behavior would be marginal.

### 4.2 ON resistance (R_ON)

R_ON at 25 °C:

| V_DD | typical | max |
|---|---|---|
| 5 V | 470 Ω | 1050 Ω |
| 10 V | 180 Ω | 400 Ω |
| 15 V | 125 Ω | 240 Ω |
| **3.3 V (extrapolated)** | **~1.5–2 kΩ** | **probably 2.5–3 kΩ** |

R_ON rises rapidly as V_DD falls because the channel transistor's gate-drive shrinks. This is the dominant source of analog-settling time at this PCB's 3.3 V V_DD.

### 4.3 Channel switching delays (from §5.6)

At V_DD = 5 V, C_L = 50 pF, R_L = 1 kΩ:

| Parameter | Typical | Max |
|---|---|---|
| t_pd (signal in → signal out, channel selected) | 30 ns | 60 ns |
| t_ph (turn-on time)  | 325 ns | 650 ns |
| t_pl (turn-off time) | 220 ns | 440 ns |

These are the chip's intrinsic switching times. They are **fast** — sub-microsecond. The 2 ms `time.sleep` in the project is not for the MUX itself; it is for the analog network (source impedance × line capacitance × any filter caps) to settle through R_ON. See Section 6 below.

### 4.4 Off-channel leakage

±100 nA max per channel at V_DD − V_SS = 18 V, 25 °C. Negligible for strain-gauge signals.

### 4.5 Off-channel capacitance

Output capacitance C_OS at 1 MHz: 35 pF (CD4097B). With 7 off-channels per bank, this adds up to ~250 pF of stray capacitance on each COMMON pin — relevant when estimating settling time (Section 6).

### 4.6 Crosstalk and feedthrough

40 dB feedthrough rejection up to 8 MHz (CD4097B on Any channel). Adequate for the strain-gauge bandwidth (DC to a few hundred Hz).

---

## 5. Supply, ground, and signal range

| Parameter | Value | Notes |
|---|---|---|
| V_DD | 3.3 V | this project |
| V_SS | 0 V (GND) | this project |
| V_DD − V_SS allowable range | 3–18 V | wider than needed |
| Analog signal range (V_S, V_D) | V_SS to V_DD | strict — signal must stay between rails |
| Absolute max for signal pins | V_SS − 0.5 V to V_DD + 0.5 V | beyond this, current flows through the chip's protection diodes (latch-up risk for unpowered chip with live signals) |

Implication for the strain-gauge bridges: bridge excitation must not exceed V_DD = 3.3 V. With excitation at 3.3 V and a perfectly balanced bridge at mid-rail (1.65 V) the signal is well inside the rails. With any practical bridge offset, even after amplification, signals stay well under 3.3 V.

---

## 6. Settling time — why the project uses 2 ms

The 2 ms delay after every MUX switch (in `set_mux`) is **analog settling time**, not MUX propagation time. The MUX itself switches in well under 1 µs. What takes 2 ms is the voltage at the ADC input pin reaching the new bridge's value after switching, through an RC network whose components are:

```
R = R_bridge_out  +  R_ON(MUX)  +  R_wire
C = C_wiring  +  C_ADC_input  +  C_off-channels  +  any explicit filter caps
```

Order-of-magnitude estimates at V_DD = 3.3 V:

| Term | Value |
|---|---|
| Bridge output impedance (350 Ω full bridge) | ~88 Ω |
| MUX R_ON | ~1.5–2 kΩ |
| Wire resistance | ~100 mΩ |
| ADC + MUX common pin capacitance | 10–20 pF |
| Wiring/ribbon capacitance per channel | 30–100 pF |
| Off-channel coupling (7 channels × ~35 pF) | ~250 pF |
| Optional filter cap on MUX input (if present) | 1–100 nF |

For 24-bit accuracy you need ~17 time constants of settling. Without any explicit filter cap, the time constant is on the order of microseconds and the 2 ms is conservative. **With** an explicit anti-alias filter cap of 10–100 nF at each MUX input, the time constant jumps into hundreds of microseconds and the 2 ms is necessary.

The code's own comment confirms this: "100 µs was sufficient for a single whisker but too short for 16 sensors due to increased wiring capacitance and source impedance on each MUX input."

The fastest way to shrink this delay is documented in the [PCB operation guide](pcb_operation_guide.md), Section 9 — buffer the bridges, remove or move any filter caps, and measure actual settling on a scope.

---

## 7. Common-mode reminders

- **Do not leave INHIBIT floating.** Tie pin 13 to GND on the board so the chip cannot randomly disable all channels.
- **Do not leave address pins floating** while V_DD is applied. CMOS inputs floating can oscillate and draw weird supply current.
- **Both chips on this board receive the same address GPIOs**, but driven through separate Pi GPIOs (SELA_1/B/C and SELA_2/B/C). The `set_mux()` function writes both sets to the same value, so the two chips step in lock-step. The Pi pins were kept separate to allow individual chip selection in the future if needed.

---

## 8. Sources

- TI CD4067B/CD4097B datasheet, SCHS052D, August 2024: <https://www.ti.com/lit/ds/symlink/cd4097b.pdf>
- TI product page: <https://www.ti.com/product/CD4097B>
