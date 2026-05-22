#!/usr/bin/env python3
"""
Independent 16-whisker / 32-channel Raspberry Pi logger.

Outputs lab-style .dat files:
    YYYY-MM-DD HH:MM:SS.mmm  Ch1  Ch2 ... Ch32

This does NOT depend on the previous master's-student Python code.
It uses:
    - pigpio only for the ADS131M04 master clock on GPIO4
    - RPi.GPIO for CS, DRDY, and MUX select GPIO
    - spidev for SPI0 communication

Stop manually with Ctrl+C.
"""

from __future__ import annotations

import csv
import os
import sys
import time
import signal
import logging
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

try:
    import pigpio
    import spidev
    import RPi.GPIO as GPIO
except ImportError as exc:
    raise SystemExit(
        "Missing Raspberry Pi hardware Python package. "
        "Install/enable pigpio, spidev, and RPi.GPIO before running."
    ) from exc

import config_32ch as cfg


# =========================
# ADS131M04 command/register constants
# =========================
CMD_RESET = 0x0011
CMD_STANDBY = 0x0022
CMD_WAKEUP = 0x0033
CMD_LOCK = 0x0555
CMD_UNLOCK = 0x0666
CMD_NULL = 0x0000

REG_ID = 0x00
REG_MODE = 0x02
REG_CLOCK = 0x03
REG_GAIN1 = 0x04

# ADS131M04 oversampling values used by the previous code:
# OSR field 011 (=3) -> decimation 1024 -> 4000 SPS at CLKIN = 8.192 MHz.
ADC_OSR_SETTING = 3


def setup_logging() -> logging.Logger:
    logger = logging.getLogger("whisker32")
    logger.setLevel(logging.INFO)
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
    logger.handlers.clear()
    logger.addHandler(handler)
    return logger


log = setup_logging()


def now_strings() -> Tuple[str, str]:
    dt = datetime.now()
    date_str = dt.strftime("%Y-%m-%d")
    time_str = dt.strftime("%H:%M:%S.%f")[:-3]
    return date_str, time_str


def conv24(b0: int, b1: int, b2: int) -> int:
    """Convert 3 bytes to signed 24-bit integer."""
    v = (b0 << 16) | (b1 << 8) | b2
    if v & 0x800000:
        v -= 1 << 24
    return v


def count_to_voltage(count: int) -> float:
    """Convert signed ADC count to ADC-input voltage."""
    return (float(count) / float(cfg.ADC_FULL_SCALE_COUNTS)) * (cfg.VREF_VOLTS / cfg.PGA_GAIN)


@dataclass
class OutputFiles:
    voltage_path: Optional[Path]
    counts_path: Optional[Path]
    voltage_file: Optional[object]
    counts_file: Optional[object]

    def close(self) -> None:
        for f in (self.voltage_file, self.counts_file):
            if f is not None:
                f.flush()
                f.close()


class ADS131M04Logger:
    def __init__(self) -> None:
        self.pi = None
        self.spi = None
        self.running = True
        self.frames_written = 0
        self.timeout_count = 0
        self.reinit_count = 0

    # -------------------------
    # Hardware setup/cleanup
    # -------------------------
    def start_pigpio_clock(self) -> None:
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError(
                "Could not connect to pigpio daemon. Start it with:\n"
                "  sudo /usr/local/bin/pigpiod\n"
                "or:\n"
                "  sudo pigpiod"
            )

        rc = self.pi.hardware_clock(cfg.GPIO_CLKIN, cfg.ADC_CLOCK_HZ)
        if rc != 0:
            raise RuntimeError(
                f"pigpio.hardware_clock(GPIO{cfg.GPIO_CLKIN}, {cfg.ADC_CLOCK_HZ} Hz) "
                f"failed with rc={rc}. Without CLKIN the ADS131M04 never produces "
                f"conversions, so DRDY will never assert."
            )

        # Verify GPIO actually entered an ALT-function (clock-output) mode.
        # pigpio mode codes: 0=INPUT, 1=OUTPUT, 2=ALT5, 3=ALT4, 4=ALT0, 5=ALT1, 6=ALT2, 7=ALT3.
        # On the Pi, GPIO4 must be in ALT0 for the GPCLK0 output to drive the pin.
        mode = self.pi.get_mode(cfg.GPIO_CLKIN)
        mode_names = {0: "INPUT", 1: "OUTPUT", 2: "ALT5", 3: "ALT4",
                      4: "ALT0", 5: "ALT1", 6: "ALT2", 7: "ALT3"}
        log.info("Started ADC CLKIN on GPIO%d at %.3f MHz (pigpio mode=%s)",
                 cfg.GPIO_CLKIN, cfg.ADC_CLOCK_HZ / 1e6,
                 mode_names.get(mode, f"unknown({mode})"))
        if mode != 4:
            log.warning("GPIO%d is in mode %s, not ALT0 — the clock output is NOT being "
                        "driven onto the pin. Check that GPIO%d is not claimed by another "
                        "overlay (e.g. dtoverlay=w1-gpio in /boot/firmware/config.txt).",
                        cfg.GPIO_CLKIN, mode_names.get(mode, str(mode)), cfg.GPIO_CLKIN)

    def stop_pigpio_clock(self) -> None:
        if self.pi is not None and self.pi.connected:
            try:
                self.pi.hardware_clock(cfg.GPIO_CLKIN, 0)
            except Exception:
                pass
            try:
                self.pi.stop()
            except Exception:
                pass
        self.pi = None

    def setup_gpio(self) -> None:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(cfg.GPIO_CS, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(cfg.GPIO_DRDY, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.setup(cfg.MUX1_PINS, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(cfg.MUX2_PINS, GPIO.OUT, initial=GPIO.LOW)

        log.info("GPIO initialized: CS=%d, DRDY=%d, MUX1=%s, MUX2=%s",
                 cfg.GPIO_CS, cfg.GPIO_DRDY, cfg.MUX1_PINS, cfg.MUX2_PINS)

    def cleanup_gpio(self) -> None:
        try:
            GPIO.output(cfg.GPIO_CS, GPIO.HIGH)
        except Exception:
            pass
        try:
            GPIO.cleanup()
        except Exception:
            pass

    def open_spi(self) -> None:
        self.spi = spidev.SpiDev()
        self.spi.open(cfg.SPI_BUS, cfg.SPI_DEVICE)
        self.spi.max_speed_hz = cfg.SPI_SPEED_HZ
        self.spi.mode = cfg.SPI_MODE
        self.spi.no_cs = True  # manual CS on GPIO27
        log.info("Opened SPI bus %d.%d at %.3f MHz, mode=%s",
                 cfg.SPI_BUS, cfg.SPI_DEVICE, cfg.SPI_SPEED_HZ / 1e6, bin(cfg.SPI_MODE))

    def close_spi(self) -> None:
        if self.spi is not None:
            try:
                self.spi.close()
            except Exception:
                pass
        self.spi = None

    def setup_all(self) -> None:
        self.start_pigpio_clock()
        # Give CLKIN time to stabilise before the ADC is poked.
        time.sleep(0.25)
        self.setup_gpio()
        self.open_spi()
        self.init_adc()

    def close_all(self) -> None:
        self.close_spi()
        self.cleanup_gpio()
        self.stop_pigpio_clock()

    # -------------------------
    # Low-level ADC operations
    # -------------------------
    def spi_transfer(self, payload: List[int]) -> List[int]:
        if self.spi is None:
            raise RuntimeError("SPI not open")

        GPIO.output(cfg.GPIO_CS, GPIO.LOW)
        time.sleep(10e-6)
        data = self.spi.xfer2(payload)
        time.sleep(10e-6)
        GPIO.output(cfg.GPIO_CS, GPIO.HIGH)
        return data

    def send_command(self, cmd: int) -> List[int]:
        payload = [(cmd >> 8) & 0xFF, cmd & 0xFF, 0x00] + [0x00] * (cfg.FRAME_BYTES - 3)
        return self.spi_transfer(payload)

    def read_register(self, reg: int) -> int:
        # ADS131M04 commands are 16-bit command + 8-bit padding in a 24-bit word.
        # Use the same two-frame read style that worked previously.
        cmd = 0xA000 | ((reg & 0x3F) << 7)
        payload1 = [(cmd >> 8) & 0xFF, cmd & 0xFF, 0x00] + [0x00] * (cfg.FRAME_BYTES - 3)
        self.spi_transfer(payload1)

        payload2 = [0x00] * cfg.FRAME_BYTES
        response = self.spi_transfer(payload2)

        # Register response appears in the first output word of the following frame.
        if len(response) >= 2:
            return (response[0] << 8) | response[1]
        return 0

    def write_register(self, reg: int, value: int) -> None:
        cmd = 0x6000 | ((reg & 0x3F) << 7)
        payload = [
            (cmd >> 8) & 0xFF, cmd & 0xFF, 0x00,
            (value >> 8) & 0xFF, value & 0xFF, 0x00,
        ] + [0x00] * (cfg.FRAME_BYTES - 6)
        self.spi_transfer(payload)

    def wait_drdy(self, timeout_s: Optional[float] = None) -> bool:
        """Wait until DRDY is low (active-low). Used after RESET and for throwaway reads."""
        if timeout_s is None:
            timeout_s = cfg.DRDY_TIMEOUT_S
        deadline = time.monotonic() + timeout_s

        while GPIO.input(cfg.GPIO_DRDY):
            if time.monotonic() > deadline:
                return False
            time.sleep(0.00005)

        return True

    def wait_drdy_fresh(self, timeout_s: Optional[float] = None) -> bool:
        """Wait for a guaranteed fresh DRDY falling edge after a MUX switch.

        After switching the MUX the ADC may already have DRDY asserted (low) from a
        conversion that started on the PREVIOUS channel. Calling wait_drdy() directly
        returns immediately in that case, so you read stale data from the wrong channel.

        This method first waits for DRDY to go HIGH (deasserted, i.e. the ADC has
        acknowledged/completed the stale conversion) and then waits for the next
        falling edge (a genuine conversion on the new channel).
        """
        if timeout_s is None:
            timeout_s = cfg.DRDY_TIMEOUT_S
        deadline = time.monotonic() + timeout_s

        # Step 1: wait for DRDY to go high (flush any pending assertion)
        while not GPIO.input(cfg.GPIO_DRDY):
            if time.monotonic() > deadline:
                return False
            time.sleep(0.00005)

        # Step 2: wait for the next falling edge (fresh conversion on new channel)
        while GPIO.input(cfg.GPIO_DRDY):
            if time.monotonic() > deadline:
                return False
            time.sleep(0.00005)

        return True

    def read_status_word(self) -> int:
        resp = self.send_command(CMD_NULL)
        if len(resp) >= 2:
            return (resp[0] << 8) | resp[1]
        return 0

    def configure_adc_registers(self) -> None:
        self.send_command(CMD_UNLOCK)

        # CLOCK register: enable all 4 channels (bits 11..8), OSR field (bits 4..2),
        # PWR=10 high-resolution (bits 1..0). 0x0F0E with OSR=011 -> 4000 SPS @ 8.192 MHz.
        ch_en_mask = 0b1111 << 8
        osr_field = (ADC_OSR_SETTING & 0b111) << 2
        pwr_hr = 0b10
        clock_val = ch_en_mask | osr_field | pwr_hr
        self.write_register(REG_CLOCK, clock_val)

        # MODE: preserve reset default (WLENGTH=01 -> 24-bit words, TIMEOUT enabled).
        self.write_register(REG_MODE, 0x0510)

        # GAIN1: all gains x1
        self.write_register(REG_GAIN1, 0x0000)

        self.send_command(CMD_LOCK)

    def init_adc(self) -> None:
        """Initialize ADC without aborting on unreliable ID read."""
        log.info("Initializing ADS131M04...")

        status_words = []
        if cfg.ADC_RESET_ON_INIT:
            # Issue RESET first thing so the chip is in a known state regardless of any
            # leftover SPI framing from a previous run.
            self.send_command(CMD_RESET)
            time.sleep(0.05)

            # The first NULL frames after reset are a cheap probe for whether SPI is alive,
            # but on this PCB the response can be unstable and DRDY may stay high after RESET.
            for _ in range(3):
                status = self.read_status_word()
                status_words.append(status)
                time.sleep(0.005)
            status_text = ", ".join(f"0x{word:04X}" for word in status_words)
            log.info("Post-RESET NULL status probes: %s", status_text)
            if any(word in (0x0000, 0xFFFF) for word in status_words):
                log.warning("At least one post-RESET status probe was 0x0000/0xFFFF; check CS, MISO, power, and CLKIN wiring.")

            if not self.wait_drdy(timeout_s=0.25):
                raise RuntimeError(
                    f"DRDY did not assert after CMD_RESET. Current DRDY={GPIO.input(cfg.GPIO_DRDY)}, "
                    f"post-RESET status probes={status_text}. "
                    f"This board is known to wedge after SPI reset; power-cycle the ADC and set "
                    f"ADC_RESET_ON_INIT = False for normal logging."
                )

            log.info("DRDY asserted after CMD_RESET.")
        else:
            log.info("Skipping CMD_RESET on this PCB; using WAKEUP/no-reset init path.")
            self.send_command(CMD_WAKEUP)
            time.sleep(0.02)
            for _ in range(3):
                status = self.read_status_word()
                status_words.append(status)
                time.sleep(0.005)
            status_text = ", ".join(f"0x{word:04X}" for word in status_words)
            log.info("Initial NULL status probes without RESET: %s", status_text)
            if GPIO.input(cfg.GPIO_DRDY):
                log.warning(
                    "DRDY is still high during no-reset startup (status probes=%s). "
                    "Continuing with register configuration and letting the first frame read decide whether conversions are flowing.",
                    status_text,
                )
            else:
                log.info("DRDY already low on no-reset startup.")

        dev_id = self.read_register(REG_ID)
        if dev_id == 0 or dev_id == 0xFFFF:
            log.warning("ADS131M04 ID read returned %s; continuing and testing DRDY/data",
                        hex(dev_id))
        else:
            log.info("ADS131M04 ID read: %s", hex(dev_id))

        self.configure_adc_registers()
        log.info("ADC initialized: OSR setting=%d, nominal SPS=4000", ADC_OSR_SETTING)

    # -------------------------
    # MUX / data acquisition
    # -------------------------
    def set_mux(self, addr: int) -> None:
        if not (0 <= addr <= 7):
            raise ValueError("MUX address must be 0-7")

        for i, pin in enumerate(cfg.MUX1_PINS):
            GPIO.output(pin, (addr >> i) & 0x01)

        for i, pin in enumerate(cfg.MUX2_PINS):
            GPIO.output(pin, (addr >> i) & 0x01)

        # Allow time for the analog input to settle after switching.
        # 100 µs was sufficient for a single whisker but too short for 16 sensors
        # due to increased wiring capacitance and source impedance on each MUX input.
        time.sleep(0.002)

    def read_adc_frame(self, fresh_edge: bool = False) -> Optional[List[int]]:
        """Read one 18-byte SPI frame and return 4 signed 24-bit channel values.

        Args:
            fresh_edge: If True, use wait_drdy_fresh() to guarantee a new conversion
                        on the current MUX channel rather than catching a stale DRDY
                        from a previous channel. Always True for the first read after
                        a MUX switch.
        """
        ok = self.wait_drdy_fresh() if fresh_edge else self.wait_drdy()
        if not ok:
            self.timeout_count += 1
            return None

        raw = self.spi_transfer([0x00] * cfg.FRAME_BYTES)
        if len(raw) < 15:
            self.timeout_count += 1
            return None

        return [
            conv24(raw[3], raw[4], raw[5]),
            conv24(raw[6], raw[7], raw[8]),
            conv24(raw[9], raw[10], raw[11]),
            conv24(raw[12], raw[13], raw[14]),
        ]

    def read_32_counts(self) -> Optional[List[int]]:
        values: List[int] = []

        for addr in range(cfg.NUM_MUX_POSITIONS):
            self.set_mux(addr)

            # First read after a MUX switch must use wait_drdy_fresh() to flush any
            # stale DRDY assertion left over from the previous channel.
            first = True
            for _ in range(cfg.THROWAWAY_CONVERSIONS):
                self.read_adc_frame(fresh_edge=first)
                first = False

            ch = self.read_adc_frame(fresh_edge=first)
            if ch is None:
                return None

            values.extend(ch)

        if len(values) != cfg.NUM_OUTPUT_CHANNELS:
            return None

        return values

    def reinitialize(self) -> None:
        self.reinit_count += 1
        log.info("Reinitializing ADC/clock/SPI path (count=%d)...", self.reinit_count)

        self.close_spi()
        self.stop_pigpio_clock()
        time.sleep(0.2)

        self.start_pigpio_clock()
        time.sleep(0.1)
        self.open_spi()
        self.init_adc()

    # -------------------------
    # Output files
    # -------------------------
    def open_output_files(self) -> OutputFiles:
        out_dir = Path(cfg.OUTPUT_DIR)
        out_dir.mkdir(parents=True, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        voltage_path = out_dir / f"whisker16_32ch_voltage_{ts}.dat"
        counts_path = out_dir / f"whisker16_32ch_counts_{ts}.dat"

        vf = open(voltage_path, "w", buffering=1) if cfg.WRITE_VOLTAGE_DAT else None
        cf = open(counts_path, "w", buffering=1) if cfg.WRITE_RAW_COUNTS_DAT else None

        log.info("Voltage .dat: %s", voltage_path if vf else "disabled")
        log.info("Counts  .dat: %s", counts_path if cf else "disabled")

        return OutputFiles(
            voltage_path=voltage_path if vf else None,
            counts_path=counts_path if cf else None,
            voltage_file=vf,
            counts_file=cf,
        )

    def write_dat_row(self, outputs: OutputFiles, counts: List[int]) -> None:
        date_str, time_str = now_strings()

        if outputs.counts_file is not None:
            vals = "".join(" " + cfg.COUNTS_FORMAT.format(v) for v in counts)
            outputs.counts_file.write(f"{date_str} {time_str}{vals}\n")

        if outputs.voltage_file is not None:
            volts = [count_to_voltage(v) for v in counts]
            vals = "".join(" " + cfg.VOLTAGE_FORMAT.format(v) for v in volts)
            outputs.voltage_file.write(f"{date_str} {time_str}{vals}\n")

        self.frames_written += 1

    # -------------------------
    # Main loop
    # -------------------------
    def run_forever(self) -> None:
        outputs = self.open_output_files()
        start = time.monotonic()
        last_reinit = start
        next_sample = start
        consecutive_timeouts = 0

        log.info("Starting 32-channel logging at %.2f Hz. Press Ctrl+C to stop.", cfg.TARGET_HZ)

        try:
            self.setup_all()

            while self.running:
                now = time.monotonic()

                # Periodic reinit avoids the long-run DRDY timeout behavior observed in testing.
                if now - last_reinit >= cfg.REINIT_EVERY_S:
                    self.reinitialize()
                    last_reinit = time.monotonic()
                    next_sample = last_reinit
                    consecutive_timeouts = 0

                # Fixed-rate pacing
                wait = next_sample - time.monotonic()
                if wait > 0:
                    time.sleep(wait)

                counts = self.read_32_counts()

                if counts is None:
                    consecutive_timeouts += 1
                    log.warning("Frame skipped due to DRDY timeout/read error (consecutive=%d)",
                                consecutive_timeouts)

                    if consecutive_timeouts >= 3:
                        self.reinitialize()
                        last_reinit = time.monotonic()
                        next_sample = last_reinit
                        consecutive_timeouts = 0
                    else:
                        next_sample = time.monotonic() + 1.0 / cfg.TARGET_HZ
                    continue

                consecutive_timeouts = 0
                self.write_dat_row(outputs, counts)

                if self.frames_written % int(max(1, cfg.TARGET_HZ * 2)) == 0:
                    elapsed = time.monotonic() - start
                    log.info("frames=%d elapsed=%.1fs timeouts=%d reinit=%d Ch1=%d",
                             self.frames_written, elapsed, self.timeout_count,
                             self.reinit_count, counts[0])

                next_sample += 1.0 / cfg.TARGET_HZ
                if next_sample < time.monotonic():
                    next_sample = time.monotonic() + 1.0 / cfg.TARGET_HZ

        except KeyboardInterrupt:
            log.info("Manual stop requested.")
        finally:
            outputs.close()
            self.close_all()
            log.info("Stopped. Frames written=%d, timeouts=%d, reinitializations=%d",
                     self.frames_written, self.timeout_count, self.reinit_count)
            if outputs.voltage_path is not None:
                log.info("Saved voltage file: %s", outputs.voltage_path)
            if outputs.counts_path is not None:
                log.info("Saved raw-count file: %s", outputs.counts_path)


def main() -> None:
    logger = ADS131M04Logger()
    logger.run_forever()


if __name__ == "__main__":
    main()
