"""
Configuration for the 16-whisker / 32-channel Raspberry Pi logger.

Hardware assumptions:
- ADS131M04 ADC connected through PCB header P1
- Two MUX select groups connected through PCB header P2
- Raspberry Pi pin numbering below is BCM GPIO numbering, not physical pin numbering.

P1 wiring used by this package:
P1-1 SCLK  -> GPIO11, physical pin 23
P1-2 DRDY  -> GPIO5,  physical pin 29
P1-3 CLKIN -> GPIO4,  physical pin 7
P1-4 DOUT  -> GPIO9,  physical pin 21, MISO
P1-5 DIN   -> GPIO10, physical pin 19, MOSI
P1-6 CS    -> GPIO27, physical pin 13

P2 wiring used by this package:
P2-1 SELC_1 -> GPIO23, physical pin 16
P2-2 SELB_1 -> GPIO22, physical pin 15
P2-3 SELA_1 -> GPIO17, physical pin 11
P2-4 SELA_2 -> GPIO24, physical pin 18
P2-5 SELB_2 -> GPIO25, physical pin 22
P2-6 SELC_2 -> GPIO26, physical pin 37
"""

# =========================
# GPIO / SPI settings
# =========================
GPIO_DRDY = 5
GPIO_CS = 27
GPIO_CLKIN = 4

# SPI0 pins are fixed on Raspberry Pi:
# MOSI GPIO10, MISO GPIO9, SCLK GPIO11
SPI_BUS = 0
SPI_DEVICE = 0
SPI_MODE = 0b01
SPI_SPEED_HZ = 4_000_000

# External master clock supplied to ADS131M04 CLKIN
ADC_CLOCK_HZ = 8_192_000

# On this PCB, sending CMD_RESET can leave DRDY stuck high until the ADC is power-cycled.
# Keep reset-based init available for bench diagnostics, but do not use it in normal logging.
ADC_RESET_ON_INIT = False

# MUX address pins: bit0, bit1, bit2
MUX1_PINS = [17, 22, 23]  # SELA_1, SELB_1, SELC_1
MUX2_PINS = [24, 25, 26]  # SELA_2, SELB_2, SELC_2

# =========================
# Acquisition settings
# =========================
NUM_MUX_POSITIONS = 8
NUM_ADC_CHANNELS = 4
NUM_OUTPUT_CHANNELS = NUM_MUX_POSITIONS * NUM_ADC_CHANNELS  # 32

TARGET_HZ = 10.0
DRDY_TIMEOUT_S = 0.20

# The previous system becomes unstable after about 18-25 seconds for a single whisker.
# With 16 whiskers (8 MUX positions) this instability appears sooner (~10-15 s), so reinit
# at 8 s to stay safely ahead of the failure window.
REINIT_EVERY_S = 8.0

# Discard N conversions after every MUX switch.
# With THROWAWAY=0 the first read after a MUX switch captures a conversion that started on
# the PREVIOUS channel (the switch happens mid-conversion). Set to 2 so two full fresh
# conversions on the new channel occur before recording data.
# At 4000 SPS each conversion is 250 µs, so 2 throwaways add ~500 µs per MUX position.
THROWAWAY_CONVERSIONS = 2

# ADS131M04 frame used here:
# 3 bytes STATUS + 4 channels * 3 bytes + 3 extra/CRC bytes = 18 bytes.
# CRC is ignored because the current board/code path produced valid data after bypassing CRC.
FRAME_BYTES = 18

# =========================
# ADC voltage conversion
# =========================
# These values should be confirmed from the PCB/reference configuration.
# If not confirmed, keep the raw-count .dat file as the trusted file and treat voltage as approximate.
VREF_VOLTS = 1.2
PGA_GAIN = 1.0
ADC_FULL_SCALE_COUNTS = 2**23

# Output format
WRITE_VOLTAGE_DAT = True
WRITE_RAW_COUNTS_DAT = True

# Decimal formatting for lab-style .dat files
VOLTAGE_FORMAT = "{: .9f}"   # volts
COUNTS_FORMAT = "{: .0f}"    # raw ADC counts

# Folder where data files are saved
OUTPUT_DIR = "data_32ch"
