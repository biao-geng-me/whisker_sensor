"""Channel naming for the 16-whisker x 2-gauge array.

The firmware streams codes in OUT order 1..32. Whisker W's gauge A is
OUT(2W-1) and gauge B is OUT(2W), so OUT order is already grouped per whisker:
W1_A, W1_B, W2_A, ... W16_B — the same column order as whisker_plot.py's CSV.
"""

from __future__ import annotations

NUM_WHISKER = 16
NCH = 2 * NUM_WHISKER

CHANNEL_NAMES = [f"W{w}_{g}" for w in range(1, NUM_WHISKER + 1) for g in "AB"]

# |code| at or above this means the INA351 output is railed: nothing plugged in.
SAT_CODE = 8_380_000


# (ADC channel, mux select) -> OUT number (1..32), mirror of OUT_OF in
# whisker_daq.ino. OUT o is channel index o - 1.
OUT_OF = (
    (32, 30, 28, 26, 31, 29, 27, 25),
    (18, 20, 22, 24, 23, 21, 19, 17),
    (5, 7, 3, 1, 2, 4, 6, 8),
    (15, 13, 11, 9, 10, 12, 16, 14),
)


def channels_of_selects(sel_mask: int) -> list[int]:
    """0-based channel indices scanned with this mux select mask (INFO sel_mask)."""
    return sorted(OUT_OF[a][s] - 1 for s in range(8) if sel_mask >> s & 1 for a in range(4))


def railed_channels(railmask: int) -> list[int]:
    """0-based channel indices whose railmask bit is set."""
    return [i for i in range(NCH) if railmask >> i & 1]
