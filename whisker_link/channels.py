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


def railed_channels(railmask: int) -> list[int]:
    """0-based channel indices whose railmask bit is set."""
    return [i for i in range(NCH) if railmask >> i & 1]
