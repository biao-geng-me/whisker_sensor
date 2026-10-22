import sys, numpy as np
sys.path.insert(0, '/home/bigeme/.herdr/worktrees/whisker_sensor_repo/chat/msd/esp32')
import burst_spectrum as bs
from pathlib import Path
# 3125 SPS bursts: each sample is one OSR-128 sinc3 result (window 3 x 320 us).
# A scheme "K" averages K consecutive results per reading, one reading per 12.48 ms (39 samples, 80.1 Hz).
runs = [('res_352_369', 0, 2, '330+22 ohm'), ('res_352_369', 0, 3, '369 ohm'),
        ('w6a_sel2', 0, 2, 'W2 gauge'), ('w6a_sel2', 0, 3, 'W6 gauge')]
print("reading = average of K consecutive OSR-128 results; span = (K+2) x 320 us; sigma in mV of an 80.1 Hz series")
print("K:      " + "".join(f"{k:>9d}" for k in (1, 2, 3, 4, 5, 8)))
print("span ms " + "".join(f"{(k+2)*0.32:9.2f}" for k in (1, 2, 3, 4, 5, 8)))
for fn, bi, c, lab in runs:
    b = bs.load(Path(f'data/{fn}.npz'))[bi]
    x = b.codes[:, c].astype(float) * bs.MV_PER_CODE
    out = []
    for K in (1, 2, 3, 4, 5, 8):
        y = np.convolve(x, np.ones(K) / K, 'valid')
        # all 39 phases, average sigma
        s = np.mean([y[ph::39].std() for ph in range(39)])
        out.append(s)
    print(f"{lab:8s}" + "".join(f"{s:9.4f}" for s in out))
