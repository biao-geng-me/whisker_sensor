import sys, numpy as np
sys.path.insert(0, '/home/bigeme/.herdr/worktrees/whisker_sensor_repo/chat/msd/esp32')
import burst_spectrum as bs
from scipy.signal import welch
from pathlib import Path
for fn in ('data/res_352_369.npz', 'data/w6a_sel2.npz'):
    for b in bs.load(Path(fn)):
        fs = b.meta['clkin_hz'] / 2 / b.meta['osr']
        mv = b.codes.astype(float) * bs.MV_PER_CODE
        bad = (b.flags != 0).sum()
        print(f"== {fn} CLKIN {b.meta['clkin_hz']/1e6:g} MHz OSR {b.meta['osr']} fs {fs:.1f} n {len(mv)} flagged {bad}")
        for c in (2, 3):
            x = mv[:, c] - mv[:, c].mean()
            f, p = welch(x, fs=fs, nperseg=2048, scaling='density')
            df = f[1] - f[0]; tot = p.sum() * df
            def band(a, z): m = (f >= a) & (f < z); return p[m].sum() * df
            # tones: bins > 6x median
            med = np.median(p[f > 5]); tone = p > 6 * med
            print(f"   CH{c} DC {mv[:,c].mean():7.1f} mV  sigma {x.std():.4f}  "
                  f"<40Hz {np.sqrt(band(0,40)):.4f}  40-500 {np.sqrt(band(40,500)):.4f}  >500 {np.sqrt(band(500,fs/2)):.4f}  "
                  f"tones {np.sqrt(p[tone].sum()*df):.4f}  floor {np.sqrt(med*df):.5f}/bin, floor dens {np.sqrt(med)*1000:.2f} uV/rtHz")
