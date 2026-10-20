"""Tiny Gerber (RS-274X, Altium output) -> primitives: strokes, regions, flashes. Units mm."""
import re, math

def parse(txt):
    """txt: the Gerber file contents."""
    fmt = re.search(r"%FSLAX(\d)(\d)Y", txt); dec = int(fmt.group(2))
    sc = 10 ** dec
    aps = {}
    for m in re.finditer(r"%ADD(\d+)([A-Za-z_0-9]+),?([^*]*)\*%", txt):
        aps[int(m.group(1))] = (m.group(2), [float(v) for v in m.group(3).split("X") if v])
    for m in re.finditer(r"%ADD(\d+)([A-Za-z_0-9]+)\*%", txt):
        aps.setdefault(int(m.group(1)), (m.group(2), []))
    body = re.sub(r"%[^%]*%", "", txt)
    x = y = 0.0; ap = None; interp = 1; region = False; cur = []
    strokes, regions, flashes = [], [], []
    for cmd in body.replace("\n", "").split("*"):
        if not cmd: continue
        for g in re.findall(r"G(\d+)", cmd):
            g = int(g)
            if g in (1, 2, 3): interp = g
            elif g == 36: region = True; cur = []
            elif g == 37:
                if cur: regions.append(cur)
                region = False; cur = []
        m = re.search(r"D(\d+)$", cmd)
        d = int(m.group(1)) if m else None
        if d is not None and d >= 10: ap = d; continue
        mx = re.search(r"X(-?\d+)", cmd); my = re.search(r"Y(-?\d+)", cmd)
        mi = re.search(r"I(-?\d+)", cmd); mj = re.search(r"J(-?\d+)", cmd)
        nx = int(mx.group(1)) / sc if mx else x; ny = int(my.group(1)) / sc if my else y
        if d is None and not (mx or my): continue
        if d is None: d = 1
        if d == 2:
            if region and cur: regions.append(cur)
            cur = [(nx, ny)] if region else cur
        elif d == 1:
            if interp == 1: pts = [(nx, ny)]
            else:
                cx = x + (int(mi.group(1)) / sc if mi else 0); cy = y + (int(mj.group(1)) / sc if mj else 0)
                a0 = math.atan2(y - cy, x - cx); a1 = math.atan2(ny - cy, nx - cx); r = math.hypot(x - cx, y - cy)
                if interp == 3:
                    while a1 <= a0: a1 += 2 * math.pi
                else:
                    while a1 >= a0: a1 -= 2 * math.pi
                k = max(4, int(abs(a1 - a0) / 0.15))
                pts = [(cx + r * math.cos(a0 + (a1 - a0) * t / k), cy + r * math.sin(a0 + (a1 - a0) * t / k)) for t in range(1, k + 1)]
            if region:
                if not cur: cur = [(x, y)]
                cur.extend(pts)
            else:
                w = aps.get(ap, ("C", [0.1]))[1]; w = w[0] if w else 0.1
                strokes.append(([(x, y)] + pts, w))
        elif d == 3:
            flashes.append((nx, ny, ap, aps.get(ap)))
        x, y = nx, ny
    return dict(strokes=strokes, regions=regions, flashes=flashes, aps=aps)
