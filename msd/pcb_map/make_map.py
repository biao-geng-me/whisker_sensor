"""Build the whisker PCB header map (standalone HTML) from the MSD P25937 Altium layout.

Reads PCB0603.PcbDoc (pad nets and positions) from the Altium zip, the top silkscreen from the
Gerber zip, and the ESP32 GPIO assignments (PIN_* defines) from the whisker_daq firmware.

    python msd/pcb_map/make_map.py out.html
    python msd/pcb_map/make_map.py out.html --altium-zip A.zip --gerber-zip G.zip --firmware F.ino
"""
import argparse, html, os, re, sys, zipfile
sys.path.insert(0, os.path.dirname(__file__))
import pcbdoc, gerber

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = "/shared/rc/whiskers/MSD_P25937"
ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
ap.add_argument("out", help="output .html")
ap.add_argument("--altium-zip", default=f"{PROJ}/P25937_Whisker_Final.zip")
ap.add_argument("--gerber-zip", default=f"{PROJ}/REV2_GERBER.zip")
ap.add_argument("--firmware", default=os.path.join(HERE, "..", "esp32", "whisker_daq", "whisker_daq.ino"))
args = ap.parse_args()


def zip_member(path, name):
    with zipfile.ZipFile(path) as z:
        hit = [n for n in z.namelist() if n.rsplit("/", 1)[-1] == name]
        if not hit:
            sys.exit(f"{name} not found in {path}")
        return z.read(hit[0])


comps, nets, pads, board = pcbdoc.load(zip_member(args.altium_zip, "PCB0603.PcbDoc"))
MM = 0.0254
HEADERS = ["J2", "J3", "J4", "J5"]
HCOL = {"J2": "var(--h1)", "J3": "var(--h2)", "J4": "var(--h3)", "J5": "var(--h4)"}


def fw(k):
    return f"W{(k + 1) // 2}_{'A' if k % 2 else 'B'}"

# --- header pins from the PCB ------------------------------------------------
hdr = {}
for i, c in enumerate(comps):
    d = c["SOURCEDESIGNATOR"]
    if d in HEADERS:
        hdr[d] = {int(q["name"]): dict(net=nets[q["net"]], x=q["x"] * MM, y=q["y"] * MM)
                  for q in pads if q["comp"] == i}

# --- ESP32 control headers: P1 = ADC SPI (to ADS131M04 U4), P2 = CD4097B selects -----
# GPIOs as in msd/esp32/whisker_daq/whisker_daq.ino (PIN_* defines). The P1 net names
# (PIN23, PIN21, ...) are Raspberry Pi 40-pin header numbers: the layout was drawn for a Pi.
ADS_PIN = {12: "CS", 13: "DRDY", 14: "SCLK", 15: "DOUT", 16: "DIN", 17: "CLKIN"}
# PIN_* defines in the firmware; P1 signal names map to the firmware's names.
_fw = dict(re.findall(r"^#define\s+PIN_(\w+)\s+(\d+)", open(args.firmware).read(), re.M))
FW_GPIO = {k: int(_fw[{"DOUT": "MISO", "DIN": "MOSI"}.get(k, k)]) for k in
           ("SCLK", "DOUT", "DIN", "CS", "DRDY", "CLKIN",
            "SELC_1", "SELB_1", "SELA_1", "SELA_2", "SELB_2", "SELC_2")}
FW_DIR = {"DOUT": "ADC → ESP32", "DRDY": "ADC → ESP32"}
CTL, CTL_SIG = {}, {"P1": {}, "P2": {}}
CTL_TITLE = {"P1": "ADC SPI", "P2": "mux selects"}
for i, c in enumerate(comps):
    d = c["SOURCEDESIGNATOR"]
    if d in ("P1", "P2", "J1", "MD1"):
        pp = {q["name"]: dict(net=nets[q["net"]] if q["net"] < len(nets) else None, x=q["x"] * MM, y=q["y"] * MM)
              for q in pads if q["comp"] == i}
        if d == "J1":
            J1 = {int(k): v for k, v in pp.items() if k.isdigit()}
        elif d == "MD1":      # TPSM84203 3.3 V buck module, through-hole: 1 VIN, 2 GND, 3 VOUT
            MD1 = {int(k): v for k, v in pp.items()}
        else:
            CTL[d] = {int(k): v for k, v in pp.items()}
u4 = {}   # net -> ADS131M04 pin
mux = {}  # net -> CD4097B pin
for q in pads:
    if q["comp"] is None or q["comp"] >= len(comps) or q["net"] is None or q["net"] >= len(nets):
        continue
    d = comps[q["comp"]]["SOURCEDESIGNATOR"]
    if d == "U4":
        u4[nets[q["net"]]] = int(q["name"])
    elif d == "U3" and nets[q["net"]].startswith("SEL"):
        mux[nets[q["net"]]] = int(q["name"])
for n, p in CTL["P1"].items():
    apin = u4[p["net"]]; name = ADS_PIN[apin]
    CTL_SIG["P1"][n] = dict(name=name, short=name, net=p["net"], chip=f"U4 pin {apin}", gpio=FW_GPIO[name],
                            dir=FW_DIR.get(name, "ESP32 → ADC"))
for n, p in CTL["P2"].items():
    name = p["net"]; m = name[-1]
    CTL_SIG["P2"][n] = dict(name=name, short=name.replace("_", ""), net=name,
                            chip=f"CD4097B #{m} pin {mux[name]}", gpio=FW_GPIO[name], dir="ESP32 → mux")
assert J1[4]["net"] == "GND" and MD1[2]["net"] == "GND"

# INA351 REF divider: MAIN.SchDoc places the Amplifier_Reference sheet twice, so there are two
# identical copies (R7 3V3->REF, R8 REF->GND, C4 REF->GND), both on the one AMP_REF_3V3 net.
REFDIV = []   # [{R7: part, R8: part, C4: part}] per copy, part = dict(x, y, nets)
_parts = []
for i, c in enumerate(comps):
    d = c["SOURCEDESIGNATOR"]
    if d in ("R7", "R8", "C4"):
        _parts.append(dict(d=d, x=float(c["X"][:-3]) * MM, y=float(c["Y"][:-3]) * MM, rot=float(c["ROTATION"]),
                           nets={q["name"]: nets[q["net"]] for q in pads if q["comp"] == i and q["net"] < len(nets)}))
for yc in sorted({round(p["y"] / 10) for p in _parts}):
    REFDIV.append({p["d"]: p for p in _parts if round(p["y"] / 10) == yc})
for cp in REFDIV:
    assert set(cp) == {"R7", "R8", "C4"}
    assert sorted(cp["R7"]["nets"].values()) == ["3V3", "AMP_REF_3V3"]
    assert sorted(cp["R8"]["nets"].values()) == ["AMP_REF_3V3", "GND"]
    assert sorted(cp["C4"]["nets"].values()) == ["AMP_REF_3V3", "GND"]

# Bench state of this board (hand-entered, 2026-10-21): what is fitted on each copy.
REFDIV_STATUS = {"top": "fitted: 45k + 100k", "bottom": "not populated"}

# gauge = (signal pin, 3V3 pin). Schematic MAIN sheet: CONN(4k+1)->pin 4k+3, CONN(4k+3)->pin 4k+2;
# the odd CONN nets are 3V3 on the PCB, the even ones carry the signal.
gauges = []
for h in HEADERS:
    pins = hdr[h]
    for k in range(4):
        for sig, vcc in ((4 * k + 1, 4 * k + 3), (4 * k + 4, 4 * k + 2)):
            s, v = pins[sig]["net"], pins[vcc]["net"]
            assert s.startswith("CONN") and v == "3V3", (h, sig, s, vcc, v)
            m = int(s[4:]); out = m // 2
            gauges.append(dict(h=h, sig=sig, vcc=vcc, conn_sig=m, conn_vcc=m - 1, out=out,
                               w=fw(out)[1:].replace("_", ""), fw=fw(out)))
assert sorted(g["out"] for g in gauges) == list(range(1, 33))
G = {(g["h"], p): g for g in gauges for p in (g["sig"], g["vcc"])}

# --- board overview SVG (mm, y up -> flip) -------------------------------------
vx = [float(board[f"VX{i}"][:-3]) * MM for i in range(9)]
vy = [float(board[f"VY{i}"][:-3]) * MM for i in range(9)]
W, H = max(vx), max(vy)
fy = lambda y: H - y
pad = 6
leg = gerber.parse(zip_member(args.gerber_zip, "PCB0603_Legend_Top.gbr").decode("latin1"))
silk = []
for pts, w in leg["strokes"]:
    silk.append("M" + " L".join(f"{x:.2f},{fy(y):.2f}" for x, y in pts))
regions = ["M" + " L".join(f"{x:.2f},{fy(y):.2f}" for x, y in r) + "Z" for r in leg["regions"]]
o = []
o.append(f'<svg class="board" viewBox="{-pad} {-pad - 4} {W + 2 * pad} {H + 2 * pad + 8}" role="img" '
         f'aria-label="Top view of the whisker PCB with headers J2 to J5 marked">')
o.append(f'<path d="M{" L".join(f"{x:.2f},{fy(y):.2f}" for x, y in zip(vx, vy))}Z" class="pcb"/>')
for c in comps:
    if c["SOURCEDESIGNATOR"].startswith("MH"):
        o.append(f'<circle cx="{float(c["X"][:-3]) * MM:.2f}" cy="{fy(float(c["Y"][:-3]) * MM):.2f}" r="2.6" class="mh"/>')
o.append(f'<path d="{" ".join(silk)}" class="silk"/>')
o.append(f'<path d="{" ".join(regions)}" class="silkfill"/>')
for h in HEADERS:
    xs = [p["x"] for p in hdr[h].values()]; ys = [p["y"] for p in hdr[h].values()]
    x0, x1, y0, y1 = min(xs) - 1.6, max(xs) + 1.6, min(ys) - 1.6, max(ys) + 1.6
    o.append(f'<rect x="{x0:.2f}" y="{fy(y1):.2f}" width="{x1 - x0:.2f}" height="{y1 - y0:.2f}" rx="0.8" '
             f'class="hbox" style="stroke:{HCOL[h]}"/>')
    under = len(o)
    for n, p in hdr[h].items():
        g = G[(h, n)]
        is_sig = n == g["sig"]
        shape = (f'<rect x="{p["x"] - .75:.2f}" y="{fy(p["y"]) - .75:.2f}" width="1.5" height="1.5"' if n == 1
                 else f'<circle cx="{p["x"]:.2f}" cy="{fy(p["y"]):.2f}" r="0.78"')
        o.append(shape + f' class="{"psig" if is_sig else "pvcc"}" style="{f"fill:{HCOL[h]}" if is_sig else ""}">'
                 f'<title>{h} pin {n}: {"CONN" + str(g["conn_sig"]) if is_sig else "3V3 (CONN" + str(g["conn_vcc"]) + ")"} · OUT{g["out"]}</title>'
                 + ('</rect>' if n == 1 else '</circle>'))
    ws = sorted({int(g["w"][:-1]) for g in gauges if g["h"] == h})
    inner_right = h in ("J2", "J5")            # title on the side facing the muxes
    tx = x1 + 4.2 if inner_right else x0 - 4.2
    anc = "start" if inner_right else "end"
    ym = (fy(y0) + fy(y1)) / 2
    o.append(f'<text x="{tx:.2f}" y="{ym:.2f}" text-anchor="{anc}" class="hlab" style="fill:{HCOL[h]}">{h}</text>')
    o.append(f'<text x="{tx:.2f}" y="{ym + 3:.2f}" text-anchor="{anc}" class="hsub">W{ws[0]}–W{ws[-1]}</text>')
    for g in [g for g in gauges if g["h"] == h]:
        a, b = hdr[h][g["sig"]], hdr[h][g["vcc"]]
        right = g["sig"] % 2 == 1
        gx = a["x"] + (1.35 if right else -1.35)
        gy = fy((a["y"] + b["y"]) / 2)
        o.insert(under, f'<path d="M{a["x"]:.2f},{fy(a["y"]):.2f} L{b["x"]:.2f},{fy(b["y"]):.2f}" class="pair" style="stroke:{HCOL[h]}"/>')
        o.append(f'<text x="{gx:.2f}" y="{gy + .5:.2f}" text-anchor="{"start" if right else "end"}" class="glab">{g["w"]}</text>')
# --- P1 (ADC SPI) / P2 (mux selects) and the GND points (MD1 pin 2, J1 pin 4) ------------------------
for ref, pins in CTL.items():
    col = f"var(--{ref.lower()})"
    xs = [p["x"] for p in pins.values()]; y = pins[1]["y"]
    x0, x1 = min(xs) - 1.6, max(xs) + 1.6
    o.append(f'<rect x="{x0:.2f}" y="{fy(y) - 1.6:.2f}" width="{x1 - x0:.2f}" height="3.2" rx="0.8" '
             f'class="hbox" style="stroke:{col}"/>')
    for n, p in pins.items():
        sig = CTL_SIG[ref][n]
        shape = (f'<rect x="{p["x"] - .75:.2f}" y="{fy(y) - .75:.2f}" width="1.5" height="1.5"' if n == 1
                 else f'<circle cx="{p["x"]:.2f}" cy="{fy(y):.2f}" r="0.78"')
        o.append(shape + f' class="psig" style="fill:{col}"><title>{ref} pin {n}: {sig["name"]} → ESP32 GPIO{sig["gpio"]}</title>'
                 + ('</rect>' if n == 1 else '</circle>'))
        o.append(f'<text transform="translate({p["x"] + .55:.2f},{fy(y) - 2.2:.2f}) rotate(-90)" class="clab">{sig["short"]}</text>')
        o.append(f'<text x="{p["x"]:.2f}" y="{fy(y) + 3.3:.2f}" text-anchor="middle" class="cgpio">{sig["gpio"]}</text>')
    o.append(f'<text x="{x1 + 1.2:.2f}" y="{fy(y) + .3:.2f}" class="hlab" style="fill:{col}">{ref}</text>')
    o.append(f'<text x="{x1 + 1.2:.2f}" y="{fy(y) + 2.6:.2f}" class="hsub">{CTL_TITLE[ref]}</text>')
g4 = J1[4]
o.append(f'<circle cx="{g4["x"]:.2f}" cy="{fy(g4["y"]):.2f}" r="1.25" class="gnd"><title>J1 pin 4: GND (through-hole)</title></circle>')
o.append(f'<text x="{g4["x"] - 2:.2f}" y="{fy(g4["y"]) + .7:.2f}" text-anchor="end" class="gndlab">GND</text>')
for cp in REFDIV:
    xs = [p["x"] for p in cp.values()]; yc = sum(p["y"] for p in cp.values()) / 3
    o.append(f'<rect x="{min(xs) - 1.6:.2f}" y="{fy(yc) - 1.3:.2f}" width="{max(xs) - min(xs) + 3.2:.2f}" height="2.6" rx=".5" class="refbox">'
             f'<title>INA351 REF divider (one of two identical copies on the AMP_REF_3V3 net)</title></rect>')
    for d, p in sorted(cp.items(), key=lambda kv: kv[1]["x"]):
        w, h = (1.6, 0.8) if abs(p["rot"]) % 180 < 1 else (0.8, 1.6)
        role = {"R7": "3V3 → REF", "R8": "REF → GND", "C4": "1 µF, REF → GND"}[d]
        o.append(f'<rect x="{p["x"] - w / 2:.2f}" y="{fy(p["y"]) - h / 2:.2f}" width="{w}" height="{h}" class="refpart refpart-{d.lower()}">'
                 f'<title>{d}: {role}</title></rect>')
        ty = fy(yc) + (3.6 if yc > H / 2 else -2.4)
        o.append(f'<text x="{p["x"]:.2f}" y="{ty:.2f}" text-anchor="middle" class="reflab">{d}</text>')
    top = yc > H / 2
    xm = (min(xs) + max(xs)) / 2
    o.append(f'<text x="{xm:.2f}" y="{fy(yc) + (5.6 if top else -6.1):.2f}" text-anchor="middle" class="reftitle">REF divider</text>')
    o.append(f'<text x="{xm:.2f}" y="{fy(yc) + (7.4 if top else -4.3):.2f}" text-anchor="middle" class="refstat">{REFDIV_STATUS["top" if top else "bottom"]}</text>')
m2 = MD1[2]
o.append(f'<circle cx="{m2["x"]:.2f}" cy="{fy(m2["y"]):.2f}" r="1.35" class="gndrec"><title>MD1 pin 2 (middle lead of the 3.3 V buck module): GND, recommended ground point</title></circle>')
for k, cls in ((1, "mdvin"), (3, "md3v3")):
    q = MD1[k]
    o.append(f'<circle cx="{q["x"]:.2f}" cy="{fy(q["y"]):.2f}" r="0.6" class="{cls}"><title>MD1 pin {k}: {"VIN (supply +)" if k == 1 else "3V3 out"}</title></circle>')
o.append(f'<text x="{m2["x"]:.2f}" y="{fy(MD1[3]["y"]) + 2.9:.2f}" text-anchor="middle" class="gndlab gndreclab">GND ★</text>')
# scale bar
o.append(f'<path d="M{W - 20},{H + 4.5} h10" class="scale"/><text x="{W - 15}" y="{H + 8.6}" text-anchor="middle" class="hsub">10 mm</text>')
o.append('</svg>')
board_svg = "\n".join(o)

# --- header detail SVGs --------------------------------------------------------
def header_svg(h):
    P = 26          # px per 2.54 mm
    cxL, cxR = 0, P
    rows = 8
    LW = 175        # label column width each side
    Wd = LW * 2 + P + 40
    Hd = rows * P + 36
    ox = LW + 20 - 0; oy = 24
    X = lambda col: ox + (cxR if col else cxL)
    Y = lambda row: oy + (rows - row) * P + P / 2   # row 1 at the bottom (pin-1 end)
    s = [f'<svg class="hdr" viewBox="0 0 {Wd} {Hd}" role="img" aria-label="{h} pinout, top view">']
    s.append(f'<rect x="{ox - P / 2 - 4}" y="{oy - 4}" width="{2 * P + 8}" height="{rows * P + 8}" rx="4" class="housing"/>')
    for g in [g for g in gauges if g["h"] == h]:
        col = g["sig"] % 2          # odd pin numbers are in the right-hand column
        rs = (g["sig"] + 1) // 2; rv = (g["vcc"] + 1) // 2
        r0, r1 = min(rs, rv), max(rs, rv)
        x = X(col)
        s.append(f'<rect x="{x - P / 2 + 2.5}" y="{Y(r1) - P / 2 + 2.5}" width="{P - 5}" height="{2 * P - 5}" rx="{(P - 5) / 2}" '
                 f'class="cap" style="stroke:{HCOL[h]}"/>')
        name = g["w"]
        ym = (Y(r0) + Y(r1)) / 2
        lx = x + (P / 2 + 14 if col else -(P / 2 + 14))
        a = "start" if col else "end"
        s.append(f'<path d="M{x + (P / 2 - 2 if col else -(P / 2 - 2))},{ym} H{lx + (-4 if col else 4)}" class="lead" style="stroke:{HCOL[h]}"/>')
        s.append(f'<text x="{lx}" y="{ym - 3}" text-anchor="{a}" class="wname">'
                 f'Whisker {html.escape(name)}</text>')
        s.append(f'<text x="{lx}" y="{ym + 12}" text-anchor="{a}" class="wmeta">OUT{g["out"]} · CONN{g["conn_vcc"]}/{g["conn_sig"]} · pins {min(g["sig"], g["vcc"])}+{max(g["sig"], g["vcc"])}</text>')
    for n, p in hdr[h].items():
        g = G[(h, n)]; col = n % 2; row = (n + 1) // 2
        x, y = X(col), Y(row)
        sig = n == g["sig"]
        if n == 1:
            s.append(f'<rect x="{x - 7}" y="{y - 7}" width="14" height="14" class="{"psig" if sig else "pvcc"}" style="{f"fill:{HCOL[h]}" if sig else ""}"/>')
        else:
            s.append(f'<circle cx="{x}" cy="{y}" r="7" class="{"psig" if sig else "pvcc"}" style="{f"fill:{HCOL[h]}" if sig else ""}"/>')
        s.append(f'<text x="{x}" y="{y + 3.2}" text-anchor="middle" class="pnum{" pnumd" if sig else ""}">{n}</text>')
    s.append(f'<text x="{ox + P / 2}" y="{oy + rows * P + 22}" text-anchor="middle" class="edge">pin 1 end ▾</text>')
    s.append('</svg>')
    return "\n".join(s)

# table rows
def wkey(g):
    return g["out"]
rows = []
for g in sorted(gauges, key=wkey):
    rows.append(f'<tr><td>{g["w"]}</td><td>{g["fw"]}</td><td>OUT{g["out"]}</td>'
                f'<td><span class="dot" style="background:{HCOL[g["h"]]}"></span>{g["h"]}</td>'
                f'<td>{g["sig"]} <span class="mut">(CONN{g["conn_sig"]})</span></td>'
                f'<td>{g["vcc"]} <span class="mut">(CONN{g["conn_vcc"]}, 3V3)</span></td></tr>')

# --- ESP32 dev board: DOIT ESP32 DevKit V1, 30-pin (15 per side) --------------------------
# Silkscreen labels, antenna end first, top view with the antenna up and USB down.
DK_L = ["EN", "VP", "VN", "D34", "D35", "D32", "D33", "D25", "D26", "D27", "D14", "D12", "D13", "GND", "VIN"]
DK_R = ["D23", "D22", "TX0", "RX0", "D21", "D19", "D18", "D5", "TX2", "RX2", "D4", "D2", "D15", "GND", "3V3"]
_DK_ALIAS = {"VP": 36, "VN": 39, "TX0": 1, "RX0": 3, "TX2": 17, "RX2": 16}
def gpio_of(lab):
    if lab in _DK_ALIAS:
        return _DK_ALIAS[lab]
    return int(lab[1:]) if re.fullmatch(r"D\d+", lab) else None
GPIO_USE = {s["gpio"]: (ref, n, s["name"]) for ref in CTL_SIG for n, s in CTL_SIG[ref].items()}
BOARD_POS = {}
for side, lst in (("left", DK_L), ("right", DK_R)):
    for i, lab in enumerate(lst):
        g = gpio_of(lab)
        if g in GPIO_USE:
            BOARD_POS[g] = f"{side} row, pin {i + 1} ({lab})"

def devkit_svg():
    P = 21; rows = len(DK_L); colL, colR = 275, 415; oy = 96
    Wd, Hd = 690, oy + rows * P + 60
    s = [f'<svg class="dk" viewBox="0 0 {Wd} {Hd}" role="img" aria-label="ESP32 DevKit V1 30-pin board, top view, with the pins used by the whisker firmware">']
    bx0, bx1 = colL - 22, colR + 22
    s.append(f'<rect x="{bx0}" y="14" width="{bx1 - bx0}" height="{Hd - 28}" rx="6" class="dkpcb"/>')
    s.append(f'<rect x="{bx0 + 16}" y="22" width="{bx1 - bx0 - 32}" height="40" rx="2" class="dkmod"/>')
    s.append(f'<text x="{(bx0 + bx1) / 2}" y="46" text-anchor="middle" class="dkt">antenna</text>')
    s.append(f'<rect x="{(bx0 + bx1) / 2 - 18}" y="{Hd - 30}" width="36" height="22" rx="3" class="dkusb"/>')
    s.append(f'<text x="{(bx0 + bx1) / 2}" y="{Hd - 15}" text-anchor="middle" class="dkt">USB</text>')
    for side, lst, cx in (("L", DK_L, colL), ("R", DK_R, colR)):
        for i, lab in enumerate(lst):
            y = oy + i * P + P / 2
            use = GPIO_USE.get(gpio_of(lab))
            gnd = lab == "GND"
            if use:
                col = f"var(--{use[0].lower()})"
                cls, sty = "dkpin", f"fill:{col}"
            elif gnd:
                cls, sty = "dkpin dkgnd", ""
            else:
                cls, sty = "dkpin dkoff", ""
            s.append(f'<circle cx="{cx}" cy="{y}" r="6.5" class="{cls}" style="{sty}"/>')
            s.append(f'<text x="{cx + (12 if side == "L" else -12)}" y="{y + 4}" text-anchor="{"start" if side == "L" else "end"}" '
                     f'class="dklab{"" if (use or gnd) else " dkdim"}">{lab}</text>')
            if use or gnd:
                tx = cx + (-40 if side == "L" else 40)
                a = "end" if side == "L" else "start"
                if use:
                    txt = f'{use[0]}-{use[1]} · {use[2]}'
                    s.append(f'<path d="M{cx + (-9 if side == "L" else 9)},{y} H{tx + (5 if side == "L" else -5)}" class="dklead" style="stroke:{col}"/>')
                    s.append(f'<text x="{tx}" y="{y + 4}" text-anchor="{a}" class="dkuse" style="fill:{col}">{txt}</text>')
                else:
                    s.append(f'<text x="{tx}" y="{y + 4}" text-anchor="{a}" class="dkuse dkg">GND → PCB ground</text>')
    s.append('</svg>')
    return "\n".join(s)

CTL_ROLE = {"SCLK": "SPI clock", "DOUT": "SPI data out (MISO)", "DIN": "SPI data in (MOSI)",
            "CS": "SPI chip select", "DRDY": "data ready, falling edge = new conversion",
            "CLKIN": "ADC master clock (LEDC PWM, 0.8 MHz now)"}
def ctl_rows(ref):
    r = []
    for n in sorted(CTL_SIG[ref]):
        s = CTL_SIG[ref][n]
        role = CTL_ROLE[s["name"]] if s["name"] in CTL_ROLE else f'mux {s["name"][-1]} select {s["name"][3]}'
        r.append(f'<tr><td>{n}</td><td>{s["name"]}</td><td><b>GPIO{s["gpio"]}</b></td>'
                 f'<td>{BOARD_POS[s["gpio"]]}</td><td>{s["dir"]}</td><td class="mut">{s["chip"]}</td><td class="mut">{role}</td></tr>')
    return "\n".join(r)

tpl = open(os.path.join(HERE, "page.html")).read()
page = (tpl.replace("{{BOARD}}", board_svg)
           .replace("{{J5}}", header_svg("J5")).replace("{{J4}}", header_svg("J4"))
           .replace("{{J2}}", header_svg("J2")).replace("{{J3}}", header_svg("J3"))
           .replace("{{ROWS}}", "\n".join(rows))
           .replace("{{DEVKIT}}", devkit_svg()).replace("{{P1ROWS}}", ctl_rows("P1")).replace("{{P2ROWS}}", ctl_rows("P2")))
page = ('<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n'
        '<meta name="viewport" content="width=device-width, initial-scale=1">\n'
        + page.replace("<main>", "</head>\n<body>\n<main>", 1) + "\n</body>\n</html>\n")
page = page.replace("body { background", "body { margin: 0; background", 1)
open(args.out, "w").write(page)
print("ok", len(page))
