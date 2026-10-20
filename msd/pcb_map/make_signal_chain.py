"""Write the whisker signal-chain schematic page (one whisker, W1, both gauges), a standalone HTML.

The companion of make_map.py: the header map shows how the board connects externally, this page
shows the internal structure for one whisker, from the gauges to the host. Hand-drawn from the
traced Altium nets (see README.md); no inputs.

    python msd/pcb_map/make_signal_chain.py whisker_signal_chain.html
"""
import sys

GA, GB = "var(--ga)", "var(--gb)"
o = []


def line(*pts, cls="w", col=None):
    d = "M" + " L".join(f"{x},{y}" for x, y in pts)
    o.append(f'<path d="{d}" class="{cls}"' + (f' style="stroke:{col}"' if col else "") + "/>")


def text(x, y, s, cls="t", anchor="start", col=None):
    o.append(f'<text x="{x}" y="{y}" class="{cls}" text-anchor="{anchor}"'
             + (f' style="fill:{col}"' if col else "") + f">{s}</text>")


def dot(x, y):
    o.append(f'<circle cx="{x}" cy="{y}" r="3" class="dot"/>')


def res_v(x, y0, y1, label, val, side="r", col=None):
    """Vertical resistor between (x,y0) and (x,y1), IEC box in the middle."""
    m = (y0 + y1) / 2
    line((x, y0), (x, m - 18), col=col); line((x, m + 18), (x, y1), col=col)
    o.append(f'<rect x="{x - 6}" y="{m - 18}" width="12" height="36" class="res"' + (f' style="stroke:{col}"' if col else "") + "/>")
    dx, a = (12, "start") if side == "r" else (-12, "end")
    text(x + dx, m - 2, label, "lab", a)
    text(x + dx, m + 12, val, "val", a)


def res_h(x0, x1, y, label, val):
    m = (x0 + x1) / 2
    line((x0, y), (m - 18, y)); line((m + 18, y), (x1, y))
    o.append(f'<rect x="{m - 18}" y="{y - 6}" width="36" height="12" class="res"/>')
    text(m, y - 11, label, "lab", "middle"); text(m, y + 22, val, "val", "middle")


def cap_v(x, y0, y1, label, val):
    m = (y0 + y1) / 2
    line((x, y0), (x, m - 4)); line((x, m + 4), (x, y1))
    line((x - 11, m - 4), (x + 11, m - 4), cls="plate"); line((x - 11, m + 4), (x + 11, m + 4), cls="plate")
    text(x + 16, m - 2, label, "lab"); text(x + 16, m + 12, val, "val")


def gnd(x, y):
    line((x - 10, y), (x + 10, y), cls="g"); line((x - 6, y + 4), (x + 6, y + 4), cls="g"); line((x - 2, y + 8), (x + 2, y + 8), cls="g")


def flag(x, y, name, cls="f3v3", up=True):
    """Net flag: a short stub and a label box, pointing up (power) or down."""
    if up:
        line((x, y), (x, y - 10))
        o.append(f'<rect x="{x - 19}" y="{y - 26}" width="38" height="16" rx="3" class="{cls}"/>')
        text(x, y - 14, name, "flagt", "middle")
    else:
        line((x, y), (x, y + 10))
        o.append(f'<rect x="{x - 19}" y="{y + 10}" width="38" height="16" rx="3" class="{cls}"/>')
        text(x, y + 22, name, "flagt", "middle")


def pin(x, y, label, below=False):
    o.append(f'<circle cx="{x}" cy="{y}" r="5" class="hpin"/>')
    text(x, y + (20 if below else -10), label, "pinl", "middle")


def channel(oy, g, col, out, sens_pin, vcc_pin, conn, sel):
    """One gauge: whisker gauge -> J2 -> quarter bridge -> INA351 -> RC -> OUTn."""
    yv, ys, yp = oy + 30, oy + 140, oy + 75          # 3V3 lead, sense line, IN+ line
    # gauge in the whisker (off board)
    text(64, oy + 8, f"gauge {g}", "gname", "middle", col)
    res_v(64, yv, ys, "", "", col=col)
    text(52, oy + 89, "~350 Ω", "val", "end")
    text(64, ys + 18, "(+ series R)", "note", "middle")
    line((64, yv), (150, yv), cls="lead", col=col); line((64, ys), (150, ys), cls="lead", col=col)
    pin(150, yv, f"J2-{vcc_pin}"); pin(150, ys, f"J2-{sens_pin}")
    # 3V3 side of the gauge
    line((155, yv), (200, yv)); flag(200, yv, "3V3")
    # sense node: R3 to GND, on to IN-
    line((155, ys), (250, ys)); dot(250, ys)
    res_v(250, ys, oy + 205, "R3", "352 Ω", "l"); gnd(250, oy + 205)
    text(200, ys - 6, f"sense", "net", "middle")
    line((250, ys), (420, ys))
    # reference arm R1/R2 -> IN+
    flag(330, oy + 5, "3V3")
    res_v(330, oy + 5, yp, "R1", "123 Ω")
    dot(330, yp)
    res_v(330, yp, oy + 118, "R2", "123 Ω"); gnd(330, oy + 118)
    line((330, yp), (420, yp)); text(385, yp - 6, "1.65 V", "net", "middle")
    # INA351
    ya = (yp + ys) / 2
    o.append(f'<path d="M420,{yp - 25} L420,{ys + 25} L510,{ya} Z" class="amp"/>')
    text(426, yp + 5, "+", "pm"); text(426, ys + 5, "−", "pm")
    text(455, ya + 3, "INA351", "ampt", "middle"); text(455, ya + 17, "G = 10", "val", "middle")
    line((462, ys + 4), (462, ys + 32)); text(468, ys + 24, "REF", "lab")
    o.append(f'<rect x="443" y="{ys + 32}" width="38" height="16" rx="3" class="fref"/>'); text(462, ys + 44, "REF", "flagt", "middle")
    # output RC
    line((510, ya), (530, ya))
    res_h(530, 610, ya, "", "1 k")
    dot(630, ya); line((610, ya), (800, ya))
    cap_v(630, ya, ya + 55, "", "0.1 µF"); gnd(630, ya + 55)
    text(715, ya - 8, f"OUT{out}", "outl", "middle", col)
    o.append(f'<path d="M800,{ya} m-7,-5 l7,5 l-7,5" class="w"/>')
    text(806, ya + 4, f"X{sel}", "lab"); text(838, ya + 4, f"(sel {sel})", "val")
    return ya


W, H = 1260, 860
o.append(f'<svg class="sch" viewBox="0 0 {W} {H}" role="img" aria-label="Signal chain schematic for whisker W1: two strain gauges through quarter bridges, INA351 amplifiers, RC filters, the CD4097B mux and the ADS131M04 ADC to the ESP32">')

o.append('<rect x="132" y="40" width="930" height="800" rx="10" class="board"/>')
text(146, 830, "whisker PCB (P25937 rev 2)", "blk")
o.append('<rect x="14" y="62" width="100" height="440" rx="14" class="whisker"/>')
text(64, 522, "whisker W1", "blk", "middle"); text(64, 538, "(off board)", "note", "middle")
# chip bodies first, so the pin labels drawn later sit on top
MX0, MX1 = 800, 900
AX0, AX1 = 950, 1066
EX0, EX1 = 1130, 1246
o.append(f'<rect x="{MX0}" y="110" width="{MX1 - MX0}" height="400" rx="4" class="chip"/>')
o.append(f'<rect x="{AX0}" y="200" width="{AX1 - AX0}" height="300" rx="4" class="chip"/>')
o.append(f'<rect x="{EX0}" y="200" width="{EX1 - EX0}" height="330" rx="8" class="esp"/>')

ya_a = channel(80, "A", GA, 1, 1, 3, "CONN2", 3)
ya_b = channel(320, "B", GB, 2, 4, 2, "CONN4", 4)

# mux (CD4097B #1, X section)
text((MX0 + MX1) / 2, 132, "CD4097B #1", "chipt", "middle"); text((MX0 + MX1) / 2, 147, "X section", "val", "middle")
for i, s_ in enumerate(("X0–X7 = OUT", "5, 7, 3, 1,", "2, 4, 6, 8")):
    text((MX0 + MX1) / 2, 290 + i * 14, s_, "note", "middle")
ycom = 300
line((MX1, ycom + 40), (950, ycom + 40)); text(925, ycom + 32, "COM_X1", "net", "middle")
text((MX0 + MX1) / 2, 498, "A   B   C", "val", "middle")
selx = (828, 850, 872)
# ADC
text((AX0 + AX1) / 2, 222, "ADS131M04", "chipt", "middle"); text((AX0 + AX1) / 2, 237, "U4", "val", "middle")
text(AX0 + 6, ycom + 44, "AIN2P", "lab")
text(AX0 + 6, ycom + 84, "AIN2N", "lab"); line((AX0, ycom + 80), (930, ycom + 80), (930, ycom + 92)); gnd(930, ycom + 92)
for i, s_ in enumerate(("±1.2 V FS", "gain 1", "int. 1.2 V ref", "OSR 128", "3125 SPS")):
    text((AX0 + AX1) / 2 - 8, 420 + i * 14 - 20, s_, "val", "middle")
text((AX0 + AX1) / 2, 482, "AIN0/1/3: other", "note", "middle"); text((AX0 + AX1) / 2, 494, "mux outputs", "note", "middle")
p1 = [("SCLK", "GPIO18"), ("DIN", "GPIO23"), ("CS", "GPIO21"), ("CLKIN", "GPIO16"), ("DOUT", "GPIO19"), ("DRDY", "GPIO4")]
for i, (s_, g) in enumerate(p1):
    y = 262 + i * 20
    text(AX1 - 5, y + 4, s_, "lab", "end")
    line((AX1, y), (EX0, y), cls="w p1w")
    text(EX0 + 5, y + 4, g, "gpio2")
text((AX1 + EX0) / 2, 250, "P1", "p1t", "middle")
# ESP32
text((EX0 + EX1) / 2, 222, "ESP32", "espt", "middle"); text((EX0 + EX1) / 2, 237, "DevKit V1", "espv", "middle")
for i, s_ in enumerate(("per select:", "250 µs settle,", "3 discards,", "keep the 4th", "", "8 selects × 4 ch", "= 32 gauges/scan", "≈76 scans/s")):
    text((EX0 + EX1) / 2, 400 + i * 13, s_, "espv", "middle")
# host above the ESP32
o.append(f'<rect x="1080" y="62" width="170" height="70" rx="6" class="host"/>')
text(1165, 82, "host", "blk", "middle")
text(1165, 99, "laptop: wavi_py", "note", "middle"); text(1165, 112, "(--transport esp32) or", "note", "middle")
text(1165, 125, "BlueOS Pi: daq_server.py", "note", "middle")
line((1188, 200), (1188, 132)); text(1182, 170, "USB serial", "lab", "end"); text(1182, 184, "921600 Bd", "val", "end")
# P2 selects: ESP32 bottom -> under the board -> mux A/B/C
for i, (x, (s_, g)) in enumerate(zip(selx, (("SELA_1", "GPIO25"), ("SELB_1", "GPIO26"), ("SELC_1", "GPIO27")))):
    yb = 560 + i * 16
    line((x, 510), (x, yb), (1150 + i * 20, yb), (1150 + i * 20, 530), cls="w p2w")
    text(1150 + i * 20 + 4, yb + 36 - i * 16 + 14, "", "val")
text(1240, 628, "P2: SELA_1 / SELB_1 / SELC_1", "p2t", "end")
text(1240, 642, "= GPIO25 / 26 / 27", "p2t", "end")
text(1240, 658, "same 3-bit select also drives mux #2", "note", "end")

# power (lower left, on the board)
o.append('<rect x="150" y="600" width="210" height="70" rx="6" class="pwr"/>')
text(160, 620, "5.5 V bench supply", "lab"); text(160, 637, "→ J1 (pin 1 +, pin 4 GND)", "val")
text(160, 653, "→ MD1 buck (TPSM84203)", "val")
line((360, 635), (400, 635)); flag(400, 635, "3V3")
text(412, 652, "bridges, INA351s,", "note"); text(412, 665, "muxes, ADC", "note")
text(160, 692, "ESP32 GND ← wire from MD1 pin 2", "note")
# REF divider
rx, ry = 620, 640
text(rx - 40, 604, "REF divider (AMP_REF_3V3): one net for all 32 INA351 REF pins", "blk")
flag(rx, ry, "3V3")
res_v(rx, ry, ry + 80, "R7", "47 k (target)")
dot(rx, ry + 80)
res_v(rx, ry + 80, ry + 160, "R8", "10 k (target)"); gnd(rx, ry + 160)
line((rx, ry + 80), (rx + 150, ry + 80)); dot(rx + 150, ry + 80)
cap_v(rx + 150, ry + 80, ry + 140, "C4", "1 µF"); gnd(rx + 150, ry + 140)
line((rx + 150, ry + 80), (rx + 200, ry + 80))
o.append(f'<rect x="{rx + 200}" y="{ry + 72}" width="38" height="16" rx="3" class="fref"/>'); text(rx + 219, ry + 84, "REF", "flagt", "middle")
text(rx + 75, ry + 74, "≈ 0.58 V", "net", "middle")
for i, s_ in enumerate(("schematic: 100 k / 100 k = 1.65 V", "(would rail every channel)", "as built: one copy empty, the", "other modified; REF floats",
                        "two copies on the board;", "one is enough")):
    text(870, 676 + i * 14 + (8 if i >= 2 else 0) + (8 if i >= 4 else 0), s_, "note")
o.append("</svg>")
svg = "\n".join(o)

page = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Whisker Signal Chain</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Sans+Condensed:wght@600&display=swap">
<style>
:root {{
  --bg: #f3f5f2; --panel: #ffffff; --fg: #18211c; --mut: #5d6a62; --line: #d5ddd7;
  --ga: #d6402f; --gb: #2f63d6; --p1: #e0507a; --p2: #1fa3b3; --ref: #c2418f; --pwr: #c0392b;
  --chip: #f4f6f8; --wire: #2a3530; --board: #eef5f0;
  --f-display: "IBM Plex Sans Condensed", "Arial Narrow", sans-serif;
  --f-body: "IBM Plex Sans", system-ui, sans-serif;
  --f-mono: "IBM Plex Mono", ui-monospace, monospace;
}}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{
  --bg: #121815; --panel: #1a221e; --fg: #e3ebe5; --mut: #93a299; --line: #2c3832;
  --ga: #f06a5a; --gb: #6b95f0; --p1: #f0708f; --p2: #45c9d8; --ref: #e27bb7; --pwr: #ef6a5a;
  --chip: #222c27; --wire: #c9d4cd; --board: #17211c; color-scheme: dark }} }}
:root[data-theme="dark"] {{
  --bg: #121815; --panel: #1a221e; --fg: #e3ebe5; --mut: #93a299; --line: #2c3832;
  --ga: #f06a5a; --gb: #6b95f0; --p1: #f0708f; --p2: #45c9d8; --ref: #e27bb7; --pwr: #ef6a5a;
  --chip: #222c27; --wire: #c9d4cd; --board: #17211c; color-scheme: dark }}
body {{ margin: 0; background: var(--bg); color: var(--fg); font: 15px/1.55 var(--f-body);
  padding-left: max(16px, env(safe-area-inset-left, 0px)); padding-right: max(16px, env(safe-area-inset-right, 0px)); padding-block: 28px 56px; }}
main {{ max-width: 1200px; margin: 0 auto; display: grid; gap: 30px; }}
h1 {{ font: 600 2rem/1.1 var(--f-display); margin: 0; }}
h2 {{ font: 600 1.25rem/1.2 var(--f-display); margin: 0 0 6px; }}
p {{ margin: 0; max-width: 72ch; }}
.lede {{ color: var(--mut); margin-top: 8px; }}
.eyebrow {{ font: 500 .75rem var(--f-mono); letter-spacing: .08em; text-transform: uppercase; color: var(--mut); }}
a {{ color: var(--gb); }}
.card {{ background: var(--panel); border: 1px solid var(--line); border-radius: 6px; padding: 12px; }}
.scroll {{ overflow-x: auto; }}
svg.sch {{ width: 100%; min-width: 900px; height: auto; display: block; }}
.w {{ fill: none; stroke: var(--wire); stroke-width: 1.6; stroke-linejoin: round; }}
.lead {{ fill: none; stroke-width: 2.4; stroke-dasharray: 6 3; }}
.p1w {{ stroke: var(--p1); }} .p2w {{ stroke: var(--p2); }}
.res {{ fill: var(--panel); stroke: var(--wire); stroke-width: 1.6; }}
.plate {{ stroke: var(--wire); stroke-width: 2.4; }}
.g {{ stroke: var(--wire); stroke-width: 1.6; }}
.dot {{ fill: var(--wire); }}
.amp {{ fill: var(--chip); stroke: var(--wire); stroke-width: 1.8; stroke-linejoin: round; }}
.chip {{ fill: var(--chip); stroke: var(--wire); stroke-width: 1.6; }}
.esp {{ fill: #1f2a44; stroke: #0e1526; stroke-width: 1.4; }}
.host {{ fill: var(--chip); stroke: var(--line); stroke-width: 1.4; }}
.pwr {{ fill: var(--chip); stroke: var(--pwr); stroke-width: 1.4; }}
.board {{ fill: var(--board); stroke: var(--line); stroke-width: 1.4; }}
.whisker {{ fill: none; stroke: var(--mut); stroke-width: 1.4; stroke-dasharray: 5 4; }}
.hpin {{ fill: #c9a24a; stroke: #23272a; stroke-width: 1.2; }}
.f3v3 {{ fill: var(--pwr); }} .fref {{ fill: var(--ref); }}
.t, .lab {{ font: 600 11px var(--f-mono); fill: var(--fg); }}
.val {{ font: 400 10.5px var(--f-mono); fill: var(--mut); }}
.note {{ font: 400 10.5px var(--f-body); fill: var(--mut); }}
.net {{ font: 600 10.5px var(--f-mono); fill: var(--mut); }}
.pinl {{ font: 600 10.5px var(--f-mono); fill: #a07d22; }}
.flagt {{ font: 600 9.5px var(--f-mono); fill: #fff; }}
.gname {{ font: 600 12px var(--f-body); }}
.pm {{ font: 600 14px var(--f-mono); fill: var(--fg); }}
.ampt, .chipt {{ font: 600 11.5px var(--f-body); fill: var(--fg); }}
.outl {{ font: 600 12px var(--f-mono); }}
.blk {{ font: 600 12px var(--f-body); fill: var(--fg); }}
.gpio {{ font: 600 10px var(--f-mono); fill: var(--p1); }}
.gpio2 {{ font: 600 10px var(--f-mono); fill: #f3a3bb; }}
.p1t {{ font: 600 11px var(--f-mono); fill: var(--p1); }}
.p2t {{ font: 600 11px var(--f-mono); fill: var(--p2); }}
.espt {{ font: 600 13px var(--f-mono); fill: #dfe6f2; }}
.espv {{ font: 400 10px var(--f-mono); fill: #aeb8cc; }}
.grid2 {{ display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 16px; }}
@media (max-width: 760px) {{ .grid2 {{ grid-template-columns: 1fr; }} }}
ul {{ margin: 0; padding-left: 1.1em; display: grid; gap: 8px; }}
li {{ max-width: 72ch; }}
code {{ font: .88em var(--f-mono); }}
.eq {{ font: 500 .95rem var(--f-mono); background: var(--chip); border: 1px solid var(--line); border-radius: 4px; padding: 8px 10px; overflow-x: auto; white-space: nowrap; }}
.src {{ font-size: .8rem; color: var(--mut); }}
</style>
</head>
<body>
<main>
  <header>
    <div class="eyebrow">MSD P25937 · whisker PCB rev 2 · one whisker, end to end</div>
    <h1>Whisker signal chain</h1>
    <p class="lede">How one whisker (W1, gauges A and B) travels through the board: quarter bridge, INA351 amplifier, RC filter, CD4097B mux and ADS131M04 ADC to the ESP32 and the host. The other 15 whiskers are identical copies on other mux inputs. For where things plug in, see the <a href="whisker_header_map.html">header map</a>.</p>
  </header>

  <section class="card"><div class="scroll">{svg}</div></section>

  <section class="grid2">
    <div>
      <h2>Per gauge</h2>
      <ul>
        <li><b>Quarter bridge.</b> The gauge (in the whisker) runs from 3V3 to the sense node, R3 (352 Ω) from the sense node to GND. A fixed arm R1/R2 makes 1.65 V. Only the sense pin carries signal; every odd header pin is the same 3V3 net, so gauge orientation does not matter.</li>
        <li><b>INA351</b> (GS = GND, gain 10) amplifies the difference of the two arms and adds REF. Higher gauge resistance → lower sense voltage → higher output.</li>
        <li><b>RC filter</b> 1 k / 0.1 µF (≈1.6 kHz) on each output, then the net OUTn goes to a mux input. W1 gauge A = OUT1 = mux #1 X3, gauge B = OUT2 = mux #1 X4; both reach the ADC on AIN2 (COM_X1), at mux selects 3 and 4.</li>
      </ul>
      <div class="eq" style="margin-top:12px">V<sub>sense</sub> = 3.3 V × 352 / (352 + R<sub>gauge</sub>)<br>OUT = 10 × (1.65 V − V<sub>sense</sub>) + REF ≈ REF + 23 mV per Ω above 352 Ω</div>
    </div>
    <div>
      <h2>Shared parts</h2>
      <ul>
        <li><b>REF</b> (<code>AMP_REF_3V3</code>) is one net for all 32 INA351s, set by R7/R8 with C4. It sets where zero bridge signal sits on the output. The ADC measures OUT against GND (AINxN grounded), so REF drift shows on every channel at once. Target 0.58 V (47 k / 10 k); as built it is unreliable (one divider copy empty, the other modified).</li>
        <li><b>Mux and ADC.</b> Two CD4097B (X and Y sections each) feed the four ADC inputs: mux #1 X → AIN2, Y → AIN3 (OUT1–16); mux #2 X → AIN0, Y → AIN1 (OUT17–32). One 3-bit select drives both muxes, so 8 selects × 4 ADC channels = 32 gauges per scan.</li>
        <li><b>ADC</b> ADS131M04: ±1.2 V full scale, gain 1, internal 1.2 V reference, OSR 128 at CLKIN 0.8 MHz (from ESP32 GPIO16) → 320 µs per conversion. Per select the firmware settles 250 µs, drops 3 conversions and keeps the 4th (sinc3 filter flushed), ≈1.6 ms per select, ≈76 scans/s.</li>
        <li><b>Power</b>: 5.5 V bench supply → J1 → MD1 buck → 3V3 for the bridges, amplifiers, muxes and ADC. The ESP32 is powered by USB and shares ground through a wire from MD1 pin 2.</li>
      </ul>
    </div>
  </section>
  <p class="src">Drawn from the traced nets of <code>PCB0603.PcbDoc</code> and the <code>MAIN</code> / <code>Amplifier_Reference</code> schematic sheets (<code>P25937_Whisker_Final.zip</code>), the mux table <code>OUT_OF</code> and timing constants in <code>msd/esp32/whisker_daq/whisker_daq.ino</code>. Generated by <code>msd/pcb_map/make_signal_chain.py</code>.</p>
</main>
</body>
</html>
"""
open(sys.argv[1], "w").write(page)
print("ok", len(page))
