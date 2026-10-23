"""Checks for signal_chain_anim.html: run its in-page self test (#selftest) in headless Chromium and
compare with the signal chain page's own worked examples (make_signal_chain.py), the settle model and
the firmware's discard rules.

    CHROMIUM=~/app/chromium pytest msd/pcb_map/test_signal_chain_anim.py

Needs a Chromium/Chrome binary (env CHROMIUM, else chromium/google-chrome on PATH); skipped without.
On the cluster run it on a compute node (srun -p debug -q qos_tier3 -A flowlab ...).
"""
import ast
import json
import math
import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
PAGE = HERE / "signal_chain_anim.html"
GEN = HERE / "make_signal_chain.py"


def chain_page_funcs():
    """VR, walk, mod_bits, movsum, worst_err from make_signal_chain.py (the module writes a file on
    import, so only these definitions are executed)."""
    tree = ast.parse(GEN.read_text())
    keep = [n for n in tree.body
            if (isinstance(n, ast.FunctionDef) and n.name in {"walk", "mod_bits", "movsum", "worst_err"})
            or (isinstance(n, ast.Assign) and any(getattr(t, "id", "") == "VR" for t in n.targets))]
    ns = {}
    exec(compile(ast.Module(body=keep, type_ignores=[]), str(GEN), "exec"), ns)
    return ns


def find_chromium():
    for c in (os.environ.get("CHROMIUM"), shutil.which("chromium"), shutil.which("google-chrome"),
              str(Path.home() / "app" / "chromium")):
        if c and Path(c).exists():
            return c
    return None


@pytest.fixture(scope="module")
def st():
    exe = find_chromium()
    if not exe:
        pytest.skip("no chromium")
    env = dict(os.environ, APPIMAGE_EXTRACT_AND_RUN="1")
    out = subprocess.run([exe, "--headless", "--no-sandbox", "--disable-gpu", "--virtual-time-budget=60000",
                          "--dump-dom", PAGE.as_uri() + "#selftest"], capture_output=True, text=True,
                         timeout=300, env=env).stdout
    m = re.search(r'<pre id="selftest"[^>]*>(.*?)</pre>', out, re.S)
    assert m, "no self-test output"
    r = json.loads(m.group(1).replace("&amp;", "&").replace("&lt;", "<").replace("&gt;", ">"))
    assert "error" not in r, r
    return r


def test_first_order_walk_matches_chain_page(st):
    ns = chain_page_funcs()
    for v in (0.5674, -0.314567):
        rows = ns["walk"](v)
        for n in (32, 64):
            k, a0, b, fb, d, a1, ones, est = rows[n - 1]
            got = st["walk"][str(v)][str(n)]
            assert got["ones"] == ones
            assert got["est"] == pytest.approx(est, abs=1e-12)


def test_sinc3_and_count_accuracy_match_chain_page(st):
    ns = chain_page_funcs()
    for v in (0.5674, -0.314567):
        for n in (32, 64, 128, 256):
            for order in (1, 2):
                for filt in ("sinc3", "count"):
                    want = ns["worst_err"](v, n, order, filt)
                    got = st["worst"][f"{v}|{n}|{order}|{filt}"]
                    assert got == pytest.approx(want, rel=1e-9, abs=1e-12), (v, n, order, filt)


def _check_dc(rows, tol_v=5e-6):
    for r in rows:
        if r is None:
            continue
        if r["rail"]:
            assert r["code"] == 2 ** 23 - 1
        else:
            assert abs(r["code"] - r["expect"]) * 1.2 / 2 ** 23 < tol_v, r


def test_dc_code_through_the_whole_pipeline(st):
    # quiet preset, no noise, no tail: a DC input comes out as round(V / 1.2 V * 2^23) within a few uV
    _check_dc(st["dc"])
    assert any(r["rail"] for r in st["dc"])


def test_real_board_topology(st):
    _check_dc(st["dcReal"])
    s = st["realShape"]
    assert s["nch"] == 4 and s["ng"] == 32
    assert s["scanned"] == 28                       # selects 0123457: select 6 skipped
    assert s["rate"] == pytest.approx(62.5, rel=1e-9)


def test_settle_tail_follows_the_model(st):
    for r in st["tail"]:
        assert r["sim"] == pytest.approx(r["model"], rel=1e-4, abs=1e-9), r
    # past the fast part, the residual is the 0.0136 x step tail decaying with tau 80 us
    u0, u1 = st["tail"][-2], st["tail"][-1]
    assert u1["sim"] / u0["sim"] == pytest.approx(math.exp(-(u1["u"] - u0["u"]) / 80), rel=1e-3)
    # a railed source holds the pin at the rail for the overload delay
    assert st["railHold"]["pin"] == pytest.approx(st["railHold"]["rail"], abs=1e-3)


def test_discard_and_keep_logic(st):
    for name, r in st["keep"].items():
        assert r["bad"] == 0, (name, r)
        assert r["kept"] > 0
    # firmware: quiet D = ceil(3 + 210/64) - 1 = 6, wide D = ceil(3 + 210/32) - 1 = 9
    assert st["keep"]["quiet"]["D"] == 6
    assert st["keep"]["wide"]["D"] == 9


def test_scan_rates(st):
    assert st["quietRate"] == pytest.approx(62.5, rel=1e-9)
    assert 72 < st["legacyRate"] < 80          # bench: ~76 scans/s with 8 selects


def test_running_sum_sinc3_equals_direct_sum(st):
    for n, r in st["sinc3Fast"].items():
        assert r["dS"] == 0, (n, r)            # integer sums: exact
        assert r["dA"] < 1e-12, (n, r)          # analog averages: rounding only


def _perf(exe, preset, width):
    env = dict(os.environ, APPIMAGE_EXTRACT_AND_RUN="1")
    out = subprocess.run([exe, "--headless", "--no-sandbox", "--disable-gpu", f"--window-size={width},800",
                          "--dump-dom", PAGE.as_uri() + f"#perf&preset={preset}&scroll=1500&pause"],
                         capture_output=True, text=True, timeout=300, env=env).stdout
    m = re.search(r'<pre id="selftest"[^>]*>(.*?)</pre>', out, re.S)
    assert m, "no perf output"
    return json.loads(m.group(1).replace("&amp;", "&").replace("&lt;", "<").replace("&gt;", ">"))


@pytest.mark.parametrize("width", [1400, 900])
@pytest.mark.parametrize("preset", ["legacy", "quiet", "wide", "real", "teach"])
def test_page_height_and_scroll_are_constant(preset, width):
    """300 frames with every lane and every text node updated each frame: the document height and
    the scroll position must not change (no layout jumps while the animation runs)."""
    exe = find_chromium()
    if not exe:
        pytest.skip("no chromium")
    r = _perf(exe, preset, width)
    assert len(r["heights"]) == 1, r
    assert r["scrollYs"] == [r["scrollTarget"]], r
    assert r["scrollTarget"] > 0


def test_adc_noise_per_bit_is_flat(st):
    # datasheet noise ~ 1/sqrt(OSR) at OSR >= 256 -> about the same per-bit noise
    a, b = st["sigmaBit"]["256"], st["sigmaBit"]["4096"]
    assert 0.1e-3 < a < 0.5e-3 and 0.7 < a / b < 1.4
