"""lk_xcore.h (the firmware's cross-core queue and park/run handshake) under
ThreadSanitizer: xcore_tsan_test.cpp with two real threads. Skips without a g++
that has libtsan.

Run: python -m pytest msd/esp32/host_sim/test_xcore_tsan.py   (CXX=... if needed)
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from cxx_probe import find_cxx  # noqa: E402


def test_xcore_under_tsan(tmp_path):
    cxx = find_cxx("-fsanitize=thread")
    if not cxx:
        pytest.skip("no C++ compiler that builds and runs -fsanitize=thread code ($CXX, conda g++, g++)")
    exe = tmp_path / "xtsan"
    r = subprocess.run([cxx, "-std=gnu++17", "-O1", "-g", "-fsanitize=thread", "-pthread",
                        f"-I{HERE.parent / 'whisker_daq'}", str(HERE / "xcore_tsan_test.cpp"),
                        "-o", str(exe)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr[-3000:]
    for _ in range(3):                       # different interleavings each run
        r = subprocess.run([str(exe)], capture_output=True, text=True, timeout=300)
        out = r.stdout + r.stderr
        assert "ThreadSanitizer" not in out, out[-4000:]
        assert r.returncode == 0 and "OK" in r.stdout, out[-2000:]
