"""Find a C++ compiler that really compiles (and links) with the given flags.

`g++ --version` isn't enough: on the cluster a "blindfold" g++ stub answers it
but can't build anything. Tries $CXX, the conda g++, then g++ / c++.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path

CANDIDATES = (os.environ.get("CXX"), str(Path.home() / "app/miniconda/envs/mpi/bin/g++"), "g++", "c++")
SRC = "#include <thread>\nint main() { std::thread t([]{}); t.join(); return 0; }\n"


def find_cxx(*flags: str) -> str | None:
    for c in CANDIDATES:
        if not c or not (shutil.which(c) or os.access(c, os.X_OK)):
            continue
        with tempfile.TemporaryDirectory() as d:
            src, exe = Path(d) / "probe.cpp", Path(d) / "probe"
            src.write_text(SRC)
            try:
                r = subprocess.run([c, "-std=gnu++17", *flags, "-pthread", str(src), "-o", str(exe)],
                                   capture_output=True, timeout=120)
                if r.returncode == 0 and subprocess.run([str(exe)], timeout=60).returncode == 0:
                    return c
            except (OSError, subprocess.TimeoutExpired):
                pass
    return None
