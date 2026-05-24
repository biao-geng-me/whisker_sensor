"""Minimum-viable example of using wavi_py as a library from an agent loop.

No Qt, no plot windows. Connects to ``rpi/daq_bridge.py`` over TCP, then runs
a control-loop-style block that reads new samples from the SampleBus and
prints a summary every batch. This is the pattern an RL/agent program would
use to consume sensor data on its own thread, fully decoupled from any GUI.

Run:

    python wavi_py/examples/headless_control_loop.py --host 192.168.2.3
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

import numpy as np

# Make wavi_py importable when running this script directly from inside the repo.
import pathlib
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from wavi_py import WaviClient  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5555)
    p.add_argument("--nsensor", type=int, default=9)
    p.add_argument("--fs", type=float, default=80.0)
    p.add_argument("--max-seconds", type=float, default=0.0,
                   help="stop after this many seconds (0 = run until Ctrl-C)")
    args = p.parse_args()

    client = WaviClient(
        transport="tcp",
        host=args.host,
        port=args.port,
        nsensor=args.nsensor,
        fs=args.fs,
    )

    stop = False

    def _sigint(_sig, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _sigint)

    print(f"connecting to {client.endpoint_str} ...")
    client.start()
    if not client.wait_ready(timeout=15.0):
        print("timed out waiting for stream to start")
        client.stop()
        return 1

    meta = client.bus.get_metadata()
    print(f"stream ready. V0={np.asarray(meta.get('V0')).round(3).tolist()}")
    print("seq        n_new   latency_ms  channel-0 voltage  channel-1 voltage")

    last_seq = 0
    t_start = time.perf_counter()
    while not stop:
        t_before = time.perf_counter()
        samples, last_seq = client.bus.read_since(last_seq, block=True, timeout=0.5)
        if samples is None:
            continue
        latency_ms = (time.perf_counter() - t_before) * 1000.0

        # Whatever the agent does with the data goes here. We just summarize.
        ch0_last = float(samples[-1, 0])
        ch1_last = float(samples[-1, 1])
        print(f"{last_seq:>9d}  {samples.shape[0]:>5d}   {latency_ms:>9.2f}      "
              f"{ch0_last:>15.4f}      {ch1_last:>15.4f}")

        if args.max_seconds and (time.perf_counter() - t_start) >= args.max_seconds:
            break

    print("stopping...")
    client.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
