"""wavi_py — Python port of wavi/wavi.m.

Public entry points:

- ``WaviClient`` — high-level facade. Wires transport + DAQ thread + sample bus.
  ``WaviClient(headless=True)`` for use as a library; ``False`` for GUI mode.
- ``SampleBus`` — thread-safe pub/sub of sensor samples. Multiple subscribers,
  each with their own read cursor. The control loop and the GUI are peers.

See ``wavi_py/examples/headless_control_loop.py`` for the agent-side pattern.
"""

from wavi_py.client import WaviClient
from wavi_py.daq.bus import SampleBus

__all__ = ["WaviClient", "SampleBus"]
