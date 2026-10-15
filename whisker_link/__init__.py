"""whisker_link — wire formats shared by the ESP32 firmware, the vehicle-side
``rpi/daq_server.py`` and topside clients (``wavi_py``, the Cockpit page).

Pure stdlib (no numpy) so the server can run on the BlueOS Pi with only
pyserial + pymavlink installed.

- ``esp32_frame``  binary frames ESP32 -> host (and the ``Decoder``)
- ``wire``         JSON-lines event stream server -> clients, also the
                   on-disk recording format
- ``channels``     gauge naming
"""
