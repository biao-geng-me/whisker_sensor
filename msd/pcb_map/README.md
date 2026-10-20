# Whisker PCB header map

`make_map.py` builds a standalone HTML page that maps the MSD P25937 whisker PCB (rev 2): which header pins each strain gauge plugs into (J2–J5), the ESP32 control headers P1 (ADC SPI) and P2 (mux selects) with the ESP32 GPIO and DevKit V1 board pin for each, and how to route the ESP32 ground wire. The published copy is `/shared/rc/whiskers/MSD_P25937/whisker_header_map.html`, and a copy is kept at [`../docs/whisker_header_map.html`](../docs/whisker_header_map.html).

```bash
pip install olefile
python msd/pcb_map/make_map.py whisker_header_map.html
```

Inputs, all read directly (no unzipping):

- `--altium-zip` (default `/shared/rc/whiskers/MSD_P25937/P25937_Whisker_Final.zip`): `PCB0603.PcbDoc` gives component positions, pad positions and pad nets.
- `--gerber-zip` (default `/shared/rc/whiskers/MSD_P25937/REV2_GERBER.zip`): `PCB0603_Legend_Top.gbr` gives the silkscreen drawn under the board view.
- `--firmware` (default `msd/esp32/whisker_daq/whisker_daq.ino`): the `PIN_*` defines give the ESP32 GPIO for each P1/P2 signal, so the page follows the firmware.

Files: `pcbdoc.py` is a minimal Altium PcbDoc reader (Components6, Nets6, Pads6, Board6 streams; binary pad layout as in KiCad's Altium importer). `gerber.py` is a minimal Gerber RS-274X reader (strokes, regions, flashes). `page.html` is the page template (styles and static text); `make_map.py` fills in the SVGs and tables.

Hand-entered facts, not read from the files: the gauge pin pairing (from the schematic MAIN sheet, checked by asserts against the PCB nets), the firmware/wavi_py whisker naming (W(n)_A = OUT(2n−1)), the ADS131M04 pin functions, and the DevKit V1 30-pin header order (DOIT layout; check your board's labels). The whisker numbering was verified on the bench: J2 pins 1–4 show up as W1 in wavi_py.

Things the map found that are easy to get wrong: the silkscreen "MUX1"/"MUX2" labels are the opposite of the net names (mux 1 = SEL*_1 = OUT1–16 is the lower chip, printed "MUX2"); P1 and P2 have no GND pin (the only through-hole GND is J1 pin 4); the P1 net names (PIN23, PIN21, …) are Raspberry Pi header positions from the original design.
