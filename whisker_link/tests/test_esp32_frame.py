"""Run: python -m pytest whisker_link/tests -q"""

import random

import numpy as np

from whisker_link import esp32_frame as ef
from whisker_link.convert import envelope_fit


def _scan(seq):
    raw = [random.randint(-(1 << 23), (1 << 23) - 1) for _ in range(32)]
    return raw, ef.encode_scan(seq, 1000 * seq, 12300, 0xF0F0F0F0, 0, raw)


def test_crc_matches_bitwise():
    data = bytes(random.randrange(256) for _ in range(200))
    assert ef.crc16_fast(data) == ef.crc16_ccitt(data)
    assert ef.crc16_fast(b"123456789") == 0x29B1        # CRC-16/CCITT-FALSE check value


def test_roundtrip_with_garbage_and_random_chunking():
    random.seed(1)
    stream, sent = bytearray(b"boot banner\r\nmore text\r\n"), []
    for seq in range(200):
        raw, fr = _scan(seq)
        sent.append(raw)
        stream += fr
        if seq % 17 == 0:
            stream += bytes([0xA5, 0x5A, 0x01, 0x70]) + b"\x00" * 7   # false sync
        if seq % 50 == 0:
            stream += ef.encode_text(f"[halt #{seq}] recovered")
    dec, got, texts = ef.Decoder(), [], []
    i = 0
    while i < len(stream):
        n = random.randint(1, 300)
        for ev in dec.feed(bytes(stream[i:i + n])):
            if isinstance(ev, ef.Scan):
                got.append(ev)
            elif isinstance(ev, ef.Text):
                texts.append(ev.text)
        i += n
    assert [s.raw for s in got] == sent
    assert [s.seq for s in got] == list(range(200))
    assert got[0].railmask == 0xF0F0F0F0 and got[0].dur_us == 12300
    assert "boot banner" in texts and "[halt #50] recovered" in texts


def test_corrupted_frame_is_dropped_not_misparsed():
    _, a = _scan(1)
    _, b = _scan(2)
    bad = bytearray(a)
    bad[20] ^= 0xFF
    evs = list(ef.Decoder().feed(bytes(bad) + b))
    scans = [e for e in evs if isinstance(e, ef.Scan)]
    assert [s.seq for s in scans] == [2]


def test_flags_roundtrip():
    flags = ef.FLAG_RECOVERED | ef.FLAG_CRC_RETRY | ef.FLAG_STALE | ef.FLAG_REINIT
    assert flags == 0x0F
    frame = ef.encode_scan(7, 1000, 12300, 0, flags, [0] * ef.NCH)
    (scan,) = list(ef.Decoder().feed(frame))
    assert scan.flags & ef.FLAG_STALE and scan.flags == flags


def test_info_roundtrip():
    info = ef.Info(ef.PROTO_VERSION, 32, 0x2240, 0xFF02, 800000, 3125, 921600, 128, 250, 3,
                   0b10111111)
    (ev,) = list(ef.Decoder().feed(ef.encode_info(info)))
    assert ev == info
    cfg = ef.info_cfg(dict(ev.__dict__))
    assert cfg["selects"] == "0123457" and cfg["osr"] == 128


def test_info_v2_still_decodes():
    info = ef.Info(2, 32, 0x2405, 0x0F02, 800000, 3125, 921600, 128, 250, 3)
    (ev,) = list(ef.Decoder().feed(ef.encode_info(info)))
    assert ev == info and ev.sel_mask is None and "selects" not in ef.info_cfg(dict(ev.__dict__))


def test_info_v1_still_decodes():
    info = ef.Info(1, 32, 0x2405, 0x0F02, 800000, 3125, 921600)
    (ev,) = list(ef.Decoder().feed(ef.encode_info(info)))
    assert ev == info and ev.osr is None


def test_micros_unwrap():
    u = ef.TimeUnwrapper()
    vals = [u(t) for t in (4294967000, 4294967290, 100, 5000)]
    assert vals == [4294967000, 4294967290, (1 << 32) + 100, (1 << 32) + 5000]


def test_envelope_fit_recovers_offset_and_drift():
    rng = np.random.default_rng(0)
    src = np.arange(0, 600, 0.0125)                      # 80 Hz for 10 min
    true_a, true_b = 3.0, 1 + 40e-6                      # 40 ppm crystal error
    rx = true_a + true_b * src + rng.exponential(0.002, src.size)   # late, never early
    a, b = envelope_fit(src, rx)
    assert abs((b - true_b) * 1e6) < 1.0                 # < 1 ppm
    assert abs(a - true_a) < 0.5e-3                      # < 0.5 ms


def test_info_v4_and_health_roundtrip():
    info = ef.Info(ef.PROTO_VERSION, 32, 0x2405, 0x0F02, 8_000_000, 31250, 921600, 128, 200, 9,
                   0b10111111, 2, 11_428_571, 125, (9, 9, 9, 9, 9, 9, 0, 8))
    (ev,) = list(ef.Decoder().feed(ef.encode_info(info)))
    assert ev == info
    cfg = ef.info_cfg(dict(ev.__dict__))
    assert cfg["mode_name"] == "wide" and cfg["scan_hz"] == 250.0 and cfg["avg_per_sel"][7] == 8
    q = ef.Info(ef.PROTO_VERSION, 32, 0x2405, 0x0F0A, 8_000_000, 7812, 921600, 512, 200, 4,
                0b10111111, 1, 8_000_000, 125, (14, 14, 13, 13, 13, 13, 0, 13))
    assert ef.info_cfg(dict(q.__dict__))["scan_hz"] == 62.5     # not 7812 / 125
    h = ef.Health(1, 2, 371, 34, 180, 160, 16, 3, 31250, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1000)
    (hv,) = list(ef.Decoder().feed(ef.encode_health(h)))
    assert hv == h and hv.summary()["busy_pct"] == 37.1 and hv.summary()["lat_avg_us"] == 18.0
    assert hv.core is None and "core" not in hv.summary()
    # version 2: dual-core fields appended; a v1 reader of the same bytes sees the prefix
    h2 = ef.Health(2, 2, 650, 24, 180, 160, 16, 3, 31250, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1000,
                   core=1, q_hwm=3, tx_win_us=100, parks=2, park_timeouts=0, host_tx_max_us=410,
                   det_max_us=5, tx_late=0)
    raw = ef.encode_health(h2)
    (hv2,) = list(ef.Decoder().feed(raw))
    assert hv2 == h2 and hv2.summary()["core"] == 1 and hv2.summary()["host_tx_max_us"] == 410
    assert len(raw) == 6 + 42 + 14
    v1 = ef._HEALTH.unpack_from(raw, 4)
    assert v1[0] == 2 and v1[20] == 1000
    # a v3 decoder would see an unknown frame type and skip it; v3 INFO still decodes
    v3 = ef.Info(3, 32, 0x2405, 0x0F02, 800000, 3125, 921600, 128, 250, 3, 0xFF)
    (ev3,) = list(ef.Decoder().feed(ef.encode_info(v3)))
    assert ev3.mode is None and "mode" not in ef.info_cfg(dict(ev3.__dict__))


def _beat(**kw):
    d = dict(version=1, uptime_ms=123456, mode=2, core=1, host_mode="B", flags=ef.BEAT_BOOT_CLOCK_OK,
             window_ms=1000, drdy_edges=31250, halts=2, unrecovered=0, adc_reinits=1,
             clock_rb=0xFF02, device_id=0x2405, scans=9999, reset_reason=3, boot_clock_rb=0xFF02)
    d.update(kw)
    return ef.Beat(**d)


def test_beat_frame_and_text_line():
    b = _beat()
    line = ef.beat_text(ef.Beat(**{**b.__dict__, "host_mode": "g"}))
    # what the firmware prints in a text mode (beatService's format), as loose text
    fw = ("#BEAT v1 up_ms=123456 mode=2 core=1 hmode=g flags=0x08 window_ms=1000 edges=31250"
          " halts=2 unrec=0 reinit=1 clock_rb=0xFF02 id=0x2405 scans=9999 reset=3"
          " boot_clock_rb=0xFF02")
    assert line == fw
    evs = list(ef.Decoder().feed(b"banner\r\n" + fw.encode() + b"\r\n" + ef.encode_beat(b)))
    assert evs[0] == ef.Text("banner")
    assert evs[1] == ef.Beat(**{**b.__dict__, "host_mode": "g"}) and evs[2] == b
    assert len(ef.encode_beat(b)) == 6 + 38                 # BEAT_LEN in whisker_daq.ino
    s = b.summary()
    assert s["conv_hz"] == 31250 and s["converting"] and s["reset_name"] == "sw"
    dead = _beat(drdy_edges=0, flags=ef.BEAT_ADC_DOWN, window_ms=1450)
    assert dead.adc_down and not dead.summary()["converting"] and dead.conv_hz == 0
    assert ef.beat_from_text("#BEAT v1 up_ms=1") is None   # truncated: not a beat
    assert ef.beat_from_text("#BURST v1 sel=2") is None


def test_unknown_frame_type_and_longer_payloads_are_safe():
    """Old decoders skip a frame type they don't know (BEAT for them); this one
    decodes longer INFO / BEAT payloads (newer firmware) as far as it knows them."""
    info = ef.Info(4, 32, 0x2405, 0xFF0A, 8_000_000, 7812, 921600, 512, 200, 4, 0xBF, 1,
                   8_000_000, 125, (18, 18, 18, 18, 17, 17, 0, 17))
    longer = ef._frame(ef.T_INFO, ef.encode_info(info)[4:-2] + b"\x01\x02\x03")
    beat = ef._frame(ef.T_BEAT, ef.encode_beat(_beat())[4:-2] + b"\x00" * 4)
    evs = list(ef.Decoder().feed(ef._frame(0x7E, b"future") + longer + beat))
    assert evs == [info, _beat()]
