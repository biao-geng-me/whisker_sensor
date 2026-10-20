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
    flags = ef.FLAG_RECOVERED | ef.FLAG_CRC_RETRY | ef.FLAG_STALE
    assert flags == 0x07
    frame = ef.encode_scan(7, 1000, 12300, 0, flags, [0] * ef.NCH)
    (scan,) = list(ef.Decoder().feed(frame))
    assert scan.flags & ef.FLAG_STALE and scan.flags == flags


def test_info_roundtrip():
    info = ef.Info(ef.PROTO_VERSION, 32, 0x2240, 0xFF02, 800000, 3125, 921600)
    (ev,) = list(ef.Decoder().feed(ef.encode_info(info)))
    assert ev == info


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
