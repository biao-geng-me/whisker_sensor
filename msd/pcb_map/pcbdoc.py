"""Minimal Altium .PcbDoc reader: components, nets, pads (layout per KiCad's altium importer)."""
import struct
import olefile   # pip install olefile

def text_records(data):
    i, out = 0, []
    while i + 4 <= len(data):
        n = struct.unpack_from("<I", data, i)[0]; i += 4
        d = {}
        for kv in data[i:i + n].rstrip(b"\x00").decode("latin1").split("|"):
            if "=" in kv:
                k, v = kv.split("=", 1); d[k.upper()] = v
        out.append(d); i += n
    return out

def mil(s):  # "1234.5mil" -> float mil
    return float(s.replace("mil", "")) if s else 0.0

def pads(data):
    i, out = 0, []
    while i < len(data):
        typ = data[i]; i += 1
        assert typ == 2, (typ, i)
        n = struct.unpack_from("<I", data, i)[0]; i += 4
        name = data[i + 1:i + 1 + data[i]].decode("latin1"); i += n
        for _ in range(3):
            n = struct.unpack_from("<I", data, i)[0]; i += 4 + n
        n = struct.unpack_from("<I", data, i)[0]; i += 4
        s = data[i:i + n]; i += n
        layer = s[0]
        net, = struct.unpack_from("<H", s, 3)
        comp, = struct.unpack_from("<H", s, 7)
        x, y = struct.unpack_from("<ii", s, 13)
        tx, ty = struct.unpack_from("<ii", s, 21)
        hole, = struct.unpack_from("<i", s, 45)
        shape = s[49]
        rot, = struct.unpack_from("<d", s, 52)
        out.append(dict(name=name, layer=layer, net=net, comp=comp, x=x / 1e4, y=y / 1e4,
                        sx=tx / 1e4, sy=ty / 1e4, hole=hole / 1e4, shape=shape, rot=rot))
        # optional subrecord 6
        if i + 4 <= len(data) and data[i] != 2:
            n = struct.unpack_from("<I", data, i)[0]; i += 4 + n
    return out

def load(src):
    """src: a path or the file's bytes."""
    o = olefile.OleFileIO(src)
    rd = lambda s: o.openstream(s + "/Data").read()
    comps = text_records(rd("Components6"))
    nets = [r.get("NAME", "") for r in text_records(rd("Nets6"))]
    board = text_records(rd("Board6"))[0]
    return comps, nets, pads(rd("Pads6")), board
