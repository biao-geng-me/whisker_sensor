"""Dome map: which gauge channels sit where on the dome, and which sensor.

A dome map is a JSON file keyed by the physical POSITION on the dome (e.g.
``LFM``: left = port, front = the hole column nearer the dome's pole, middle).
Each position names, per bend axis, the DAQ gauge channel that carries it and
its polarity, the physical sensor mounted there (``sensor_id``, optional), a
free-form display ``label`` and the hole's geometry in the dome asset's
collar frame (``wavi_py/assets/dome/whiskers.json``: mm, +z forward along the
tube axis, +y up, +x port):

    {
      "format": "wavi-dome-map/2",
      "calibration_file": "whisker_calibration.json",   # optional, relative
      "whisker_length_mm": 130.0,                       # default per position
      "unused_gauges": {"W3_B": "why"},                 # optional bookkeeping
      "positions": {
        "LFM": {"daq": {"drag": "W6_A",
                        "lift": {"ch": "W6_B", "polarity": -1}},
                "sensor_id": "S07", "label": "LFM",
                "root": [x, y, z], "axis": [x, y, z],
                "e1": [x, y, z], "length_mm": 130.0, "note": "..."}
      }
    }

``daq`` has the two axes, ``drag`` (front-back bend) and ``lift`` (up-down
bend). Each is a gauge channel as the stream names it (``W1_A`` .. ``W16_B``;
``W1A`` is accepted and normalised), a bare string meaning polarity +1, or
``{"ch": ..., "polarity": 1 | -1}``. The two axes may come from different
whiskers' slots. A dead gauge: ``{"ch": null, "reason": "..."}`` (the reason is
required); that axis then reads as zero bend. At least one axis must have a
channel.

Geometry: ``root`` is where the whisker leaves the dome, ``axis`` its outward
rest direction, ``e1`` (optional) the drag direction (default: +z projected
normal to the axis, i.e. meridional, toward the front, as in the asset).

Calibration belongs to the sensor, not the position, so it lives in a
separate file keyed by ``sensor_id`` and follows the sensor when it moves:

    {
      "format": "wavi-whisker-calibration/2",
      "sensors": {
        "S07": {"deg_per_mv": [[a, b], [c, d]], "date": "2026-11-02",
                "source": "...", "note": "..."}
      }
    }

``deg_per_mv`` is in the sensor's own axes: (drag, lift) mV, after tare and
polarity, -> (drag, lift) deg, as a 2x2 matrix or [g_drag, g_lift] for a
diagonal one. A position with no sensor_id, or a sensor with no entry, uses
the placeholder gain ``dome_pose.DEG_PER_MV``.

The order, per position (``DomeMap.calibration`` folds steps 3-5 into the one
2x2 matrix per hole that ``dome_pose.Calibration`` applies):

    1. raw mV of the drag and lift gauge channels (``hole_channels``)
    2. tare (``dome_pose.Tare``) and smoothing -> delta mV
    3. polarity: delta * (p_drag, p_lift)
    4. per-sensor 2x2 ``deg_per_mv`` -> (drag, lift) deg; drag + = tip
       toward the front (e1), lift + = tip up (toward +y)
    5. dome frame: theta_e1 = drag, theta_e2 = s * lift, where s = +-1 makes
       the hole's e2 = axis x e1 point up (``lift_sign``: e2 points down on
       the port side, up on starboard)

``load`` validates everything and raises ``DomeMapError`` naming the file
and the entry: duplicate keys, unknown fields, gauge names that are not
``W1_A`` .. ``W16_B``, a gauge used twice or both used and listed unused, a
sensor_id used twice, a polarity other than +-1, bad vectors.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

from whisker_link.channels import CHANNEL_NAMES, NUM_WHISKER
from whisker_link.dome_pose import DEG_PER_MV, Calibration, DomeLayout

MAP_FORMAT = "wavi-dome-map/2"
CAL_FORMAT = "wavi-whisker-calibration/2"
DEFAULT_LENGTH_MM = 130.0
AXES = ("drag", "lift")

# the map shipped with wavi_py (the ROV dome as wired on 2026-10-29)
DEFAULT_MAP = Path(__file__).resolve().parents[1] / "wavi_py" / "assets" / "dome" / "dome_map.json"

_MAP_KEYS = {"format", "description", "frame", "coordinates", "polarity_note",
             "calibration_file", "whisker_length_mm", "unused_gauges", "positions"}
_POS_KEYS = {"daq", "sensor_id", "label", "root", "axis", "e1", "length_mm", "note"}
_AXIS_KEYS = {"ch", "polarity", "reason"}
_CAL_KEYS = {"format", "description", "sensors"}
_SENSOR_KEYS = {"deg_per_mv", "date", "source", "note"}
_GAUGE_RE = re.compile(r"W(\d+)_?([AB])")


class DomeMapError(ValueError):
    """A dome map or calibration file that cannot be used; the message says
    which file and entry."""


@dataclass(frozen=True)
class Gauge:
    """One bend axis of a position: its channel (index into CHANNEL_NAMES,
    None for a dead gauge) and polarity."""
    ch: Optional[int]
    polarity: int = 1
    reason: str = ""

    @property
    def name(self) -> str:
        return CHANNEL_NAMES[self.ch] if self.ch is not None else "none"


@dataclass(frozen=True)
class Position:
    key: str                     # the position, e.g. "LFM"
    daq: dict                    # "drag" / "lift" -> Gauge
    label: str
    sensor_id: Optional[str]
    root: np.ndarray             # (3,) mm
    axis: np.ndarray             # (3,) unit, outward
    e1: np.ndarray               # (3,) unit, normal to axis: drag + direction
    length: float                # mm
    note: str = ""

    @property
    def slots(self) -> str:
        """``W6`` when both axes are one whisker's gauges, else the gauges."""
        chans = [g.ch for g in self.daq.values() if g.ch is not None]
        whiskers = {c // 2 for c in chans}
        if len(whiskers) == 1:
            return f"W{whiskers.pop() + 1}"
        return "/".join(CHANNEL_NAMES[c] for c in chans)

    @property
    def display(self) -> str:
        """``LFM (W6)``: the label with its channel(s), for the Dome view and
        per-whisker plots."""
        return f"{self.label} ({self.slots})"

    def gauge_display(self, axis: str) -> str:
        """``LFM drag (W6_A)``, with ``-`` for a flipped polarity."""
        g = self.daq[axis]
        return f"{self.label} {axis} ({'-' if g.polarity < 0 else ''}{g.name})"


@dataclass(frozen=True)
class SensorCal:
    sensor_id: str
    deg_per_mv: np.ndarray       # (2, 2): (drag, lift) mV after polarity -> (drag, lift) deg
    date: str = ""
    source: str = ""
    note: str = ""


@dataclass
class DomeMap:
    positions: list                               # [Position], file order = hole order
    sensors: dict = field(default_factory=dict)   # sensor_id -> SensorCal
    unused: dict = field(default_factory=dict)    # channel index -> reason
    path: Optional[Path] = None
    calibration_path: Optional[Path] = None

    # ----- loading
    @classmethod
    def load(cls, path) -> "DomeMap":
        path = Path(path)
        doc = _read_json(path)
        cal_path = None
        sensors: dict = {}
        if isinstance(doc, dict) and doc.get("calibration_file"):
            name = doc["calibration_file"]
            if not isinstance(name, str):
                raise DomeMapError(f"{path}: calibration_file must be a file name")
            cal_path = (path.parent / name).resolve()
            sensors = load_calibration(cal_path)
        m = cls.from_dict(doc, sensors, where=str(path))
        m.path, m.calibration_path = path, cal_path
        return m

    @classmethod
    def from_dict(cls, doc, sensors: Optional[dict] = None, where: str = "dome map") -> "DomeMap":
        def err(msg: str) -> DomeMapError:
            return DomeMapError(f"{where}: {msg}")

        if not isinstance(doc, dict):
            raise err("the top level must be a JSON object")
        if doc.get("format") != MAP_FORMAT:
            raise err(f"format must be {MAP_FORMAT!r} (got {doc.get('format')!r})")
        _no_unknown(doc, _MAP_KEYS, err, "top level")
        length0 = _positive(doc.get("whisker_length_mm", DEFAULT_LENGTH_MM), err,
                            "whisker_length_mm")
        unused = {}
        ug = doc.get("unused_gauges", {})
        if not isinstance(ug, dict):
            raise err("unused_gauges must be an object {\"W3_B\": \"reason\"}")
        for ch, why in ug.items():
            c = _gauge(ch, err, f"unused_gauges {ch!r}")
            if c in unused:
                raise err(f"unused_gauges: {CHANNEL_NAMES[c]} is listed twice")
            unused[c] = str(why)
        pos_doc = doc.get("positions")
        if not isinstance(pos_doc, dict) or not pos_doc:
            raise err("positions must be a non-empty object keyed by position")
        positions = []
        by_gauge: dict = {}
        by_sensor: dict = {}
        for key, p in pos_doc.items():
            what = f"position {key!r}"
            if not key.strip():
                raise err("a position key is empty")
            if not isinstance(p, dict):
                raise err(f"{what} must be an object")
            _no_unknown(p, _POS_KEYS, err, what)
            for k in ("daq", "root", "axis"):
                if k not in p:
                    raise err(f"{what} has no {k!r}")
            daq = _daq(p["daq"], err, what)
            for ax, g in daq.items():
                if g.ch is None:
                    continue
                if g.ch in by_gauge:
                    raise err(f"{what} {ax}: gauge {g.name} is also {by_gauge[g.ch]}")
                if g.ch in unused:
                    raise err(f"{what} {ax}: gauge {g.name} is also listed in unused_gauges")
                by_gauge[g.ch] = f"{key} {ax}"
            sid = p.get("sensor_id")
            if sid is not None:
                if not isinstance(sid, str) or not sid.strip():
                    raise err(f"{what}: sensor_id must be a non-empty string or null")
                if sid in by_sensor:
                    raise err(f"{what}: sensor_id {sid!r} is also at position {by_sensor[sid]!r}")
                by_sensor[sid] = key
            label = p.get("label", key)
            if not isinstance(label, str) or not label.strip():
                raise err(f"{what}: label must be a non-empty string")
            root = _vec(p["root"], err, f"{what} root")
            axis = _unit(_vec(p["axis"], err, f"{what} axis"), err, f"{what} axis")
            if "e1" in p:
                e1 = _vec(p["e1"], err, f"{what} e1")
            else:
                e1 = np.array([0.0, 0.0, 1.0]) if abs(axis[2]) < 0.999 else np.array([0.0, 1.0, 0.0])
            e1 = e1 - (e1 @ axis) * axis
            if np.linalg.norm(e1) < 1e-6:
                raise err(f"{what}: e1 is parallel to the axis")
            e1 = e1 / np.linalg.norm(e1)
            length = _positive(p.get("length_mm", length0), err, f"{what} length_mm")
            positions.append(Position(key, daq, label.strip(), sid, root, axis, e1, length,
                                      str(p.get("note", ""))))
        return cls(positions, dict(sensors or {}), unused)

    # ----- queries
    def gauge_owner(self) -> dict:
        """channel index -> (Position, axis)"""
        return {g.ch: (p, ax) for p in self.positions for ax, g in p.daq.items()
                if g.ch is not None}

    def gauge_names(self, nch: int = 2 * NUM_WHISKER) -> list:
        """Display names of the gauge channels: ``LFM drag (W6_A)`` where
        mapped, ``W3 A`` where not."""
        own = self.gauge_owner()
        return [own[c][0].gauge_display(own[c][1]) if c in own else CHANNEL_NAMES[c].replace("_", " ")
                for c in range(nch)]

    def whisker_names(self, n: int = NUM_WHISKER) -> list:
        """Display names of whisker slots 1..n (a slot = gauges W<n>_A, W<n>_B):
        ``LFM (W6)``, ``LFM/LBM (W2)`` when its gauges serve two positions,
        ``W3`` when neither is mapped."""
        own = self.gauge_owner()
        out = []
        for w in range(n):
            labels = []
            for c in (2 * w, 2 * w + 1):
                if c in own and own[c][0].label not in labels:
                    labels.append(own[c][0].label)
            out.append(f"{'/'.join(labels)} (W{w + 1})" if labels else f"W{w + 1}")
        return out

    def sensor_cal(self, pos: Position) -> Optional[SensorCal]:
        return self.sensors.get(pos.sensor_id) if pos.sensor_id else None

    def uncalibrated(self) -> list:
        """Positions on the placeholder gain: no sensor_id, or no calibration for it."""
        return [p.key for p in self.positions if self.sensor_cal(p) is None]

    # ----- the dome pose model's inputs
    def layout(self, center=None, radius: float = 0.0) -> DomeLayout:
        """One hole per position, in file order; ``center`` / ``radius`` are
        the dome hemisphere's (from the CAD asset)."""
        P = self.positions
        e2 = [np.cross(p.axis, p.e1) for p in P]
        return DomeLayout(np.array([p.root for p in P]), np.array([p.axis for p in P]),
                          np.array([p.e1 for p in P]), np.array(e2),
                          np.array([p.length for p in P]), [p.display for p in P],
                          None if center is None else np.asarray(center, dtype=np.float64),
                          float(radius))

    def hole_channels(self) -> np.ndarray:
        """(H, 2) gauge channel of each hole's (drag, lift); -1 = none."""
        return np.array([[-1 if p.daq[ax].ch is None else p.daq[ax].ch for ax in AXES]
                         for p in self.positions], dtype=np.intp)

    @staticmethod
    def lift_sign(pos: Position) -> int:
        """+1 if the hole's e2 = axis x e1 points up (+y), -1 if down (step 5)."""
        return -1 if np.cross(pos.axis, pos.e1)[1] < 0 else 1

    def calibration(self, deg_per_mv: float = DEG_PER_MV) -> Calibration:
        """Per hole, steps 3-5 of the module docstring as one matrix:
        diag(1, lift_sign) @ sensor deg_per_mv (or the placeholder gain) @
        diag(polarities), in rad / mV."""
        m = np.zeros((len(self.positions), 2, 2))
        for i, p in enumerate(self.positions):
            cal = self.sensor_cal(p)
            sensor = cal.deg_per_mv if cal is not None else np.eye(2) * deg_per_mv
            pol = np.diag([p.daq[ax].polarity for ax in AXES])
            m[i] = np.diag([1, self.lift_sign(p)]) @ sensor @ pol
        return Calibration(np.radians(m))

    def summary(self) -> str:
        """One line for a status bar / tooltip."""
        name = self.path.name if self.path else "dome map"
        n_cal = len(self.positions) - len(self.uncalibrated())
        return f"{name}: {len(self.positions)} positions, {n_cal} calibrated"


def load_calibration(path) -> dict:
    """sensor_id -> SensorCal from a calibration file."""
    path = Path(path)
    doc = _read_json(path)

    def err(msg: str) -> DomeMapError:
        return DomeMapError(f"{path}: {msg}")

    if not isinstance(doc, dict):
        raise err("the top level must be a JSON object")
    if doc.get("format") != CAL_FORMAT:
        raise err(f"format must be {CAL_FORMAT!r} (got {doc.get('format')!r})")
    _no_unknown(doc, _CAL_KEYS, err, "top level")
    sensors = doc.get("sensors", {})
    if not isinstance(sensors, dict):
        raise err("sensors must be an object keyed by sensor_id")
    out = {}
    for sid, s in sensors.items():
        what = f"sensor {sid!r}"
        if not sid.strip():
            raise err("a sensor_id is empty")
        if not isinstance(s, dict):
            raise err(f"{what} must be an object")
        _no_unknown(s, _SENSOR_KEYS, err, what)
        if "deg_per_mv" not in s:
            raise err(f"{what} has no deg_per_mv (leave the sensor out until it is calibrated)")
        out[sid] = SensorCal(sid, _cal_matrix(s["deg_per_mv"], err, what), str(s.get("date", "")),
                             str(s.get("source", "")), str(s.get("note", "")))
    return out


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def _read_json(path: Path):
    def no_dupes(pairs):
        seen = {}
        for k, v in pairs:
            if k in seen:
                raise DomeMapError(f"{path}: duplicate key {k!r}")
            seen[k] = v
        return seen

    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise DomeMapError(f"{path}: cannot read: {exc.strerror or exc}") from None
    try:
        return json.loads(text, object_pairs_hook=no_dupes)
    except json.JSONDecodeError as exc:
        raise DomeMapError(f"{path}: not valid JSON: {exc}") from None


def _no_unknown(d: dict, allowed: set, err, what: str) -> None:
    extra = sorted(set(d) - allowed)
    if extra:
        raise err(f"{what}: unknown field(s) {', '.join(map(repr, extra))} "
                  f"(allowed: {', '.join(sorted(allowed))})")


def _gauge(ch, err, what: str) -> int:
    """``W6_A`` / ``W6A`` -> channel index."""
    m = _GAUGE_RE.fullmatch(ch.strip()) if isinstance(ch, str) else None
    if m is None or not 1 <= int(m.group(1)) <= NUM_WHISKER:
        hint = ""
        if isinstance(ch, str) and re.fullmatch(r"W\d+", ch.strip()):
            hint = "; name one gauge, e.g. 'W7_A'"
        raise err(f"{what}: {ch!r} is not a gauge channel W1_A .. W{NUM_WHISKER}_B{hint}")
    return CHANNEL_NAMES.index(f"W{int(m.group(1))}_{m.group(2)}")


def _daq(d, err, what: str) -> dict:
    if not isinstance(d, dict):
        raise err(f"{what}: daq must be an object {{\"drag\": \"W1_A\", \"lift\": \"W1_B\"}}")
    _no_unknown(d, set(AXES), err, f"{what} daq")
    out = {}
    for ax in AXES:
        w = f"{what} {ax}"
        if ax not in d:
            raise err(f"{w}: missing (a dead gauge: {{\"ch\": null, \"reason\": \"...\"}})")
        a = d[ax]
        if isinstance(a, str):
            a = {"ch": a}
        if not isinstance(a, dict):
            raise err(f"{w}: a gauge name or {{\"ch\": ..., \"polarity\": 1 | -1}}")
        _no_unknown(a, _AXIS_KEYS, err, w)
        if "ch" not in a:
            raise err(f"{w}: no 'ch'")
        pol = a.get("polarity", 1)
        if isinstance(pol, bool) or pol not in (1, -1):
            raise err(f"{w}: polarity must be 1 or -1 (got {pol!r})")
        reason = str(a.get("reason", ""))
        if a["ch"] is None:
            if not reason.strip():
                raise err(f"{w}: no channel needs a 'reason' (e.g. dead gauge)")
            out[ax] = Gauge(None, int(pol), reason)
        else:
            out[ax] = Gauge(_gauge(a["ch"], err, w), int(pol), reason)
    if all(g.ch is None for g in out.values()):
        raise err(f"{what}: both axes have no channel")
    return out


def _number(x, err, what: str) -> float:
    if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
        raise err(f"{what}: {x!r} is not a finite number")
    return float(x)


def _positive(x, err, what: str) -> float:
    v = _number(x, err, what)
    if v <= 0:
        raise err(f"{what} must be > 0")
    return v


def _vec(x, err, what: str) -> np.ndarray:
    if not isinstance(x, list) or len(x) != 3:
        raise err(f"{what} must be [x, y, z]")
    return np.array([_number(v, err, what) for v in x])


def _unit(v: np.ndarray, err, what: str) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise err(f"{what} is zero")
    return v / n


def _cal_matrix(x, err, what: str) -> np.ndarray:
    bad = f"{what}: deg_per_mv must be [g_drag, g_lift] or [[a, b], [c, d]]"
    if not isinstance(x, list) or len(x) != 2:
        raise err(bad)
    if all(isinstance(r, list) for r in x):
        if any(len(r) != 2 for r in x):
            raise err(bad)
        return np.array([[_number(v, err, what) for v in r] for r in x])
    return np.diag([_number(v, err, what) for v in x])
