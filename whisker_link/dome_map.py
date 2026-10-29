"""Dome map: which whisker channel sits where on the dome, and which sensor.

A dome map is a JSON file keyed by the physical POSITION on the dome (e.g.
``LFU``: left, front, upper). Each position names the DAQ whisker channel
wired to it (``W1`` .. ``W16``: both gauges, ``W<n>_A`` and ``W<n>_B`` of the
stream), the physical sensor mounted there (``sensor_id``, optional), a
free-form display ``label`` and the hole's geometry in the dome asset's
collar frame (``wavi_py/assets/dome/whiskers.json``: mm, +z forward along the
tube axis, +y up, +x port):

    {
      "format": "wavi-dome-map/1",
      "calibration_file": "whisker_calibration.json",   # optional, relative
      "whisker_length_mm": 130.0,                       # default per position
      "unused_channels": {"W3": "why"},                 # optional, informative
      "positions": {
        "LFU": {"channel": "W7", "sensor_id": "S07", "label": "LFU",
                "root": [x, y, z], "axis": [x, y, z],
                "e1": [x, y, z], "length_mm": 130.0, "note": "..."}
      }
    }

``root`` is where the whisker leaves the dome, ``axis`` its outward rest
direction; ``e1`` (optional) is the bending frame's first axis, gauge A's bend
direction (default: +z projected normal to the axis, i.e. meridional, as in the
asset). ``sensor_id``, ``label``, ``e1``, ``length_mm`` and ``note`` may be
left out.

Calibration belongs to the sensor, not the position, so it lives in a
separate file keyed by ``sensor_id`` and follows the sensor when it is moved:

    {
      "format": "wavi-whisker-calibration/1",
      "sensors": {
        "S07": {"deg_per_mv": [[a, b], [c, d]], "date": "2026-11-02",
                "source": "...", "note": "..."}
      }
    }

``deg_per_mv`` is the existing ``dome_pose.Calibration`` model, (dA, dB) mV
after tare -> (theta_e1, theta_e2) deg, as a 2x2 matrix or [gA, gB] for a
diagonal one; it assumes the sensor is mounted with gauge A toward e1. A
position with no sensor_id, or a sensor with no entry, falls back to the
placeholder gain ``dome_pose.DEG_PER_MV``.

``load`` validates everything and raises ``DomeMapError`` naming the file
and the entry: duplicate keys (positions, sensors, fields), unknown fields,
channels that are not ``W1`` .. ``W16``, a channel used twice or both used
and listed unused, a sensor_id used twice, bad vectors.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

from whisker_link.channels import NUM_WHISKER
from whisker_link.dome_pose import DEG_PER_MV, Calibration, DomeLayout

MAP_FORMAT = "wavi-dome-map/1"
CAL_FORMAT = "wavi-whisker-calibration/1"
DEFAULT_LENGTH_MM = 130.0

# the map shipped with wavi_py (the ROV dome as wired on 2026-10-29)
DEFAULT_MAP = Path(__file__).resolve().parents[1] / "wavi_py" / "assets" / "dome" / "dome_map.json"

_MAP_KEYS = {"format", "description", "frame", "coordinates", "calibration_file",
             "whisker_length_mm", "unused_channels", "positions"}
_POS_KEYS = {"channel", "sensor_id", "label", "root", "axis", "e1", "length_mm", "note"}
_CAL_KEYS = {"format", "description", "sensors"}
_SENSOR_KEYS = {"deg_per_mv", "date", "source", "note"}
_CHANNEL_RE = re.compile(r"W(\d+)")


class DomeMapError(ValueError):
    """A dome map or calibration file that cannot be used; the message says
    which file and entry."""


@dataclass(frozen=True)
class Position:
    key: str                     # the position, e.g. "LFU"
    whisker: int                 # DAQ whisker channel 1..16 (gauges W<n>_A, W<n>_B)
    label: str
    sensor_id: Optional[str]
    root: np.ndarray             # (3,) mm
    axis: np.ndarray             # (3,) unit, outward
    e1: np.ndarray               # (3,) unit, normal to axis: gauge A's bend direction
    length: float                # mm
    note: str = ""

    @property
    def channel(self) -> str:
        return f"W{self.whisker}"

    @property
    def display(self) -> str:
        """``LFU (W7)``: the label with the channel, for plot axes and titles."""
        return f"{self.label} ({self.channel})"


@dataclass(frozen=True)
class SensorCal:
    sensor_id: str
    deg_per_mv: np.ndarray       # (2, 2): (dA, dB) mV -> (theta_e1, theta_e2) deg
    date: str = ""
    source: str = ""
    note: str = ""


@dataclass
class DomeMap:
    positions: list                               # [Position], file order = hole order
    sensors: dict = field(default_factory=dict)   # sensor_id -> SensorCal
    unused: dict = field(default_factory=dict)    # whisker number -> reason
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
        uc = doc.get("unused_channels", {})
        if not isinstance(uc, dict):
            raise err("unused_channels must be an object {\"W3\": \"reason\"}")
        for ch, why in uc.items():
            unused[_whisker(ch, err, f"unused_channels {ch!r}")] = str(why)
        pos_doc = doc.get("positions")
        if not isinstance(pos_doc, dict) or not pos_doc:
            raise err("positions must be a non-empty object keyed by position")
        positions = []
        by_whisker: dict = {}
        by_sensor: dict = {}
        for key, p in pos_doc.items():
            what = f"position {key!r}"
            if not key.strip():
                raise err("a position key is empty")
            if not isinstance(p, dict):
                raise err(f"{what} must be an object")
            _no_unknown(p, _POS_KEYS, err, what)
            for k in ("channel", "root", "axis"):
                if k not in p:
                    raise err(f"{what} has no {k!r}")
            w = _whisker(p["channel"], err, what)
            if w in by_whisker:
                raise err(f"{what}: channel W{w} is also position {by_whisker[w]!r}")
            if w in unused:
                raise err(f"{what}: channel W{w} is also listed in unused_channels")
            by_whisker[w] = key
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
            positions.append(Position(key, w, label.strip(), sid, root, axis, e1, length,
                                      str(p.get("note", ""))))
        return cls(positions, dict(sensors or {}), unused)

    # ----- queries
    def by_whisker(self) -> dict:
        """whisker number -> Position"""
        return {p.whisker: p for p in self.positions}

    def whisker_names(self, n: int = NUM_WHISKER) -> list:
        """Display names of whisker channels 1..n: ``LFU (W7)`` where mapped,
        ``W3`` where not."""
        bw = self.by_whisker()
        return [bw[w].display if w in bw else f"W{w}" for w in range(1, n + 1)]

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

    def hole_of(self) -> dict:
        """whisker number -> hole index of ``layout()``"""
        return {p.whisker: i for i, p in enumerate(self.positions)}

    def calibration(self, deg_per_mv: float = DEG_PER_MV) -> Calibration:
        """Per hole: the sensor's calibration, or the placeholder diagonal gain."""
        m = np.tile(np.eye(2) * deg_per_mv, (len(self.positions), 1, 1))
        for i, p in enumerate(self.positions):
            cal = self.sensor_cal(p)
            if cal is not None:
                m[i] = cal.deg_per_mv
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


def _whisker(ch, err, what: str) -> int:
    m = _CHANNEL_RE.fullmatch(ch.strip()) if isinstance(ch, str) else None
    if m is None or not 1 <= int(m.group(1)) <= NUM_WHISKER:
        hint = ""
        if isinstance(ch, str) and re.fullmatch(r"W\d+_[AB]", ch.strip()):
            hint = "; a position takes both gauges, so name the whisker, e.g. 'W7'"
        raise err(f"{what}: channel {ch!r} is not W1 .. W{NUM_WHISKER}{hint}")
    return int(m.group(1))


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
    bad = f"{what}: deg_per_mv must be [gA, gB] or [[a, b], [c, d]]"
    if not isinstance(x, list) or len(x) != 2:
        raise err(bad)
    if all(isinstance(r, list) for r in x):
        if any(len(r) != 2 for r in x):
            raise err(bad)
        return np.array([[_number(v, err, what) for v in r] for r in x])
    return np.diag([_number(v, err, what) for v in x])
