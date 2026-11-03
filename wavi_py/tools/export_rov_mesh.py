"""Export a light BlueROV2 (frame, thrusters, enclosures) for wavi_py's ROV and
Dome views.

    python wavi_py/tools/export_rov_mesh.py BLUEROV2_R4_NPZ [out_dir]

Input: the tessellated Blue Robotics BlueROV2 R4 assembly that rov-cad's
``render/tessellate_bluerov2.py`` writes (``bluerov2_r4.npz``: per solid
instance its vertices in mm in R4's own frame, triangles, colours and the
product-name path), made from Blue Robotics' public STEP
(https://cad.bluerobotics.com/BLUEROV2-R4.zip, ``BLUEROV2-R4-PUBLIC.STEP``,
CC BY 4.0). That step needs the CAD env (b123d013); this one is plain numpy::

    ~/app/miniconda/envs/b123d013/bin/python -I \\
        ~/Projects/rov-cad/render/tessellate_bluerov2.py \\
        ~/Projects/rov-cad/bluerobotics/BLUEROV2-R4-PUBLIC.STEP /scratch/.../bluerov2_r4.npz

Output (default ``wavi_py/assets/rov/``):

  rov_mesh.npz    per group g in ``groups`` (JSON list of names): ``v_<g>`` (N, 3)
                  float32 mm, ``f_<g>`` (M, 3) uint32, ``rgb_<g>`` (3,) float32;
                  ``source`` (JSON: where it came from, the licence, the frame)
  SOURCE.md       the same, for people

The solids are picked by product name (``GROUPS``; screws, nuts, o-rings,
cables, ballast, electronics and the stock dome are left out), merged per
group and decimated by vertex clustering (export_dome_assets.cluster_decimate)
to about ``TARGETS`` triangles: a silhouette to look at, not CAD. The vertices
go into rov-cad's collar frame (the whisker dome's: tube axis on z, the seal
face at z = 0, forward +z, up +y, port +x) by R4 - ``COLLAR_TO_R4``
(render_rov.py), so the dome mesh sits on the ROV without a transform.
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
from wavi_py.tools.export_dome_assets import cluster_decimate  # noqa: E402

DEFAULT_OUT = os.path.join(os.path.dirname(HERE), "assets", "rov")
COLLAR_TO_R4 = np.array([0.0, 0.65, 165.628])      # rov-cad render/render_rov.py
# group -> (product-name keys, any of which selects a solid), in priority order
GROUPS = {
    "thrusters": ("T200",),
    "enclosures": ("LOCKING-TUBE", "BATTERY-TUBE", "FLANGE", "END-CAP"),
    "buoyancy": ("FAIRING", "BUOYANCY-FOAM"),
    "frame": ("SIDE-PANEL", "FRONT-PANEL", "REAR-PANEL", "BOTTOMPANEL",
              "CRADLE-EXTRUSION", "CRADLE-CLIP", "BATTERY-CRADLE"),
}
SKIP = ("SCREW", "NUT", "STANDOFF", "ORING", "O-RING", "CORD", "BALLAST", "PAINT",
        "WTE4-P-DOME-R4", "WTE4-P-DOME-RETAINING-RING-R1")
TARGETS = {"thrusters": 9000, "enclosures": 5000, "buoyancy": 5000, "frame": 9000}
COLORS = {"thrusters": (0.30, 0.33, 0.38), "enclosures": (0.62, 0.66, 0.72),
          "buoyancy": (0.80, 0.80, 0.82), "frame": (0.13, 0.13, 0.15)}


def group_of(path: list[str]) -> str | None:
    names = " ".join(path).upper()
    if any(s in names for s in SKIP) and "T200" not in names:
        return None
    for g, keys in GROUPS.items():
        if any(k in names for k in keys):
            return g
    return None


def decimate_to(V, F, target):
    """The smallest clustering cell (bisection, mm) that brings F under ``target``."""
    if len(F) <= target:
        return V, F, 0.0
    lo, hi = 0.2, 40.0
    best = cluster_decimate(V, F, hi)[:2]
    for _ in range(16):
        mid = (lo * hi) ** 0.5
        Vn, Fn, _ = cluster_decimate(V, F, mid)
        if len(Fn) <= target:
            hi, best = mid, (Vn, Fn)
        else:
            lo = mid
    return best[0], best[1], hi


def main(src: str, out_dir: str = DEFAULT_OUT) -> None:
    d = np.load(src)
    items = json.loads(str(d["index"]))
    parts: dict[str, list] = {g: [] for g in GROUPS}
    seen: dict[str, set] = {g: set() for g in GROUPS}
    for k, it in enumerate(items):
        g = group_of(it["path"])
        if g is None:
            continue
        parts[g].append((d[f"v{k}"].astype(np.float64) - COLLAR_TO_R4, d[f"f{k}"]))
        seen[g].add(it["path"][-1])
    os.makedirs(out_dir, exist_ok=True)
    out, report = {}, []
    for g, lst in parts.items():
        if not lst:
            continue
        Vs, Fs, off = [], [], 0
        for V, F in lst:
            Vs.append(V)
            Fs.append(F.astype(np.int64) + off)
            off += len(V)
        V, F = np.concatenate(Vs), np.concatenate(Fs)
        n0 = len(F)
        V, F, cell = decimate_to(V, F, TARGETS[g])
        used = np.unique(F)
        remap = np.full(len(V), -1, np.int64)
        remap[used] = np.arange(len(used))
        V, F = V[used], remap[F]
        out[f"v_{g}"] = V.astype(np.float32)
        out[f"f_{g}"] = F.astype(np.uint32)
        out[f"rgb_{g}"] = np.asarray(COLORS[g], np.float32)
        line = (f"{g}: {len(lst)} solids, {n0} -> {len(F)} triangles (cell {cell:.1f} mm), "
                f"{len(seen[g])} products")
        report.append(line)
        print(line)
        print("   ", ", ".join(sorted(seen[g]))[:600])
    lo = np.min([out[f"v_{g}"].min(0) for g in parts if f"v_{g}" in out], axis=0)
    hi = np.max([out[f"v_{g}"].max(0) for g in parts if f"v_{g}" in out], axis=0)
    source = {
        "model": "Blue Robotics BlueROV2 R4",
        "url": "https://cad.bluerobotics.com/BLUEROV2-R4.zip",
        "file": "BLUEROV2-R4-PUBLIC.STEP",
        "license": "CC BY 4.0 (Blue Robotics)",
        "via": "rov-cad render/tessellate_bluerov2.py -> wavi_py/tools/export_rov_mesh.py",
        "frame": "rov-cad collar frame: forward +z, up +y, port +x; mm",
        "collar_to_r4_mm": COLLAR_TO_R4.tolist(),
        "groups": report,
        "bbox_mm": [np.round(lo.astype(float), 1).tolist(), np.round(hi.astype(float), 1).tolist()],
    }
    np.savez_compressed(os.path.join(out_dir, "rov_mesh.npz"),
                        groups=json.dumps([g for g in parts if f"v_{g}" in out]),
                        source=json.dumps(source), **out)
    with open(os.path.join(out_dir, "SOURCE.md"), "w") as f:
        f.write("# rov_mesh.npz: where it came from\n\n")
        f.write(f"{source['model']}, from Blue Robotics' public CAD ({source['url']}, "
                f"`{source['file']}`), licensed {source['license']}. Tessellated by "
                "~/Projects/rov-cad `render/tessellate_bluerov2.py`, then reduced to the "
                "frame, thrusters, enclosures and buoyancy and decimated by vertex clustering "
                "with `wavi_py/tools/export_rov_mesh.py` (re-run it to rebuild). A display "
                "silhouette, not CAD.\n\n")
        f.write(f"Frame: {source['frame']} (R4 minus {source['collar_to_r4_mm']} mm), so it "
                f"shares the whisker dome mesh's frame. Bounding box {source['bbox_mm']} mm.\n\n")
        for line in report:
            f.write(f"- {line}\n")
    print(f"bbox {source['bbox_mm']} -> {out_dir}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    main(*sys.argv[1:3])
