"""Export the whisker dome as display assets for wavi_py's Dome view.

    ~/app/miniconda/envs/b123d013/bin/python wavi_py/tools/export_dome_assets.py [out_dir]

Run OFFLINE in the CAD env (b123d013, build123d). wavi_py itself never imports
build123d: it only reads what this writes (default ``wavi_py/assets/dome/``):

  dome_mesh.npz   ``vertices`` (N, 3) float32 mm, ``normals`` (N, 3) float32
                  unit, ``faces`` (M, 3) uint32, ``part`` (M,) uint8 index into
                  ``part_names`` (one entry per exported CAD part), ``part_names``
                  (JSON string)
  whiskers.json   one record per dome hole: ``hole`` (build order), ``side``
                  (L / R, from the root's x sign), ``row``, ``col``, ``root``
                  and ``tip`` (mm), ``axis`` (unit, outward), ``e1`` / ``e2``
                  (the bending frame, see below), plus ``frame`` / ``units`` /
                  ``source`` metadata and ``dome``: the hemisphere's ``center``
                  (on the tube axis, at the equator) and outer ``radius``, mm

Both are plain arrays / JSON, so a glTF or web front end can reuse them as is.

Geometry comes from ~/Projects/rov-cad exactly as render/export_dome_meshes.py
builds it (whisker_dome.build on the committed dome + sensor-case baselines via
clamp_collar.dome_full_params), in the COLLAR's frame: tube axis on z, the BR
seal flange's sealing face at z = 0, the dome toward +z, the ROV's underside
toward -y. The rov-cad sources are read, never written.

The display mesh is the dome shell + cap plates + compartment floors (the
``dome`` group of export_dome_meshes), tessellated coarsely and, if still above
``TARGET_TRIS``, decimated by vertex clustering (plain numpy: no mesh library
is installed in either env). Good enough to look at, not for measurement.
build123d tessellates each B-rep face on its own, so vertices are not shared
across faces; the normals (area-weighted over each vertex's triangles) are
therefore smooth within a face and sharp at the CAD edges. Decimation welds
the mesh, which would smooth across the edges too, so it is skipped when the
coarse tessellation already fits.

Bending frame per whisker: ``axis`` = n, outward along the rod; ``e1`` = the
tube axis (+z, "forward") projected into the plane normal to n, i.e. the
meridional direction; ``e2`` = n x e1 (circumferential). (e1, e2, n) is
right-handed and orthonormal. theta_x tilts the tip toward +e1, theta_y toward
+e2 (whisker_link/dome_pose.py).
"""

from __future__ import annotations

import json
import os
import sys
import time

import numpy as np

ROV_CAD = os.path.expanduser("~/Projects/rov-cad")
for sub in ("render", "dome", "collar", "sensor_case", ""):
    p = os.path.join(ROV_CAD, sub)
    if p not in sys.path:
        sys.path.insert(0, p)

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.join(os.path.dirname(HERE), "assets", "dome")

TARGET_TRIS = 40_000
# coarse tessellation: chordal mm, angular rad
TOL, ANG = 0.25, 0.30
DOME_PARTS = ("dome_shell", "cap_plate", "comp_floor")


def cluster_decimate(V, F, cell):
    """Vertex-clustering decimation on a ``cell`` mm grid: every vertex in a
    cell collapses to the cell's mean; degenerate and duplicate faces drop."""
    key = np.floor(V / cell).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.ravel()
    n = inv.max() + 1
    Vn = np.zeros((n, 3))
    np.add.at(Vn, inv, V)
    Vn /= np.bincount(inv, minlength=n)[:, None]
    Fn = inv[F]
    ok = (Fn[:, 0] != Fn[:, 1]) & (Fn[:, 1] != Fn[:, 2]) & (Fn[:, 0] != Fn[:, 2])
    Fn = Fn[ok]
    srt = np.sort(Fn, axis=1)
    _, first = np.unique(srt, axis=0, return_index=True)
    keep = np.zeros(len(Fn), bool)
    keep[first] = True
    return Vn, Fn[keep], ok.nonzero()[0][keep]


def decimate_to(V, F, target):
    """Smallest clustering cell (bisection) that brings F under ``target``.
    Returns (V, F, kept-face index into the input F)."""
    if len(F) <= target:
        return V, F, np.arange(len(F))
    lo, hi = 0.05, 5.0
    best = cluster_decimate(V, F, hi)
    for _ in range(14):
        mid = (lo * hi) ** 0.5
        out = cluster_decimate(V, F, mid)
        if len(out[1]) <= target:
            hi, best = mid, out
        else:
            lo = mid
    print(f"  decimated {len(F)} -> {len(best[1])} tris (cell {hi:.2f} mm)")
    return best


def vertex_normals(V, F):
    """Area-weighted vertex normals (unit; zero for an unused vertex)."""
    fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    vn = np.zeros_like(V, dtype=np.float64)
    for k in range(3):
        np.add.at(vn, F[:, k], fn)
    return vn / np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-12)


def bending_frame(axis, fwd=(0.0, 0.0, 1.0)):
    n = np.asarray(axis, float)
    n = n / np.linalg.norm(n)
    f = np.asarray(fwd, float)
    e1 = f - (f @ n) * n
    if np.linalg.norm(e1) < 1e-6:          # whisker along the tube axis: any e1
        e1 = np.cross(n, [1.0, 0.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    return n, e1, e2


def main(out_dir=DEFAULT_OUT):
    from build123d import Pos                       # noqa: WPS433 (CAD env only)
    import clamp_collar as cc
    import export_dome_meshes as edm
    import whisker_dome as wd

    t0 = time.time()
    full = cc.dome_full_params()
    out = wd.build(dict(full))
    meta = out["meta"]
    print("dome build", f"{time.time() - t0:.0f}s", "errors:", meta.get("build_errors"))
    shift = Pos(0, 0, -float(meta["seal_face_z"]))

    verts, faces, part_id, names = [], [], [], []
    off = 0
    for name, part in out["parts"].items():
        if part is None or not name.startswith(DOME_PARTS):
            continue
        moved = [shift * s for s in part] if isinstance(part, list) else shift * part
        V, F = wd.mesh_of(moved, tolerance=TOL, angular_tolerance=ANG)
        if len(F) == 0:
            continue
        print(f"  {name:24s} {len(F):8d} tris")
        verts.append(V)
        faces.append(F + off)
        part_id.append(np.full(len(F), len(names), np.uint8))
        names.append(name)
        off += len(V)
    V, F, P = np.concatenate(verts), np.concatenate(faces), np.concatenate(part_id)
    if len(F) > TARGET_TRIS:
        # weld coincident vertices first, so clustering sees one connected mesh
        V, F, kept = cluster_decimate(V, F, 1e-3)
        P = P[kept]
        V, F, kept = decimate_to(V, F, TARGET_TRIS)
        P = P[kept]
    N = vertex_normals(V, F)

    rods = [edm.rod_axis(shift * r) for r in out["parts"].get("whisker_guides") or []]
    rows = int(round(float(meta["params"].get("ROWS", full.get("ROWS", 2)))))
    cols = int(round(float(meta["params"].get("COLS", full.get("COLS", 2)))))
    table = []
    for k, (root, tip) in enumerate(rods):
        root, tip = np.array(root), np.array(tip)
        n, e1, e2 = bending_frame(tip - root)
        i, j = divmod(k % (rows * cols), cols)
        table.append({"hole": k, "side": "L" if root[0] < 0 else "R", "row": i, "col": j,
                      "root": root.round(4).tolist(), "tip": tip.round(4).tolist(),
                      "axis": n.round(6).tolist(), "e1": e1.round(6).tolist(),
                      "e2": e2.round(6).tolist(),
                      "length": round(float(np.linalg.norm(tip - root)), 3)})

    os.makedirs(out_dir, exist_ok=True)
    np.savez_compressed(os.path.join(out_dir, "dome_mesh.npz"),
                        vertices=V.astype(np.float32), normals=N.astype(np.float32),
                        faces=F.astype(np.uint32),
                        part=P.astype(np.uint8), part_names=np.array(json.dumps(names)))
    doc = {"frame": "collar: tube axis +z (dome forward), BR seal face z=0, ROV underside -y",
           "units": "mm",
           "bending_frame": "axis n outward; e1 = +z projected normal to n (meridional); "
                            "e2 = n x e1 (circumferential)",
           "source": "rov-cad whisker_dome.build(clamp_collar.dome_full_params()), "
                     "via wavi_py/tools/export_dome_assets.py",
           "dome": {"center": [0.0, 0.0, round(-float(meta["seal_face_z"]), 4)],
                    "radius": float(meta["params"].get("R", full.get("R", 0)))},
           "grid": {"rows": rows, "cols": cols,
                    "pitch": float(meta["params"].get("PITCH", 0)),
                    "wlen": float(meta["params"].get("WLEN", 0))},
           "whiskers": table}
    with open(os.path.join(out_dir, "whiskers.json"), "w") as fh:
        json.dump(doc, fh, indent=1)
    print("wrote", out_dir, f"{len(V)} verts, {len(F)} tris, {len(table)} whiskers,",
          f"{time.time() - t0:.0f}s")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_OUT)
