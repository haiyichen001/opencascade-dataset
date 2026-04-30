"""
Web 四格式对比查看器 — Flask + Three.js, localhost:8005
"""
import sys, json, struct, io, math, os
from pathlib import Path
import numpy as np
from flask import Flask, jsonify, request, send_file

DATA = Path(r"D:\opencascade-dataset")
app = Flask(__name__)

# ── 工具：单例加载 STEP shape ──
_shape_cache = {}

def get_shape(part):
    if part not in _shape_cache:
        from OCC.Core.STEPControl import STEPControl_Reader
        sr = STEPControl_Reader()
        st = sr.ReadFile(str(DATA / "step" / f"{part}.step"))
        if st != 1:
            return None
        sr.TransferRoots()
        _shape_cache[part] = sr.OneShape()
    return _shape_cache[part]


WEB_ROOT = Path(__file__).parent

@app.route("/")
def index():
    return send_file(str(WEB_ROOT / "web_index.html"))


# ── API ──

@app.route("/api/parts")
def api_parts():
    parts = sorted([f.stem for f in (DATA / "step").glob("*.step")])
    return jsonify(parts)


@app.route("/api/mesh/<part>")
def api_mesh(part):
    """STEP 高精度 tessellation → Three.js JSON"""
    from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
    from OCC.Core.TopExp import TopExp_Explorer
    from OCC.Core.TopAbs import TopAbs_FACE
    from OCC.Core.TopLoc import TopLoc_Location
    from OCC.Core.BRep import BRep_Tool
    from OCC.Core.Bnd import Bnd_Box
    from OCC.Core.BRepBndLib import brepbndlib

    shape = get_shape(part)
    if shape is None:
        return jsonify({"error": "part not found"}), 404

    bbox = Bnd_Box(); brepbndlib.Add(shape, bbox)
    x1, y1, z1, x2, y2, z2 = bbox.Get()
    span = max(x2 - x1, y2 - y1, z2 - z1)

    BRepMesh_IncrementalMesh(shape, span / 100.0).Perform()

    all_verts = []
    all_norms = []
    all_tris = []
    vi = 0

    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = exp.Current()
        loc = TopLoc_Location()
        tri = BRep_Tool().Triangulation(face, loc)
        if tri is None:
            exp.Next(); continue

        trsf = loc.Transformation()
        nv = tri.NbNodes()
        local_idx = {}
        for i in range(1, nv + 1):
            p = tri.Node(i)
            p.Transform(trsf)
            all_verts.extend([p.X(), p.Y(), p.Z()])
            local_idx[i] = vi
            vi += 1

        has_norms = tri.HasNormals()
        for i in range(1, nv + 1):
            if has_norms:
                n = tri.Normal(i)
                all_norms.extend([n.X(), n.Y(), n.Z()])
            else:
                all_norms.extend([0, 0, 1])

        nt = tri.NbTriangles()
        for i in range(1, nt + 1):
            t = tri.Triangle(i)
            a, b, c = t.Value(1), t.Value(2), t.Value(3)
            all_tris.extend([local_idx[a], local_idx[b], local_idx[c]])

        exp.Next()

    return jsonify({
        "vertices": all_verts,
        "normals": all_norms,
        "triangles": all_tris,
        "center": [(x1 + x2) / 2, (y1 + y2) / 2, (z1 + z2) / 2],
        "span": span,
    })


@app.route("/api/stl/<part>")
def api_stl(part):
    """STL 三角网格 → Three.js JSON"""
    stl_path = DATA / "stl" / f"{part}.stl"
    if not stl_path.exists():
        return jsonify({"error": "STL not found"}), 404

    import trimesh as tm
    m = tm.load(str(stl_path))
    verts = m.vertices.astype(np.float32)
    faces = m.faces.astype(np.int32)

    # compute normals
    if hasattr(m, 'vertex_normals') and m.vertex_normals is not None:
        norms = m.vertex_normals.astype(np.float32)
    else:
        norms = np.zeros_like(verts)

    return jsonify({
        "vertices": verts.flatten().tolist(),
        "normals": norms.flatten().tolist(),
        "triangles": faces.flatten().tolist(),
        "center": m.centroid.tolist() if hasattr(m, 'centroid') else [0, 0, 0],
        "span": float(m.extents.max()) if hasattr(m, 'extents') else 1.0,
    })


@app.route("/api/points/<part>")
def api_points(part):
    """点云 → JSON"""
    ply_path = DATA / "ply" / f"{part}_16384.ply"
    if not ply_path.exists():
        return jsonify({"error": "PLY not found"}), 404

    pts = []
    with open(ply_path) as f:
        for line in f:
            if line == "end_header\n":
                break
        for line in f:
            parts = line.split()
            if len(parts) >= 3:
                pts.append([float(parts[0]), float(parts[1]), float(parts[2])])

    # 全部点
    arr = np.array(pts, dtype=np.float32)
    return jsonify({
        "points": arr.flatten().tolist(),
        "center": arr.mean(axis=0).tolist(),
        "span": float((arr.max(axis=0) - arr.min(axis=0)).max()),
    })


@app.route("/api/voxels/<part>")
def api_voxels(part):
    """体素 → JSON (表面体素)"""
    npy_path = DATA / "npy" / f"{part}_128.npy"
    if not npy_path.exists():
        return jsonify({"error": "NPY not found"}), 404

    arr = np.load(str(npy_path)).astype(bool)
    R = arr.shape[0]

    # 加载 STEP 算 bbox
    shape = get_shape(part)
    from OCC.Core.Bnd import Bnd_Box
    from OCC.Core.BRepBndLib import brepbndlib
    bbox = Bnd_Box(); brepbndlib.Add(shape, bbox)
    x1, y1, z1, x2, y2, z2 = bbox.Get()
    span = max(x2 - x1, y2 - y1, z2 - z1) * 1.05
    step = span / R
    ox = x1 - (span - (x2 - x1)) / 2
    oy = y1 - (span - (y2 - y1)) / 2
    oz = z1 - (span - (z2 - z1)) / 2

    voxels = []
    step_v = 1
    for i in range(0, R, step_v):
        for j in range(0, R, step_v):
            for k in range(0, R, step_v):
                if not arr[i, j, k]:
                    continue
                cx = float(ox + (i + step_v / 2) * step)
                cy = float(oy + (j + step_v / 2) * step)
                cz = float(oz + (k + step_v / 2) * step)
                hs = float(step * step_v / 2)
                voxels.append([cx, cy, cz, hs])

    if not voxels:
        return jsonify({"error": "no voxels"}), 404

    return jsonify({
        "voxels": voxels,
        "center": [(x1 + x2) / 2, (y1 + y2) / 2, (z1 + z2) / 2],
        "span": float(span),
    })


if __name__ == "__main__":
    print("ISO-Mech Web Viewer → http://localhost:8005")
    app.run(host="127.0.0.1", port=8005, debug=False)
