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
    return send_file(str(WEB_ROOT / "multi_viewer.html"))


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


@app.route("/api/mesh_l<level>/<part>")
def api_mesh_reverse(level, part):
    """逆向 STEP 的 mesh"""
    from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
    from OCC.Core.TopExp import TopExp_Explorer
    from OCC.Core.TopAbs import TopAbs_FACE
    from OCC.Core.TopLoc import TopLoc_Location
    from OCC.Core.BRep import BRep_Tool
    from OCC.Core.Bnd import Bnd_Box
    from OCC.Core.BRepBndLib import brepbndlib

    # 确定逆向 STEP 文件路径
    if level == "1":
        rev_path = DATA / "step_reverse" / f"{part}_reverse.step"
    elif level == "2":
        rev_path = DATA / "step_reverse_l2" / f"{part}_l2.step"
    elif level == "3":
        rev_path = DATA / "step_reverse_l3" / f"{part}_l3.step"
    else:
        return jsonify({"error": "invalid level"}), 404

    if not rev_path.exists():
        return jsonify({"error": "reverse file not found"}), 404

    from OCC.Core.STEPControl import STEPControl_Reader
    sr = STEPControl_Reader()
    if sr.ReadFile(str(rev_path)) != 1:
        return jsonify({"error": "failed to read reverse step"}), 500
    sr.TransferRoots()
    shape = sr.OneShape()

    bbox = Bnd_Box(); brepbndlib.Add(shape, bbox)
    x1, y1, z1, x2, y2, z2 = bbox.Get()
    span = max(x2 - x1, y2 - y1, z2 - z1)

    BRepMesh_IncrementalMesh(shape, span / 100.0).Perform()

    all_verts = []; all_norms = []; all_tris = []; vi = 0
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = exp.Current()
        loc = TopLoc_Location()
        tri = BRep_Tool().Triangulation(face, loc)
        if tri is None: exp.Next(); continue
        trsf = loc.Transformation(); nv = tri.NbNodes()
        local_idx = {}
        for i in range(1, nv + 1):
            p = tri.Node(i); p.Transform(trsf)
            all_verts.extend([p.X(), p.Y(), p.Z()])
            local_idx[i] = vi; vi += 1
        has_n = tri.HasNormals()
        for i in range(1, nv + 1):
            if has_n:
                n = tri.Normal(i); all_norms.extend([n.X(), n.Y(), n.Z()])
            else:
                all_norms.extend([0, 0, 1])
        nt = tri.NbTriangles()
        for i in range(1, nt + 1):
            t = tri.Triangle(i)
            a, b, c = t.Value(1), t.Value(2), t.Value(3)
            all_tris.extend([local_idx[a], local_idx[b], local_idx[c]])
        exp.Next()

    return jsonify({
        "vertices": all_verts, "normals": all_norms, "triangles": all_tris,
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
    ply_path = DATA / "ply" / f"{part}_65536.ply"
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


@app.route("/api/fit/<part>")
def api_fit(part):
    """点云曲面拟合 — 多次 RANSAC 提取所有平面+柱面，无限边界"""
    ply_path = DATA / "ply" / f"{part}_65536.ply"
    if not ply_path.exists():
        return jsonify({"error": "PLY not found"}), 404

    import numpy as np

    pts = []
    with open(ply_path) as f:
        for line in f:
            if line == "end_header\n": break
        for line in f:
            parts = line.split()
            if len(parts) >= 3:
                pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
    arr = np.array(pts, dtype=np.float32)
    if len(arr) > 4096:
        step = len(arr) // 4096
        arr = arr[::step][:4096]

    bbox_min = arr.min(axis=0).tolist()
    bbox_max = arr.max(axis=0).tolist()
    center = arr.mean(axis=0).tolist()
    span = float((arr.max(axis=0) - arr.min(axis=0)).max())
    threshold = span * 0.015
    remaining = arr.copy()
    surfaces = []

    # 估算点云法线（局部平面拟合）
    from scipy.spatial import KDTree
    kdt = KDTree(arr)
    _, knn_idx = kdt.query(arr, k=12)
    normals = np.zeros_like(arr)
    for i in range(len(arr)):
        nb = arr[knn_idx[i]] - arr[i]
        _, _, vh = np.linalg.svd(nb)
        normals[i] = vh[2]

    # ---- 1st order: RANSAC planes ----
    for _ in range(8):
        n = len(remaining)
        if n < 20: break
        best_inl, best_nn, best_dd = [], None, 0
        for _ in range(min(150, n*2)):
            i = np.random.choice(n, 3, replace=False)
            p0, p1, p2 = remaining[i]
            nn = np.cross(p1-p0, p2-p0)
            nl = np.linalg.norm(nn)
            if nl < 1e-10: continue
            nn /= nl; dd = -np.dot(nn, p0)
            inl = np.where(np.abs(np.dot(remaining, nn)+dd) < threshold)[0]
            if len(inl) > len(best_inl): best_inl, best_nn, best_dd = inl, nn, dd
        if len(best_inl) < 20: break
        surfaces.append({"type":"plane","normal":best_nn.tolist(),"d":float(best_dd),"n_points":int(len(best_inl))})
        mask = np.ones(n, dtype=bool); mask[best_inl]=False; remaining=remaining[mask]

    # ---- 2nd order: RANSAC cylinders on remaining ----
    cyl_thresh = threshold * 2
    for _ in range(5):
        n = len(remaining)
        if n < 20: break
        # 用 KDTree 找 remaining 中的点的法线
        remaining_normals = np.array([normals[kdt.query([p])[1][0]] for p in remaining])
        best_inl, best_ax, best_ctr, best_r = [], None, None, 0
        for _ in range(min(100, n)):
            i = np.random.choice(n, min(5, n), replace=False)
            pts_sample = remaining[i]
            n_sample = remaining_normals[i]
            # SVD of normals -> axis
            _, _, vh = np.linalg.svd(n_sample - n_sample.mean(axis=0))
            ax = vh[2]; al = np.linalg.norm(ax)
            if al < 0.3: continue
            ax /= al
            proj = remaining - np.outer(np.dot(remaining, ax), ax)
            ctr = proj.mean(axis=0)
            r = np.median(np.linalg.norm(proj - ctr, axis=1))
            if r < cyl_thresh: continue
            d = np.abs(np.linalg.norm(proj - ctr, axis=1) - r)
            inl = np.where(d < cyl_thresh)[0]
            if len(inl) > len(best_inl): best_inl, best_ax, best_ctr, best_r = inl, ax, ctr, r
        if len(best_inl) < 15: break
        surfaces.append({"type":"cylinder","axis":best_ax.tolist(),"center":best_ctr.tolist(),"radius":float(best_r),"n_points":int(len(best_inl))})
        mask = np.ones(n, dtype=bool); mask[best_inl]=False; remaining=remaining[mask]

    # ---- 2nd order: RANSAC spheres on remaining ----
    for _ in range(3):
        n = len(remaining)
        if n < 15: break
        best_inl, best_sc, best_sr = [], None, 0
        for _ in range(min(100, n)):
            i = np.random.choice(n, min(5, n), replace=False)
            p = remaining[i]
            # Least squares sphere fit
            A = np.column_stack([p, np.ones(len(p))])
            b = -(p[:,0]**2 + p[:,1]**2 + p[:,2]**2)
            try:
                x, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            except: continue
            sc = -x[:3]/2; sr = np.sqrt(np.sum(sc**2) - x[3])
            if sr < threshold or sr > span: continue
            d = np.abs(np.linalg.norm(remaining - sc, axis=1) - sr)
            inl = np.where(d < threshold * 1.5)[0]
            if len(inl) > len(best_inl): best_inl, best_sc, best_sr = inl, sc, sr
        if len(best_inl) < 12: break
        surfaces.append({"type":"sphere","center":best_sc.tolist(),"radius":float(best_sr),"n_points":int(len(best_inl))})
        mask = np.ones(n, dtype=bool); mask[best_inl]=False; remaining=remaining[mask]

    return jsonify({
        "points": arr.tolist(),
        "center": center,
        "span": span,
        "bbox_min": bbox_min,
        "bbox_max": bbox_max,
        "surfaces": surfaces,
    })


@app.route("/api/labels/<part>")
def api_labels(part):
    """STEP 面类型标注 — 每面单独 mesh + 类型标签"""
    shape = get_shape(part)
    if shape is None:
        return jsonify({"error": "part not found"}), 404

    from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
    from OCC.Core.TopExp import TopExp_Explorer
    from OCC.Core.TopAbs import TopAbs_FACE
    from OCC.Core.TopLoc import TopLoc_Location
    from OCC.Core.BRep import BRep_Tool
    from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
    from OCC.Core.GeomAbs import (GeomAbs_Plane, GeomAbs_Cylinder, GeomAbs_Cone,
        GeomAbs_Sphere, GeomAbs_Torus, GeomAbs_BezierSurface, GeomAbs_BSplineSurface)
    from OCC.Core.Bnd import Bnd_Box
    from OCC.Core.BRepBndLib import brepbndlib

    bbox = Bnd_Box(); brepbndlib.Add(shape, bbox)
    x1,y1,z1,x2,y2,z2 = bbox.Get()
    span = max(x2-x1, y2-y1, z2-z1)
    BRepMesh_IncrementalMesh(shape, span / 100.0).Perform()

    type_names = {GeomAbs_Plane: "Plane", GeomAbs_Cylinder: "Cylinder",
        GeomAbs_Cone: "Cone", GeomAbs_Sphere: "Sphere", GeomAbs_Torus: "Torus",
        GeomAbs_BezierSurface: "Bezier", GeomAbs_BSplineSurface: "BSpline"}
    type_colors = {GeomAbs_Plane: "#4db8ff", GeomAbs_Cylinder: "#44cc44",
        GeomAbs_Cone: "#ff8844", GeomAbs_Sphere: "#ff44ff", GeomAbs_Torus: "#ffcc00",
        GeomAbs_BezierSurface: "#888888", GeomAbs_BSplineSurface: "#aaaaaa"}

    faces_data = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = exp.Current()
        adapt = BRepAdaptor_Surface(face, True)
        st = adapt.GetType()
        loc = TopLoc_Location()
        tri = BRep_Tool().Triangulation(face, loc)
        if tri is None: exp.Next(); continue
        trsf = loc.Transformation()
        nv = tri.NbNodes(); nt = tri.NbTriangles()
        verts = []; tris_idx = []; vi = 0; local = {}
        for i in range(1, nv+1):
            p = tri.Node(i); p.Transform(trsf)
            verts.extend([p.X(), p.Y(), p.Z()]); local[i] = vi; vi += 1
        for i in range(1, nt+1):
            t = tri.Triangle(i)
            tris_idx.extend([local[t.Value(1)], local[t.Value(2)], local[t.Value(3)]])
        # 面中心
        cx = sum(verts[::3])/vi; cy = sum(verts[1::3])/vi; cz = sum(verts[2::3])/vi
        faces_data.append({
            "vertices": verts, "triangles": tris_idx,
            "type": type_names.get(st, "Other"),
            "color": type_colors.get(st, "#ffffff"),
            "center": [cx, cy, cz],
        })
        exp.Next()

    return jsonify({
        "faces": faces_data,
        "center": [(x1+x2)/2, (y1+y2)/2, (z1+z2)/2],
        "span": span,
    })


@app.route("/labels")
def labels_page():
    return send_file(str(WEB_ROOT / "labels_viewer.html"))




if __name__ == "__main__":
    print("ISO-Mech Web Viewer → http://localhost:8005")
    app.run(host="127.0.0.1", port=8005, debug=False)
