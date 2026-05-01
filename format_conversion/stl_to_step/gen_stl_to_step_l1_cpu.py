"""
L1: STL → STEP — 三角面→B-Rep缝合 + 自适应容差 + 退化过滤 + 实体化
"""
import os, time
from pathlib import Path
from multiprocessing import Pool, cpu_count
import numpy as np
import trimesh
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
    BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_MakeSolid)
from OCC.Core.gp import gp_Pnt
from OCC.Core.TopoDS import topods
from OCC.Core.ShapeFix import ShapeFix_Shell, ShapeFix_Solid
from OCC.Core.TopAbs import TopAbs_SHELL

STL_DIR = r"D:\opencascade-dataset\stl"
STEP_DIR = r"D:\opencascade-dataset\step_reverse"
os.makedirs(STEP_DIR, exist_ok=True)


def triangle_area(p0, p1, p2):
    a = np.array([p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2]])
    b = np.array([p2[0]-p0[0], p2[1]-p0[1], p2[2]-p0[2]])
    return np.linalg.norm(np.cross(a, b)) * 0.5


def process_one(stl_path):
    stem = Path(stl_path).stem
    out = os.path.join(STEP_DIR, f"{stem}_reverse.step")
    if os.path.exists(out):
        return (stem, "skip")

    try:
        mesh = trimesh.load(stl_path)
        if not mesh.is_watertight:
            mesh.fill_holes()
        verts = mesh.vertices
        faces = mesh.faces

        bbox_span = (verts.max(axis=0) - verts.min(axis=0)).max()
        # 自适应缝合容差
        sew_tol = max(1e-6, bbox_span * 1e-8)
        # 退化三角面阈值
        min_area = bbox_span * bbox_span * 1e-10

        sewer = BRepBuilderAPI_Sewing(sew_tol, True, True, True, False)
        skipped = 0
        for tri in faces:
            v0, v1, v2 = verts[tri[0]], verts[tri[1]], verts[tri[2]]
            if triangle_area(v0, v1, v2) < min_area:
                skipped += 1
                continue
            p0 = gp_Pnt(*v0); p1 = gp_Pnt(*v1); p2 = gp_Pnt(*v2)
            e0 = BRepBuilderAPI_MakeEdge(p0, p1).Edge()
            e1 = BRepBuilderAPI_MakeEdge(p1, p2).Edge()
            e2 = BRepBuilderAPI_MakeEdge(p2, p0).Edge()
            w = BRepBuilderAPI_MakeWire(e0, e1, e2).Wire()
            sewer.Add(BRepBuilderAPI_MakeFace(w).Face())

        sewer.Perform()
        result = sewer.SewedShape()

        # 尝试实体化
        is_solid = False
        if result.ShapeType() == TopAbs_SHELL:
            try:
                fixer = ShapeFix_Shell(topods.Shell(result))
                fixer.Perform()
                fixed = fixer.Shell()
                sm = BRepBuilderAPI_MakeSolid()
                sm.Add(topods.Shell(fixed))
                if sm.IsDone():
                    solid = sm.Solid()
                    sf = ShapeFix_Solid(solid)
                    sf.Perform()
                    result = sf.Solid()
                    is_solid = True
            except:
                pass

        writer = STEPControl_Writer()
        writer.Transfer(result, STEPControl_AsIs)
        writer.Write(out)

        return (stem, "ok")

    except Exception as e:
        return (stem, str(e)[:120])


if __name__ == "__main__":
    files = [str(f) for f in sorted(Path(STL_DIR).glob("*.stl"))]
    n = max(1, cpu_count() - 1)
    print(f"L1: Triangle→BRep with adaptive tol + degenerate filter + solidify")
    print(f"Files: {len(files)}, Workers: {n}")

    t0 = time.time()
    ok = fail = skip = 0
    with Pool(n) as p:
        for stem, status in p.imap_unordered(process_one, files, chunksize=20):
            if status == "ok": ok += 1
            elif status == "skip": skip += 1
            else: fail += 1
            if (ok + fail) % 200 == 0:
                e = time.time() - t0
                r = e / (ok + fail) if (ok + fail) > 0 else 0
                eta = r * (len(files) - ok - fail - skip)
                print(f"  [{ok+fail}/{len(files)}] ok={ok} fail={fail} | {r:.1f}s/p | ETA {eta/60:.0f}min", flush=True)

    e = time.time() - t0
    print(f"\nDONE: ok={ok} fail={fail} skip={skip} in {e/60:.1f}min", flush=True)
