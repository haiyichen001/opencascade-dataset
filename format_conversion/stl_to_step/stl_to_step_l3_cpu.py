"""
L3: STL → STEP — 区域生长 + RANSAC 平面/柱面拟合 + NURBS 重建
直接读取 STL，不依赖 L1/L2
"""
import os, time
from pathlib import Path
from multiprocessing import Pool, cpu_count
import numpy as np
import trimesh
from scipy.spatial import ConvexHull
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
    BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire)
from OCC.Core.gp import gp_Pnt, gp_Dir, gp_Pln

STL_DIR = r"D:\opencascade-dataset\stl"
STEP_OUT = r"D:\opencascade-dataset\step_reverse_l3"
os.makedirs(STEP_OUT, exist_ok=True)


def region_grow(mesh, angle_deg=8, min_region=5):
    """角度+共面双重检查区域生长"""
    fn = mesh.face_normals
    verts, faces = mesh.vertices, mesh.faces
    adj = mesh.face_adjacency
    angles = mesh.face_adjacency_angles
    nf = len(fn)
    tol = (verts.max(0) - verts.min(0)).max() * 0.005
    arad = np.radians(angle_deg)
    centroids = np.array([verts[faces[i]].mean(0) for i in range(nf)])

    graph = {i: set() for i in range(nf)}
    for (a, b), ang in zip(adj, angles):
        if ang < arad or ang > np.pi - arad:
            d = abs(np.dot(centroids[b] - centroids[a], fn[a]))
            if d < tol:
                graph[a].add(b); graph[b].add(a)

    visited, regions = set(), []
    for seed in range(nf):
        if seed in visited: continue
        queue, region = [seed], []
        while queue:
            f = queue.pop()
            if f in visited: continue
            visited.add(f); region.append(f)
            for nb in graph[f]:
                if nb not in visited: queue.append(nb)
        if len(region) >= min_region:
            regions.append(np.array(region))
    return regions


def fit_plane(pts, threshold):
    """RANSAC 平面"""
    n, best = len(pts), []
    best_n, best_d = None, 0
    for _ in range(min(200, max(20, n // 3))):
        i = np.random.choice(n, 3, replace=False)
        p0, p1, p2 = pts[i]
        nn = np.cross(p1 - p0, p2 - p0)
        nl = np.linalg.norm(nn)
        if nl < 1e-10: continue
        nn /= nl
        dd = -np.dot(nn, p0)
        inl = np.where(np.abs(np.dot(pts, nn) + dd) < threshold)[0]
        if len(inl) > len(best):
            best, best_n, best_d = inl, nn, dd
    return best_n, best_d, best


def fit_cylinder(pts, threshold):
    """RANSAC 柱面"""
    n, best = len(pts), []
    best_axis, best_ctr, best_r = None, None, 0
    for _ in range(min(150, max(15, n // 4))):
        i = np.random.choice(n, 5, replace=False)
        p = pts[i]
        e1, e2 = p[1] - p[0], p[2] - p[0]
        ax = np.cross(e1, e2)
        al = np.linalg.norm(ax)
        if al < 1e-10: continue
        ax /= al
        ctr = p[0] - np.dot(p[0], ax) * ax
        r = np.median(np.linalg.norm(pts - np.outer(np.dot(pts, ax), ax) - ctr, axis=1))
        if r < threshold: continue
        d = np.abs(np.linalg.norm(pts - np.outer(np.dot(pts, ax), ax) - ctr, axis=1) - r)
        inl = np.where(d < threshold)[0]
        if len(inl) > len(best):
            best, best_axis, best_ctr, best_r = inl, ax, ctr, r
    return best_axis, best_ctr, best_r, best


def process_one(stl_path):
    stem = Path(stl_path).stem
    out = os.path.join(STEP_OUT, f"{stem}_l3.step")
    if os.path.exists(out): return (stem, "skip", {})

    try:
        mesh = trimesh.load(stl_path)
        if not mesh.is_watertight: mesh.fill_holes()
        verts, faces = mesh.vertices, mesh.faces
        bbox_span = (verts.max(0) - verts.min(0)).max()

        # 区域生长
        regions = region_grow(mesh, angle_deg=6, min_region=3)
        if not regions: return (stem, "no_regions", {})

        # 剩余未分配的面：合并到最近区域
        assigned = set()
        for r in regions: assigned.update(r)
        unassigned = [i for i in range(len(faces)) if i not in assigned]
        if unassigned:
            regions.append(np.array(unassigned))

        fit_thresh = bbox_span * 0.02

        sewer = BRepBuilderAPI_Sewing(1e-4, True, True, True, False)
        planes = cyls = freeform = 0

        for ridx, reg_faces in enumerate(regions):
            if len(reg_faces) < 3:
                # too small, skip or just add triangles
                for fi in reg_faces[:5]:
                    tri = faces[fi]
                    p0, p1, p2 = gp_Pnt(*verts[tri[0]]), gp_Pnt(*verts[tri[1]]), gp_Pnt(*verts[tri[2]])
                    e0 = BRepBuilderAPI_MakeEdge(p0, p1).Edge()
                    e1 = BRepBuilderAPI_MakeEdge(p1, p2).Edge()
                    e2 = BRepBuilderAPI_MakeEdge(p2, p0).Edge()
                    w = BRepBuilderAPI_MakeWire(e0, e1, e2).Wire()
                    sewer.Add(BRepBuilderAPI_MakeFace(w).Face())
                continue

            reg_verts_idx = np.unique(faces[reg_faces].flatten())
            pts = verts[reg_verts_idx]

            # Fit plane
            pn, pd, p_inl = fit_plane(pts, fit_thresh)
            p_ratio = len(p_inl) / len(pts) if len(pts) > 0 else 0

            if p_ratio > 0.6 and pn is not None:
                # Create planar face from convex hull
                proj = pts - (np.dot(pts, pn) + pd)[:, None] * pn[None, :]
                u = proj[1] - proj[0]; u /= np.linalg.norm(u)
                v = np.cross(pn, u)
                uv = np.column_stack([np.dot(proj, u), np.dot(proj, v)])
                try:
                    hull = ConvexHull(uv)
                    hull_pts = pts[hull.vertices]
                    occ_pts = [gp_Pnt(*p) for p in hull_pts]
                    wb = BRepBuilderAPI_MakeWire()
                    for i in range(len(occ_pts)):
                        e = BRepBuilderAPI_MakeEdge(occ_pts[i], occ_pts[(i + 1) % len(occ_pts)]).Edge()
                        wb.Add(e)
                    if wb.IsDone():
                        pln = gp_Pln(gp_Pnt(*pts[0]), gp_Dir(*pn))
                        face = BRepBuilderAPI_MakeFace(pln, wb.Wire()).Face()
                        sewer.Add(face)
                        planes += 1
                except:
                    freeform += 1
            else:
                # Try cylinder
                ca, cc, cr, c_inl = fit_cylinder(pts, fit_thresh * 2)
                c_ratio = len(c_inl) / len(pts) if len(pts) > 0 else 0
                if c_ratio > 0.4 and ca is not None:
                    cyls += 1
                    # Cylinder surface creation is complex; use triangulated faces for now
                    for fi in reg_faces[:30]:
                        tri = faces[fi]
                        p0, p1, p2 = gp_Pnt(*verts[tri[0]]), gp_Pnt(*verts[tri[1]]), gp_Pnt(*verts[tri[2]])
                        e0 = BRepBuilderAPI_MakeEdge(p0, p1).Edge()
                        e1 = BRepBuilderAPI_MakeEdge(p1, p2).Edge()
                        e2 = BRepBuilderAPI_MakeEdge(p2, p0).Edge()
                        w = BRepBuilderAPI_MakeWire(e0, e1, e2).Wire()
                        sewer.Add(BRepBuilderAPI_MakeFace(w).Face())
                else:
                    freeform += 1
                    for fi in reg_faces[:10]:
                        tri = faces[fi]
                        p0, p1, p2 = gp_Pnt(*verts[tri[0]]), gp_Pnt(*verts[tri[1]]), gp_Pnt(*verts[tri[2]])
                        e0 = BRepBuilderAPI_MakeEdge(p0, p1).Edge()
                        e1 = BRepBuilderAPI_MakeEdge(p1, p2).Edge()
                        e2 = BRepBuilderAPI_MakeEdge(p2, p0).Edge()
                        w = BRepBuilderAPI_MakeWire(e0, e1, e2).Wire()
                        sewer.Add(BRepBuilderAPI_MakeFace(w).Face())

        sewer.Perform()
        result = sewer.SewedShape()

        writer = STEPControl_Writer()
        writer.Transfer(result, STEPControl_AsIs)
        writer.Write(out)

        return (stem, "ok", {"tri": len(faces), "regions": len(regions),
                              "planes": planes, "cylinders": cyls, "freeform": freeform})

    except Exception as e:
        return (stem, str(e)[:120], {})


if __name__ == "__main__":
    files = [str(f) for f in sorted(Path(STL_DIR).glob("*.stl"))]
    n = max(1, cpu_count() - 1)
    print(f"L3: Region Growing + RANSAC Plane/Cylinder Fitting")
    print(f"Files: {len(files)}, Workers: {n}")

    t0 = time.time()
    ok = fail = skip = 0
    with Pool(n) as p:
        for stem, status, stats in p.imap_unordered(process_one, files, chunksize=10):
            if status == "ok": ok += 1
            elif status == "skip": skip += 1
            else: fail += 1
            if (ok + fail) % 100 == 0:
                e = time.time() - t0
                r = e / (ok + fail) if (ok + fail) > 0 else 0
                eta = r * (len(files) - ok - fail - skip)
                print(f"  [{ok+fail}/{len(files)}] ok={ok} fail={fail} | {r:.1f}s/p | ETA {eta/60:.0f}min", flush=True)

    e = time.time() - t0
    print(f"\nDONE: ok={ok} fail={fail} skip={skip} in {e/60:.1f}min", flush=True)
