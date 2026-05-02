"""
L3: STL -> STEP: region grow + dual plane/cylinder classification + NURBS faces
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


def region_grow(mesh, angle_deg=5, min_region=3):
    fn = mesh.face_normals
    verts, faces = mesh.vertices, mesh.faces
    adj = mesh.face_adjacency
    angles = mesh.face_adjacency_angles
    nf = len(fn)
    tol = (verts.max(0) - verts.min(0)).max() * 0.003
    arad = np.radians(angle_deg)
    centroids = np.array([verts[faces[i]].mean(0) for i in range(nf)])

    graph = {i: set() for i in range(nf)}
    for (a, b), ang in zip(adj, angles):
        if ang < arad or ang > np.pi - arad:
            da = abs(np.dot(centroids[b] - centroids[a], fn[a]))
            db = abs(np.dot(centroids[a] - centroids[b], fn[b]))
            if da < tol and db < tol:
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


def cylinder_clusters(faces_list, all_faces, verts, fn, fit_thresh):
    """SVD of remaining face normals -> cluster by angle around cylinder axis"""
    remaining = list(set(faces_list))
    if len(remaining) < 10: return []
    rem_n = fn[remaining]
    _, _, vh = np.linalg.svd(rem_n - rem_n.mean(axis=0))
    axis = vh[2]
    al = np.linalg.norm(axis)
    if al < 0.3: return []
    axis /= al

    # project normals -> angle around axis
    ref = np.array([1,0,0]) if abs(np.dot(axis,[1,0,0])) < 0.9 else np.array([0,1,0])
    u = np.cross(axis, ref); u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    proj = np.column_stack([np.dot(rem_n, u), np.dot(rem_n, v)])
    angles = np.arctan2(proj[:,1], proj[:,0])

    n_clusters = max(2, min(16, len(remaining) // 5))
    bins = np.linspace(-np.pi, np.pi, n_clusters + 1)
    clusters = []
    for bi in range(n_clusters):
        mask = (angles >= bins[bi]) & (angles < bins[bi + 1])
        cf = np.array(remaining)[mask]
        if len(cf) < 3: continue
        cv = np.unique(all_faces[cf].flatten())
        cpts = verts[cv]
        proj_pts = cpts - np.outer(np.dot(cpts, axis), axis)
        ctr = proj_pts.mean(axis=0)
        r = np.median(np.linalg.norm(proj_pts - ctr, axis=1))
        err = np.abs(np.linalg.norm(proj_pts - ctr, axis=1) - r).mean()
        if err < fit_thresh * 3:
            clusters.append((axis, ctr, r, cf))
    return clusters


def classify(pts, normals, threshold):
    """Return 'plane', 'cylinder', or 'freeform'"""
    # fit plane
    n = len(pts)
    best_pn, best_pd, best_inl = None, 0, []
    for _ in range(min(150, max(20, n // 3))):
        i = np.random.choice(n, 3, replace=False)
        p0, p1, p2 = pts[i]
        nn = np.cross(p1 - p0, p2 - p0)
        nl = np.linalg.norm(nn)
        if nl < 1e-10: continue
        nn /= nl
        dd = -np.dot(nn, p0)
        inl = np.where(np.abs(np.dot(pts, nn) + dd) < threshold)[0]
        if len(inl) > len(best_inl): best_inl, best_pn, best_pd = inl, nn, dd

    p_ratio = len(best_inl) / n if n > 0 else 0
    p_err = np.abs(np.dot(pts, best_pn) + best_pd).mean() if best_pn is not None else 1e9

    # fit cylinder via SVD of normals
    c_ratio, c_err = 0, 1e9
    ca, cc, cr = None, None, 0
    if len(normals) >= 3:
        ctr = normals - normals.mean(axis=0)
        svd_ok = ctr.shape[0] >= 3
        if svd_ok:
            _, _, vh = np.linalg.svd(ctr)
            cyl_axis = vh[2]
            al = np.linalg.norm(cyl_axis)
            if al > 0.3:
                proj = pts - np.outer(np.dot(pts, cyl_axis), cyl_axis)
                ctr_est = proj.mean(axis=0)
                r_est = np.median(np.linalg.norm(proj - ctr_est, axis=1))
                c_err = np.abs(np.linalg.norm(proj - ctr_est, axis=1) - r_est).mean()
                inl = np.where(np.abs(np.linalg.norm(proj - ctr_est, axis=1) - r_est) < threshold * 2)[0]
                c_ratio = len(inl) / n if n > 0 else 0
                ca, cc, cr = cyl_axis, ctr_est, r_est

    # pick: cylinder wins only if clearly better (error < 1/3 of plane)
    if c_err < p_err * 0.33 and c_err < threshold * 2 and cr > 0:
        return ("cylinder", ca, cc, cr, c_err)
    if p_err < threshold * 0.05:
        return ("plane", best_pn, best_pd, 0, p_err)
    if c_err < threshold * 3 and cr > 0:
        return ("cylinder", ca, cc, cr, c_err)
    return ("cylinder_candidate", ca if ca is not None else best_pn, cc if cc is not None else 0, cr, c_err)


def process_one(stl_path):
    stem = Path(stl_path).stem
    out = os.path.join(STEP_OUT, f"{stem}_l3.step")
    if os.path.exists(out): return (stem, "skip", {})

    try:
        mesh = trimesh.load(stl_path)
        if not mesh.is_watertight: mesh.fill_holes()
        verts, faces = mesh.vertices, mesh.faces
        fn = mesh.face_normals
        bbox_span = (verts.max(0) - verts.min(0)).max()
        fit_thresh = bbox_span * 0.015

        regions = region_grow(mesh, angle_deg=4, min_region=3)
        if not regions: return (stem, "no_regions", {})

        sewer = BRepBuilderAPI_Sewing(1e-4, True, True, True, False)
        planes = cyls = ff = 0
        cyl_candidates = []  # faces that might be cylindrical

        # Phase 1: extract planes
        for reg_faces in regions:
            if len(reg_faces) < 3: continue
            reg_verts = np.unique(faces[reg_faces].flatten())
            pts = verts[reg_verts]
            reg_normals = fn[reg_faces]
            surf_type, a, b, c, err = classify(pts, reg_normals, fit_thresh)

            if surf_type == "plane":
                pn, pd = a, b
                proj = pts - (np.dot(pts, pn) + pd)[:, None] * pn[None, :]
                u = proj[1] - proj[0]; u /= np.linalg.norm(u) + 1e-10
                v = np.cross(pn, u)
                uv = np.column_stack([np.dot(proj, u), np.dot(proj, v)])
                try:
                    hull = ConvexHull(uv)
                    hpts = pts[hull.vertices]
                    occ_pts = [gp_Pnt(*p) for p in hpts]
                    wb = BRepBuilderAPI_MakeWire()
                    for i in range(len(occ_pts)):
                        e = BRepBuilderAPI_MakeEdge(occ_pts[i], occ_pts[(i+1)%len(occ_pts)]).Edge()
                        wb.Add(e)
                    if wb.IsDone():
                        pln = gp_Pln(gp_Pnt(*pts[0]), gp_Dir(*pn))
                        sewer.Add(BRepBuilderAPI_MakeFace(pln, wb.Wire()).Face())
                        planes += 1
                except Exception:
                    cyl_candidates.extend(reg_faces)
            else:
                cyl_candidates.extend(reg_faces)

        # Phase 2: SVD cluster remaining faces into cylinders
        if cyl_candidates:
            clusters = cylinder_clusters(cyl_candidates, faces, verts, fn, fit_thresh)
            for axis, ctr, radius, cf in clusters:
                cyls += 1
                count = min(50, len(cf))
                for fi in cf[:count]:
                    tri = faces[fi]
                    p0 = gp_Pnt(*verts[tri[0]]); p1 = gp_Pnt(*verts[tri[1]]); p2 = gp_Pnt(*verts[tri[2]])
                    e0 = BRepBuilderAPI_MakeEdge(p0, p1).Edge()
                    e1 = BRepBuilderAPI_MakeEdge(p1, p2).Edge()
                    e2 = BRepBuilderAPI_MakeEdge(p2, p0).Edge()
                    w = BRepBuilderAPI_MakeWire(e0, e1, e2).Wire()
                    sewer.Add(BRepBuilderAPI_MakeFace(w).Face())
            if not clusters:
                ff += 1

        sewer.Perform()
        result = sewer.SewedShape()
        writer = STEPControl_Writer()
        writer.Transfer(result, STEPControl_AsIs)
        writer.Write(out)

        return (stem, "ok", {"tri": len(faces), "regions": len(regions),
                              "planes": planes, "cylinders": cyls, "freeform": ff})

    except Exception as e:
        return (stem, str(e)[:200], {})


if __name__ == "__main__":
    files = [str(f) for f in sorted(Path(STL_DIR).glob("*.stl"))]
    n = max(1, cpu_count() - 1)
    print(f"L3: Region Grow + Dual Plane/Cylinder + NURBS")
    print(f"Files: {len(files)}, Workers: {n}")

    t0 = time.time()
    ok = fail = skip = 0
    with Pool(n) as p:
        for stem, status, stats in p.imap_unordered(process_one, files, chunksize=10):
            if status == "ok": ok += 1
            elif status == "skip": skip += 1
            else:
                fail += 1
                if fail <= 3: print(f"  FAIL {stem}: {status[:100]}", flush=True)
            if (ok + fail) % 100 == 0:
                e = time.time() - t0
                r = e / max(1, ok + fail)
                eta = r * max(0, len(files) - ok - fail - skip)
                print(f"  [{ok+fail}/{len(files)}] ok={ok} fail={fail} | {r:.1f}s/p | ETA {eta/60:.0f}min", flush=True)

    e = time.time() - t0
    print(f"\nDONE: ok={ok} fail={fail} skip={skip} in {e/60:.1f}min", flush=True)
