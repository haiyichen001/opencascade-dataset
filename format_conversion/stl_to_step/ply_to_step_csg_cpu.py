"""
PLY -> STEP via Primitive Fitting + CSG Half-Space Intersection
"""
import os, time
from pathlib import Path
from multiprocessing import Pool, cpu_count
import numpy as np
from scipy.spatial import KDTree
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeHalfSpace
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeFace
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common
from OCC.Core.gp import gp_Pnt, gp_Dir, gp_Pln, gp_Ax2, gp_Ax3
from OCC.Core.Geom import Geom_CylindricalSurface
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs, STEPControl_Reader
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import brepgprop

PLY_DIR = r"D:\opencascade-dataset\ply"
STEP_REF_DIR = r"D:\opencascade-dataset\step"
STEP_OUT = r"D:\opencascade-dataset\step_reverse_csg"
os.makedirs(STEP_OUT, exist_ok=True)


def fit_surfaces(points, span):
    """Normal clustering + spatial split -> plane/cylinder fit"""
    arr = points
    threshold = span * 0.015
    thresh3 = threshold * 3

    kdt = KDTree(arr)
    _, knn_idx = kdt.query(arr, k=12)
    normals = np.zeros_like(arr)
    for i in range(len(arr)):
        nb = arr[knn_idx[i]] - arr[i]
        _, _, vh = np.linalg.svd(nb)
        normals[i] = vh[2]

    n_clusters = 3
    theta = np.arccos(np.clip(normals[:, 2], -1, 1))
    phi = np.arctan2(normals[:, 1], normals[:, 0])
    t_idx = np.floor(theta / np.pi * n_clusters).astype(int).clip(0, n_clusters - 1)
    p_idx = np.floor((phi + np.pi) / (2 * np.pi) * n_clusters).astype(int).clip(0, n_clusters - 1)
    cluster_id = t_idx * n_clusters + p_idx

    surfaces = []
    for cid in np.unique(cluster_id):
        mask = cluster_id == cid
        if mask.sum() < 10:
            continue
        cluster_pts = arr[mask]
        ctr_all = cluster_pts.mean(axis=0)
        _, _, vh = np.linalg.svd(cluster_pts - ctr_all)
        nn = vh[2]
        proj_d = np.dot(cluster_pts, nn)
        d_sorted = np.sort(proj_d)
        gaps = np.diff(d_sorted)
        split_idx = np.where(gaps > threshold * 2)[0] + 1
        order = np.argsort(proj_d)
        groups = np.split(order, split_idx) if len(split_idx) > 0 else [order]

        for group in groups:
            if len(group) < 30:
                continue
            sub_pts = cluster_pts[group]
            ctr = sub_pts.mean(axis=0)
            _, _, vh2 = np.linalg.svd(sub_pts - ctr)
            nn2 = vh2[2]
            dd = -np.dot(nn2, ctr)
            err = np.abs(np.dot(sub_pts, nn2) + dd).mean()

            if err < thresh3:
                surfaces.append({"type": "plane", "normal": nn2, "d": float(dd), "n_points": int(len(sub_pts))})
            else:
                sub_n = normals[mask][group]
                if len(sub_n) >= 3:
                    _, _, vh3 = np.linalg.svd(sub_n - sub_n.mean(axis=0))
                    ax = vh3[2]
                    if np.linalg.norm(ax) > 0.3:
                        ax /= np.linalg.norm(ax)
                        proj = sub_pts - np.outer(np.dot(sub_pts, ax), ax)
                        pctr = proj.mean(axis=0)
                        r = np.median(np.linalg.norm(proj - pctr, axis=1))
                        c_err = np.abs(np.linalg.norm(proj - pctr, axis=1) - r).mean()
                        if c_err < thresh3:
                            surfaces.append({"type": "cylinder", "axis": ax, "center": pctr,
                                              "radius": float(r), "n_points": int(len(sub_pts))})
    return surfaces


def csg_from_fits(points, span, surfaces):
    """Build CSG solid from fitted surfaces using half-space intersection"""
    center = points.mean(axis=0)
    ref_pt = gp_Pnt(float(center[0]), float(center[1]), float(center[2]))
    size = span * 10

    shapes = []
    for s in surfaces:
        if s["type"] == "plane":
            n = np.array(s["normal"])
            d = s["d"]
            origin = -n * d
            pln = gp_Pln(gp_Pnt(float(origin[0]), float(origin[1]), float(origin[2])),
                         gp_Dir(float(n[0]), float(n[1]), float(n[2])))
            face = BRepBuilderAPI_MakeFace(pln, -size, size, -size, size).Face()
            try:
                shapes.append(BRepPrimAPI_MakeHalfSpace(face, ref_pt).Shape())
            except:
                pass

        elif s["type"] == "cylinder":
            ax = np.array(s["axis"])
            ctr = np.array(s["center"])
            r = s["radius"]
            h = span * 20
            p0 = ctr - ax * h / 2
            ax3 = gp_Ax3(gp_Pnt(float(p0[0]), float(p0[1]), float(p0[2])),
                         gp_Dir(float(ax[0]), float(ax[1]), float(ax[2])))
            cyl_surf = Geom_CylindricalSurface(ax3, r)
            face_cyl = BRepBuilderAPI_MakeFace(cyl_surf, -size, size, -size, size, 1e-4).Face()
            try:
                shapes.append(BRepPrimAPI_MakeHalfSpace(face_cyl, ref_pt).Shape())
            except:
                pass

    if len(shapes) < 3:
        return None

    result = shapes[0]
    for s in shapes[1:]:
        result = BRepAlgoAPI_Common(result, s).Shape()

    return result


def process_one(ply_path):
    stem = Path(ply_path).stem.replace("_16384", "")
    out = os.path.join(STEP_OUT, f"{stem}_csg.step")
    if os.path.exists(out):
        return (stem, "skip", 0)

    try:
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
        span = float((arr.max(axis=0) - arr.min(axis=0)).max())

        surfaces = fit_surfaces(arr, span)
        if not surfaces: return (stem, "no_surfaces", 0)

        result = csg_from_fits(arr, span, surfaces)
        if result is None: return (stem, "csg_failed", 0)

        writer = STEPControl_Writer()
        writer.Transfer(result, STEPControl_AsIs)
        writer.Write(out)

        ref_path = os.path.join(STEP_REF_DIR, f"{stem}.step")
        ref_vol = 1
        if os.path.exists(ref_path):
            sr = STEPControl_Reader(); sr.ReadFile(ref_path); sr.TransferRoots()
            props = GProp_GProps(); brepgprop.VolumeProperties(sr.OneShape(), props)
            ref_vol = props.Mass()
        props2 = GProp_GProps(); brepgprop.VolumeProperties(result, props2)
        err = abs(props2.Mass() - ref_vol) / ref_vol * 100 if ref_vol > 0 else 0
        return (stem, "ok", err)

    except Exception as e:
        return (stem, str(e)[:100], 0)


if __name__ == "__main__":
    files = [str(f) for f in sorted(Path(PLY_DIR).glob("*_16384.ply"))]
    n = max(1, cpu_count() - 1)
    print(f"PLY -> CSG Half-Space Intersection (Normal Clustering + Spatial Split)")
    print(f"Files: {len(files)}, Workers: {n}")

    t0 = time.time(); ok = fail = skip = 0; total_err = 0
    with Pool(n) as p:
        for r in p.imap_unordered(process_one, files, chunksize=5):
            stem, status = r[0], r[1]
            if status == "ok":
                ok += 1; total_err += r[2]
            elif status == "skip": skip += 1
            else:
                fail += 1
                if fail <= 5: print(f"  FAIL {stem}: {status}", flush=True)
            if (ok + fail) % 100 == 0:
                avg = total_err / ok if ok > 0 else 0
                e = time.time() - t0
                rate = e / max(1, ok + fail)
                eta = rate * max(0, len(files) - ok - fail - skip)
                print(f"  [{ok+fail}/{len(files)}] ok={ok} fail={fail} avg_err={avg:.1f}% | {rate:.1f}s/p | ETA {eta/60:.0f}min", flush=True)

    e = time.time() - t0
    avg = total_err / ok if ok > 0 else 0
    print(f"\nDONE: ok={ok} fail={fail} skip={skip} avg_err={avg:.1f}% in {e/60:.1f}min", flush=True)
