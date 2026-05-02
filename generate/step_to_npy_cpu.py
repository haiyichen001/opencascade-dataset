"""
STEP 直接体素化 — OCC BRepClass3d_SolidClassifier, 零近似, 零射线
"""
import os, time
from pathlib import Path
from multiprocessing import Pool, cpu_count
import numpy as np
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.BRepClass3d import BRepClass3d_SolidClassifier
from OCC.Core.gp import gp_Pnt
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.TopAbs import TopAbs_IN

STEP_DIR = r"D:\opencascade-dataset\step"
NPY_DIR = r"D:\opencascade-dataset\npy"
os.makedirs(NPY_DIR, exist_ok=True)
RES = 128


def process_one(step_path):
    stem = Path(step_path).stem
    out = os.path.join(NPY_DIR, f"{stem}_{RES}.npy")
    if os.path.exists(out):
        return (stem, "skip")

    try:
        sr = STEPControl_Reader()
        if sr.ReadFile(step_path) != 1:
            return (stem, "read_fail")
        sr.TransferRoots()
        shape = sr.OneShape()

        bbox = Bnd_Box()
        brepbndlib.Add(shape, bbox)
        x1, y1, z1, x2, y2, z2 = bbox.Get()
        span = max(x2 - x1, y2 - y1, z2 - z1) * 1.05
        step = span / RES
        ox = x1 - (span - (x2 - x1)) / 2
        oy = y1 - (span - (y2 - y1)) / 2
        oz = z1 - (span - (z2 - z1)) / 2

        classifier = BRepClass3d_SolidClassifier(shape)
        arr = np.zeros((RES, RES, RES), dtype=np.int8)

        # 只检测 bbox + margin 内的点
        margin_idx = int(2 / step)
        i0 = max(0, int((x1 - ox) / step) - margin_idx)
        i1 = min(RES, int((x2 - ox) / step) + margin_idx + 1)
        j0 = max(0, int((y1 - oy) / step) - margin_idx)
        j1 = min(RES, int((y2 - oy) / step) + margin_idx + 1)
        k0 = max(0, int((z1 - oz) / step) - margin_idx)
        k1 = min(RES, int((z2 - oz) / step) + margin_idx + 1)

        for i in range(i0, i1):
            cx = ox + (i + 0.5) * step
            for j in range(j0, j1):
                cy = oy + (j + 0.5) * step
                for k in range(k0, k1):
                    cz = oz + (k + 0.5) * step
                    classifier.Perform(gp_Pnt(cx, cy, cz), 1e-4)
                    if classifier.State() == TopAbs_IN:
                        arr[i, j, k] = 1

        np.save(out, arr)
        return (stem, "ok")

    except Exception as e:
        return (stem, str(e)[:120])


if __name__ == "__main__":
    files = sorted(Path(STEP_DIR).glob("*.step"))
    todo = [str(f) for f in files
            if not os.path.exists(os.path.join(NPY_DIR, f"{f.stem}_{RES}.npy"))]
    n = max(1, cpu_count() - 1)
    print(f"Method: OCC BRepClass3d_SolidClassifier (direct STEP)")
    print(f"Resolution: {RES}^3, Workers: {n}")
    print(f"Total: {len(files)}, To do: {len(todo)}")

    t0 = time.time()
    ok = fail = skip = 0
    with Pool(n) as p:
        for stem, status in p.imap_unordered(process_one, todo, chunksize=10):
            if status == "ok":
                ok += 1
            elif status == "skip":
                skip += 1
            else:
                fail += 1
            if (ok + fail) % 200 == 0:
                e = time.time() - t0
                r = e / (ok + fail)
                eta = r * (len(todo) - ok - fail)
                print(f"  [{ok+fail}/{len(todo)}] ok={ok} fail={fail} skip={skip} | {r:.1f}s/p | ETA {eta/60:.0f}min", flush=True)

    e = time.time() - t0
    print(f"\nDONE: ok={ok} fail={fail} skip={skip} in {e/60:.1f}min", flush=True)
    count = len(list(Path(NPY_DIR).glob("*.npy")))
    print(f"NPY files: {count}", flush=True)
