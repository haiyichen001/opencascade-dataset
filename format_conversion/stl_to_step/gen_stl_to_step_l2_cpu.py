"""
L2: STL → STEP — 三角缝合 + 共面合并 (ShapeUpgrade_UnifySameDomain)
依赖 L1 输出
"""
import os, time
from pathlib import Path
from multiprocessing import Pool, cpu_count
from OCC.Core.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_AsIs
from OCC.Core.ShapeUpgrade import ShapeUpgrade_UnifySameDomain

STEP_IN = r"D:\opencascade-dataset\step_reverse"
STEP_OUT = r"D:\opencascade-dataset\step_reverse_l2"
os.makedirs(STEP_OUT, exist_ok=True)


def process_one(step_path):
    stem = Path(step_path).stem.replace("_reverse", "")
    out = os.path.join(STEP_OUT, f"{stem}_l2.step")
    if os.path.exists(out):
        return (stem, "skip")

    try:
        sr = STEPControl_Reader()
        if sr.ReadFile(step_path) != 1:
            return (stem, "read_fail")
        sr.TransferRoots()
        shape = sr.OneShape()

        unifier = ShapeUpgrade_UnifySameDomain(shape, True, True, True)
        unifier.Build()
        unified = unifier.Shape()

        writer = STEPControl_Writer()
        writer.Transfer(unified, STEPControl_AsIs)
        writer.Write(out)
        return (stem, "ok")
    except Exception as e:
        return (stem, str(e)[:120])


if __name__ == "__main__":
    files = [str(f) for f in sorted(Path(STEP_IN).glob("*_reverse.step"))]
    n = max(1, cpu_count() - 1)
    print(f"L2: UnifySameDomain (coplanar/coaxial merge)")
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
