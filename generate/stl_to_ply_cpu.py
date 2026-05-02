"""STL → 点云 PLY (65536点)"""
import os
from pathlib import Path
from multiprocessing import Pool, cpu_count
import trimesh
import numpy as np

ROOT = Path(__file__).parent.parent
STL_DIR = ROOT / "stl"
PLY_DIR = ROOT / "ply"
os.makedirs(PLY_DIR, exist_ok=True)

def save_ply(pts, path):
    # Binary PLY format, little-endian
    import struct
    n = len(pts)
    header = f"ply\nformat binary_little_endian 1.0\nelement vertex {n}\nproperty float x\nproperty float y\nproperty float z\nend_header\n"
    with open(path, 'wb') as f:
        f.write(header.encode())
        for p in pts:
            f.write(struct.pack('<fff', p[0], p[1], p[2]))

def process_one(stl_path_str):
    stem = Path(stl_path_str).stem
    out = os.path.join(PLY_DIR, f"{stem}_65536.ply")
    if os.path.exists(out): return (stem, "skip")
    try:
        mesh = trimesh.load(stl_path_str)
        pts = mesh.sample(65536)
        save_ply(pts, out)
        return (stem, "ok")
    except Exception as e:
        return (stem, str(e)[:80])

if __name__ == "__main__":
    files = [str(f) for f in sorted(Path(STL_DIR).glob("*.stl"))
             if not os.path.exists(os.path.join(PLY_DIR, f"{f.stem}_65536.ply"))]
    print(f"STL: {len(list(Path(STL_DIR).glob('*.stl')))} total, {len(files)} to do")

    ok = fail = skip = 0
    n = max(1, cpu_count() - 1)
    print(f"Workers: {n}")
    with Pool(n) as p:
        for stem, status in p.imap_unordered(process_one, files, chunksize=20):
            if status == "ok": ok += 1
            elif status == "skip": skip += 1
            else: fail += 1
            if (ok + fail) % 200 == 0:
                print(f"  ok={ok} fail={fail} skip={skip}")

    print(f"\nDONE: ok={ok} fail={fail} skip={skip}")
    print(f"PLY: {len(list(Path(PLY_DIR).glob('*.ply')))} files")
