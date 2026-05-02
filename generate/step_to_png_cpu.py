"""批量生成 24 视图 PNG — 复用 Viewer + 多进程"""
import os, math
from pathlib import Path
from multiprocessing import Pool, cpu_count
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.Graphic3d import Graphic3d_NOM_PLASTIC
from OCC.Core.gp import gp_Dir
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Display.OCCViewer import Viewer3d

ROOT = Path(__file__).parent.parent
STEP_DIR = ROOT / "step"
PNG_DIR = ROOT / "png"
os.makedirs(PNG_DIR, exist_ok=True)

def fibonacci_sphere(n=24):
    v = []
    phi = math.pi * (3 - math.sqrt(5))
    for i in range(n):
        y = 1 - (i / (n - 1)) * 2
        r = math.sqrt(1 - y * y)
        theta = phi * i
        v.append(gp_Dir(math.cos(theta)*r, y, math.sin(theta)*r))
    return v

VIEW_DIRS = fibonacci_sphere(24)

def process_one(step_path_str):
    step_path = Path(step_path_str)
    stem = step_path.stem
    out_path = os.path.join(PNG_DIR, f"{stem}_24.png")
    if os.path.exists(out_path):
        return (stem, "skip")

    try:
        reader = STEPControl_Reader()
        if reader.ReadFile(str(step_path)) != IFSelect_RetDone:
            return (stem, "read_fail")
        reader.TransferRoots()
        shape = reader.OneShape()
        if shape.IsNull():
            return (stem, "null_shape")

        viewer = Viewer3d()
        viewer.Create()
        viewer.View.SetBackgroundColor(Quantity_TOC_RGB, 0.0, 0.0, 0.0)
        viewer.DisplayShape(shape, material=Graphic3d_NOM_PLASTIC,
                            color=Quantity_Color(0.55, 0.75, 0.95, Quantity_TOC_RGB), update=False)
        viewer.SetSize(300, 300)
        viewer.FitAll()
        viewer.SetModeShaded()

        for i, d in enumerate(VIEW_DIRS):
            viewer.View.SetProj(d.X(), d.Y(), d.Z())
            viewer.FitAll()
            viewer.View.Dump(os.path.join(PNG_DIR, f"{stem}_{i+1}.png"))

        return (stem, "ok")
    except Exception as e:
        return (stem, f"err:{e}")

if __name__ == "__main__":
    all_files = sorted(Path(STEP_DIR).glob("*.step"))
    to_do = [str(f) for f in all_files if not os.path.exists(os.path.join(PNG_DIR, f"{f.stem}_24.png"))]
    print(f"Total: {len(all_files)}, to do: {len(to_do)}")

    n_proc = 24
    print(f"Workers: {n_proc}")
    ok = fail = skip = 0
    with Pool(n_proc) as p:
        for stem, status in p.imap_unordered(process_one, to_do, chunksize=10):
            if status == "ok": ok += 1
            elif status == "skip": skip += 1
            else: fail += 1
            if (ok + fail) % 50 == 0:
                print(f"  ok={ok} fail={fail} skip={skip}")

    print(f"\nDONE: ok={ok} fail={fail} skip={skip}")
    print(f"PNGs: {len(list(Path(PNG_DIR).glob('*.png')))}")
