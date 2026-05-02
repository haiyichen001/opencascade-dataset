"""
ISO-Mech: ISO 标准机械零件 STEP 生成器
自动检测 CPU 核心数，留 1 核
"""
import os, cadquery as cq
from multiprocessing import Pool, cpu_count
from cq_warehouse.fastener import *
from cq_warehouse.bearing import *
from cq_gears import SpurGear, BevelGear, CrossedHelicalGear, RackGear, RingGear, Worm

ROOT = Path(__file__).parent.parent
STEP_DIR = ROOT / "step"
os.makedirs(STEP_DIR, exist_ok=True)

ISO_LENGTHS = [8,10,12,16,20,25,30,35,40,45,50,55,60,65,70,80,90,100,110,120,130,140,150,160,180,200]
GEAR_WIDTH = 10

def iso_min_len(d):
    if d <= 2: return 5
    if d <= 3: return 8
    if d <= 4: return 10
    if d <= 5: return 12
    if d <= 6: return 16
    if d <= 8: return 20
    if d <= 10: return 25
    if d <= 12: return 30
    if d <= 16: return 35
    if d <= 20: return 40
    if d <= 24: return 50
    if d <= 30: return 60
    return 80

def iso_max_len(d):
    if d <= 1.6: return 16
    if d <= 2: return 20
    if d <= 2.5: return 25
    if d <= 3: return 30
    if d <= 3.5: return 35
    if d <= 4: return 40
    if d <= 5: return 50
    if d <= 6: return 60
    if d <= 8: return 80
    if d <= 10: return 100
    if d <= 12: return 120
    if d <= 14: return 140
    if d <= 16: return 160
    if d <= 18: return 180
    return 200

def parse_d(size_str):
    try: return float(size_str.split('-')[0].replace('M','').replace(',','.'))
    except: return 0

# ====== 构建任务列表 ======
tasks = []

# Nuts
for t in ['iso4032','iso4033','iso4035']:
    for s in HexNut.sizes(t):
        tasks.append(('nut', t, s, 0))

# Screws
screw_types = [
    ("HexHeadBolt", HexHeadScrew, ['iso4014','iso4017']),
    ("SocketHeadCapScrew", SocketHeadCapScrew, ['iso4762']),
    ("CountersunkScrew", CounterSunkScrew, ['iso14581','iso10642','iso2009','iso14582','iso7046']),
    ("PanHeadScrew", PanHeadScrew, ['iso14583','iso1580']),
    ("SetScrew", SetScrew, ['iso4026']),
]
for label, cls, types in screw_types:
    for t in types:
        for s in cls.sizes(t):
            d = parse_d(s)
            minL = iso_min_len(d)
            maxL = iso_max_len(d)
            for L in ISO_LENGTHS:
                if L < minL or L > maxL: continue
                tasks.append(('screw', label, cls.__name__, t, s, L))

# Washers
for t in ['iso7089','iso7091','iso7093','iso7094']:
    for s in PlainWasher.sizes(t):
        tasks.append(('washer', 'PlainWasher', t, s))
for t in ['iso7090']:
    for s in ChamferedWasher.sizes(t):
        tasks.append(('washer', 'ChamferedWasher', t, s))

# Bearings
for bname, bcls in [
    ("DeepGrooveBallBearing", SingleRowDeepGrooveBallBearing),
    ("CappedDeepGrooveBallBearing", SingleRowCappedDeepGrooveBallBearing),
    ("AngularContactBallBearing", SingleRowAngularContactBallBearing),
    ("CylindricalRollerBearing", SingleRowCylindricalRollerBearing),
    ("TaperedRollerBearing", SingleRowTaperedRollerBearing),
]:
    for size in bcls.sizes("SKT"):
        tasks.append(('bearing', bname, size))

# Gears
for m in [0.5,0.8,1.0,1.25,1.5,2.0,2.5,3,4,5,6,8,10]:
    for z in [12,15,18,20,24,30,36,40,48,60]:
        tasks.append(('gear', 'SpurGear', m, z))

for m in [1,1.5,2,2.5,3,4,5]:
    for z in [15,20,24,30,36,40]:
        tasks.append(('gear', 'BevelGear', m, z))

for m in [1,1.5,2,2.5,3,4,5]:
    for z in [15,20,24,30]:
        tasks.append(('gear', 'CrossedHelicalGear', m, z))

for m in [1,1.5,2,2.5,3,4,5]:
    for L in [30,50,80]:
        tasks.append(('gear', 'RackGear', m, L))

for m in [1,1.5,2,2.5,3,4,5]:
    for z in [24,30,36,48,60]:
        tasks.append(('gear', 'RingGear', m, z))

for m in [1,1.5,2,2.5,3,4]:
    for n in [1,2,4]:
        tasks.append(('gear', 'WormGear', m, n))

# ====== 工作函数 ======
def do_task(task):
    t = task[0]
    try:
        if t == 'nut':
            _, std, size, _ = task
            fname = f"HexagonNut_{std}_{size}"
            path = os.path.join(STEP_DIR, fname + ".step")
            if os.path.exists(path): return None
            obj = HexNut(size=size, fastener_type=std)
            cq.exporters.export(obj, path)
            return fname

        elif t == 'screw':
            _, label, cls_name, std, size, L = task
            fname = f"{label}_{std}_{size}x{L}"
            path = os.path.join(STEP_DIR, fname + ".step")
            if os.path.exists(path): return None
            cls = globals()[cls_name]
            obj = cls(size=size, length=L, fastener_type=std)
            cq.exporters.export(obj, path)
            return fname

        elif t == 'washer':
            _, wtype, std, size = task
            fname = f"{wtype}_{std}_{size}"
            path = os.path.join(STEP_DIR, fname + ".step")
            if os.path.exists(path): return None
            cls = globals()[wtype]
            obj = cls(size=size, fastener_type=std)
            cq.exporters.export(obj, path)
            return fname

        elif t == 'bearing':
            _, bname, size = task
            fname = f"{bname}_{size}"
            path = os.path.join(STEP_DIR, fname + ".step")
            if os.path.exists(path): return None
            bmap = {
                "DeepGrooveBallBearing": SingleRowDeepGrooveBallBearing,
                "CappedDeepGrooveBallBearing": SingleRowCappedDeepGrooveBallBearing,
                "AngularContactBallBearing": SingleRowAngularContactBallBearing,
                "CylindricalRollerBearing": SingleRowCylindricalRollerBearing,
                "TaperedRollerBearing": SingleRowTaperedRollerBearing,
            }
            obj = bmap[bname](size=size, bearing_type="SKT")
            cq.exporters.export(obj, path)
            return fname

        elif t == 'gear':
            _, gtype, *args = task
            fname = f"{gtype}_M{args[0]}"
            if gtype == 'SpurGear':
                fname += f"_Z{args[1]}"
                path = os.path.join(STEP_DIR, fname + ".step")
                if os.path.exists(path): return None
                obj = SpurGear(module=args[0], teeth_number=args[1], width=GEAR_WIDTH).build()
            elif gtype == 'BevelGear':
                fname += f"_Z{args[1]}"
                path = os.path.join(STEP_DIR, fname + ".step")
                if os.path.exists(path): return None
                obj = BevelGear(module=args[0], teeth_number=args[1], cone_angle=45, face_width=GEAR_WIDTH).build()
            elif gtype == 'CrossedHelicalGear':
                fname += f"_Z{args[1]}"
                path = os.path.join(STEP_DIR, fname + ".step")
                if os.path.exists(path): return None
                obj = CrossedHelicalGear(module=args[0], teeth_number=args[1], width=GEAR_WIDTH).build()
            elif gtype == 'RackGear':
                fname += f"_L{args[1]}"
                path = os.path.join(STEP_DIR, fname + ".step")
                if os.path.exists(path): return None
                obj = RackGear(module=args[0], length=args[1], width=GEAR_WIDTH, height=15).build()
            elif gtype == 'RingGear':
                fname += f"_Z{args[1]}"
                path = os.path.join(STEP_DIR, fname + ".step")
                if os.path.exists(path): return None
                obj = RingGear(module=args[0], teeth_number=args[1], width=GEAR_WIDTH, rim_width=5).build()
            elif gtype == 'WormGear':
                fname += f"_T{args[1]}"
                path = os.path.join(STEP_DIR, fname + ".step")
                if os.path.exists(path): return None
                obj = Worm(module=args[0], lead_angle=10, n_threads=args[1], length=30).build()
            cq.exporters.export(obj, path)
            return fname
    except:
        return None

if __name__ == "__main__":
    n = max(1, cpu_count() - 1)
    print(f"Tasks: {len(tasks)}, Workers: {n}")
    ok = fail = 0
    with Pool(n) as p:
        for r in p.imap_unordered(do_task, tasks, chunksize=50):
            if r: ok += 1
            else: fail += 1
            if (ok + fail) % 500 == 0:
                print(f"  ok={ok} fail={fail}")
    print(f"\nDONE: {ok} generated, {fail} failed/dupes")
    print(f"STEP: {len(os.listdir(STEP_DIR))} files")
