# ISO-Mech

ISO 标准机械零件数据集。2,372 个 STEP B-Rep 实体，5 种 3D 格式，面向 AI 训练。所有尺寸严格对应 ISO 标准长度系列。

## 零件目录

| 英文名 | 中文名 | 标准 | 数量 |
|--------|--------|------|------|
| HexagonNut | 六角螺母 | ISO 4032/4033/4035 | 67 |
| HexHeadBolt | 六角头螺栓 | ISO 4014/4017 | ~600 |
| SocketHeadCapScrew | 内六角圆柱头螺钉 | ISO 4762 | ~200 |
| CountersunkScrew | 沉头螺钉 | ISO 14581/10642/2009/14582/7046 | ~600 |
| PanHeadScrew | 盘头螺钉 | ISO 14583/1580 | ~200 |
| SetScrew | 紧定螺钉 | ISO 4026 | ~200 |
| PlainWasher | 平垫圈 | ISO 7089/7091/7093/7094 | 75 |
| ChamferedWasher | 倒角垫圈 | ISO 7090 | 52 |
| DeepGrooveBallBearing | 深沟球轴承 | SKT | 31 |
| CappedDeepGrooveBallBearing | 带盖深沟球轴承 | SKT | 25 |
| AngularContactBallBearing | 角接触球轴承 | SKT | 9 |
| CylindricalRollerBearing | 圆柱滚子轴承 | SKT | 9 |
| TaperedRollerBearing | 圆锥滚子轴承 | SKT | 26 |
| SpurGear | 直齿轮 | ISO 54 | 130 |
| BevelGear | 锥齿轮 | ISO 54 | 42 |
| CrossedHelicalGear | 交叉斜齿轮 | ISO 54 | 28 |
| RackGear | 齿条 | ISO 54 | 21 |
| RingGear | 齿圈 | ISO 54 | 35 |
| WormGear | 蜗轮 | ISO 54 | 18 |
| | | **合计** | **2,372** |

螺栓/螺钉长度严格限制在 ISO 标准范围内（直径-长度上限校验）。

## 数据格式

| 格式 | 说明 | 数量 | 参数 |
|------|------|------|------|
| STEP (.step) | B-Rep 实体，精确 NURBS | 2,372 | CadQuery 生成 |
| STL (.stl) | 三角网格 | 2,372 | CadQuery tessellation |
| 点云 (.ply) | 表面均匀采样 | 2,372 | 16,384 点 |
| 体素 (.npy) | 128^3 二值体素 | 2,372 | OCC 实体判点 |
| 多视图 (.png) | 24 视角截图 | 56,928 | 300x300，黑底 |

## 生成

环境：Python 3.11, CadQuery, pythonocc-core, trimesh, numpy

```bash
conda activate occ

# 按顺序执行
python format_conversion/generate/gen_step_cpu.py              # CadQuery 生成 STEP
python format_conversion/step_to_stl/gen_step_to_stl_cpu.py    # STEP → STL
python format_conversion/stl_to_ply/gen_stl_to_ply_cpu.py      # STL → PLY 16384点
python format_conversion/step_to_npy/gen_step_to_npy_cpu.py    # STEP → NPY 体素
python format_conversion/step_to_png/gen_step_to_png_cpu.py    # PNG 24视角

# STL 逆向 STEP
python format_conversion/stl_to_step/gen_stl_to_step_l1_cpu.py  # L1: 三角缝合
python format_conversion/stl_to_step/gen_stl_to_step_l2_cpu.py  # L2: 共面合并
python format_conversion/stl_to_step/gen_stl_to_step_l3_cpu.py  # L3: 曲面拟合
```
```

全部 CPU 多进程（`cpu_count() - 1`）。

### 体素生成

直接从 STEP B-Rep 实体生成，不经 STL 中转：

1. `STEPControl_Reader` 读取 STEP
2. `BRepClass3d_SolidClassifier` 判定每个网格点是否在实体内部
3. 零近似、零射线、零精度问题

## Web 对比查看器

`http://localhost:8005` — Flask + Three.js，4 视口 2x2 网格，双向联动。

**架构：**
- 后端 Flask (`viewer/web_server.py`)，OCC 读 STEP 并 tessellate，trimesh 读 STL，numpy 读点云/体素
- 前端 Three.js 0.160 (`viewer/web_index.html`)，4 个 WebGL 渲染器，OrbitControls 同步
- ANGLE (NVIDIA RTX 5060 Ti) WebGL 2.0
- 黑底 UI，零件下拉列表

**各格式渲染：**

| 位置 | 格式 | 渲染方式 |
|------|------|----------|
| 左上 | STEP | BRepMesh tessellation，Phong 光滑，蓝色 |
| 右上 | STL | trimesh 加载，flat shading + 线框叠加，橙色 |
| 左下 | 点云 | 16,384 点，绿色 |
| 右下 | 体素 | 128^3 全实体，Lambert + 白色线框，紫色 |

**启动：**
```bash
conda activate occ
python viewer/web_server.py
# 浏览器打开 http://localhost:8005
```

## 数据质量

- 统一坐标系，CadQuery 生成
- 尺寸严格对应 ISO 标准
- 体素零近似（OCC 实体判点）

## 许可

MIT
