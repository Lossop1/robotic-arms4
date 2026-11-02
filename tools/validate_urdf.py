"""
URDF 校验：不依赖 ROS，也不用 urdfpy（本机 urdfpy/networkx 与 numpy2 不兼容）。

检查项
 1. XML 可解析、名字唯一、运动树单根连通
 2. 惯量：正定 + 三角不等式（URDF/Gazebo 的实际物理可行性）
 3. 通用 URDF 正运动学（从文件里的 origin/axis 读，不是复用生成脚本的假设）
 4. 零位几何：tool_link 是否落在 (0.304, 0, 0.530) m
 5. 独立交叉验证：整体包围盒 vs R4_COMPLETE.STEP 的整体包围盒
 6. 渲染：零位 + 演示位姿，输出 PNG（侧视图便于和 preview/complete.png 对比）
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from lxml import etree

_ROOT = Path(__file__).resolve().parents[1]
VERSION = "0.1.0"

if "--version" in sys.argv[1:]:
    print(f"{Path(__file__).name} {VERSION}")
    raise SystemExit(0)

URDF_DIR = _ROOT / "data" / "models" / "r4_urdf"
PKG = _ROOT / "data" / "cad" / "R4_Integrated_20261029"
OUT = URDF_DIR / "preview"


# ---------------------------------------------------------------- 基础
def rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rot_axis(axis, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


def tf(R=None, p=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


def parse(path: Path):
    tree = etree.parse(str(path))
    robot = tree.getroot()
    links, joints = {}, []
    for e in robot.findall("link"):
        name = e.get("name")
        vis = [m.get("filename") for m in e.findall("visual/geometry/mesh")]
        col = [m.get("filename") for m in e.findall("collision/geometry/mesh")]
        ine = e.find("inertial")
        mass = None
        inertia = None
        if ine is not None:
            mass = float(ine.find("mass").get("value"))
            o = ine.find("origin")
            inertia = dict(
                xyz=[float(v) for v in o.get("xyz").split()] if o is not None else [0, 0, 0],
                rpy=[float(v) for v in o.get("rpy").split()] if o is not None else [0, 0, 0],
                m=[[float(ine.find("inertia").get(k)) for k in ("ixx", "ixy", "ixz")],
                   [float(ine.find("inertia").get("ixy")), float(ine.find("inertia").get("iyy")),
                    float(ine.find("inertia").get("iyz"))],
                   [float(ine.find("inertia").get("ixz")), float(ine.find("inertia").get("iyz")),
                    float(ine.find("inertia").get("izz"))]],
            )
        links[name] = dict(visual=vis, collision=col, mass=mass, inertia=inertia)
    for e in robot.findall("joint"):
        o = e.find("origin")
        ax = e.find("axis")
        lim = e.find("limit")
        joints.append(dict(
            name=e.get("name"), type=e.get("type"),
            parent=e.find("parent").get("link"), child=e.find("child").get("link"),
            xyz=[float(v) for v in o.get("xyz").split()] if o is not None else [0, 0, 0],
            rpy=[float(v) for v in o.get("rpy").split()] if o is not None else [0, 0, 0],
            axis=[float(v) for v in ax.get("xyz").split()] if ax is not None else [0, 0, 1],
            limit=None if lim is None else dict(
                lower=float(lim.get("lower")), upper=float(lim.get("upper")),
                effort=float(lim.get("effort")), velocity=float(lim.get("velocity"))),
        ))
    return robot.get("name"), links, joints


# ---------------------------------------------------------------- 检查
def check_structure(links, joints):
    errs = []
    children = [j["child"] for j in joints]
    if len(set(children)) != len(children):
        errs.append("同一 link 被多个 joint 认领")
    unknown = [j["name"] for j in joints if j["parent"] not in links or j["child"] not in links]
    if unknown:
        errs.append(f"joint 引用了不存在的 link: {unknown}")
    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        errs.append(f"根节点数 = {len(roots)}（应为 1）: {roots}")
    # 连通性
    if roots:
        seen, stack = {roots[0]}, [roots[0]]
        while stack:
            cur = stack.pop()
            for j in joints:
                if j["parent"] == cur and j["child"] not in seen:
                    seen.add(j["child"])
                    stack.append(j["child"])
        if seen != set(links):
            errs.append(f"孤岛 link: {sorted(set(links) - seen)}")
    return errs, roots


def classify_inertia(I):
    w = np.linalg.eigvalsh(I)
    if w.min() <= 0:
        return "非正定"
    if w[0] + w[1] < w[2]:
        return "违反三角不等式"
    return "ok"


# ---------------------------------------------------------------- FK
def fk(links, joints, q):
    children = {j["child"] for j in joints}
    roots = [n for n in links if n not in children]
    T = {roots[0]: np.eye(4)}
    order, pending = [], list(joints)
    while pending:
        prog = False
        for j in list(pending):
            if j["parent"] in T:
                Tj = T[j["parent"]] @ tf(rpy_to_R(*j["rpy"]), np.array(j["xyz"]))
                T[j["child"]] = Tj @ tf(rot_axis(j["axis"], q.get(j["name"], 0.0))
                                        if j["type"] in ("revolute", "continuous") else None)
                order.append(j)
                pending.remove(j)
                prog = True
        if not prog:
            raise SystemExit("运动树有环/断链: " + str([j['name'] for j in pending]))
    return T


def load_meshes(links, base: Path):
    import trimesh
    out = {}
    for name, L in links.items():
        for f in L["visual"]:
            p = (base / f.replace("package://r4_urdf/", "")).resolve()
            if not p.exists():
                p = (base / ".." / f.replace("package://r4_urdf/", "")).resolve()
            if not p.exists():
                raise SystemExit(f"缺网格: {f}")
            out[name] = trimesh.load_mesh(str(p), process=False)
    return out


# ---------------------------------------------------------------- 渲染
def render(scene_items, path: Path, cam=(1.25, -1.15, 0.78), look=(0.12, 0.0, 0.28), size=(1400, 800)):
    """vtk 离屏渲染。scene_items: [(vertices Nx3, faces Mx3, rgba)]"""
    import vtk
    ren = vtk.vtkRenderer()
    ren.SetBackground(0.93, 0.93, 0.94)
    ren.SetBackground2(0.80, 0.82, 0.86)
    ren.GradientBackgroundOn()
    for V, F, rgb in scene_items:
        pts = vtk.vtkPoints()
        pts.SetNumberOfPoints(len(V))
        flat = np.asarray(V, dtype=np.float64).ravel()
        for i in range(0, len(flat), 3):
            pts.SetPoint(i // 3, flat[i], flat[i + 1], flat[i + 2])
        cells = vtk.vtkCellArray()
        for tri in np.asarray(F):
            cells.InsertNextCell(3)
            for k in tri:
                cells.InsertCellPoint(int(k))
        pd = vtk.vtkPolyData()
        pd.SetPoints(pts)
        pd.SetPolys(cells)
        nrm = vtk.vtkPolyDataNormals()
        nrm.SetInputData(pd)
        nrm.ConsistencyOn()
        nrm.SplittingOff()
        nrm.Update()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(nrm.GetOutputPort())
        mapper.ScalarVisibilityOff()
        act = vtk.vtkActor()
        act.SetMapper(mapper)
        p = act.GetProperty()
        p.SetColor(*rgb)
        p.SetSpecular(0.25)
        p.SetSpecularPower(24)
        p.SetAmbient(0.28)
        p.SetDiffuse(0.78)
        p.EdgeVisibilityOff()
        ren.AddActor(act)

    for i, (pos, inten) in enumerate([((2, -2, 3), 1.0), ((-2, -1, 1.5), 0.45)]):
        l = vtk.vtkLight()
        l.SetPosition(*pos)
        l.SetFocalPoint(*look)
        l.SetIntensity(inten)
        ren.AddLight(l)
    ren.SetLightFollowCamera(False)

    cam_ = ren.GetActiveCamera()
    cam_.SetPosition(*cam)
    cam_.SetFocalPoint(*look)
    cam_.SetViewUp(0, 0, 1)
    cam_.SetViewAngle(30)
    ren.ResetCameraClippingRange()

    rw = vtk.vtkRenderWindow()
    rw.SetOffScreenRendering(1)
    rw.AddRenderer(ren)
    rw.SetSize(*size)
    rw.Render()
    w2i = vtk.vtkWindowToImageFilter()
    w2i.SetInput(rw)
    w2i.SetInputBufferTypeToRGB()
    w2i.ReadFrontBufferOn()
    w2i.Update()
    wr = vtk.vtkPNGWriter()
    wr.SetFileName(str(path))
    wr.SetInputConnection(w2i.GetOutputPort())
    wr.Write()
    return path


PALETTE = [(0.62, 0.66, 0.72), (0.24, 0.42, 0.28), (0.30, 0.45, 0.62),
           (0.42, 0.48, 0.58), (0.70, 0.45, 0.20)]


def scene_from_fk(T, meshes, order):
    items = []
    for i, name in enumerate(order):
        if name not in meshes:
            continue
        m = meshes[name]
        V = np.asarray(m.vertices) @ T[name][:3, :3].T + T[name][:3, 3]
        items.append((V, np.asarray(m.faces), PALETTE[i % len(PALETTE)]))
    return items


def main():
    deep = "--deep" in sys.argv
    OUT.mkdir(parents=True, exist_ok=True)
    urdf = URDF_DIR / "urdf" / "r4.urdf"
    name, links, joints = parse(urdf)
    print(f"# robot={name}  links={len(links)}  joints={len(joints)}")

    errs, roots = check_structure(links, joints)
    print(f"[1] 结构: {'OK' if not errs else 'FAIL'}  根={roots}")
    for e in errs:
        print("     ✗", e)

    print("[2] 惯量:")
    for n, L in links.items():
        if L["mass"] is None:
            print(f"     {n:10s} 无惯性（纯坐标系 link）")
            continue
        I = np.array(L["inertia"]["m"])
        w = np.linalg.eigvalsh(I)
        print(f"     {n:10s} m={L['mass']:7.3f} kg  主惯量={np.round(w, 6)}  {classify_inertia(I)}")

    T = fk(links, joints, {})
    tip = T["tool_link"][:3, 3] * 1000
    want = np.array([304.0, 0.0, 530.0])
    err = np.linalg.norm(tip - want)
    print(f"[3] 零位 tool_link = {np.round(tip, 3)} mm  期望 {want}  误差 {err:.2e} mm  "
          f"{'OK' if err < 1e-6 else 'FAIL'}")

    meshes = load_meshes(links, URDF_DIR)
    order = ["base_link", "link1", "link2", "link3", "link4"]
    T0 = fk(links, joints, {})
    allV = np.vstack([np.asarray(meshes[n].vertices) @ T0[n][:3, :3].T + T0[n][:3, 3]
                      for n in order])
    bb_min, bb_max = allV.min(0) * 1000, allV.max(0) * 1000
    print(f"[4] URDF 包围盒 mm: min={np.round(bb_min,1)} max={np.round(bb_max,1)}")

    if deep and (PKG / "CAD" / "R4_COMPLETE.STEP").exists():
        import cadquery as cq
        shp = cq.importers.importStep(str(PKG / "CAD" / "R4_COMPLETE.STEP")).val()
        b = shp.BoundingBox()
        print(f"[5] STEP 包围盒 mm(独立源): min=[{b.xmin:.1f} {b.ymin:.1f} {b.zmin:.1f}] "
              f"max=[{b.xmax:.1f} {b.ymax:.1f} {b.zmax:.1f}]")
        d = max(abs(bb_min[0] - b.xmin), abs(bb_min[1] - b.ymin), abs(bb_min[2] - b.zmin),
                abs(bb_max[0] - b.xmax), abs(bb_max[1] - b.ymax), abs(bb_max[2] - b.zmax))
        print(f"     包围盒最大偏差 = {d:.2f} mm  {'OK' if d < 2.0 else 'CHECK'}")
    else:
        print("[5] 包围盒交叉验证跳过（加 --deep 会导入 R4_COMPLETE.STEP）")

    p1 = render(scene_from_fk(T0, meshes, order), OUT / "urdf_zero.png")
    q_demo = {"joint1": math.radians(25), "joint2": math.radians(-35),
              "joint3": math.radians(55), "joint4": math.radians(40)}
    T1 = fk(links, joints, q_demo)
    p2 = render(scene_from_fk(T1, meshes, order), OUT / "urdf_demo.png")
    print(f"[6] 渲染: {p1.name}, {p2.name}")

    masses = {k: v["mass"] for k, v in links.items() if v["mass"]}
    print(f"[7] 总质量 {sum(masses.values()):.3f} kg  详情 {json.dumps({k: round(v,3) for k,v in masses.items()})}")
    return 0 if not errs else 1


if __name__ == "__main__":
    sys.exit(main())
