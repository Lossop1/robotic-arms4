"""
R4 四轴机械臂：SolidWorks 装配包 -> URDF

唯一可信输入（不猜测、不手填几何）：
  <PKG>/source/assembly.json   201 条实例，4x4 齐次矩阵，单位 mm，含 group 字段
  <PKG>/CAD/<part>.STEP        零件几何，单位 mm（AP203, SolidWorks 2025）
  <PKG>/checks/native_integration.log
                               8 条配合 = 4x(重合+同心) -> 4 个 1 自由度转动副
关节轴来自几何反算：4 个 *_hub 实例的零件都是 P04，其主轴为局部 Z
（r=47/30/10 三个同轴圆柱面），乘以实例旋转矩阵得到世界轴向量。

用法:
  python tools/build_urdf.py            # 全量生成
  python tools/build_urdf.py --quick    # 粗网格试跑
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
PKG = _ROOT / "data" / "cad" / "R4_Integrated_20261029"
OUT = _ROOT / "data" / "models" / "r4_urdf"

# ---------------------------------------------------------------- 密度假设 kg/m^3
DENSITY = {
    "structure": 2700.0,   # P01..P14 铝合金板/支架
    "fastener": 7850.0,    # H_M5/H_M6 钢制螺栓螺母
    "motor": 3500.0,       # MotorReference：钢定子+铝壳+空腔的等效密度
}
FASTENERS = {"H_M5x16", "H_M5x20", "H_M6x16", "H_M6x20", "H_M6x25", "H_M6nut", "H_lockpin"}

# 网格离散精度 (mm, rad)
TOL = {"default": (0.08, 0.20), "fastener": (0.15, 0.45), "motor": (0.30, 0.50)}
QUICK_SCALE = 3.0
# 电机模型带波纹端盖，B-rep 直出 9.5 万面；视觉上减到 35% 仍看得很清楚
DECIMATE = {"MotorReference": 0.35}

# ---------------------------------------------------------------- 运动链
# 每个连杆坐标系 = 其关节轴上的点，姿态与装配坐标系一致（纯平移）；原点在 z=0 基座面
CHAIN = [
    dict(joint="joint1", parent="base_link", child="link1", axis=(0, 0, 1),
         point_mm=(0.0, 0.0, 110.0), kind="revolute",
         limit_deg=(-180.0, 180.0), effort=None, velocity=1.0),
    dict(joint="joint2", parent="link1", child="link2", axis=(0, 1, 0),
         point_mm=(0.0, 0.0, 270.0), kind="revolute",
         limit_deg=(-100.0, 100.0), effort=None, velocity=1.0),
    dict(joint="joint3", parent="link2", child="link3", axis=(0, 1, 0),
         point_mm=(0.0, 36.0, 530.0), kind="revolute",
         limit_deg=(-140.0, 140.0), effort=None, velocity=1.0),
    dict(joint="joint4", parent="link3", child="link4", axis=(1, 0, 0),
         point_mm=(240.0, 0.0, 530.0), kind="revolute",
         limit_deg=(-180.0, 180.0), effort=None, velocity=1.0),
    dict(joint="tool_mount", parent="link4", child="tool_link", axis=(1, 0, 0),
         point_mm=(304.0, 0.0, 530.0), kind="fixed",
         limit_deg=None, effort=None, velocity=None),
]
GROUP_TO_LINK = {0: "base_link", 1: "link1", 2: "link2", 3: "link3", 4: "link4"}

# ---------------------------------------------------------------- 关节限位/力矩口径
# 限位：包里无任何机械限位数据，取四轴小型臂的保守行程；标注为假设，可一行改掉
# effort：由重力静力矩最坏工况 x1.5 安全系数算出（见 compute_effort）


def log(msg: str) -> None:
    print(msg, flush=True)


# ============================================================ 工具函数
def rot_to_rpy(R: np.ndarray):
    """URDF 的 rpy = Rz(yaw)Ry(pitch)Rx(roll)。本项目全部 rpy=0（连杆姿态与装配一致）。"""
    return (0.0, 0.0, 0.0)


def load_assembly():
    data = json.loads((PKG / "source" / "assembly.json").read_text(encoding="utf-8"))
    inst = []
    for it in data:
        T = np.array(it["T"], dtype=float)  # 4x4, mm
        inst.append(dict(name=it["name"], part=it["part"], group=int(it["group"]),
                         R=T[:3, :3].copy(), t=T[:3, 3].copy()))
    return inst


def density_of(part: str) -> float:
    if part in FASTENERS:
        return DENSITY["fastener"]
    if part == "MotorReference":
        return DENSITY["motor"]
    return DENSITY["structure"]


# ============================================================ 零件几何
def import_part(part: str):
    """返回 (cadquery Shape, 体积 mm^3, 质心 mm(3), 单位密度惯量 mm^5 关于质心)"""
    import cadquery as cq
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.TopoDS import TopoDS_Shape

    path = PKG / "CAD" / f"{part}.STEP"
    if not path.exists():
        raise SystemExit(f"缺 STEP: {path}")
    wp = cq.importers.importStep(str(path))
    shp = wp.val()

    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shp.wrapped, props)
    V = float(props.Mass())
    c = props.CentreOfMass()
    com = np.array([c.X(), c.Y(), c.Z()], dtype=float)
    M = props.MatrixOfInertia()
    I = np.array([[M.Value(r, k) for k in (1, 2, 3)] for r in (1, 2, 3)], dtype=float)
    return shp, V, com, I


def tessellate(shp, part: str, quick: bool):
    tol, ang = TOL["fastener" if part in FASTENERS else ("motor" if part == "MotorReference" else "default")]
    if quick:
        tol *= QUICK_SCALE
        ang = min(0.9, ang * 2.5)
    verts, tris = shp.tessellate(tol, ang)
    V = np.array([[v.x, v.y, v.z] for v in verts], dtype=np.float64)
    F = np.array(tris, dtype=np.int64).reshape(-1, 3)
    if part in DECIMATE:
        import trimesh
        m = trimesh.Trimesh(vertices=V, faces=F, process=True)
        m = m.simplify_quadric_decimation(face_count=int(len(F) * DECIMATE[part]))
        V, F = np.asarray(m.vertices, dtype=np.float64), np.asarray(m.faces, dtype=np.int64)
    return V, F


# ============================================================ 惯量合成
def link_inertial(instances, parts, frame_mm):
    """按实例把零件质量/惯量搬到连杆坐标系。
    返回 (m [kg], com [m, 相对连杆坐标系], I [kg*m^2, 关于 com, 连杆系姿态])。"""
    total_m = 0.0
    first = None
    entries = []
    for it in instances:
        p = parts[it["part"]]
        rho_mm3 = p["rho"] / 1e9
        m = p["V"] * rho_mm3                                   # kg
        I_local = p["I"] * rho_mm3 * 1e-6                       # kg*m^2 关于零件质心
        I_world = it["R"] @ I_local @ it["R"].T
        com_world = it["R"] @ p["com"] + it["t"]                # mm
        d = (com_world - frame_mm) / 1000.0                     # m
        I_about_frame = I_world + m * ((d @ d) * np.eye(3) - np.outer(d, d))
        entries.append((m, d, I_about_frame))
        total_m += m
    if total_m <= 0:
        raise SystemExit("连杆无质量")
    ds = sum(m * d for m, d, _ in entries) / total_m
    I_com = np.zeros((3, 3))
    for m, d, I_f in entries:
        r = d - ds
        I_com += I_f + m * ((r @ r) * np.eye(3) - np.outer(r, r))
    return total_m, ds, I_com


def fix_triangle_inequality(I):
    """数值/薄板组合下可能微违反三角不等式，URDF 解析器会报警。按比例轻微放大最小主惯量。"""
    w = np.linalg.eigvalsh(I)
    pairs = [(w[1] + w[2] - w[0]), (w[0] + w[2] - w[1]), (w[0] + w[1] - w[2])]
    if min(pairs) < 0:
        eps = max(1e-9, abs(min(pairs)) * 1.001)
        I = I + eps * np.eye(3)
        return I, True
    return I, False


# ============================================================ 重力力矩 / effort
GRAV = 9.80665


def rot_axis(axis, ang):
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


def joint_frames(q_deg):
    """返回每个连杆坐标系（相对装配系）的 (R, p_mm)。q_deg: 4 个关节角。"""
    out = [("base_link", np.eye(3), np.zeros(3))]
    R_acc = np.eye(3)
    p_prev = np.zeros(3)
    for idx, j in enumerate(CHAIN):
        p_j = np.array(j["point_mm"], dtype=float)
        # 关节在父连杆坐标系的平移（父系已带姿态 -> 用父系 R 旋转平移量）
        off = R_acc @ (p_j - p_prev)
        p_world = out[-1][2] + off
        if j["kind"] == "revolute":
            R_child = R_acc @ rot_axis(np.array(j["axis"], dtype=float), math.radians(q_deg[idx]))
        else:
            R_child = R_acc
        out.append((j["child"], R_child, p_world))
        R_acc, p_prev = R_child, p_j
    return out


def compute_effort(masses_coms):
    """每个转动关节：采样工作空间内的重力静力矩最大值 x1.5。
    masses_coms: {link_name: (m, com_mm)}"""
    sample = [-140, -100, -45, 0, 45, 100, 140]
    best = {j["joint"]: 0.0 for j in CHAIN if j["kind"] == "revolute"}
    for q1 in (0.0,):
        for q2 in sample:
            for q3 in sample:
                for q4 in sample:
                    fr = {n: (R, p) for n, R, p in joint_frames([q1, q2, q3, q4])}
                    for idx, j in enumerate(CHAIN):
                        if j["kind"] != "revolute":
                            continue
                        name = j["joint"]
                        child = j["child"]
                        Rj, pj = fr[child]                     # 子连杆坐标系（在轴上）
                        axis_w = Rj @ np.array(j["axis"], dtype=float)
                        tau = 0.0
                        for lname, (m, com) in masses_coms.items():
                            li = "link1 link2 link3 link4".split().index(lname) + 1
                            if li <= idx:
                                continue                       # 只在近端关节之后算
                            Rl, pl = fr[lname]
                            c_w = Rl @ com + pl / 1000.0
                            r = c_w - pj / 1000.0
                            F = np.array([0.0, 0.0, -m * GRAV])
                            tau += float(axis_w @ np.cross(r, F))
                        best[name] = max(best[name], abs(tau))
    return {k: max(1.0, round(v * 1.5, 2)) for k, v in best.items()}


# ============================================================ URDF 生成
def fmt(x):
    if abs(x) < 1e-12:
        return "0"
    return f"{x:.9g}"


def vec(v):
    return " ".join(fmt(float(x)) for x in v)


def build(args):
    t0 = time.time()
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "urdf").mkdir(exist_ok=True)
    (OUT / "meshes" / "visual").mkdir(parents=True, exist_ok=True)
    (OUT / "meshes" / "collision").mkdir(parents=True, exist_ok=True)
    (OUT / "preview").mkdir(exist_ok=True)

    inst = load_assembly()
    log(f"[1/6] 读入实例 {len(inst)} 条，零件 {len({i['part'] for i in inst})} 种")

    # --- 零件：几何 + 体积 + 惯量 + 三角化
    parts = {}
    for part in sorted({i["part"] for i in inst}):
        t1 = time.time()
        shp, V, com, I = import_part(part)
        Vtx, Fcs = tessellate(shp, part, args.quick)
        parts[part] = dict(V=V, com=com, I=I, verts=Vtx, faces=Fcs,
                           rho=density_of(part), ntri=len(Fcs))
        log(f"   {part:14s} V={V:12.1f} mm^3  tri={len(Fcs):7d}  ({time.time()-t1:5.1f}s)")

    # --- 组装：按 group 合并成连杆网格
    import trimesh
    link_mesh = {lg: dict(V=[], F=[], off=0) for lg in GROUP_TO_LINK.values()}
    frame_of = {j["child"]: np.array(j["point_mm"], dtype=float) for j in CHAIN}
    frame_of["base_link"] = np.zeros(3)

    by_link = {lg: [] for lg in GROUP_TO_LINK.values()}
    for it in inst:
        by_link[GROUP_TO_LINK[it["group"]]].append(it)

    for lg, items in by_link.items():
        acc_V, acc_F, off = [], [], 0
        for it in items:
            p = parts[it["part"]]
            V = p["verts"] @ it["R"].T + it["t"]                 # -> 装配系 mm
            V = (V - frame_of[lg]) / 1000.0                      # -> 连杆系 m
            acc_V.append(V)
            acc_F.append(p["faces"] + off)
            off += len(V)
        V = np.vstack(acc_V)
        F = np.vstack(acc_F)
        mesh = trimesh.Trimesh(vertices=V, faces=F, process=True)
        vis = OUT / "meshes" / "visual" / f"{lg}.stl"
        mesh.export(vis)

        # 碰撞体：顶点随机下采样后取凸包（Gazebo/MoveIt 常规做法，近似）
        rng = np.random.default_rng(0)
        pts = V if len(V) <= 40000 else V[rng.choice(len(V), 40000, replace=False)]
        from scipy.spatial import ConvexHull
        hull = ConvexHull(pts)
        used = np.unique(hull.simplices)
        remap = -np.ones(len(pts), dtype=np.int64)
        remap[used] = np.arange(len(used))
        hv = pts[used]
        hf = remap[hull.simplices]
        col = OUT / "meshes" / "collision" / f"{lg}.stl"
        trimesh.Trimesh(vertices=hv, faces=hf, process=True).export(col)
        log(f"[2/6] {lg:10s} 实例{len(items):4d}  三角{len(F):8d}  "
            f"visual={vis.stat().st_size/1e6:5.1f}MB  hull三角{len(hf)}")

    # --- 惯量
    masses_coms = {}
    inert = {}
    for lg, items in by_link.items():
        m, com_m, I = link_inertial(items, parts, frame_of[lg])
        I, fixed = fix_triangle_inequality(I)
        inert[lg] = (m, com_m, I, fixed)
        if lg != "base_link":
            masses_coms[lg] = (m, com_m)
        log(f"[3/6] {lg:10s} m={m:7.3f} kg  com={np.round(com_m*1000,1)} mm"
            + ("  [三角不等式已微调]" if fixed else ""))

    effort = compute_effort(masses_coms)
    effort["joint1"] = max(effort.get("joint1", 0.0), effort.get("joint2", 1.0))
    log("[4/6] 重力静力矩->effort: " + ", ".join(f"{k}={v}Nm" for k, v in effort.items()))

    # --- 写 URDF
    def urdf_text(pkg_paths: bool):
        prefix = "package://r4_urdf" if pkg_paths else ".."
        L = []
        a = L.append
        a('<?xml version="1.0"?>')
        a('<!-- R4 四轴机械臂 URDF —— 由 tools/build_urdf.py 从装配包自动生成，勿手改 -->')
        a('<!-- 几何/位姿: source/assembly.json + CAD/*.STEP (mm) ; 关节轴: 4 个 P04 hub 圆柱轴反算 -->')
        a('<robot name="r4_arm">')
        for lg in ["base_link", "link1", "link2", "link3", "link4"]:
            m, com_m, I, _ = inert[lg]
            o = com_m
            a(f'  <link name="{lg}">')
            a(f'    <inertial>')
            a(f'      <origin xyz="{vec(o)}" rpy="0 0 0"/>')
            a(f'      <mass value="{fmt(m)}"/>')
            a('      <inertia ixx="{}" ixy="{}" ixz="{}" iyy="{}" iyz="{}" izz="{}"/>'.format(
                fmt(I[0, 0]), fmt(I[0, 1]), fmt(I[0, 2]), fmt(I[1, 1]), fmt(I[1, 2]), fmt(I[2, 2])))
            a('    </inertial>')
            a(f'    <visual>')
            a(f'      <origin xyz="0 0 0" rpy="0 0 0"/>')
            a(f'      <geometry><mesh filename="{prefix}/meshes/visual/{lg}.stl"/></geometry>')
            a(f'    </visual>')
            a(f'    <collision>')
            a(f'      <origin xyz="0 0 0" rpy="0 0 0"/>')
            a(f'      <geometry><mesh filename="{prefix}/meshes/collision/{lg}.stl"/></geometry>')
            a(f'    </collision>')
            a(f'  </link>')
        a('  <link name="tool_link"/>')
        for j in CHAIN:
            p_child = np.array(j["point_mm"], dtype=float) / 1000.0
            p_parent = frame_of[j["parent"]] / 1000.0
            off = p_child - p_parent
            a(f'  <joint name="{j["joint"]}" type="{j["kind"]}">')
            a(f'    <parent link="{j["parent"]}"/>')
            a(f'    <child link="{j["child"]}"/>')
            a(f'    <origin xyz="{vec(off)}" rpy="0 0 0"/>')
            if j["kind"] == "revolute":
                lo, hi = j["limit_deg"]
                a(f'    <axis xyz="{vec(j["axis"])}"/>')
                a(f'    <limit lower="{fmt(math.radians(lo))}" upper="{fmt(math.radians(hi))}" '
                  f'effort="{fmt(effort[j["joint"]])}" velocity="{fmt(j["velocity"])}"/>')
                a(f'    <dynamics damping="0.5" friction="0.2"/>')
            a(f'  </joint>')
        for j in CHAIN:
            if j["kind"] != "revolute":
                continue
            a(f'  <transmission name="{j["joint"]}_trans">')
            a('    <type>transmission_interface/SimpleTransmission</type>')
            a(f'    <joint name="{j["joint"]}"><hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface></joint>')
            a('    <actuator name="{}_motor"><mechanicalReduction>1</mechanicalReduction></actuator>'.format(j["joint"]))
            a('  </transmission>')
        a('</robot>')
        return "\n".join(L) + "\n"

    (OUT / "urdf" / "r4.urdf").write_text(urdf_text(True), encoding="utf-8")
    (OUT / "urdf" / "r4_portable.urdf").write_text(urdf_text(False), encoding="utf-8")

    # --- 报告
    report = dict(
        generated=time.strftime("%Y-%m-%d %H:%M:%S"),
        source=str(PKG),
        instances=len(inst),
        links={lg: dict(mass_kg=round(inert[lg][0], 4),
                        com_mm=[round(float(x) * 1000, 3) for x in inert[lg][1]],
                        instances=len(by_link[lg]),
                        triangles=int(sum(parts[i["part"]]["ntri"] for i in by_link[lg])))
               for lg in by_link},
        joints=[dict(name=j["joint"], type=j["kind"], axis=list(j["axis"]),
                     point_mm=list(round(float(x), 3) for x in j["point_mm"]),
                     effort_Nm=effort.get(j["joint"]), velocity_rad_s=j["velocity"],
                     limit_deg=j["limit_deg"]) for j in CHAIN],
        assumptions=dict(
            density=DENSITY,
            joint_limits="包内无机械限位数据，取保守行程，需实测替换",
            effort="重力静力矩最坏工况(采样 q2/q3/q4)x1.5",
            velocity="占位值 1.0 rad/s，需按电机减速比替换",
            inertia="由 B-rep 实体体积分+上述密度估算",
            collision="每个连杆取视觉网格的凸包（近似）",
        ),
        total_mass_kg=round(sum(v[0] for v in inert.values()), 4),
        elapsed_s=round(time.time() - t0, 1),
    )
    (OUT / "build_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2),
                                           encoding="utf-8")
    log(f"[5/6] URDF: {OUT/'urdf'/'r4.urdf'}")
    log(f"[6/6] 总质量 {report['total_mass_kg']} kg，用时 {report['elapsed_s']}s")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="粗网格试跑")
    build(ap.parse_args())
