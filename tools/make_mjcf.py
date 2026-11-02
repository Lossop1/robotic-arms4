# -*- coding: utf-8 -*-
"""URDF -> MJCF（MuJoCo），供可视化仿真用。

只做这个项目需要的事：
  - 关节一律 hinge（continuous 没有限位）
  - 惯性参数照抄 URDF
  - 视觉用 visual mesh，物理用 collision mesh（contype 默认关接触，避免网格碰撞不稳定）
  - 每轴一个 position 执行器，模拟响应器里的 CSP 位置环

用法: python tools/make_mjcf.py [--kp 500] [--out data/models/r4_urdf/urdf/r4_mjcf.xml]
"""
from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "data" / "models" / "r4_urdf" / "urdf" / "r4.urdf"
MESH_ROOT = URDF.parent.parent      # data/models/r4_urdf


def rpy_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def vec(s, default="0 0 0"):
    if not s:
        return np.array([float(x) for x in default.split()])
    return np.array([float(x) for x in s.split()])


def mesh_file(uri: str) -> Path:
    """package://r4_urdf/meshes/visual/x.stl -> 绝对路径"""
    rel = uri.split("r4_urdf/", 1)[-1]
    return (MESH_ROOT / rel).resolve()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(URDF.with_name("r4_mjcf.xml")))
    ap.add_argument("--kp", type=float, default=800.0, help="位置执行器刚度 N·m/rad")
    ap.add_argument("--kv", type=float, default=20.0, help="阻尼 N·m·s/rad")
    args = ap.parse_args()

    tree = ET.parse(URDF)
    robot = tree.getroot()

    links = {}
    for link in robot.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        mass = float(inertial.find("mass").get("value")) if inertial is not None else 0.0
        ixyz = vec(inertial.find("origin").get("xyz")) if inertial is not None and inertial.find("origin") is not None else np.zeros(3)
        I = np.zeros((3, 3))
        if inertial is not None and inertial.find("inertia") is not None:
            ie = inertial.find("inertia")
            ixx, iyy, izz = (float(ie.get(k)) for k in ("ixx", "iyy", "izz"))
            ixy, ixz, iyz = (float(ie.get(k)) for k in ("ixy", "ixz", "iyz"))
            I = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        visual = link.find("visual")
        mesh = None
        if visual is not None:
            m = visual.find("geometry/mesh")
            if m is not None:
                mesh = mesh_file(m.get("filename"))
        links[name] = dict(mass=mass, ixyz=ixyz, I=I, mesh=mesh)

    joints = []
    for j in robot.findall("joint"):
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        o = j.find("origin")
        xyz = vec(o.get("xyz")) if o is not None else np.zeros(3)
        rpy = vec(o.get("rpy")) if o is not None else np.zeros(3)
        axis = j.find("axis")
        axyz = vec(axis.get("xyz")) if axis is not None else np.array([0.0, 0.0, 1.0])
        joints.append(dict(name=j.get("name"), parent=parent, child=child,
                           type=j.get("type"), xyz=xyz, rpy=rpy, axis=axyz))

    lines = []
    a = lines.append
    a('<mujoco model="r4">')
    a('  <compiler angle="radian" autolimits="true"/>')
    a('  <option gravity="0 0 -9.81" timestep="0.001" integrator="implicitfast"/>')
    a('  <visual><global offwidth="1280" offheight="800"/></visual>')
    a('  <asset>')
    for name, L in links.items():
        if L["mesh"]:
            a(f'    <mesh name="m_{name}" file="{L["mesh"].as_posix()}"/>')
    a('  </asset>')
    a('  <worldbody>')
    a('    <light pos="1 1 2" dir="-0.4 -0.4 -1" diffuse="0.7 0.7 0.7"/>')
    a('    <geom name="floor" type="plane" size="3 3 0.05" pos="0 0 -0.001" rgba=".88 .88 .9 1"/>')

    children = {j["child"]: j for j in joints}
    root_link = next(l for l in links if l not in children)

    def emit_body(name, indent, pos=None, quat=None, joint=None):
        L = links[name]
        pad = "  " * indent
        attrs = f' name="{name}"'
        if pos is not None:
            attrs += f' pos="{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}"'
        if quat is not None:
            attrs += f' quat="{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}"'
        a(f"{pad}<body{attrs}>")
        if joint is not None:
            ax = joint["axis"]
            a(f'{pad}  <joint name="{joint["name"]}" type="hinge" '
              f'axis="{ax[0]:.6f} {ax[1]:.6f} {ax[2]:.6f}"/>')
        if L["mass"] > 0:
            w, v = np.linalg.eigh(L["I"])
            # MJCF fullinertia 需要对角项 + 非对角项
            a(f'{pad}  <inertial pos="{L["ixyz"][0]:.6f} {L["ixyz"][1]:.6f} {L["ixyz"][2]:.6f}" '
              f'mass="{L["mass"]:.6f}" '
              f'fullinertia="{L["I"][0,0]:.6f} {L["I"][1,1]:.6f} {L["I"][2,2]:.6f} '
              f'{L["I"][0,1]:.6f} {L["I"][0,2]:.6f} {L["I"][1,2]:.6f}"/>')
        if L["mesh"]:
            a(f'{pad}  <geom type="mesh" mesh="m_{name}" contype="0" conaffinity="0" '
              f'rgba="0.72 0.75 0.8 1"/>')
        for j in joints:
            if j["parent"] == name:
                q = rpy_to_quat(*j["rpy"])
                emit_body(j["child"], indent + 1, j["xyz"], q,
                          j if j["type"] in ("revolute", "continuous") else None)
        a(f"{pad}</body>")

    emit_body(root_link, 2)

    a('  </worldbody>')
    a('  <actuator>')
    for j in joints:
        if j["type"] in ("revolute", "continuous"):
            a(f'    <position name="act_{j["name"]}" joint="{j["name"]}" '
              f'kp="{args.kp}" kv="{args.kv}" ctrlrange="-3.15 3.15"/>')
    a('  </actuator>')
    a('</mujoco>')

    out = Path(args.out)
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"MJCF -> {out}")

    import mujoco
    m = mujoco.MjModel.from_xml_path(str(out))
    print(f"编译通过: nq={m.nq} nu={m.nu} nbody={m.nbody} 总质量={m.body_mass.sum():.2f} kg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
