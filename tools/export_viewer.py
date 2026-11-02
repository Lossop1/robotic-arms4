# -*- coding: utf-8 -*-
"""
把 URDF 网格 + 控制仿真结果打包成一个不依赖任何后端的网页数据文件。

输出 control/viewer/data.js：
  CHAIN  链参数（关节原点、轴、限位）与低模网格（三角形位置 + 索引）
  DEMOS  三个控制场景的时间序列（q(t)、跟踪误差）
网页 index.html 用 three.js 按 URDF 语义搭 Object3D 层级，直接旋转即可，
所以浏览器里拖动滑块 = 真实 FK。
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
URDF_DIR = ROOT / "data" / "models" / "r4_urdf"
OUT = ROOT / "tools" / "viewer"
sys.path.insert(0, str(ROOT))

from r4ctl import ik as IK  # noqa: E402
from r4ctl.model import chain  # noqa: E402
from r4ctl.servo import Servo  # noqa: E402
from r4ctl.sim import Sim  # noqa: E402
from r4ctl.traj import CartesianMove, JointMove, circle  # noqa: E402

ch = chain()
LINKS = ["base_link", "link1", "link2", "link3", "link4"]
COLORS = ["#98a2ad", "#3d6b47", "#4c7398", "#7d8894", "#c77c2e"]
TARGET_FACES = {"base_link": 3500, "link1": 4500, "link2": 4500, "link3": 4500, "link4": 2500}


def mesh_of(name):
    m = trimesh.load_mesh(str(URDF_DIR / "meshes" / "visual" / f"{name}.stl"), process=True)
    target = TARGET_FACES.get(name, 4000)
    if len(m.faces) > target:
        m = m.simplify_quadric_decimation(face_count=target)
    v = np.round(np.asarray(m.vertices), 6)
    f = np.asarray(m.faces)
    # 法线用面的法线（three 会自己算，这里不需要）
    return dict(pos=[round(float(x), 6) for x in v.ravel()],
                idx=[int(i) for i in f.ravel()],
                n=len(f))


def chain_json():
    joints = []
    for i, j in enumerate(ch.movable):
        joints.append(dict(
            name=j["name"],
            origin=[round(float(x), 6) for x in j["xyz"]],
            axis=[round(float(x), 6) for x in j["axis"]],
            qmin=round(math.degrees(ch.q_min[i]), 2),
            qmax=round(math.degrees(ch.q_max[i]), 2),
        ))
    info = dict(
        links=[dict(name=n, color=COLORS[i], mesh=mesh_of(n)) for i, n in enumerate(LINKS)],
        joints=joints,
        geom=dict(L1=IK.L1, L3=IK.L3, Z_J2=IK.Z_J2,
                  r_min=round(IK.R_MIN, 4), r_max=round(IK.R_MAX, 4)),
        total_mass=round(float(sum(ch.link[i + 1]["mass"] for i in range(ch.n))), 3),
    )
    return info


def pack(log, n=160):
    idx = np.linspace(0, len(log["t"]) - 1, n).astype(int)
    return dict(
        t=[round(float(log["t"][k]), 4) for k in idx],
        q=[[round(math.degrees(float(x)), 3) for x in log["q"][k]] for k in idx],
        qd=[[round(math.degrees(float(x)), 2) for x in log["qd"][k]] for k in idx],
        err_deg=round(float(np.degrees(np.abs(log["q"] - log["q_des"]).max())), 3),
        tip=[[round(float(y) * 1000, 2) for y in log["tip"][k]] for k in idx],
    )


def demos_json():
    out = {}

    # 1 关节 S 曲线
    q0 = np.radians([0.0, -45.0, 40.0, 0.0])
    q1 = np.radians([35.0, -15.0, 5.0, 60.0])
    mv = JointMove(q0, q1)

    def ref1(t):
        if t <= mv.T:
            return mv.sample(t)
        return mv.sample(mv.T)[0], np.zeros(ch.n), np.zeros(ch.n)
    out["joint"] = dict(label="① 关节 S 曲线（含重力+摩擦前馈）",
                        desc=f"时长 {mv.T:.2f}s，四轴时间同步",
                        data=pack(Sim().run(ref1, Servo("pd_g_ff"), T=mv.T + 0.4, q0=q0)))

    # 2 笛卡尔圆
    q0b = np.array([0.0, 0.55, -0.15, 0.0])
    p0 = IK.fk(q0b)
    radius = 0.08
    center = p0[:2] - np.array([radius, 0.0])
    mv2 = CartesianMove(q0b, circle(center, radius, p0[2], roll=0.0,
                                    plane="xy", ang0=0.0, ang1=1.6 * math.pi), n=320)

    def ref2(t):
        if t <= mv2.T:
            return mv2.sample(t)
        return mv2.sample(mv2.T)[0], np.zeros(ch.n), np.zeros(ch.n)
    out["circle"] = dict(label="② 笛卡尔圆弧 r=80mm",
                         desc=f"弧长 {mv2.path_len*1000:.0f}mm，时长 {mv2.T:.2f}s，CTC 控制器",
                         data=pack(Sim().run(ref2, Servo("ctc"), T=mv2.T + 0.3, q0=q0b)))

    # 3 重力补偿 vs 断电
    q0c = np.radians([20.0, -60.0, 55.0, 15.0])

    def hold(t):
        return q0c, np.zeros(ch.n), np.zeros(ch.n)

    def push(t):
        d = np.zeros(ch.n)
        if 0.9 <= t < 1.3:
            d[1] = 25.0
        return d
    out["gravity_off"] = dict(label="③ 断电 τ=0（会掉）", desc="关掉重力补偿",
                              data=pack(Sim().run(hold, Servo("pd", kp=np.zeros(ch.n),
                                                              kd=np.zeros(ch.n)),
                                                  T=1.5, q0=q0c)))
    out["gravity_on"] = dict(label="④ 重力补偿 + 手推 25 N·m", desc="τ=g(q)，0.9–1.3 s 受扰动",
                             data=pack(Sim().run(hold, Servo("pd_g", kp=np.zeros(ch.n),
                                                             kd=np.zeros(ch.n)),
                                                 T=1.5, q0=q0c, disturbance=push)))
    return out


def main():
    OUT.mkdir(exist_ok=True)
    info = chain_json()
    demos = demos_json()
    js = ("// 自动生成：control/tools/export_viewer.py\n"
          "const CHAIN = " + json.dumps(info, separators=(",", ":")) + ";\n"
          "const DEMOS = " + json.dumps(demos, ensure_ascii=False, separators=(",", ":")) + ";\n")
    p = OUT / "data.js"
    p.write_text(js, encoding="utf-8")
    tris = sum(l["mesh"]["n"] for l in info["links"])
    print(f"data.js {p.stat().st_size/1024:.0f} KB   网格三角 {tris}   演示 {len(demos)} 段")


if __name__ == "__main__":
    main()
