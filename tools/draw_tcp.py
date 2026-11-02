# -*- coding: utf-8 -*-
"""把机械臂的关节位置与"末端"画出来，用于确认 TCP 到底在哪。

输出 data/models/r4_urdf/tcp_definition.png：三个视角 + 标注 J1..J4、TCP、基座。
"""
from __future__ import annotations

import math
import struct
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "robot"))
sys.path.insert(0, str(ROOT / "tools"))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

from r4ctl.model import Chain  # noqa: E402
from r4ctl.ik import fk  # noqa: E402

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams["axes.unicode_minus"] = False

MESH = ROOT / "data" / "models" / "r4_urdf" / "meshes" / "visual"


def load_stl(p: Path, max_tri=1500):
    with open(p, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        step = max(1, n // max_tri)
        tris = []
        for i in range(n):
            d = f.read(50)
            if i % step:
                continue
            v = np.array([struct.unpack("<3f", d[12 + 12 * k:24 + 12 * k]) for k in range(3)])
            tris.append(v)
    return np.array(tris)          # (m, 3, 3)


def main():
    ch = Chain()
    q = np.zeros(ch.n)
    frames = {}
    # 用 model 的 FK 拿每个连杆的世界位姿
    for i, link in enumerate(["base_link", "link1", "link2", "link3", "link4"]):
        pass
    T = ch.fk_all(q) if hasattr(ch, "fk_all") else None

    # 直接用链上每段的局部变换拼出世界位姿
    world = {}
    R = np.eye(3)
    p = np.zeros(3)
    world["base_link"] = (R.copy(), p.copy())
    for i, seg in enumerate(ch.segments if hasattr(ch, "segments") else []):
        pass

    # 简化：从 URDF 里读 origin/axis，自己串一遍
    import xml.etree.ElementTree as ET
    root = ET.parse(ROOT / "data" / "models" / "r4_urdf" / "urdf" / "r4.urdf").getroot()
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        joints[j.find("child").get("link")] = dict(
            parent=j.find("parent").get("link"),
            xyz=np.array([float(x) for x in o.get("xyz").split()]),
            rpy=np.array([float(x) for x in (o.get("rpy") or "0 0 0").split()]),
            axis=np.array([float(x) for x in j.find("axis").get("xyz").split()]) if j.find("axis") is not None else None,
            name=j.get("name"), type=j.get("type"))

    def rpy_R(r, p_, y):
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p_), math.sin(p_)
        cy, sy = math.cos(y), math.sin(y)
        return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                         [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                         [-sp,   cp*sr,            cp*cr]])

    poses = {"base_link": (np.eye(3), np.zeros(3))}
    order = ["link1", "link2", "link3", "link4", "tool_link"]
    for link in order:
        j = joints[link]
        Rp, pp = poses[j["parent"]]
        Rw = Rp @ rpy_R(*j["rpy"])
        pw = pp + Rp @ j["xyz"]
        poses[link] = (Rw, pw)

    tcp = poses["tool_link"][1]

    fig = plt.figure(figsize=(15, 5.2))
    views = [(18, -60), (18, 20), (60, -90)]
    titles = ["侧视（Y轴看）", "斜视", "俯视（Z轴往下）"]
    for k, ((elev, azim), title) in enumerate(zip(views, titles)):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        chain_pts = [poses["base_link"][1], poses["link1"][1], poses["link2"][1],
                     poses["link3"][1], poses["link4"][1], poses["tool_link"][1]]
        P = np.array(chain_pts) * 1000.0
        ax.plot(P[:, 0], P[:, 1], P[:, 2], "o-", color="#1f4e79", lw=3.2,
                ms=7, markerfacecolor="white", zorder=5)
        for a_, b_ in zip(P[:-1], P[1:]):
            mid = (a_ + b_) / 2
            L = np.linalg.norm(b_ - a_)
            ax.text(mid[0], mid[1], mid[2], f"{L:.0f}", fontsize=7, color="#555")
        # 关节位置
        for link in ["link1", "link2", "link3", "link4"]:
            pj = poses[link][1] * 1000
            ax.scatter(*pj, c="#1f4e79", s=45, depthshade=False)
            ax.text(pj[0], pj[1], pj[2] + 18, link.replace("link", "J"), fontsize=9,
                    color="#1f4e79", weight="bold")
        # TCP
        ax.scatter(*tcp * 1000, c="red", s=130, marker="*", depthshade=False)
        ax.text(tcp[0] * 1000, tcp[1] * 1000, tcp[2] * 1000 + 30,
                f"TCP\n({tcp[0]*1000:+.0f},{tcp[1]*1000:+.0f},{tcp[2]*1000:+.0f})mm",
                color="red", fontsize=9, weight="bold")
        ax.scatter(0, 0, 0, c="k", s=30, marker="s", depthshade=False)
        ax.set_xlim(-200, 400)
        ax.set_ylim(-400, 400)
        ax.set_zlim(0, 750)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title, fontsize=10)

    out = ROOT / "data" / "models" / "r4_urdf" / "tcp_definition.png"
    fig.suptitle("R4：关节位置与末端定义（红★ = URDF 的 tool_link 原点 = J4 输出面）",
                 fontsize=12)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print("图 ->", out)
    print(f"TCP（q=0）= {np.round(tcp*1000,1)} mm")
    for link in ["link1", "link2", "link3", "link4"]:
        print(f"  {link} 原点 = {np.round(poses[link][1]*1000,1)} mm")


if __name__ == "__main__":
    main()
