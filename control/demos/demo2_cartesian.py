# -*- coding: utf-8 -*-
"""
Demo 2：笛卡尔空间运动 —— 直线与圆弧。

流程就是真实控制器里的那条链：
  笛卡尔路径 → 逐点闭式逆解 → 按 dq/ds 折算标量速度/加速度/加加速度约束 → S 曲线时间律
  → 1 kHz 伺服 → 力矩
验收看"末端实际轨迹"而不是关节给定：把仿真出来的 q 做 FK，量它偏离理想路径多少。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import COLORS, save  # noqa: E402

import matplotlib.pyplot as plt  # noqa: E402

from r4ctl import ik as IK  # noqa: E402
from r4ctl.model import chain  # noqa: E402
from r4ctl.servo import Servo  # noqa: E402
from r4ctl.sim import Sim  # noqa: E402
from r4ctl.traj import CartesianMove, circle, line  # noqa: E402

ch = chain()
Q0 = np.array([0.0, 0.55, -0.15, 0.0])        # 一个可达的起手位姿
P0 = IK.fk(Q0)


def run_path(name, mv, t_extra=0.4, mode="ctc"):
    def ref(t):
        if t <= mv.T:
            return mv.sample(t)
        return mv.sample(mv.T)[0], np.zeros(ch.n), np.zeros(ch.n)

    log = Sim().run(ref, Servo(mode), T=mv.T + t_extra, q0=Q0)
    t = log["t"]
    s = np.array([mv.profile.sample(min(tt, mv.T))[0] for tt in t])
    # 用连续路径求理想点（不要用离散采样点，否则量化误差本身就有半个步长）
    ideal = np.array([mv.path_fn(min(max(si, 0.0), 1.0))[0] for si in s])
    dev = np.linalg.norm(log["tip"] - ideal, axis=1) * 1000     # mm
    in_move = t <= mv.T
    return dict(name=name, T=mv.T, log=log, dev=dev, ideal=ideal, s=s,
                max_dev_mm=float(dev[in_move].max()),
                rms_dev_mm=float(np.sqrt((dev[in_move] ** 2).mean())),
                length_mm=mv.path_len * 1000,
                vmax_mm_s=float(mv.profile.sample(mv.T / 2)[1] * mv.path_len * 1000))


def main():
    print("== Demo 2: 笛卡尔直线 / 圆弧 ==")
    p1 = P0 + np.array([0.10, 0.06, -0.05])
    mv_line = CartesianMove(Q0, line(P0, p1, 0.0, 0.0), n=200, kind="scurve")
    r1 = run_path("直线", mv_line)
    print(f"  直线: 长度 {r1['length_mm']:.0f} mm  时长 {r1['T']:.3f}s  "
          f"路径速度峰值 {r1['vmax_mm_s']:.0f} mm/s")
    print(f"        末端偏离理想直线 最大 {r1['max_dev_mm']*1000:.1f} um  "
          f"RMS {r1['rms_dev_mm']*1000:.1f} um")

    # 圆弧必须从当前末端点出发：圆心放在 P0 侧向 80 mm 处，起点角 = 0
    radius = 0.08
    center_xy = P0[:2] - np.array([radius, 0.0])
    mv_arc = CartesianMove(Q0, circle(center_xy, radius, P0[2], roll=0.0,
                                      plane="xy", ang0=0.0, ang1=math.pi), n=240)
    r2 = run_path("圆弧", mv_arc)
    print(f"  圆弧: 弧长 {r2['length_mm']:.0f} mm  时长 {r2['T']:.3f}s")
    print(f"        末端偏离理想圆 最大 {r2['max_dev_mm']*1000:.1f} um  "
          f"RMS {r2['rms_dev_mm']*1000:.1f} um")

    r1p = run_path("直线", mv_line, mode="pd_g_ff")
    print(f"  （对照）同一路径用 pd_g_ff：最大 {r1p['max_dev_mm']*1000:.1f} um  "
          f"RMS {r1p['rms_dev_mm']*1000:.1f} um")

    # ---------------- 图
    fig, axes = plt.subplots(2, 2, figsize=(10.5, 6.8))

    ax = axes[0][0]
    for i, r in enumerate((r1, r2)):
        ax.plot(r["ideal"][:, 0] * 1000, r["ideal"][:, 1] * 1000, "--", lw=1.0,
                color=COLORS[i], label=f"{r['name']} 理想")
        ax.plot(r["log"]["tip"][:, 0] * 1000, r["log"]["tip"][:, 1] * 1000, lw=1.6,
                color=COLORS[i], alpha=0.75, label=f"{r['name']} 实际")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title("A 末端轨迹（俯视 X-Y）")
    ax.legend(fontsize=7)
    ax.axis("equal")

    ax = axes[0][1]
    for i, r in enumerate((r1, r2)):
        ax.semilogy(r["log"]["t"], np.maximum(r["dev"], 1e-4), color=COLORS[i], lw=1.2, label=r["name"])
    ax.semilogy(r1p["log"]["t"], np.maximum(r1p["dev"], 1e-4), "--", color="0.45", lw=1.0,
                label="直线 (pd_g_ff)")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("偏离理想路径 (mm, log)")
    ax.set_title("B 路径跟踪偏差（实机 = 机械精度 + 伺服误差）")
    ax.legend()

    ax = axes[1][0]
    for i in range(ch.n):
        ax.plot(r1["log"]["t"], np.degrees(r1["log"]["qd"][:, i]), color=COLORS[i], lw=1.1,
                label=f"J{i+1}")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("关节速度 (deg/s)")
    ax.set_title("C 直线运动时的关节速度（时间同步：同时到、同时停）")
    ax.legend(ncol=4)

    ax = axes[1][1]
    for i, r in enumerate((r1, r2)):
        e = np.degrees(np.abs(r["log"]["q"] - r["log"]["q_des"]).max(axis=1))
        ax.semilogy(r["log"]["t"], np.maximum(e, 1e-5), color=COLORS[i], lw=1.1, label=r["name"])
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("最大关节跟踪误差 (deg, log)")
    ax.set_title("D 关节误差（复用同一套伺服环，不额外调参）")
    ax.legend()

    save(fig, "demo2_cartesian.png")
    return dict(line=r1, arc=r2, line_pd=r1p)


if __name__ == "__main__":
    main()
