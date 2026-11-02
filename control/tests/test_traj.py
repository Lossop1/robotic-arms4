# -*- coding: utf-8 -*-
"""轨迹自检：三种时间律的边界、端点、以及关节空间同步性是否满足全部约束。"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from r4ctl import traj as TR  # noqa: E402
from r4ctl.model import chain  # noqa: E402

ch = chain()
rng = np.random.default_rng(3)

print("== 标量时间律边界检查 ==")
for kind in ("trapezoid", "scurve"):
    worst = 0.0
    bad = 0
    for _ in range(400):
        D = rng.uniform(0.05, 2.0)
        v = rng.uniform(0.2, 4.0)
        a = rng.uniform(0.5, 20.0)
        j = rng.uniform(2.0, 200.0)
        p = TR.make_profile(D, v, a, j, kind)
        ts, vals = p.grid(4000)
        s, sd, sdd, sddd = vals.T
        err = max(abs(sd).max() - v, abs(sdd).max() - a,
                  (abs(sddd).max() - j) if kind == "scurve" else -1.0)
        worst = max(worst, err)
        if s[0] > 1e-9 or abs(s[-1] - D) > 1e-6 * max(1.0, D):
            bad += 1
    print(f"  {kind:10s} 最大越限 {worst:+.3e}（<=0 为合格），端点错误 {bad}")

print("\n== 关节运动同步性（随机 200 组目标）==")
worst_v = worst_a = 0.0
viol = 0
for _ in range(200):
    q0 = rng.uniform(ch.q_min * 0.8, ch.q_max * 0.8)
    q1 = rng.uniform(ch.q_min * 0.8, ch.q_max * 0.8)
    mv = TR.JointMove(q0, q1)
    ts = np.linspace(0, mv.T, 3000)
    Q, QD, QDD = [], [], []
    for t in ts:
        q, qd, qdd = mv.sample(t)
        Q.append(q); QD.append(qd); QDD.append(qdd)
    Q, QD, QDD = np.array(Q), np.array(QD), np.array(QDD)
    worst_v = max(worst_v, float((np.abs(QD) / ch.v_max).max()))
    worst_a = max(worst_a, float((np.abs(QDD) / ch.a_max).max()))
    if (np.abs(QD) > ch.v_max * 1.001).any() or (np.abs(QDD) > ch.a_max * 1.01).any():
        viol += 1
print(f"  速度占用峰值 {worst_v*100:.1f}% (<=100%)，加速度占用峰值 {worst_a*100:.1f}%，越限次数 {viol}")

print("\n== 笛卡尔直线 ==")
q0 = np.array([0.0, 0.6, -0.2, 0.0])
p0 = TR.IK.fk(q0)
p1 = p0 + np.array([0.08, 0.03, -0.06])
mv = TR.CartesianMove(q0, TR.line(p0, p1, 0.0, 0.5), n=200)
print(f"  时长 {mv.T:.3f}s，标量速度上限 {mv.sd_max:.3f}/s")
maxdev = 0.0
for t in np.linspace(0, mv.T, 400):
    q, qd, _ = mv.sample(t)
    s, _, _, _ = mv.profile.sample(t)
    p = TR.IK.fk(q)
    p_ref = p0 + (p1 - p0) * s
    maxdev = max(maxdev, float(np.linalg.norm(p - p_ref)))
print(f"  路径跟踪最大偏差(纯几何插值) {maxdev*1e6:.2f} µm")
print(f"  末端起点 {np.round(TR.IK.fk(mv.qs[0])*1000,2)} mm，终点 {np.round(TR.IK.fk(mv.qs[-1])*1000,2)} mm")

print("\n== 笛卡尔圆弧 ==")
fn = TR.circle((0.30, 0.0), 0.06, 0.35, roll=0.3, plane="xy", ang0=0.0, ang1=math.pi)
mv = TR.CartesianMove(np.array([0.0, 0.55, -0.15, 0.3]), fn, n=200)
print(f"  时长 {mv.T:.3f}s  首点 {np.round(mv.points[0]*1000,1)}  末点 {np.round(mv.points[-1]*1000,1)}")
