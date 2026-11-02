# -*- coding: utf-8 -*-
"""
轨迹生成。

分两层：
1) 标量时间律 s(t)：梯形速度 或 加加速度受限 S 曲线。Profile 由「分段恒定加加速度 +
   每段起点的加速度跳变」组成，解析积分（每段闭式），所以采样是精确的，
   梯形速度的加速度阶跃也能如实表达。
2) 用 s(t) 驱动关节空间（q = q0 + Δq·s）或笛卡尔路径（按弧长参数化后逐点逆解）。

多轴同步是天然的：先把各关节约束折算成标量约束
    ṡ_max = min_i v_max_i/|Δq_i|,  s̈_max = min_i a_max_i/|Δq_i|,  s⃛_max = min_i j_max_i/|Δq_i|
再用这组标量约束造一条 s(t)，所有关节自动同时启停、且都不越限。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import config as C
from . import ik as IK
from .model import chain


# ============================================================ 标量时间律
@dataclass
class Segment:
    dt: float
    j: float = 0.0        # 段内恒定加加速度
    da: float = 0.0       # 段起点的加速度跳变


@dataclass
class Profile:
    """s(t)，t ∈ [0, T]，终点 s = D"""
    segments: list = field(default_factory=list)
    D: float = 1.0

    @property
    def T(self):
        return sum(s.dt for s in self.segments)

    def sample(self, t):
        """返回 (s, ṡ, s̈, s⃛)"""
        t = min(max(t, 0.0), self.T)
        s = sd = sdd = 0.0
        left = t
        for seg in self.segments:
            sdd += seg.da
            if left <= seg.dt:
                h = left
                j = seg.j
                s = s + sd * h + 0.5 * sdd * h * h + j * h ** 3 / 6.0
                sd = sd + sdd * h + 0.5 * j * h * h
                sdd = sdd + j * h
                return s, sd, sdd, j
            h = seg.dt
            j = seg.j
            s = s + sd * h + 0.5 * sdd * h * h + j * h ** 3 / 6.0
            sd = sd + sdd * h + 0.5 * j * h * h
            sdd = sdd + j * h
            left -= h
        return s, sd, sdd, 0.0

    def grid(self, n=2000):
        ts = np.linspace(0, self.T, n)
        return ts, np.array([self.sample(t) for t in ts])

    def distance(self):
        return self.sample(self.T)[0]


def _zero_profile():
    return Profile([Segment(1e-9, 0.0, 0.0)], D=0.0)


def make_trapezoid(D, vmax, amax):
    """梯形速度：加速-匀速-减速（加速度是阶跃，所以 jerk 无穷大）。"""
    D = abs(D)
    if D < 1e-12:
        return _zero_profile()
    vmax = max(vmax, 1e-9)
    amax = max(amax, 1e-9)
    ta = vmax / amax
    if vmax * ta > D:                       # 到不了 vmax：三角形速度
        vp = math.sqrt(D * amax)
        ta = vp / amax
        tc = 0.0
    else:
        tc = (D - vmax * ta) / vmax
    segs = [Segment(ta, 0.0, +amax)]
    if tc > 0:
        segs.append(Segment(tc, 0.0, -amax))
        segs.append(Segment(ta, 0.0, -amax))
    else:
        segs.append(Segment(ta, 0.0, -2.0 * amax))   # 加速直接接减速
    return Profile(segs, D)


def _accel_segments(vp, ap, jmax):
    """0 -> vp 的加速段（jerk 上、恒 a、jerk 下）"""
    tj = ap / jmax
    tp = vp / ap - tj                       # 恒加速度段时长（>=0）
    segs = [Segment(tj, jmax)]
    if tp > 1e-12:
        segs.append(Segment(tp, 0.0))
    segs.append(Segment(tj, -jmax))
    return segs


def _mirror(segs):
    """减速段：加速度取反、时间段顺序不变（时间反转等价于同一时刻对 a 取负）。"""
    return [Segment(s.dt, -s.j) for s in segs]


def make_scurve(D, vmax, amax, jmax):
    """加加速度受限 S 曲线。用「构造候选 + 对峰值速度二分」实现，
    避免七段情形分析写错：距离对峰值速度单调，二分 60 次即收敛到机器精度。"""
    D = abs(D)
    if D < 1e-12:
        return _zero_profile()
    vmax = max(vmax, 1e-9)
    amax = max(amax, 1e-9)
    jmax = max(jmax, 1e-9)

    def build(vp):
        ap = min(amax, math.sqrt(vp * jmax))   # 峰值加速度：要么撞 amax，要么三角形
        ap = max(ap, 1e-12)
        segs = _accel_segments(vp, ap, jmax)
        d_acc = Profile(segs).distance()
        return segs, d_acc

    segs, d_acc = build(vmax)
    if 2 * d_acc <= D:                          # 能到 vmax，中间加匀速段
        tv = (D - 2 * d_acc) / vmax
        full = segs + ([Segment(tv, 0.0)] if tv > 1e-12 else []) + _mirror(segs)
        return Profile(full, D)

    lo, hi = 1e-9, vmax                         # 二分峰值速度
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        _, d = build(mid)
        if 2 * d < D:
            lo = mid
        else:
            hi = mid
    vp = lo
    segs, d_acc = build(vp)
    tv = max(0.0, (D - 2 * d_acc) / max(vp, 1e-12))
    full = segs + ([Segment(tv, 0.0)] if tv > 1e-9 else []) + _mirror(segs)
    return Profile(full, D)


def make_profile(D, vmax, amax, jmax, kind=None):
    kind = kind or C.TRAJ["profile"]
    if kind == "trapezoid":
        return make_trapezoid(D, vmax, amax)
    return make_scurve(D, vmax, amax, jmax)


# ============================================================ 关节空间
class JointMove:
    """q0 -> q1 的时间同步关节运动。"""

    def __init__(self, q0, q1, v_max=None, a_max=None, j_max=None, kind=None):
        ch = chain()
        self.q0 = np.asarray(q0, float)
        self.q1 = np.asarray(q1, float)
        dq = self.q1 - self.q0
        v_max = ch.v_max if v_max is None else np.asarray(v_max, float)
        a_max = ch.a_max if a_max is None else np.asarray(a_max, float)
        j_max = ch.j_max if j_max is None else np.asarray(j_max, float)

        mag = np.abs(dq)
        mag = np.where(mag < 1e-12, 1e-12, mag)
        self.profile = make_profile(1.0, float(np.min(v_max / mag)),
                                    float(np.min(a_max / mag)),
                                    float(np.min(j_max / mag)), kind)
        self.dq = dq

    @property
    def T(self):
        return self.profile.T

    def sample(self, t):
        s, sd, sdd, _ = self.profile.sample(t)
        return self.q0 + self.dq * s, self.dq * sd, self.dq * sdd


# ============================================================ 笛卡尔路径
class CartesianMove:
    """笛卡尔直线 / 圆弧：逐点逆解，再按 dq/ds 的最坏值折算标量约束重新定时间律。"""

    def __init__(self, q0, path_fn, n=200, v_max=None, a_max=None, j_max=None, kind=None):
        ch = chain()
        self.q0 = np.asarray(q0, float)
        ss = np.linspace(0.0, 1.0, n)
        qs, pts = [], []
        q_ref = self.q0
        for s in ss:
            p, roll = path_fn(s)
            res = IK.ik(p, roll, q_ref=q_ref)
            if not res.ok:
                raise ValueError(f"笛卡尔路径在 s={s:.3f} 处不可达/超限: {res.reason}")
            qs.append(res.q)
            pts.append(np.asarray(p, float))
            q_ref = res.q
        self.ss = ss
        self.qs = np.array(qs)
        self.points = np.array(pts)
        self.path_fn = path_fn

        dqds = np.gradient(self.qs, ss, axis=0)
        self.dqds = dqds
        self.d2qds2 = np.gradient(dqds, ss, axis=0)
        v_max = ch.v_max if v_max is None else np.asarray(v_max, float)
        a_max = ch.a_max if a_max is None else np.asarray(a_max, float)
        j_max = ch.j_max if j_max is None else np.asarray(j_max, float)
        mag = np.maximum(np.abs(dqds).max(axis=0), 1e-12)
        self.sd_max = float(np.min(v_max / mag))
        self.sdd_max = float(np.min(a_max / mag))
        self.sddd_max = float(np.min(j_max / mag))
        self.profile = make_profile(1.0, self.sd_max, self.sdd_max, self.sddd_max, kind)
        self.path_len = float(np.sum(np.linalg.norm(np.diff(self.points, axis=0), axis=1)))

    @property
    def T(self):
        return self.profile.T

    def sample(self, t):
        s, sd, sdd, _ = self.profile.sample(t)
        s = min(max(s, 0.0), 1.0)
        q = np.array([np.interp(s, self.ss, self.qs[:, i]) for i in range(4)])
        idx = int(np.clip(np.searchsorted(self.ss, s), 0, len(self.ss) - 1))
        # q̇ = q_s·ṡ,  q̈ = q_ss·ṡ² + q_s·s̈
        qd = self.dqds[idx] * sd
        qdd = self.d2qds2[idx] * sd * sd + self.dqds[idx] * sdd
        return q, qd, qdd

    def deviation(self, q, s):
        p = IK.fk(q)
        idx = int(np.clip(round(s * (len(self.ss) - 1)), 0, len(self.ss) - 1))
        return float(np.linalg.norm(p - self.points[idx]))


def line(p0, p1, roll0=0.0, roll1=None):
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    r1 = roll0 if roll1 is None else roll1

    def fn(s):
        return p0 + (p1 - p0) * s, roll0 + (r1 - roll0) * s
    return fn


def circle(center, radius, z, roll=0.0, plane="xy", ang0=0.0, ang1=math.pi):
    c = np.asarray(center, float)

    def fn(s):
        a = ang0 + (ang1 - ang0) * s
        if plane == "xy":
            return np.array([c[0] + radius * math.cos(a), c[1] + radius * math.sin(a), z]), roll
        return np.array([c[0] + radius * math.cos(a), c[1], z + radius * math.sin(a)]), roll
    return fn
