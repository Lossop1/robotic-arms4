# -*- coding: utf-8 -*-
"""
4-DOF 闭式逆运动学。

构型（链1坐标系里，与 URDF 完全一致的推导见 README 附 A）：
    J1 腰转  q1  绕世界 Z
    J2 肩    q2  绕 +Y，臂杆长度 L1 = |J3-J2| 面内分量 = 260 mm
    J3 肘    q3  绕 +Y
    J4 腕滚  q4  绕前臂轴（+X），工具点在 J3 之后 L3 = 304 mm
臂面内（u = 半径，z 向上，原点在 J2 轴线）：

    u = L1·sin(q2) + L3·cos(q2+q3)
    z = L1·cos(q2) - L3·sin(q2+q3)      (相对 J2 轴点)

r² = L1² + L3² - 2·L1·L3·sin(q3)：q3 = ±90° 时达到工作空间边界（r = 44 / 564 mm），
这就是两个奇异点。任务空间是 4 维（位置 3 + 工具自转 1），与关节维数相同，
所以 IK 闭式、无冗余，解由「肘前/肘后」×「腰转 0°/180°」四支候选组成，
按限位 + 最近原则挑选，不需要数值迭代。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from . import config as C
from .model import chain

L1 = 0.260      # J2 -> J3（面内）
L3 = 0.304      # J3 -> 工具点（面内，含法兰 64 mm）
Z_J2 = 0.270    # J2 轴点高度
R_MIN = abs(L3 - L1)
R_MAX = L1 + L3
C_MARGIN = C.SAFETY["q_margin_deg"]


@dataclass
class IKResult:
    q: np.ndarray
    ok: bool
    reason: str = ""
    branch: int = 1          # +1 肘前（q3=asin）, -1 肘后（q3=pi-asin）
    yaw_flip: bool = False   # True 表示用了 q1+180° 的镜像解
    r: float = 0.0           # 工具点到 J2 轴的水平距离
    sin_q3: float = 0.0
    candidates: int = 0      # 候选解个数（可达性诊断）


def _planar(du, dz, elbow):
    """面内 2R，返回 (q2, q3, r, sin_q3)；不可达返回 None。"""
    r2 = du * du + dz * dz
    r = math.sqrt(r2)
    s = (L1 * L1 + L3 * L3 - r2) / (2.0 * L1 * L3)     # = sin(q3)
    if s > 1.0 or s < -1.0:
        return None
    s = max(-1.0, min(1.0, s))
    q3 = math.asin(s) if elbow > 0 else math.pi - math.asin(s)
    c3 = math.cos(q3)
    phi = math.atan2(dz, du)
    A1 = phi - math.atan2(-L3 * c3, L1 - L3 * s)        # 上臂方向角
    q2 = math.pi / 2.0 - A1
    return q2, q3, r, s


def wrap_near(q, q_ref, period=2 * math.pi):
    """把 q 折到离 q_ref 最近的等价角上，避开 ±180° 跳变。"""
    q = np.asarray(q, float).copy()
    if q_ref is None:
        return q
    return q + period * np.round((np.asarray(q_ref, float) - q) / period)


def ik(p, roll=0.0, elbow="auto", q_ref=None, use_limits=True, margin_deg=None):
    """位置 p (m, 世界系) + 工具自转角 roll -> 关节角 q (4,)。

    elbow: +1 肘前 | -1 肘后 | "auto"
    q_ref: 参考关节角，用于在等价解中挑最近的一支（连续运动必须给）
    """
    ch = chain()
    p = np.asarray(p, float)
    x, y, z = p
    q1_0 = math.atan2(y, x)
    u0 = math.hypot(x, y)
    dz = z - Z_J2
    if u0 < 1e-9:
        dz = z - Z_J2
    margin = C_MARGIN if margin_deg is None else margin_deg
    lo = ch.q_min + math.radians(margin)
    hi = ch.q_max - math.radians(margin)

    cands = []
    yaws = [(q1_0, u0)]
    if use_limits:
        yaws.append((q1_0 + math.pi, -u0))     # 反身解（腰转 +180°）
    for q1, u in yaws:
        branches = (1, -1) if elbow == "auto" else (elbow,)
        for e in branches:
            pl = _planar(u, dz, e)
            if pl is None:
                continue
            q2, q3, r, s = pl
            q = np.array([q1, q2, q3, roll], float)
            q = wrap_near(q, q_ref)
            cands.append(dict(q=q, e=e, flip=(u < 0), r=r, s=s))

    if not cands:
        r = math.hypot(u0, dz)
        return IKResult(np.zeros(4), False,
                        f"不可达：工具点到 J2 轴距离 r={r*1000:.1f} mm，"
                        f"可达 {R_MIN*1000:.0f}..{R_MAX*1000:.0f} mm", r=r)

    def in_lim(q):
        return bool(np.all(q >= lo) and np.all(q <= hi))

    def dist(c):
        return 0.0 if q_ref is None else float(np.abs(c["q"] - np.asarray(q_ref)).sum())

    valid = [c for c in cands if in_lim(c["q"])]
    if valid:
        valid.sort(key=lambda c: (dist(c), 0 if c["e"] > 0 else 1))
        c = valid[0]
        return IKResult(c["q"], True, "", c["e"], c["flip"], c["r"], c["s"], len(cands))

    # 全部超限：给出最接近限位的一支，标记失败
    cands.sort(key=lambda c: float(np.sum(np.maximum(0, lo - c["q"]) +
                                         np.maximum(0, c["q"] - hi))))
    c = cands[0]
    bad = [(ch.movable[i]["name"], round(float(np.degrees(c["q"][i])), 1)) for i in range(ch.n)
           if not (lo[i] <= c["q"][i] <= hi[i])]
    return IKResult(c["q"], False, f"关节超限: {bad}", c["e"], c["flip"], c["r"], c["s"], len(cands))


def fk(q):
    return chain().tool_point(q)


def manipulability(q):
    """|det(J J^T)|^0.5（数值）"""
    J = chain().jacobian(q)
    return math.sqrt(max(0.0, float(np.linalg.det(J @ J.T))))


def singular_margin(q):
    """0 = 条件良好，1 = 奇异。平面 2R 的奇异在 q3 = ±90°（|cos q3| -> 0）。"""
    return float(max(0.0, 1.0 - abs(math.cos(q[2]))))


def workspace_samples(n2=41, n3=41, n1=12):
    """工作空间采样（工具点，世界系），用于画图与可达性统计。"""
    ch = chain()
    pts = []
    for q1 in np.linspace(ch.q_min[0], ch.q_max[0], n1):
        for q2 in np.linspace(ch.q_min[1], ch.q_max[1], n2):
            for q3 in np.linspace(ch.q_min[2], ch.q_max[2], n3):
                pts.append(fk([q1, q2, q3, 0.0]))
    return np.array(pts)
