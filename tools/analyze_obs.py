# -*- coding: utf-8 -*-
"""用 1 kHz 观测数据（含主站每拍 target）算回差与频响。

数据来自 tools/obs_record.py 采的 obs_steps_sine.csv。
切段不靠时间表，靠目标信号形态：
  - 正弦段：目标在原位停留 ≥1 s 的平台把五段分开；
  - 微步段：目标跑出中位数 ±0.8° 的那部分。
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CPD = 16384 * 28 / 360.0
SRC = ROOT / "data" / "bench" / "obs_steps_sine.csv"


def load(p):
    a = np.genfromtxt(p, delimiter=",", names=True, dtype=None, encoding="utf-8")
    t = a["t_s"] - a["t_s"][0]
    return dict(t=t, pos=a["pos"] / CPD, tgt=a["target"] / CPD,
                vel=a["vel"] / CPD, tau=a["torque"])


def split_phases(d):
    """返回 (steps_mask, [sine_segments])。"""
    t, tgt = d["t"], d["tgt"]
    center = float(np.median(tgt[int(len(tgt) * 0.7):]))
    far = np.abs(tgt - center) > 0.8
    if not far.any():
        return np.zeros(len(t), bool), []
    steps_end = int(np.where(far)[0].max()) + 200
    steps = np.zeros(len(t), bool)
    steps[:steps_end] = True

    sine = ~steps
    # 正弦段：把"目标几乎不动 ≥1 s"的平台当分隔
    v = np.gradient(tgt, t)
    still = np.abs(v) < 0.05
    segs, start = [], None
    i = 0
    n = len(t)
    while i < n:
        if not sine[i]:
            i += 1
            continue
        if still[i]:
            i += 1
            continue
        j = i
        while j < n and sine[j] and not still[j]:
            j += 1
        if j - i > 200:                       # 至少 0.2 s 的振荡段
            segs.append((i, j))
        i = j
    return steps, segs


def backlash(d, mask):
    """滞回环：把升支和降支分别按目标排序插值，取公共区间的平均水平差。"""
    pos, tgt = d["pos"][mask], d["tgt"][mask]
    dp = np.diff(tgt, prepend=tgt[0])
    up, dn = dp > 0.01, dp < -0.01
    if up.sum() < 10 or dn.sum() < 10:
        return float("nan"), 0
    xu, yu = tgt[up], pos[up]
    xd, yd = tgt[dn], pos[dn]
    lo = max(xu.min(), xd.min())
    hi = min(xu.max(), xd.max())
    if hi <= lo:
        return float("nan"), 0
    xs = np.linspace(lo, hi, 40)
    gu = np.interp(xs, xu, yu)
    gd = np.interp(xs, xd, yd)
    return float(np.median(np.abs(gu - gd))) * 1000, 40


def frf(d, segs):
    """每段单独估频率、幅值比、相位滞后。"""
    out = []
    for i, (a, b) in enumerate(segs):
        t = d["t"][a:b] - d["t"][a]
        tgt, pos = d["tgt"][a:b], d["pos"][a:b]
        # 用上升过零点估频率
        z = np.where(np.diff(np.sign(tgt - tgt.mean())) > 0)[0]
        if len(z) < 3:
            continue
        f = (len(z) - 1) / (t[z[-1]] - t[z[0]])
        if f < 0.05:
            continue
        # 跳过第一个周期
        m = t > t[z[0]]
        if m.sum() < 100:
            continue
        tt = t[m]
        M = np.column_stack([np.cos(2 * np.pi * f * tt), np.sin(2 * np.pi * f * tt)])
        res = []
        for y in (tgt[m], pos[m]):
            c, *_ = np.linalg.lstsq(M, y - y.mean(), rcond=None)
            res.append((float(np.hypot(*c)), float(np.degrees(np.arctan2(c[0], c[1])))))
        (ap, pp), (am, pm) = res
        if ap < 1e-4:
            continue
        lag = (pp - pm + 180) % 360 - 180
        out.append(dict(seg=i, f_hz=round(f, 2), amp_ratio=am / ap, phase_lag_deg=lag))
    return out


def main():
    d = load(SRC)
    print(f"样本 {len(d['t'])}  时长 {d['t'][-1]:.1f}s  "
          f"采样 {1/np.median(np.diff(d['t'])):.0f} Hz")

    steps, segs = split_phases(d)
    print(f"切段：微步段 {steps.sum()} 帧，正弦段 {len(segs)} 个")

    err = (d["pos"] - d["tgt"]) * 1000
    print(f"跟随误差 RMS {np.sqrt(np.mean(err**2)):.1f} mdeg  "
          f"P95 {np.percentile(np.abs(err), 95):.1f} mdeg")

    bl, n = backlash(d, steps)
    print(f"\n== 回差（滞回环，{n} 个位姿）==\n  {bl:.0f} mdeg = {bl/1000:.4f}°")

    print("\n== 频响（分段、用真实 target）==")
    for p in frf(d, segs):
        print(f"  {p['f_hz']:5.2f} Hz  幅值比 {p['amp_ratio']:.3f}  "
              f"相位滞后 {p['phase_lag_deg']:+7.1f}°")

    if segs:
        i, j = segs[0]
        print(f"\n段首目标幅度 ±{(d['tgt'][i:j].max()-d['tgt'][i:j].min())/2:.3f}°")


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    main()
