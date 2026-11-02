# -*- coding: utf-8 -*-
"""用打标记的正弦数据算频响。

标记文件的时刻是发送端自己的时钟，与记录端有未知偏移，所以在 0–4 s 内搜索
使"各段实测频率与标记频率最吻合"的偏移，再按段算幅值比与相位滞后。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CPD = 16384 * 28 / 360.0


def load(csv):
    a = np.genfromtxt(csv, delimiter=",", names=True, dtype=None, encoding="utf-8")
    t = a["t_s"] - a["t_s"][0]
    return t, a["pos"] / CPD, a["target"] / CPD


def freq_of(t, y):
    z = np.where(np.diff(np.sign(y - y.mean())) > 0)[0]
    if len(z) < 3:
        return None
    dt = t[z[-1]] - t[z[0]]
    return (len(z) - 1) / dt if dt > 0 else None


def best_offset(t, tgt, marks):
    best, best_score = 0.0, 1e9
    for off in np.arange(0.0, 4.0, 0.005):
        score, n = 0.0, 0
        for t0, t1, f in marks:
            m = (t >= t0 + off) & (t <= t1 + off)
            if m.sum() < 200:
                continue
            fm = freq_of(t[m], tgt[m])
            if fm is None:
                score += 10.0
                continue
            score += abs(fm - f) / max(f, 0.1)
            n += 1
        if n:
            score /= n
            if score < best_score:
                best_score, best = score, off
    return best, best_score


def main():
    csv = Path(sys.argv[1])
    marks = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
    t, pos, tgt = load(csv)
    off, score = best_offset(t, tgt, marks)
    print(f"记录 {len(t)} 帧  {1/np.median(np.diff(t)):.0f} Hz   "
          f"标记对齐偏移 {off*1000:.0f} ms（拟合残差 {score:.3f}）\n")
    print(f"{'频段':>6s}{'实测f':>8s}{'幅值比':>9s}{'相位滞后':>10s}")
    for t0, t1, f in marks:
        m = (t >= t0 + off) & (t <= t1 + off)
        if m.sum() < 200:
            continue
        tt = t[m]
        fm = freq_of(tt, tgt[m]) or f
        M = np.column_stack([np.cos(2 * np.pi * fm * tt), np.sin(2 * np.pi * fm * tt)])
        out = []
        for y in (tgt[m], pos[m]):
            c, *_ = np.linalg.lstsq(M, y - y.mean(), rcond=None)
            out.append((float(np.hypot(*c)), float(np.degrees(np.arctan2(c[0], c[1])))))
        (ap, pp), (am, pm) = out
        lag = (pp - pm + 180) % 360 - 180
        print(f"{f:6.1f}{fm:8.2f}{am/ap:9.3f}{lag:10.1f}")


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    main()
