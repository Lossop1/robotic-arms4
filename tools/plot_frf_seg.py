# -*- coding: utf-8 -*-
"""把 1 Hz 那段的 target 与 pos 画出来，定位幅值比 >1 的原因（不动电机）。"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["axes.grid"] = True
plt.rcParams["grid.alpha"] = 0.3

ROOT = Path(__file__).resolve().parents[1]
CPD = 16384 * 28 / 360.0
CSV = ROOT / "data" / "bench" / "obs_sine.csv"
MARKS = ROOT / "data" / "bench" / "sine_marks.json"
OFF = 0.275


def main():
    a = np.genfromtxt(CSV, delimiter=",", names=True, dtype=None, encoding="utf-8")
    t = a["t_s"] - a["t_s"][0]
    pos = a["pos"] / CPD
    tgt = a["target"] / CPD
    marks = json.loads(MARKS.read_text(encoding="utf-8"))

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=False)
    for k, (t0, t1, f) in enumerate(marks):
        m = (t >= t0 + OFF) & (t <= t1 + OFF)
        if m.sum() < 100:
            continue
        tt = t[m] - t[m][0]
        amp_t = (tgt[m].max() - tgt[m].min()) / 2
        amp_p = (pos[m].max() - pos[m].min()) / 2
        print(f"{f:4.1f} Hz  目标半幅 {amp_t:.3f}°  实际半幅 {amp_p:.3f}°  "
              f"比值 {amp_p/amp_t:.3f}")
        if f in (1.0, 4.0, 8.0):
            ax = axes[(0, 1, 2)[(1.0, 4.0, 8.0).index(f)]]
            ax.plot(tt, tgt[m] - tgt[m].mean(), lw=1.0, label="命令")
            ax.plot(tt, pos[m] - tgt[m].mean(), lw=1.0, alpha=0.85, label="实际")
            ax.set_title(f"{f} Hz 段：目标半幅 {amp_t:.3f}° / 实际半幅 {amp_p:.3f}°")
            ax.set_xlabel("段内时间 (s)")
            ax.set_ylabel("角度 (°)")
            ax.legend()

    out = ROOT / "data" / "bench" / "frf_time_domain.png"
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print("图 ->", out)


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    main()
