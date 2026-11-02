# -*- coding: utf-8 -*-
"""画图与指标公共工具。"""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

OUT = Path(__file__).resolve().parents[3] / "out"
OUT.mkdir(exist_ok=True)

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 130
plt.rcParams["axes.grid"] = True
plt.rcParams["grid.alpha"] = 0.25
plt.rcParams["axes.labelsize"] = 9
plt.rcParams["legend.fontsize"] = 8
plt.rcParams["xtick.labelsize"] = 8
plt.rcParams["ytick.labelsize"] = 8

COLORS = ["#1f4e79", "#c0392b", "#1e8449", "#8e44ad", "#b9770e", "#2c3e50"]


def save(fig, name):
    p = OUT / name
    fig.tight_layout()
    fig.savefig(p)
    plt.close(fig)
    print(f"    图 -> {p.name}")
    return p


def deg(x):
    return np.degrees(x)


def settling_time(t, y, target, band=0.02, t_from=0.0):
    """进入并保持在 ±band*|step| 内的时间"""
    y = np.asarray(y)
    step = abs(target) if abs(target) > 1e-12 else 1.0
    inside = np.abs(y - target) <= band * step
    idx = np.where(t >= t_from)[0]
    if len(idx) == 0:
        return float("nan")
    for k in reversed(idx):
        if not inside[k]:
            if k + 1 >= len(t):
                return float("nan")
            return float(t[k + 1])
    return float(t[idx[0]])


def overshoot(y, target):
    y = np.asarray(y)
    if target >= 0:
        return float(max(0.0, (y.max() - target) / max(target, 1e-12)) * 100)
    return float(max(0.0, (target - y.min()) / max(abs(target), 1e-12)) * 100)


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))
