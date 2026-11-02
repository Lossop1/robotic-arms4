# -*- coding: utf-8 -*-
"""拉回台架 A/B 两次采集，算跟随误差并作图。

用法: python tools/analyze_ab.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from opssh import connect  # noqa: E402

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["axes.grid"] = True
plt.rcParams["grid.alpha"] = 0.3

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "data" / "bench"
OUT.mkdir(parents=True, exist_ok=True)
CPD = 16384 * 28 / 360.0


def fetch(remote: str, local: Path):
    c = connect()
    try:
        sftp = c.open_sftp()
        sftp.get(remote, str(local))
        sftp.close()
    finally:
        c.close()
    return local


def load(p: Path):
    a = np.genfromtxt(p, delimiter=",", names=True, dtype=None, encoding="utf-8")
    d = {k: np.asarray(a[k], dtype=float) for k in a.dtype.names}
    t = d["t_s"] - d["t_s"][0]
    pos = (d["pos"] - d["pos"][0]) / CPD          # 度，相对起点
    plan = (d["planned"] - d["pos"][0]) / CPD
    err = pos - plan                               # 度
    return dict(t=t, pos=pos, plan=plan, err=err, tau=d["torque"])


def stats(d, name):
    t, err = d["t"], d["err"]
    moving = np.abs(np.gradient(d["plan"], t)) > 0.5      # >0.5 °/s 视为运动段
    m = moving & (t > t[0] + 0.05)
    lag = float(np.median(np.abs(err[m]))) if m.any() else float("nan")
    return dict(name=name,
                peak=float(np.max(np.abs(err))),
                rms=float(np.sqrt(np.mean(err[np.isfinite(err)] ** 2))),
                lag_med=lag,
                tau_min=float(np.min(d["tau"])), tau_max=float(np.max(d["tau"])))


def main():
    a_p = fetch("/tmp/a_noff.csv", OUT / "a_noff.csv")
    b_p = fetch("/tmp/a_ff.csv", OUT / "a_ff.csv")
    A, B = load(a_p), load(b_p)
    sa, sb = stats(A, "无前馈"), stats(B, "速度前馈")

    print(f"{'工况':10s}{'峰值(°)':>10s}{'RMS(°)':>10s}{'巡航滞后(°)':>14s}"
          f"{'力矩范围(‰)':>18s}")
    for s in (sa, sb):
        print(f"{s['name']:10s}{s['peak']:10.3f}{s['rms']:10.3f}{s['lag_med']:14.3f}"
              f"{s['tau_min']:8.0f}..{s['tau_max']:<8.0f}")

    fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    for d, c, lb in ((A, "#1f4e79", "无前馈"), (B, "#c0392b", "速度前馈")):
        ax[0].plot(d["t"], d["plan"], "--", lw=0.9, color=c)
        ax[0].plot(d["t"], d["pos"], lw=1.4, color=c, label=lb + " 实际")
    ax[0].set_ylabel("角度 (°)")
    ax[0].legend()
    ax[0].set_title("台架 A/B：S 曲线 ±10° / 2 s（200 Hz 设点，1000 Hz 采样）")

    for d, c, lb in ((A, "#1f4e79", "无前馈"), (B, "#c0392b", "速度前馈")):
        ax[1].plot(d["t"], d["err"] * 1000, lw=1.1, color=c, label=lb)
    ax[1].axhline(0, color="k", lw=0.5)
    ax[1].set_ylabel("跟随误差 (mdeg)")
    ax[1].legend()

    for d, c, lb in ((A, "#1f4e79", "无前馈"), (B, "#c0392b", "速度前馈")):
        ax[2].plot(d["t"], d["tau"], lw=1.0, color=c, label=lb)
    ax[2].set_ylabel("力矩 (‰ 额定)")
    ax[2].set_xlabel("时间 (s)")
    ax[2].legend()

    p = OUT / "ab_compare.png"
    fig.tight_layout()
    fig.savefig(p, dpi=130)
    print("图 ->", p)
    return sa, sb


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    main()
