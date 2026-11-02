# -*- coding: utf-8 -*-
"""把台架采集的 CSV 拉回本地并画图。

用法: python bench/fetch_plot.py <远端路径> <本地名字>
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

VERSION = "0.1.0"

if "--version" in sys.argv[1:]:
    print(f"{Path(__file__).name} {VERSION}")
    raise SystemExit(0)

DATA = Path(__file__).resolve().parent / "data"
DATA.mkdir(exist_ok=True)
CPR = 16384 * 28          # counts / 输出圈


def fetch(remote: str, name: str) -> Path:
    c = connect()
    try:
        sftp = c.open_sftp()
        local = DATA / name
        sftp.get(remote, str(local))
        sftp.close()
    finally:
        c.close()
    return local


def load(path: Path):
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines()[1:]:
        f = line.split(",")
        def num(i):
            try:
                return float(f[i])
            except (ValueError, IndexError):
                return np.nan
        # 列: t_s,pos,vel,torque,planned,target,state,(后面被 enabled 字段污染)
        rows.append((num(0), num(1), num(2), num(3), num(4), num(5), num(6)))
    a = np.array(rows, dtype=float)
    return dict(t=a[:, 0], pos=a[:, 1], vel=a[:, 2], torque=a[:, 3],
                planned=a[:, 4], target=a[:, 5], state=a[:, 6])


def main():
    if len(sys.argv) < 3:
        print("用法: fetch_plot.py \u003c远端路径\u003e \u003c本地名字\u003e", file=sys.stderr)
        return 2
    remote, name = sys.argv[1], sys.argv[2]
    p = fetch(remote, name)
    d = load(p)
    pos_deg = (d["pos"] - d["pos"][0]) / CPR * 360
    plan_deg = (d["planned"] - d["pos"][0]) / CPR * 360
    err_deg = (d["pos"] - d["planned"]) / CPR * 360

    print(f"样本 {len(d['t'])}，时长 {d['t'][-1]-d['t'][0]:.2f}s")
    print(f"位移 {pos_deg[-1]:.3f}°  终点误差 {err_deg[-1]*1000:+.0f} mdeg")
    print(f"跟踪误差 峰值 {np.nanmax(np.abs(err_deg)):.3f}°  RMS {np.sqrt(np.nanmean(err_deg**2)):.3f}°")
    print(f"力矩 per-mille: min {np.nanmin(d['torque']):.0f} / max {np.nanmax(d['torque']):.0f}")

    fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    ax[0].plot(d["t"], plan_deg, "k--", lw=1.0, label="驱动器计划")
    ax[0].plot(d["t"], pos_deg, lw=1.6, label="实际")
    ax[0].set_ylabel("角度 (°)")
    ax[0].legend()
    ax[0].set_title(f"第一段 CSP 运动：+36°，5 s（{name}）")

    ax[1].plot(d["t"], err_deg * 1000, lw=1.2, color="#c0392b")
    ax[1].axhline(0, color="k", lw=0.5)
    ax[1].set_ylabel("跟随误差 (mdeg)")

    ax[2].plot(d["t"], d["torque"], lw=1.2, color="#1f4e79")
    ax[2].axhline(0, color="k", lw=0.5)
    ax[2].set_ylabel("力矩 (‰ 额定)")
    ax[2].set_xlabel("时间 (s)")

    out = DATA / (Path(name).stem + ".png")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print("图 ->", out)


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    sys.exit(main())