# -*- coding: utf-8 -*-
"""拉回四组辨识数据并算出控制需要的参数（v2：修正频响分段、重复性取样、回差判定）。

输出 data/bench/ident_params.json + data/bench/ident.png + 终端表。
"""
from __future__ import annotations

import json
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

FREQS = (0.5, 1, 2, 4, 8)
AMP_DEG = 0.5
CYCLES = 6
DWELL_S = 2.0


def fetch(remote: str, name: str) -> Path:
    c = connect()
    try:
        sftp = c.open_sftp()
        local = OUT / name
        if not local.exists():          # 本地已有就不重复下载
            sftp.get(remote, str(local))
        sftp.close()
    finally:
        c.close()
    return local


def load(p: Path):
    a = np.genfromtxt(p, delimiter=",", names=True, dtype=None, encoding="utf-8")
    return {k: np.asarray(a[k], dtype=float) for k in a.dtype.names}


# ---------------------------------------------------------------- 摩擦
def do_ramp(d):
    t = d["t_s"] - d["t_s"][0]
    pos = d["pos"] / CPD
    plan = d["planned"] / CPD
    tau = d["torque"]
    v = np.gradient(plan, t)
    out = []
    for vn in (0.5, 1, 2, 4, 8):
        for sgn in (1, -1):
            m = (np.abs(v - sgn * vn) < 0.25 * vn) & (np.abs(pos - plan) < 1.0)
            if m.sum() < 50:
                continue
            out.append(dict(v=sgn * vn, tau=float(np.median(tau[m])), n=int(m.sum())))
    return out


def fit_friction(pts):
    if len(pts) < 4:
        return None
    A = np.array([[np.sign(p["v"]), p["v"], 1.0] for p in pts])
    y = np.array([p["tau"] for p in pts])
    sol, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = A @ sol
    r2 = 1.0 - float(np.sum((y - pred) ** 2)) / float(np.sum((y - y.mean()) ** 2))
    return dict(fc_per_mille=float(sol[0]), fv_per_mille_per_deg_s=float(sol[1]),
                bias_per_mille=float(sol[2]), r2=r2)


# ---------------------------------------------------------------- 回差
def do_backlash(d):
    """滞回环宽度：同一位姿在升/降两支上的实际位置差。"""
    plan = d["planned"]
    pos = d["pos"]
    dp = np.diff(plan, prepend=plan[0])
    up = dp > 1.0
    dn = dp < -1.0
    levels = np.unique(plan)
    gaps = []
    lo, hi = np.percentile(levels, 30), np.percentile(levels, 70)
    for c in levels[(levels >= lo) & (levels <= hi)]:
        au = up & (np.abs(plan - c) < 1)
        ad = dn & (np.abs(plan - c) < 1)
        if au.sum() >= 3 and ad.sum() >= 3:
            gaps.append(abs(float(np.median(pos[au]) - np.median(pos[ad]))))
    return dict(backlash_deg=float(np.median(gaps)) / CPD if gaps else float("nan"),
                n=len(gaps), gaps_counts=[float(g) for g in gaps])


def do_step(d):
    """从阶跃响应提取：上升时间、超调、稳态误差 → 等效带宽。"""
    t = d["t_s"] - d["t_s"][0]
    plan = d["planned"]
    pos = d["pos"]
    dp = np.diff(plan, prepend=plan[0])
    edges = np.where(dp > 400)[0]
    out = []
    for e in edges:
        if e < 80 or e > len(t) - 120:
            continue
        pre = float(np.mean(pos[e - 60:e - 5]))
        tgt = float(plan[min(e + 40, len(plan) - 1)])
        if abs(tgt - pre) < 400:
            continue
        seg = pos[e:e + 100]
        y = (seg - pre) / (tgt - pre)
        try:
            t10 = t[e + int(np.argmax(y >= 0.1))]
            t90 = t[e + int(np.argmax(y >= 0.9))]
        except ValueError:
            continue
        tr = float(t90 - t10)
        over = float((np.max(seg) - tgt) / (tgt - pre) * 100)
        ss = float((np.mean(pos[e + 70:e + 95]) - tgt) / CPD * 1000)
        out.append(dict(rise_ms=tr * 1000, overshoot_pct=over, ss_err_mdeg=ss))
    bw = 0.35 / (np.median([o["rise_ms"] for o in out]) / 1000) if out else float("nan")
    return dict(n=len(out), step=out,
                rise_ms_med=float(np.median([o["rise_ms"] for o in out])) if out else float("nan"),
                overshoot_pct_med=float(np.median([o["overshoot_pct"] for o in out])) if out else float("nan"),
                ss_err_mdeg_med=float(np.median([o["ss_err_mdeg"] for o in out])) if out else float("nan"),
                bw_est_hz=float(bw))


# ---------------------------------------------------------------- 频响
def do_frf(d, skip_cycles=2):
    t = d["t_s"] - d["t_s"][0]
    pos = d["pos"] / CPD
    plan = d["planned"] / CPD
    res, t0 = [], 0.0
    for f in FREQS:
        dur = CYCLES / f
        lo, hi = t0 + DWELL_S + skip_cycles / f, t0 + DWELL_S + dur
        m = (t >= lo) & (t <= hi)
        if m.sum() > 30:
            tt = t[m]

            def amp_phase(y):
                y = y[m]
                y = y - y.mean()
                M = np.column_stack([np.cos(2 * np.pi * f * tt), np.sin(2 * np.pi * f * tt)])
                c, *_ = np.linalg.lstsq(M, y, rcond=None)
                return float(np.hypot(*c)), float(np.degrees(np.arctan2(c[0], c[1])))

            ap, pp = amp_phase(plan)
            am, pm = amp_phase(pos)
            if ap > 1e-4:
                lag = (pp - pm + 180) % 360 - 180
                res.append(dict(f_hz=f, amp_ratio=am / ap, phase_lag_deg=lag))
        t0 += DWELL_S + dur
    return res


# ---------------------------------------------------------------- 重复性
def do_repeat(d, tail_s=0.2):
    t = d["t_s"] - d["t_s"][0]
    pos = d["pos"] / CPD
    plan = d["planned"]
    tgt = np.round(plan).astype(int)
    out = []
    for u in np.unique(tgt):
        idx = np.where(tgt == u)[0]
        if len(idx) < 200:
            continue
        breaks = np.where(np.diff(idx) > 5)[0]
        segs = np.split(idx, breaks + 1)
        ends = []
        for s in segs:
            if len(s) * (t[1] - t[0]) < 0.6:
                continue
            tt = t[s]
            m = tt > (tt[-1] - tail_s)
            ends.append(pos[s][m].mean())
        if len(ends) >= 3:
            e = np.array(ends)
            out.append(dict(target_deg=float(u) / CPD, n=len(ends),
                            mean_deg=float(e.mean()),
                            std_mdeg=float(e.std(ddof=1) * 1000),
                            range_mdeg=float((e.max() - e.min()) * 1000)))
    return out


def main():
    files = {k: fetch(f"/tmp/ident_{k}.csv", f"ident_{k}.csv")
             for k in ("ramp", "steps", "sine", "repeat")}
    d = {k: load(v) for k, v in files.items()}

    ramp = do_ramp(d["ramp"])
    fric = fit_friction(ramp)
    back = do_backlash(d["steps"])
    frf = do_frf(d["sine"])
    step = do_step(d["repeat"])
    rep = do_repeat(d["repeat"])

    print("== 摩擦（‰ 额定）==")
    for p in ramp:
        print(f"  v={p['v']:+5.1f} °/s  τ={p['tau']:+7.1f}  n={p['n']}")
    if fric:
        print(f"  拟合: fc={fric['fc_per_mille']:.1f} ‰  "
              f"fv={fric['fv_per_mille_per_deg_s']:.2f} ‰/(°/s)  "
              f"bias={fric['bias_per_mille']:.1f} ‰  R²={fric['r2']:.4f}")

    print(f"\n== 回差 ==\n  {back['backlash_deg']*1000:.0f} mdeg "
          f"({back['backlash_deg']:.4f}°)  样本 {back['n']}")

    print("\n== 频响（分段拟合，跳过前 2 周期）==")
    for p in frf:
        print(f"  {p['f_hz']:4.1f} Hz  幅值比 {p['amp_ratio']:.3f}  "
              f"相位滞后 {p['phase_lag_deg']:+7.1f}°")

    print("\n== 重复性（平台段末 0.2 s 均值）==")
    for p in rep:
        print(f"  目标 {p['target_deg']:7.2f}°  n={p['n']:2d}  "
              f"σ={p['std_mdeg']:6.2f} mdeg  极差 {p['range_mdeg']:6.2f} mdeg")

    params = dict(step=step, friction=fric, ramp=ramp, backlash=back, frf=frf, repeatability=rep,
                  note="空载、低速(≤8°/s)、±15° 内；力矩单位 ‰ 额定")
    (OUT / "ident_params.json").write_text(json.dumps(params, ensure_ascii=False, indent=2),
                                           encoding="utf-8")

    fig, ax = plt.subplots(2, 2, figsize=(10.5, 7))
    v = [p["v"] for p in ramp]
    ax[0][0].plot(v, [p["tau"] for p in ramp], "o-")
    if fric:
        vv = np.linspace(min(v) - 0.5, max(v) + 0.5, 60)
        ax[0][0].plot(vv, fric["fc_per_mille"] * np.sign(vv) +
                      fric["fv_per_mille_per_deg_s"] * vv + fric["bias_per_mille"],
                      "--", label="拟合")
    ax[0][0].axhline(0, color="k", lw=0.5)
    ax[0][0].set_xlabel("速度 (°/s)")
    ax[0][0].set_ylabel("力矩 (‰ 额定)")
    ax[0][0].set_title(f"摩擦：fc={fric['fc_per_mille']:.0f}‰  R²={fric['r2']:.3f}")
    ax[0][0].legend()

    st = d["steps"]
    ts = st["t_s"] - st["t_s"][0]
    ax[0][1].plot(ts, st["planned"] / CPD, lw=0.9, label="命令")
    ax[0][1].plot(ts, st["pos"] / CPD, lw=0.9, label="实际")
    ax[0][1].set_xlabel("时间 (s)")
    ax[0][1].set_ylabel("角度 (°)")
    ax[0][1].set_title(f"双向微步：回差 {back['backlash_deg']*1000:.0f} mdeg")
    ax[0][1].legend()

    f = [p["f_hz"] for p in frf]
    ax[1][0].semilogx(f, [p["amp_ratio"] for p in frf], "o-")
    ax[1][0].axhline(1, color="k", lw=0.5)
    ax[1][0].axhline(1 / np.sqrt(2), color="r", lw=0.6, ls=":")
    ax[1][0].set_xlabel("频率 (Hz)")
    ax[1][0].set_ylabel("幅值比")
    ax[1][0].set_title("幅频（红点线 = −3 dB）")

    if rep:
        ax[1][1].bar([f"{p['target_deg']:.1f}°" for p in rep],
                     [p["std_mdeg"] for p in rep])
        ax[1][1].set_ylabel("σ (mdeg)")
        ax[1][1].set_title("重复性")

    fig.tight_layout()
    png = OUT / "ident.png"
    fig.savefig(png, dpi=130)
    print("\n图 ->", png)
    print("参数 ->", OUT / "ident_params.json")


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    main()
