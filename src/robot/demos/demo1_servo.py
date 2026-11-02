# -*- coding: utf-8 -*-
"""
Demo 1：伺服闭环对比（PD / PD+重力 / PD+重力+摩擦 / 计算力矩 CTC）

三件事分开看：
 A 轨迹跟踪：S 曲线走 J2 -45°→-20°，看跟踪误差（这决定加工/抓取精度）
 B 静态保持：在伸展位姿停住 2 s，只看稳态误差（这决定重力补偿有没有用）
 C 小阶跃：0.5° 位置阶跃（10 ms 斜坡），看闭环带宽/上升时间/超调

注：Kp 是按 M_eff 最大位形整定的（最保守），所以大幅阶跃会打力矩饱和（72 N·m），
    这不是算法问题——快速大行程要么靠轨迹规划限速，要么靠力矩裕度。
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import COLORS, deg, overshoot, rms, save, settling_time  # noqa: E402

import matplotlib.pyplot as plt  # noqa: E402

from r4ctl import config as C  # noqa: E402
from r4ctl.model import chain  # noqa: E402
from r4ctl.servo import Servo  # noqa: E402
from r4ctl.sim import Sim  # noqa: E402
from r4ctl.traj import JointMove  # noqa: E402

ch = chain()
TAU_MAX = C.SAFETY["tau_margin"] * C.ACTUATOR["tau_cont"]
Q_START = np.radians([0.0, -45.0, 40.0, 0.0])
Q_GOAL = np.radians([0.0, -20.0, 40.0, 0.0])
MODES = ["pd", "pd_g", "pd_g_ff", "ctc"]


def track_ref(move, q0, t_end):
    def ref(t):
        if t <= move.T:
            return move.sample(t)
        return move.sample(move.T)[0], np.zeros(ch.n), np.zeros(ch.n)
    return ref


def run_track(mode):
    move = JointMove(Q_START, Q_GOAL)
    sim = Sim()
    log = sim.run(track_ref(move, Q_START, move.T + 0.6), Servo(mode),
                  T=move.T + 0.6, q0=Q_START)
    t = log["t"]
    e = log["q"] - log["q_des"]
    tail = t > move.T + 0.2
    return log, dict(
        T=move.T,
        rms_deg=float(np.degrees(rms(e))),
        max_deg=float(np.degrees(np.abs(e).max())),
        static_deg=float(np.degrees(np.abs(e[tail]).max())),
        tau_peak=float(np.abs(log["tau"]).max()),
        q_after=log["q"][-1],
    )


def main():
    print("== Demo 1: 伺服闭环对比 ==")
    print(f"   饱和力矩 {TAU_MAX:.0f} N·m（{C.ACTUATOR['model']} S1 {C.ACTUATOR['tau_cont']:.0f} N·m × 0.9）")

    logs, res = {}, {}
    for mode in MODES:
        log, r = run_track(mode)
        logs[mode], res[mode] = log, r
        print(f"  {mode:8s} 行程 {r['T']:.3f}s  跟踪RMS {r['rms_deg']:.4f}°  "
              f"最大 {r['max_deg']:6.3f}°  末端静态 {r['static_deg']:.4f}°  "
              f"力矩峰值 {r['tau_peak']:5.1f} N·m")

    # ---- C 扰动抑制：保持位姿时给 J2 打 12 N·m 方波（0.3~0.5 s）
    def hold_ref(q0):
        def ref(t):
            return q0, np.zeros(ch.n), np.zeros(ch.n)
        return ref

    def dist(t):
        d = np.zeros(ch.n)
        if 0.3 <= t < 0.5:
            d[1] = 12.0
        return d

    dist_res = {}
    for mode in ("pd", "pd_g_ff"):
        lg = Sim().run(hold_ref(Q_START), Servo(mode), T=1.2, q0=Q_START, disturbance=dist)
        t = lg["t"]
        e = deg(lg["q"][:, 1] - lg["q_des"][:, 1])
        base = float(np.mean(e[(t > 0.25) & (t < 0.30)]))          # 扰动前的静态误差
        rel = e - base
        peak = float(np.abs(rel).max())
        after = (t > 0.5) & (np.abs(rel) > 0.1 * peak)
        rec = float(t[after].max()) if after.any() else 0.5
        dist_res[mode] = dict(base_err_deg=base, peak_dev_deg=peak,
                              recover_ms=(rec - 0.5) * 1000,
                              stiffness_Nm_per_rad=12.0 / np.radians(peak) if peak > 0 else float("inf"))
        print(f"  扰动抑制 {mode:8s} 扰动前静态 {base:+.3f}°  扰动峰值 {peak:.3f}°  "
              f"衰减 {(rec-0.5)*1000:5.0f} ms  闭环刚度 {dist_res[mode]['stiffness_Nm_per_rad']:.0f} N·m/rad")

    # ================= 图 =================
    fig, axes = plt.subplots(2, 2, figsize=(10.5, 6.6))

    ax = axes[0][0]
    for i, mode in enumerate(MODES):
        lg = logs[mode]
        ax.plot(lg["t"], deg(lg["q"][:, 1]), color=COLORS[i], lw=1.3, label=mode)
    ax.plot(logs["ctc"]["t"], deg(logs["ctc"]["q_des"][:, 1]), "k--", lw=0.8, label="给定")
    ax.set_ylabel("J2 角度 (deg)")
    ax.set_title("A 轨迹跟踪（S 曲线，-45°→-20°）")
    ax.legend()

    ax = axes[0][1]
    for i, mode in enumerate(MODES):
        lg = logs[mode]
        ax.plot(lg["t"], deg(lg["q"][:, 1] - lg["q_des"][:, 1]), color=COLORS[i], lw=1.1, label=mode)
    ax.axhline(0, color="k", lw=0.6)
    ax.set_yscale("symlog", linthresh=1e-3)
    ax.set_ylabel("J2 跟踪误差 (deg)")
    ax.set_xlabel("时间 (s)")
    ax.set_title("跟踪误差（symlog）")
    ax.legend(ncol=2)

    ax = axes[1][0]
    x = np.arange(len(MODES))
    v1 = [res[m]["rms_deg"] for m in MODES]
    v2 = [res[m]["static_deg"] for m in MODES]
    ax.bar(x - 0.2, np.maximum(v1, 1e-5), 0.4, label="运动段 RMS", color="#1f4e79")
    ax.bar(x + 0.2, np.maximum(v2, 1e-5), 0.4, label="末端静态", color="#c0392b")
    ax.set_yscale("log")
    ax.set_xticks(x)
    ax.set_xticklabels(MODES)
    ax.set_ylabel("误差 (deg, log)")
    ax.set_title("B 误差对比（重力前馈把静态误差压到量化以下）")
    ax.legend()

    ax = axes[1][1]
    lg = logs["pd_g_ff"]
    for i in range(ch.n):
        ax.plot(lg["t"], lg["tau"][:, i], color=COLORS[i], lw=1.1, label=f"J{i+1}")
    ax.axhline(TAU_MAX, color="r", lw=0.7, ls=":")
    ax.axhline(-TAU_MAX, color="r", lw=0.7, ls=":")
    ax.set_ylabel("关节力矩 (N·m)")
    ax.set_xlabel("时间 (s)")
    ax.set_title("C 力矩（pd+重力+摩擦）")
    ax.legend(ncol=4)

    save(fig, "demo1_servo.png")

    # ---- 扰动抑制单独一张
    fig, ax = plt.subplots(figsize=(6.6, 3.2))
    for i, mode in enumerate(("pd", "pd_g_ff")):
        lg = Sim().run(hold_ref(Q_START), Servo(mode), T=1.2, q0=Q_START, disturbance=dist)
        ax.plot(lg["t"], deg(lg["q"][:, 1] - lg["q_des"][:, 1]), color=COLORS[i], lw=1.3, label=mode)
    ax.axvspan(0.3, 0.5, color="0.85", alpha=0.6)
    ax.set_ylabel("J2 误差 (deg)")
    ax.set_xlabel("时间 (s)")
    ax.set_title("D 扰动抑制：0.3~0.5 s 给 J2 施加 12 N·m")
    ax.legend()
    save(fig, "demo1_disturbance.png")

    return res, dist_res


if __name__ == "__main__":
    main()
