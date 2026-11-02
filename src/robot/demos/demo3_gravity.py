# -*- coding: utf-8 -*-
"""
Demo 3：重力补偿与柔顺 —— 断电 / 纯重力补偿 / 软伺服拖拽示教。

三种工况从同一个位姿（伸出去的横臂，重力力矩最大）出发，1.5 s 内给 J2 施加冲击，
看机械臂怎么动：
  A 断电（τ=0）        → 直接掉下来，只有摩擦挡着
  B τ = g(q)（kp=kd=0）→ 停在原地（这就是"零重力"手感，也是拖拽示教的基础）
  C 软伺服（kp 取 5%，kd 取 15%）→ 被推动、松手后弹回，可做手动示教
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import COLORS, deg, save  # noqa: E402

import matplotlib.pyplot as plt  # noqa: E402

from r4ctl.model import chain  # noqa: E402
from r4ctl.servo import Servo  # noqa: E402
from r4ctl.sim import Sim  # noqa: E402

ch = chain()
Q_START = np.radians([20.0, -60.0, 55.0, 15.0])     # 伸出去的位姿
T = 3.0


def hold_ref(q0):
    def ref(t):
        return q0, np.zeros(ch.n), np.zeros(ch.n)
    return ref


def push(t):
    d = np.zeros(ch.n)
    if 1.0 <= t < 1.4:                     # 0.4 s 的外力脉冲（模拟手推）
        d[1] = 25.0
    return d


def main():
    print("== Demo 3: 重力补偿与柔顺 ==")
    runs = {}

    # A 断电
    servo_off = Servo("pd", kp=np.zeros(ch.n), kd=np.zeros(ch.n))
    runs["A 断电 (τ=0)"] = Sim().run(hold_ref(Q_START), servo_off, T=T, q0=Q_START)

    # B 纯重力补偿
    servo_g = Servo("pd_g", kp=np.zeros(ch.n), kd=np.zeros(ch.n))
    runs["B 重力补偿"] = Sim().run(hold_ref(Q_START), servo_g, T=T, q0=Q_START)

    # C 软伺服 + 推
    servo_soft = Servo("pd_g_ff", kp_scale=0.05, kd_scale=0.15)
    runs["C 软伺服+手推"] = Sim().run(hold_ref(Q_START), servo_soft, T=T, q0=Q_START, disturbance=push)

    for name, log in runs.items():
        e = np.degrees(np.abs(log["q"] - log["q_des"]).max(axis=1))
        print(f"  {name:14s} 最大偏离 {e.max():7.2f}°  末端最大偏离起始点 "
              f"{np.linalg.norm(log['tip'] - log['tip'][0], axis=1).max()*1000:7.1f} mm")

    # ---------------- 图
    fig, axes = plt.subplots(2, 2, figsize=(10.5, 6.6))
    ax = axes[0][0]
    for i, (name, log) in enumerate(runs.items()):
        ax.plot(log["t"], np.degrees(log["q"][:, 1]), color=COLORS[i], lw=1.4, label=name)
    ax.axhline(deg(ch.q_min[1]), color="r", lw=0.8, ls=":",
               label=f"J2 机械限位 {deg(ch.q_min[1]):.0f}°")
    ax.axvspan(1.0, 1.4, color="0.85", alpha=0.5)
    ax.set_ylabel("J2 角度 (deg)")
    ax.set_xlabel("时间 (s)")
    ax.set_title("A J2 角度（灰区 = 外力脉冲 25 N·m）")
    ax.legend(fontsize=7)

    ax = axes[0][1]
    for i, (name, log) in enumerate(runs.items()):
        d = np.linalg.norm(log["tip"] - log["tip"][0], axis=1) * 1000
        ax.plot(log["t"], d, color=COLORS[i], lw=1.3, label=name)
    ax.set_ylabel("末端偏离起始点 (mm)")
    ax.set_xlabel("时间 (s)")
    ax.set_title("B 末端位移")
    ax.legend(fontsize=7)

    ax = axes[1][0]
    for i, (name, log) in enumerate(runs.items()):
        ax.plot(log["t"], log["tau"][:, 1], color=COLORS[i], lw=1.2, label=name)
    ax.axvspan(1.0, 1.4, color="0.85", alpha=0.5)
    ax.set_ylabel("J2 力矩 (N·m)")
    ax.set_xlabel("时间 (s)")
    ax.set_title("C 力矩（断电时恒 0；重力补偿 ≈ 保持位姿所需的重力矩）")
    ax.legend(fontsize=7)

    ax = axes[1][1]
    for i, (name, log) in enumerate(runs.items()):
        e = np.degrees(np.abs(log["q"] - log["q_des"]).max(axis=1))
        ax.semilogy(log["t"], np.maximum(e, 1e-4), color=COLORS[i], lw=1.1, label=name)
    ax.set_ylabel("最大关节偏离 (deg, log)")
    ax.set_xlabel("时间 (s)")
    ax.set_title("D 偏离给定（对数）")
    ax.legend(fontsize=7)

    save(fig, "demo3_gravity.png")

    # 数值小结
    drift = {}
    for name, log in runs.items():
        drift[name] = float(np.degrees(np.abs(log["q"][-1] - log["q"][0]).max()))
    return runs, drift


if __name__ == "__main__":
    main()
