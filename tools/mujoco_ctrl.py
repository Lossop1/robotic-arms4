# -*- coding: utf-8 -*-
"""MuJoCo 里跑**我们自己的控制律**（力矩级），而不是 MuJoCo 自带的位置伺服。

四条律对比：
  p      纯位置伺服（等效你现在的 CSP：只在驱动器里 P 一下）
  pd     PD 反馈
  pdg    PD + 重力前馈
  pdgf   PD + 重力 + 摩擦前馈
  ctc    计算力矩（反馈线性化）

用法:
  python tools/mujoco_ctrl.py --a "304 0 530" --b "400 200 600" --t 2.0
  python tools/mujoco_ctrl.py --law pdgf          # 只跑一条
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "robot"))
from r4ctl.ik import fk, ik, singular_margin  # noqa: E402
from r4ctl.model import Chain  # noqa: E402

MJCF = ROOT / "data" / "models" / "r4_urdf" / "urdf" / "r4_mjcf.xml"
TAU_MAX = 100.0          # 额定力矩 N·m（厂家规格表）
FC = 2.6                 # 库仑摩擦 N·m（实测 259.5‰ × 10 N·m）
FV = 0.104               # 粘滞摩擦 N·m/(°/s)


def tcp_of(q):
    return np.asarray(fk(np.asarray(q, dtype=float)), dtype=float).reshape(-1)[:3]


def quintic(u):
    u = min(max(u, 0.0), 1.0)
    return 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5


def tcp_path(A, B, N):
    return np.array([A + (B - A) * quintic(k / N) for k in range(N + 1)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--a", default="304 0 530")
    ap.add_argument("--b", default="400 200 600")
    ap.add_argument("--t", type=float, default=2.0)
    ap.add_argument("--law", default="all")
    ap.add_argument("--wn", type=float, default=8.0, help="目标带宽 Hz")
    args = ap.parse_args()

    from r4ctl import config as C
    ch = Chain()
    M_eff = np.asarray(ch.effective_inertia(), dtype=float)

    omega = 2 * math.pi * args.wn
    kp = M_eff * omega ** 2
    kd = 2.0 * M_eff * omega

    A = np.array([float(v) for v in args.a.split()]) / 1000.0
    B = np.array([float(v) for v in args.b.split()]) / 1000.0

    import mujoco
    laws = ["p", "pd", "pdg", "pdgf", "ctc"] if args.law == "all" else [args.law]
    results = {}

    for law in laws:
        model = mujoco.MjModel.from_xml_path(str(MJCF))
        data = mujoco.MjData(model)
        # 把 position 执行器改成力矩执行器：gain=1, bias=0
        for i in range(model.nu):
            model.actuator_gainprm[i, :] = 0.0
            model.actuator_gainprm[i, 0] = 1.0
            model.actuator_biasprm[i, :] = 0.0
            model.actuator_ctrlrange[i] = (-TAU_MAX, TAU_MAX)
            model.dof_frictionloss[i] = FC          # 让摩擦前馈有意义
        N = int(args.t / model.opt.timestep)
        path = tcp_path(A, B, N)

        q = np.zeros(4)
        qd = np.zeros(4)
        rows = []
        ik_fail = 0
        for k in range(N + 1):
            r = ik(path[k], q_ref=q)
            if r is not None and r.ok:
                q_d = np.asarray(r.q, dtype=float)
            else:
                q_d = q.copy()
                ik_fail += 1
            # 期望速度：用差分
            qd_d = (q_d - rows[-1][1]) / model.opt.timestep if rows else np.zeros(4)

            e = q_d - q
            ed = qd_d - qd
            if law == "p":
                tau = kp * e
            elif law == "pd":
                tau = kp * e + kd * ed
            elif law == "pdg":
                tau = kp * e + kd * ed + ch.gravity(q_d)
            elif law == "pdgf":
                # 用 tanh 平滑 sign，避免零速附近抖振
                tau = (kp * e + kd * ed + ch.gravity(q_d)
                       + FC * np.tanh(np.degrees(qd) / 1.0) + FV * np.degrees(qd))
            else:   # ctc
                Mq = np.asarray(ch.mass_matrix(q_d), dtype=float)
                bias = np.asarray(ch.bias(q_d, qd_d), dtype=float)
                tau = Mq @ (omega ** 2 * e + 2 * omega * ed) + bias

            tau = np.clip(tau, -TAU_MAX, TAU_MAX)
            data.ctrl[:] = tau
            mujoco.mj_step(model, data)
            q = np.array([data.qpos[i] for i in range(4)])
            qd = np.array([data.qvel[i] for i in range(4)])
            rows.append((k * model.opt.timestep, q_d.copy(), q.copy(),
                         float(np.linalg.norm(path[k] - tcp_of(q)) * 1000),
                         float(np.linalg.norm(tau))))

        err = np.array([r[3] for r in rows])
        tau_n = np.array([r[4] for r in rows])
        tail = err[N // 2:]
        results[law] = dict(err_max=float(err.max()), err_tail=float(tail.mean()),
                            tau_max=float(tau_n.max()), ik_fail=ik_fail,
                            rows=rows)

    print(f"轨迹 {np.round(A*1000,0)} -> {np.round(B*1000,0)} mm，{args.t}s，"
          f"带宽 {args.wn} Hz，kp={np.round(kp,1)}，kd={np.round(kd,1)}")
    print(f"摩擦摩擦：dof_frictionloss = {FC} N·m\n")
    print(f"{'控制律':>6s}{'误差峰值(mm)':>14s}{'后半段均值(mm)':>16s}"
          f"{'力矩峰值(N·m)':>14s}{'IK失败':>8s}")
    for law in laws:
        r = results[law]
        print(f"{law:>6s}{r['err_max']:14.1f}{r['err_tail']:16.1f}"
              f"{r['tau_max']:14.1f}{r['ik_fail']:8d}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["axes.grid"] = True
    plt.rcParams["grid.alpha"] = 0.3

    fig, ax = plt.subplots(1, 2, figsize=(12, 4.6))
    for law in laws:
        rows = results[law]["rows"]
        t = [r[0] for r in rows]
        ax[0].plot(t, [r[3] for r in rows], label=law)
        ax[1].plot(t, [r[4] for r in rows], label=law)
    ax[0].set_title("末端跟踪误差")
    ax[0].set_xlabel("时间 (s)")
    ax[0].set_ylabel("误差 (mm)")
    ax[0].legend()
    ax[1].set_title(f"控制力矩（上限 {TAU_MAX:.0f} N·m）")
    ax[1].set_xlabel("时间 (s)")
    ax[1].set_ylabel("力矩 (N·m)")
    ax[1].legend()
    out = ROOT / "data" / "bench" / "ctrl_laws.png"
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print("\n图 ->", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
