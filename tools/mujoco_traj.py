# -*- coding: utf-8 -*-
"""直线轨迹回放：这才是"真实的控制"——给起点/终点/时间，逐拍插补 + IK + 伺服。

用法:
  python tools/mujoco_traj.py --a "304 0 530" --b "400 200 600" --t 2.0
  python tools/mujoco_traj.py --a ... --b ... --t 3 --csv out.csv

输出：终端统计 + data/bench/traj_check.png（关节角、关节速度、误差、奇异裕度）
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

MJCF = ROOT / "data" / "models" / "r4_urdf" / "urdf" / "r4_mjcf.xml"
V_RATED_DEG_S = 480.0        # LZ12028CH 额定输出转速


def tcp_of(q):
    return np.asarray(fk(np.asarray(q, dtype=float)), dtype=float).reshape(-1)[:3]


def quintic(u):
    u = min(max(u, 0.0), 1.0)
    return 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--a", default="304 0 530", help="起点 TCP (mm)")
    ap.add_argument("--b", default="400 200 600", help="终点 TCP (mm)")
    ap.add_argument("--t", type=float, default=2.0, help="总时长 (s)")
    ap.add_argument("--kp", type=float, default=800.0)
    ap.add_argument("--csv", default="")
    args = ap.parse_args()

    A = np.array([float(v) for v in args.a.split()]) / 1000.0
    B = np.array([float(v) for v in args.b.split()]) / 1000.0

    import mujoco
    model = mujoco.MjModel.from_xml_path(str(MJCF))
    # 应用 --kp：position 执行器的 gainprm[0]=kp，biasprm[1]=-kp
    for i in range(model.nu):
        model.actuator_gainprm[i, 0] = args.kp
        model.actuator_biasprm[i, 1] = -args.kp
    data = mujoco.MjData(model)
    N = int(args.t / model.opt.timestep)

    q_cmd = np.zeros(4)
    rows = []
    ik_fail = 0
    for k in range(N + 1):
        u = k / N
        p = A + (B - A) * quintic(u)
        r = ik(p, q_ref=q_cmd)
        if r is not None and r.ok:
            q_cmd = np.asarray(r.q, dtype=float)
        else:
            ik_fail += 1
        for i in range(model.nu):
            data.ctrl[i] = q_cmd[i]
        mujoco.mj_step(model, data)
        q_act = np.array([data.qpos[i] for i in range(4)])
        rows.append((k * model.opt.timestep, p.copy(), q_cmd.copy(), q_act,
                     float(np.linalg.norm(p - tcp_of(q_act)) * 1000),
                     singular_margin(q_cmd)))

    t = np.array([r[0] for r in rows])
    qc = np.array([r[2] for r in rows])
    qa = np.array([r[3] for r in rows])
    err = np.array([r[4] for r in rows])
    sm = np.array([r[5] for r in rows])
    qd = np.degrees(np.gradient(qc, t, axis=0))

    print("=" * 66)
    print(f"直线 {np.round(A*1000,0)} -> {np.round(B*1000,0)} mm，{args.t:.1f} s，"
          f"{N} 拍（{1/model.opt.timestep:.0f} Hz）")
    print(f"IK 失败 {ik_fail} 拍")
    print(f"末端跟踪误差：最大 {err.max():.1f} mm  稳态 {err[-1]:.1f} mm")
    print(f"最小奇异裕度：{sm.min():.3f}（0=好，1=奇异）")
    print(f"{'关节':>4s}{'峰值角速度':>12s}{'额定占比':>10s}{'总转角':>10s}")
    for i in range(4):
        peak = np.abs(qd[:, i]).max()
        print(f"{'J'+str(i+1):>4s}{peak:10.1f}°/s{peak/V_RATED_DEG_S*100:9.1f}%"
              f"{np.degrees(np.abs(qc[-1, i]-qc[0, i])):9.1f}°")

    if args.csv:
        np.savetxt(args.csv, np.column_stack([t, qc, qa, err, sm]),
                   delimiter=",", header="t,q1..q4_cmd,q1..q4_act,err_mm,singular",
                   comments="", fmt="%.6f")
        print("CSV ->", args.csv)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["axes.grid"] = True
    plt.rcParams["grid.alpha"] = 0.3

    fig, ax = plt.subplots(2, 2, figsize=(11, 7))
    for i in range(4):
        ax[0][0].plot(t, np.degrees(qc[:, i]), label=f"J{i+1}")
        ax[0][1].plot(t, qd[:, i], label=f"J{i+1}")
    ax[0][0].set_title("关节角指令")
    ax[0][0].set_ylabel("角度 (°)")
    ax[0][1].set_title("关节角速度")
    ax[0][1].set_ylabel("速度 (°/s)")
    ax[0][1].axhline(V_RATED_DEG_S, color="r", ls=":", lw=0.8, label="额定 480")
    ax[0][1].axhline(-V_RATED_DEG_S, color="r", ls=":", lw=0.8)
    for a_ in (ax[0][0], ax[0][1]):
        a_.set_xlabel("时间 (s)")
        a_.legend(fontsize=8, ncol=4)

    ax[1][0].plot(t, err)
    ax[1][0].set_title(f"末端跟踪误差（最大 {err.max():.1f} mm）")
    ax[1][0].set_xlabel("时间 (s)")
    ax[1][0].set_ylabel("误差 (mm)")

    ax[1][1].plot(t, sm)
    ax[1][1].axhline(0.9, color="r", ls=":", lw=0.8)
    ax[1][1].set_title(f"奇异裕度（最小 {sm.min():.2f}，>0.9 危险）")
    ax[1][1].set_xlabel("时间 (s)")
    ax[1][1].set_ylabel("裕度")

    out = ROOT / "data" / "bench" / "traj_check.png"
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print("图 ->", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
