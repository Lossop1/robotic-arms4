# -*- coding: utf-8 -*-
"""MuJoCo 可视化：**操控末端（TCP）**，IK 解关节——用来验证规划算法。

流程：键盘改目标 TCP -> ik() 解出四个关节角 -> position 执行器（CSP 位置环）跟踪。
屏幕上打印 目标/实际 TCP、误差、关节角、奇异裕度。

用法:
  python tools/mujoco_tcp.py             # 打开窗口，键盘操控末端
  python tools/mujoco_tcp.py --headless  # 自检：扫一个方块轨迹，报告误差与不可达点

按键:
  A / D   TCP 沿 X −/+
  W / S   TCP 沿 Y +/−
  Q / E   TCP 沿 Z −/+
  J / L   工具自转角 −/+
  R       目标回到当前末端
  G       重力开关
  ESC     退出
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "robot"))
from r4ctl.ik import fk, ik, singular_margin  # noqa: E402

MJCF = ROOT / "data" / "models" / "r4_urdf" / "urdf" / "r4_mjcf.xml"


class Keys:
    def __init__(self):
        try:
            import msvcrt
            self.m = msvcrt
            self.ok = True
        except ImportError:
            self.m = None
            self.ok = False

    def poll(self):
        out = []
        if not self.ok:
            return out
        while self.m.kbhit():
            out.append(self.m.getwch().lower())
        return out


def tcp_of(q):
    """正解 -> 工具点 (m)"""
    return np.asarray(fk(np.asarray(q, dtype=float)), dtype=float).reshape(-1)[:3]


def solve(p, roll, q_ref):
    """返回 (q, ok)。ok=False 表示不可达/超限，保留上一次有效解。"""
    try:
        r = ik(np.asarray(p, dtype=float), roll=roll, q_ref=q_ref)
    except Exception:
        return None, False
    if r is None or not getattr(r, "ok", False):
        return None, False
    return np.asarray(r.q, dtype=float), True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", default=str(MJCF))
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--step-mm", type=float, default=5.0)
    args = ap.parse_args()

    import mujoco
    model = mujoco.MjModel.from_xml_path(args.mjcf)
    data = mujoco.MjData(model)
    nq = min(model.nq, 4)

    q_now = np.zeros(4)
    p_now = tcp_of(q_now)
    p_tgt = p_now.copy()
    roll = 0.0
    q_des = q_now.copy()

    def step_once():
        nonlocal q_des
        q, ok = solve(p_tgt, roll, q_des)
        if ok:
            q_des = q
        for i in range(model.nu):
            data.ctrl[i] = q_des[i] if i < len(q_des) else 0.0
        mujoco.mj_step(model, data)

    def report():
        q_act = np.array([data.qpos[i] for i in range(nq)])
        p_act = tcp_of(q_act)
        err = np.linalg.norm(p_tgt - p_act) * 1000
        sm = singular_margin(q_des)
        qd = "  ".join(f"J{i+1}={math.degrees(q_des[i]):+6.1f}" for i in range(4))
        return (f"TCP 目标 ({p_tgt[0]*1000:+6.1f},{p_tgt[1]*1000:+6.1f},{p_tgt[2]*1000:+6.1f}) mm"
                f"  误差 {err:5.1f} mm  奇异裕度 {sm:.2f}  {qd}")

    if args.headless:
        base = p_now.copy()
        ok_n, bad_n, errs = 0, 0, []
        for k in range(240):
            t = k / 60.0
            p_tgt[:] = base + np.array([0.03 * math.sin(2 * math.pi * 0.3 * t),
                                        0.03 * math.cos(2 * math.pi * 0.3 * t),
                                        0.02 * math.sin(2 * math.pi * 0.5 * t)])
            q, ok = solve(p_tgt, roll, q_des)
            ok_n += ok
            bad_n += (not ok)
            if ok:
                q_des = q
            for i in range(model.nu):
                data.ctrl[i] = q_des[i]
            for _ in range(4):
                mujoco.mj_step(model, data)
            errs.append(np.linalg.norm(
                p_tgt - tcp_of(np.array([data.qpos[i] for i in range(nq)]))) * 1000)
        print(f"headless：可达 {ok_n} 帧 / 不可达 {bad_n} 帧，"
              f"末端误差 中位 {np.median(errs):.2f} mm 最大 {np.max(errs):.2f} mm")
        print(report())
        return 0

    import mujoco.viewer as mjv
    keys = Keys()
    with mjv.launch_passive(model, data) as viewer:
        viewer.cam.distance = 1.6
        viewer.cam.elevation = -18
        viewer.cam.azimuth = 130
        d = args.step_mm / 1000.0
        last = time.perf_counter()
        while viewer.is_running():
            for ch in keys.poll():
                if ch == "\x1b":
                    return 0
                elif ch == "a":
                    p_tgt[0] -= d
                elif ch == "d":
                    p_tgt[0] += d
                elif ch == "w":
                    p_tgt[1] += d
                elif ch == "s":
                    p_tgt[1] -= d
                elif ch == "q":
                    p_tgt[2] -= d
                elif ch == "e":
                    p_tgt[2] += d
                elif ch == "j":
                    roll -= math.radians(5)
                elif ch == "l":
                    roll += math.radians(5)
                elif ch == "r":
                    p_tgt[:] = tcp_of(np.array([data.qpos[i] for i in range(nq)]))
                elif ch == "g":
                    model.opt.gravity[2] *= -1.0
            step_once()
            viewer.sync()
            if time.perf_counter() - last > 0.5:
                last = time.perf_counter()
                print("\r" + report(), end="", flush=True)
            time.sleep(model.opt.timestep)
    return 0


if __name__ == "__main__":
    sys.exit(main())
