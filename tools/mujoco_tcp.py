# -*- coding: utf-8 -*-
"""MuJoCo 末端控制：在坐标系里改末端目标（红球），IK 解关节，机械臂去追。

可见性设计（针对"不知道发生了什么"）：
  - 红球 = 目标 TCP（直径 30 mm），开局就在末端下方 6 cm，一定看得见
  - 绿球 = 实际末端（直径 20 mm）
  - 终端每条命令都回显：收到什么 -> IK 结果 -> 当前误差

两种输入方式：
  1) 数字命令（无歧义，推荐）：在终端里输入
        p 400 0 600            设定目标 TCP（单位 mm，世界系）
        x 400 / y 0 / z 600    只改一个轴
        +x 20 / -z 20          相对移动（mm）
        roll 90                工具自转角（度）
        home                   目标回到当前末端
        status                 打印一次状态
        quit                   退出
  2) 键盘（可选）：W/S/A/D/Q/E 按住连续移动（按当前视角分解）

用法:
  python tools/mujoco_tcp.py             # 窗口 + 终端命令
  python tools/mujoco_tcp.py --selftest  # 不开窗口，跑命令自检
"""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "robot"))
from r4ctl.ik import fk, ik, singular_margin  # noqa: E402

MJCF = ROOT / "data" / "models" / "r4_urdf" / "urdf" / "r4_mjcf.xml"


def add_capsule(mujoco, scn, p0, p1, radius, rgba):
    """在 user_scn 里加一条胶囊（用于坐标轴和误差连线）。"""
    p0 = np.asarray(p0, dtype=float); p1 = np.asarray(p1, dtype=float)
    d = p1 - p0
    L = float(np.linalg.norm(d))
    if L < 1e-6 or scn.ngeom >= len(scn.geoms):
        return
    z = d / L
    tmp = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(tmp, z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE,
                        np.array([radius, L / 2, 0.0]),
                        ((p0 + p1) / 2).astype(np.float64),
                        np.column_stack([x, y, z]).ravel().astype(np.float64),
                        np.array(rgba, dtype=np.float32))
    scn.ngeom += 1


def tcp_of(q):
    return np.asarray(fk(np.asarray(q, dtype=float)), dtype=float).reshape(-1)[:3]


class Ctl:
    """目标状态 + 最近的求解结果，供渲染和命令线程共享。"""

    def __init__(self, p0):
        self.lock = threading.Lock()
        self.p = np.asarray(p0, dtype=float).copy()
        self.roll = 0.0
        self.q = np.zeros(4)
        self.ok = True
        self.err_mm = float("nan")
        self.rate = 0.04          # 按住键盘时的移动速度 m/s

    def set_p(self, p):
        with self.lock:
            self.p = np.asarray(p, dtype=float).copy()


class Keys:
    VK = {"a": 0x41, "d": 0x44, "w": 0x57, "s": 0x53, "q": 0x51, "e": 0x45}

    def __init__(self):
        try:
            import ctypes
            self.u = ctypes.windll.user32
        except Exception:
            self.u = None

    def down(self, k):
        if self.u is None:
            return False
        return bool(self.u.GetAsyncKeyState(self.VK[k]) & 0x8000)


def apply_command(ctl, line, q_now):
    parts = line.strip().split()
    if not parts:
        return ""
    cmd = parts[0].lower()
    try:
        with ctl.lock:
            if cmd == "p" and len(parts) == 4:
                ctl.p = np.array([float(v) for v in parts[1:4]]) / 1000.0
                return f"目标 = {np.round(ctl.p*1000,1)} mm"
            if cmd in ("x", "y", "z") and len(parts) == 2:
                i = "xyz".index(cmd)
                ctl.p[i] = float(parts[1]) / 1000.0
                return f"目标 = {np.round(ctl.p*1000,1)} mm"
            if cmd[0] in "+-" and cmd[1:] in ("x", "y", "z") and len(parts) == 2:
                i = "xyz".index(cmd[1:])
                sgn = 1.0 if cmd[0] == "+" else -1.0
                ctl.p[i] += sgn * float(parts[1]) / 1000.0
                return f"目标 = {np.round(ctl.p*1000,1)} mm"
            if cmd == "roll" and len(parts) == 2:
                ctl.roll = math.radians(float(parts[1]))
                return f"工具角 = {float(parts[1]):.1f}°"
            if cmd == "home":
                ctl.p = np.asarray(tcp_of(q_now)).copy()
                return f"目标 = {np.round(ctl.p*1000,1)} mm（当前末端）"
            if cmd == "speed" and len(parts) == 2:
                return f"__SPEED__{float(parts[1])}"
            if cmd == "frame":
                return "基座坐标系：原点在基座底面，X 指臂伸出方向，Y 左，Z 上"
            if cmd == "status":
                return "__STATUS__"
            if cmd in ("quit", "exit"):
                return "__QUIT__"
            return "无法识别：用 p x y z（mm）或 +x 20、home、status"
    except ValueError:
        return "数字格式不对"


def stdin_thread(ctl, stop_evt):
    for line in sys.stdin:
        if stop_evt.is_set():
            return
        msg = apply_command(ctl, line, ctl.q)
        if msg.startswith("__SPEED__"):
            try:
                spd = float(msg.replace("__SPEED__", ""))
                with ctl.lock:
                    ctl.rate = spd / 1000.0
                print(f"\n[命令] 速度 -> {spd:.0f} mm/s", flush=True)
            except ValueError:
                print("\n速度值不对", flush=True)
            continue
        if msg == "__QUIT__":
            stop_evt.set()
            return
        if msg and msg != "__STATUS__":
            print(f"\n[命令] {line.strip():22s} -> {msg}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", default=str(MJCF))
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    import mujoco
    model = mujoco.MjModel.from_xml_path(args.mjcf)
    data = mujoco.MjData(model)
    nq = min(model.nq, 4)

    p_now = tcp_of(np.zeros(4))
    # 默认目标：末端下方 6 cm —— 开局就看得见，且离机械臂有间距
    ctl = Ctl(p_now)

    def solve_and_step():
        with ctl.lock:
            p_tgt, roll = ctl.p.copy(), ctl.roll
        r = ik(p_tgt, roll=roll, q_ref=ctl.q)
        with ctl.lock:
            if r is not None and r.ok:
                ctl.q = np.asarray(r.q, dtype=float)
                ctl.ok = True
            else:
                ctl.ok = False
        for i in range(model.nu):
            data.ctrl[i] = ctl.q[i]
        mujoco.mj_step(model, data)
        q_act = np.array([data.qpos[i] for i in range(nq)])
        with ctl.lock:
            ctl.err_mm = float(np.linalg.norm(p_tgt - tcp_of(q_act)) * 1000)
        return q_act

    def status_line(q_act):
        with ctl.lock:
            p_tgt, qd, ok, err = ctl.p.copy(), ctl.q.copy(), ctl.ok, ctl.err_mm
        p_act = tcp_of(q_act)
        js = " ".join(f"J{i+1}={math.degrees(qd[i]):+6.1f}" for i in range(4))
        flag = "OK " if ok else "IK失败(保留上次解)"
        with ctl.lock:
            spd = ctl.rate * 1000
        return (f"目标({p_tgt[0]*1000:+.0f},{p_tgt[1]*1000:+.0f},{p_tgt[2]*1000:+.0f}) "
                f"实际({p_act[0]*1000:+.0f},{p_act[1]*1000:+.0f},{p_act[2]*1000:+.0f}) "
                f"误差{err:5.1f}mm 奇异{singular_margin(qd):.2f} 速度{spd:3.0f}mm/s {flag} {js}")

    if args.selftest:
        base = ctl.p.copy()
        print(f"起点目标 {np.round(base*1000,1)} mm")
        for dx in (0, 20, 40, 60):
            ctl.set_p(base + np.array([dx / 1000.0, 0, 0]))
            for _ in range(1200):          # 1.2 s，让伺服收敛到稳态
                q_act = solve_and_step()
            print(f"  X+{dx:3d}mm -> {status_line(q_act)}")
        ctl.set_p(base + np.array([0, 0, -0.25]))
        for _ in range(1200):
            q_act = solve_and_step()
        print(f"  超范围   -> {status_line(q_act)}")
        return 0

    import mujoco.viewer as mjv
    keys = Keys()
    stop_evt = threading.Event()
    threading.Thread(target=stdin_thread, args=(ctl, stop_evt), daemon=True).start()
    print("命令示例：p 400 0 600 / +x 20 / home / status / quit     （也可按住 W/S/A/D/Q/E）")

    with mjv.launch_passive(model, data) as viewer:
        viewer.cam.distance = 1.7
        viewer.cam.elevation = -15
        viewer.cam.azimuth = 130
        viewer.cam.lookat[:] = np.array([0.15, 0.0, 0.35])
        rate = 0.04
        last_t = time.perf_counter()
        last_print = 0.0
        while viewer.is_running() and not stop_evt.is_set():
            now = time.perf_counter()
            dt = now - last_t
            last_t = now
            az = math.radians(viewer.cam.azimuth)
            fwd = np.array([-math.sin(az), math.cos(az), 0.0])
            right = np.array([math.cos(az), math.sin(az), 0.0])
            with ctl.lock:
                if keys.down("w"): ctl.p += fwd * ctl.rate * dt
                if keys.down("s"): ctl.p -= fwd * ctl.rate * dt
                if keys.down("a"): ctl.p -= right * ctl.rate * dt
                if keys.down("d"): ctl.p += right * ctl.rate * dt
                if keys.down("q"): ctl.p[2] -= ctl.rate * dt
                if keys.down("e"): ctl.p[2] += ctl.rate * dt

            q_act = solve_and_step()

            scn = viewer.user_scn
            scn.ngeom = 0
            # 基座坐标系：X 红 / Y 绿 / Z 蓝，各 120 mm
            for ax, col in ((np.array([1.0,0,0]), (1.0,0.2,0.2,1.0)),
                            (np.array([0,1.0,0]), (0.2,1.0,0.2,1.0)),
                            (np.array([0,0,1.0]), (0.3,0.5,1.0,1.0))):
                add_capsule(mujoco, scn, np.zeros(3), ax*0.12, 0.004, col)
            with ctl.lock:
                _spd = rate
            with ctl.lock:
                p_tgt = ctl.p.copy()
            p_act3 = tcp_of(q_act)
            tgt_rgba = (1.0, 0.15, 0.15, 1.0) if ctl.ok else (1.0, 0.6, 0.0, 1.0)
            # 误差连线：实际末端 -> 目标
            add_capsule(mujoco, scn, p_act3, p_tgt, 0.004, (0.95, 0.85, 0.1, 1.0))
            for pos, rad, rgba in ((p_tgt, 0.015, tgt_rgba),
                                   (p_act3, 0.010, (0.15, 1.0, 0.25, 1.0))):
                if scn.ngeom < len(scn.geoms):
                    g = scn.geoms[scn.ngeom]
                    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE,
                                        np.array([rad, rad, rad]),
                                        np.asarray(pos, dtype=np.float64),
                                        np.eye(3).reshape(-1),
                                        np.array(rgba, dtype=np.float32))
                    scn.ngeom += 1
            viewer.sync()

            if now - last_print > 0.4:
                last_print = now
                print("\r" + status_line(q_act), end="", flush=True)
            time.sleep(model.opt.timestep)
    return 0


if __name__ == "__main__":
    sys.exit(main())
