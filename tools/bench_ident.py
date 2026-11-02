# -*- coding: utf-8 -*-
"""台架参数辨识采集：一次跑完控制设计需要的四组数据。

空载、低速、小幅，不追求标准验收，只要模型参数：
  1 ramp   恒速正反向扫描  -> 摩擦 τ(v) + 重力矩随位姿变化
  2 steps  双向 0.05° 微步   -> 回差/死区
  3 sine   0.5–8 Hz 小幅正弦 -> 幅值比与相位（带宽）
  4 repeat 同一目标 10 次    -> 重复性（控制能达到的下限）

限幅：|速度| ≤ 10 °/s，|位移| ≤ 15°。
输出 /tmp/ident_<phase>.csv，每行 t_s,pos,planned,torque,state。
"""
from __future__ import annotations

import argparse
import math
import socket
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bench_traj import COUNTS_PER_DEG, command_cmd, parse_axis  # noqa: E402
from setpoint_send import encode_record  # noqa: E402

MAX_VEL = 10.0          # °/s
MAX_TRAVEL = 15.0       # °


class Rig:
    def __init__(self, deployment, hz=200.0):
        self.dep = deployment
        self.cmd_sock = f"/tmp/emaster-{deployment}.sock"
        self.sp_sock = f"/tmp/emaster-{deployment}-sp.sock"
        st = parse_axis(command_cmd(self.cmd_sock, "status"))
        self.p0 = int(float(st["pos"]))
        self.sp = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.sp.connect(self.sp_sock)
        self.hz = hz
        self.seq = 0
        self.rows = []
        self.stop = threading.Event()
        self.t0 = 0.0
        self.th = None
        self.min_pos = self.p0
        self.max_pos = self.p0

    def _poll(self):
        while not self.stop.is_set():
            t = time.perf_counter() - self.t0
            a = parse_axis(command_cmd(self.cmd_sock, "status"))
            try:
                self.rows.append((t, float(a.get("pos")), float(a.get("planned")),
                                  float(a.get("torque")), a.get("state")))
            except (TypeError, ValueError):
                continue

    def start(self):
        self.rows = []
        self.stop.clear()
        self.t0 = time.perf_counter()
        self.th = threading.Thread(target=self._poll, daemon=True)
        self.th.start()

    def send_deg(self, deg, dq_deg=0.0):
        if abs(deg) > MAX_TRAVEL:
            raise RuntimeError(f"位移越限：{deg:.2f}° > {MAX_TRAVEL}°")
        self.seq += 1
        q = int(round(self.p0 + deg * COUNTS_PER_DEG))
        dq = int(round(dq_deg * COUNTS_PER_DEG))
        self.sp.send(encode_record(self.seq, time.time_ns(), [(q, dq, 0, 0, 0)], mode=8))
        self.min_pos = min(self.min_pos, q)
        self.max_pos = max(self.max_pos, q)
        return q

    def sleep_until(self, t):
        d = self.t0 + t - time.perf_counter()
        if d > 0:
            time.sleep(d)

    def finish(self, name):
        time.sleep(0.15)
        self.stop.set()
        if self.th:
            self.th.join(timeout=2.0)
        out = Path(f"/tmp/ident_{name}.csv")
        with open(out, "w", encoding="utf-8") as f:
            f.write("t_s,pos,planned,torque,state\n")
            for r in self.rows:
                f.write(f"{r[0]:.4f},{r[1]:.1f},{r[2]:.1f},{r[3]:.1f},{r[4]}\n")
        span = (self.max_pos - self.min_pos) / COUNTS_PER_DEG
        print(f"[{name}] {len(self.rows)} 行，行程 {span:.2f}°，-> {out}", file=sys.stderr)
        return out


def phase_ramp(rig, speeds=(0.5, 1, 2, 4, 8), travel=6.0):
    """恒速正反向：每档先 +travel 再 -travel，稳态段用于摩擦与重力矩。"""
    rig.start()
    t = 0.0
    q = 0.0
    dt = 1.0 / rig.hz
    for v in speeds:
        for sgn in (+1, -1):
            dur = travel / v
            rig.send_deg(q)                     # 起点先停一拍
            t += 0.2
            rig.sleep_until(t)
            n = int(dur * rig.hz)
            for k in range(n):
                q += sgn * v * dt
                rig.send_deg(q, sgn * v)
                t += dt
                rig.sleep_until(t)
            t += 0.5                             # 每档停顿，便于取稳态
            rig.send_deg(q)
            rig.sleep_until(t)
    return rig.finish("ramp")


def phase_steps(rig, step=0.05, n=30, settle=0.15):
    """双向微步：从当前位往 +1.5° 走，再回去，测死区。"""
    rig.start()
    t = 0.0
    q = 0.0
    rig.send_deg(q); t += 0.3; rig.sleep_until(t)
    for k in range(1, n + 1):
        q = k * step
        rig.send_deg(q); t += settle; rig.sleep_until(t)
    for k in range(n - 1, -1, -1):
        q = k * step - 0.0
        rig.send_deg(q); t += settle; rig.sleep_until(t)
    for k in range(-1, -n - 1, -1):
        q = k * step
        rig.send_deg(q); t += settle; rig.sleep_until(t)
    return rig.finish("steps")


def phase_sine(rig, freqs=(0.5, 1, 2, 4, 8), amp=3.0, cycles=6, dwell=2.0):
    """小幅正弦：每段前先回 0 停 dwell 秒，避免追赶瞬态污染窗口。"""
    rig.start()
    t = 0.0
    marks = []
    for f in freqs:
        rig.send_deg(0.0)
        t += dwell
        rig.sleep_until(t)
        T = 1.0 / f
        n = int(cycles * T * rig.hz)
        marks.append([round(t, 4), round(t + n / rig.hz, 4), f])
        for k in range(n):
            tt = k / rig.hz
            q = amp * math.sin(2 * math.pi * f * tt)
            dq = amp * 2 * math.pi * f * math.cos(2 * math.pi * f * tt)
            rig.send_deg(q, dq)
            t += 1.0 / rig.hz
            rig.sleep_until(t)
    rig.send_deg(0.0)
    import json
    Path("/tmp/ident_sine_marks.json").write_text(json.dumps(marks), encoding="utf-8")
    return rig.finish("sine")


def phase_repeat(rig, travel=3.0, v=2.0, n=10):
    """同一目标往返 n 次：重复性。"""
    rig.start()
    t = 0.0
    dwell = travel / v + 0.5
    for _ in range(n):
        rig.send_deg(travel)
        t += dwell
        rig.sleep_until(t)
        rig.send_deg(0.0)
        t += dwell
        rig.sleep_until(t)
    return rig.finish("repeat")


def main():
    ap = argparse.ArgumentParser(description="台架参数辨识采集（空载低速）")
    ap.add_argument("--deployment", required=True)
    ap.add_argument("--what", default="all",
                    choices=["all", "ramp", "steps", "sine", "repeat"])
    ap.add_argument("--hz", type=float, default=200.0)
    args = ap.parse_args()

    rig = Rig(args.deployment, args.hz)
    print(f"起点 {rig.p0} counts（0°），限速 {MAX_VEL} °/s，限幅 {MAX_TRAVEL}°", file=sys.stderr)
    try:
        if args.what in ("all", "ramp"):
            phase_ramp(rig)
        if args.what in ("all", "steps"):
            phase_steps(rig)
        if args.what in ("all", "sine"):
            phase_sine(rig)
        if args.what in ("all", "repeat"):
            phase_repeat(rig)
    finally:
        try:
            rig.send_deg(0.0)
            time.sleep(0.2)
        except Exception:
            pass
        rig.sp.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
