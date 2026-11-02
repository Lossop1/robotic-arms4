# -*- coding: utf-8 -*-
"""台架轨迹客户端：设点口发 S 曲线位置（可选速度/力矩前馈），同时独立线程采样状态。

在香橙派上跑。线格式借用主站自带的 tools/setpoint_send.py，保证只有一份实现。

命令口是"一次连接一条命令"，所以采样线程每次新建连接；发送线程只走设点口，
两者互不阻塞。

用法:
  python3 bench_traj.py --deployment <ID> --deg 10 --duration-ms 2000 --hz 200 [--vel-ff]
"""
from __future__ import annotations

import argparse
import os
import socket
import sys
import threading
import time
from pathlib import Path

EMASTER = Path(os.environ.get("EMASTER_SRC", "/home/orangepi/ethercat-master"))
sys.path.insert(0, str(EMASTER / "tools"))
from setpoint_send import encode_record, setpoint_socket_path  # noqa: E402

COUNTS_PER_DEG = 16384 * 28 / 360.0


def command_cmd(sock_path, cmd, timeout=1.0):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect(sock_path)
        s.sendall((cmd + "\n").encode())
        data = b""
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                ch = s.recv(8192)
            except socket.timeout:
                break
            if not ch:
                break
            data += ch
            if data.endswith(b"\n"):
                break
        return data.decode("utf-8", "replace").strip()
    except OSError:
        return ""
    finally:
        s.close()


def parse_axis(line):
    for part in line.split("|"):
        if part.startswith("a1:"):
            out = {}
            for kv in part[3:].split(","):
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    out[k] = v
            return out
    return {}


def scurve(u):
    return (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5,
            30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4)


def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def main():
    ap = argparse.ArgumentParser(description="台架 S 曲线轨迹客户端")
    ap.add_argument("--deployment", required=True)
    ap.add_argument("--deg", type=float, default=10.0)
    ap.add_argument("--duration-ms", type=int, default=2000)
    ap.add_argument("--hold-ms", type=int, default=600)
    ap.add_argument("--hz", type=float, default=200.0)
    ap.add_argument("--vel-ff", action="store_true")
    ap.add_argument("--tau-ff-permille", type=float, default=0.0)
    ap.add_argument("--mode", type=int, default=8)
    ap.add_argument("--max-err-deg", type=float, default=5.0)
    ap.add_argument("--out", default="/tmp/bench-traj.csv")
    args = ap.parse_args()

    cmd_sock = f"/tmp/emaster-{args.deployment}.sock"
    sp_sock = setpoint_socket_path(args.deployment)
    if not Path(sp_sock).exists():
        print(f"设点口不存在：{sp_sock}（主站要以 EMASTER_SETPOINT=1 启动）", file=sys.stderr)
        return 2

    st0 = parse_axis(command_cmd(cmd_sock, "status"))
    if "pos" not in st0:
        print(f"读不到位置：{st0!r}", file=sys.stderr)
        return 1
    p0 = int(float(st0["pos"]))
    total = args.deg * COUNTS_PER_DEG
    print(f"起点 {p0}，目标 {p0 + total:.0f}，{args.duration_ms} ms，{args.hz:.0f} Hz，"
          f"速度前馈={args.vel_ff}", file=sys.stderr)

    T = args.duration_ms / 1000.0
    n_move = int(round(T * args.hz))
    n_hold = int(round(args.hold_ms / 1000.0 * args.hz))
    dt = 1.0 / args.hz

    rows = []
    stop = threading.Event()

    def poller():
        while not stop.is_set():
            t = time.perf_counter() - t0
            a = parse_axis(command_cmd(cmd_sock, "status"))
            rows.append((t, num(a.get("pos")), num(a.get("planned")),
                         num(a.get("torque")), num(a.get("err")),
                         a.get("state"), a.get("mode")))

    sp = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    sp.connect(sp_sock)
    t0 = time.perf_counter()
    th = threading.Thread(target=poller, daemon=True)
    th.start()

    aborted = ""
    for k in range(n_move + n_hold):
        if k < n_move:
            u = min(1.0, k / max(1, n_move))
            s, ds = scurve(u)
            q = p0 + total * s
            dq = total * ds / T if args.vel_ff else 0.0
        else:
            q, dq = p0 + total, 0.0
        sp.send(encode_record(k + 1, time.time_ns(),
                              [(int(round(q)), int(dq), int(args.tau_ff_permille), 0, 0)],
                              mode=args.mode))
        target_end = t0 + (k + 1) * dt
        while time.perf_counter() < target_end:
            time.sleep(0.0005)

        if rows:
            _, pos, plan, _, _, _, _ = rows[-1]
            if pos == pos and abs(pos - p0) > abs(total) * 1.5 + 3000:
                aborted = f"越界保护 pos={pos:.0f}"
                break
            if pos == pos and plan == plan and abs(pos - plan) > args.max_err_deg * COUNTS_PER_DEG:
                aborted = f"跟随误差 {abs(pos - plan) / COUNTS_PER_DEG:.2f}°"
                break

    time.sleep(0.15)
    stop.set()
    th.join(timeout=2.0)
    sp.close()

    with open(args.out, "w", encoding="utf-8") as f:
        f.write("t_s,pos,planned,torque,err,state,mode\n")
        for r in rows:
            f.write(",".join(f"{v:.4f}" if isinstance(v, float) else str(v) for v in r) + "\n")

    good = [r for r in rows if r[1] == r[1]]
    print(f"采 {len(rows)} 行（有效 {len(good)}）-> {args.out}", file=sys.stderr)
    if len(good) >= 2:
        print(f"位移 {(good[-1][1] - p0) / COUNTS_PER_DEG:.3f}°（目标 {args.deg:.3f}°），"
              f"轮询 {len(good) / max(0.1, good[-1][0]):.0f} Hz", file=sys.stderr)
    if aborted:
        print("中止：" + aborted, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
