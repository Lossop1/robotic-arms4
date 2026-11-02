# -*- coding: utf-8 -*-
"""CSP 位置保持下阶梯施加 0x6071（力矩前馈），看位置是否偏移。

判据：若 0x6071 生效，位置偏移 ≈ T_ff / K_eff（尺度无关）；
      若被忽略，位置不动。

安全：CSP 位置模式，目标固定，偏差超 limit 立即归零。
用法: python3 csp_torque_ff.py --deployment <ID>
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bench_traj import COUNTS_PER_DEG, command_cmd, parse_axis  # noqa: E402
from setpoint_send import encode_record, setpoint_socket_path  # noqa: E402

import socket  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--deployment", required=True)
    ap.add_argument("--levels", default="100,200,300,400,500")
    ap.add_argument("--hold-s", type=float, default=1.0)
    ap.add_argument("--hz", type=float, default=200.0)
    ap.add_argument("--limit-deg", type=float, default=2.0)
    ap.add_argument("--out", default="/tmp/torque_ff.csv")
    args = ap.parse_args()

    cmd_sock = f"/tmp/emaster-{args.deployment}.sock"
    st0 = parse_axis(command_cmd(cmd_sock, "status"))
    if "pos" not in st0:
        print(f"读不到状态: {st0!r}", file=sys.stderr)
        return 1
    p0 = int(float(st0["pos"]))
    print(f"目标位置锁定 {p0} counts（CSP 保持）", file=sys.stderr)

    sp = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    sp.connect(setpoint_socket_path(args.deployment))

    rows = []
    seq = 0
    dt = 1.0 / args.hz
    t0 = time.perf_counter()

    def pump(tau, seconds):
        nonlocal seq
        n = int(seconds * args.hz)
        for k in range(n):
            seq += 1
            sp.send(encode_record(seq, time.time_ns(), [(p0, 0, int(tau), 0, 0)], mode=8))
            if k % 10 == 0:
                st = parse_axis(command_cmd(cmd_sock, "status"))
                try:
                    pos = float(st["pos"])
                    tau_act = float(st["torque"])
                except (KeyError, TypeError, ValueError):
                    pos, tau_act = float("nan"), float("nan")
                t = time.perf_counter() - t0
                rows.append((t, tau, pos, tau_act, (pos - p0) / COUNTS_PER_DEG))
                if pos == pos and abs(pos - p0) / COUNTS_PER_DEG > args.limit_deg:
                    print(f"位置越限，归零：{(pos-p0)/COUNTS_PER_DEG:.2f}°", file=sys.stderr)
                    return False
            target = t0 + seq * dt
            d = target - time.perf_counter()
            if d > 0:
                time.sleep(d)
        return True

    ok = True
    try:
        pump(0, 1.0)                       # 基线
        for lv in [int(x) for x in args.levels.split(",")]:
            print(f"+{lv}‰ ...", file=sys.stderr)
            if not pump(lv, args.hold_s):
                ok = False
                break
        pump(0, 1.5)                       # 卸载，看是否回位
    finally:
        for _ in range(30):
            seq += 1
            sp.send(encode_record(seq, time.time_ns(), [(p0, 0, 0, 0, 0)], mode=8))
            time.sleep(1.0 / args.hz)
        sp.close()

    with open(args.out, "w", encoding="utf-8") as f:
        f.write("t_s,tau_cmd,pos,actual_torque,offset_deg\n")
        for r in rows:
            f.write(f"{r[0]:.4f},{r[1]},{r[2]:.0f},{r[3]:.0f},{r[4]:.4f}\n")

    print("\n力矩指令  位置偏移(°)  实际力矩(‰)")
    for lv in [int(x) for x in args.levels.split(",")] + [0]:
        seg = [r for r in rows if r[1] == lv]
        if len(seg) < 3:
            continue
        tail = seg[len(seg) * 2 // 3:]
        off = sum(x[4] for x in tail if x[4] == x[4]) / max(1, len(tail))
        tq = sum(x[3] for x in tail if x[3] == x[3]) / max(1, len(tail))
        print(f"  {lv:+5d}‰     {off:+7.3f}     {tq:+7.0f}")
    print(f"\n数据 -> {args.out}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
