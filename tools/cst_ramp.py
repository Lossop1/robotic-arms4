# -*- coding: utf-8 -*-
"""CST 力矩斜坡：用缓慢变化的力矩探测齿隙（位置闭环下测不到）。

做法：mode=10（CST），力矩从 0 缓慢升到 +amp，再降到 −amp，再回 0。
轴自由，位置会随力矩"粘滑"式移动；力矩-位置滞回环的宽度就是 lost motion。
带位置守卫：偏离起点超过 limit 度立即归零退出。

用法: python3 cst_ramp.py --deployment <ID> --amp 250 --ramp-s 8 --limit-deg 3
"""
from __future__ import annotations

import argparse
import os
import socket
import sys
import time
from pathlib import Path

EMASTER = Path(os.environ.get("EMASTER_SRC", "/home/orangepi/ethercat-master"))
sys.path.insert(0, str(EMASTER / "tools"))
from setpoint_send import encode_record, setpoint_socket_path  # noqa: E402


def status(cmd_sock):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(1.0)
    try:
        s.connect(cmd_sock)
        s.sendall(b"status\n")
        data = b""
        while not data.endswith(b"\n"):
            ch = s.recv(8192)
            if not ch:
                break
            data += ch
    except OSError:
        return None
    finally:
        s.close()
    for part in data.decode("utf-8", "replace").split("|"):
        if part.startswith("a1:"):
            d = {}
            for kv in part[3:].split(","):
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    d[k] = v
            return d
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--deployment", required=True)
    ap.add_argument("--amp", type=float, default=250.0, help="力矩幅值（‰ 额定）")
    ap.add_argument("--ramp-s", type=float, default=8.0)
    ap.add_argument("--hold-s", type=float, default=1.0)
    ap.add_argument("--limit-deg", type=float, default=3.0)
    ap.add_argument("--hz", type=float, default=200.0)
    ap.add_argument("--mode", type=int, default=10)
    args = ap.parse_args()

    cmd_sock = f"/tmp/emaster-{args.deployment}.sock"
    sp = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    sp.connect(setpoint_socket_path(args.deployment))

    st = status(cmd_sock)
    if not st:
        print("读不到状态", file=sys.stderr)
        return 1
    p0 = int(float(st["pos"]))
    print(f"起点 {p0} counts，力矩 ±{args.amp}‰，斜坡 {args.ramp_s}s/段", file=sys.stderr)

    seq = 0
    dt = 1.0 / args.hz
    t0 = time.perf_counter()
    plan = ([0.0] * 1)  # 占位
    # 序列：(目标力矩, 持续秒数)
    segs = []
    for frac, dur in ((1.0, args.ramp_s), (1.0, args.hold_s),
                      (-2.0, 2 * args.ramp_s), (-1.0, args.hold_s),
                      (1.0, args.ramp_s)):
        segs.append((frac, dur))

    tau_last = 0.0
    try:
        for frac, dur in segs:
            n = int(dur * args.hz)
            start = tau_last
            for k in range(n):
                u = (k + 1) / n
                tau = start + (frac * args.amp - start) * u
                seq += 1
                sp.send(encode_record(seq, time.time_ns(),
                                      [(p0, 0, int(round(tau)), 0, 0)], mode=args.mode))
                tau_last = tau
                if k % 20 == 0:
                    st = status(cmd_sock)
                    if st and "pos" in st:
                        deg = (float(st["pos"]) - p0) / (16384 * 28 / 360.0)
                        if abs(deg) > args.limit_deg:
                            print(f"位置守卫触发：{deg:.2f}°", file=sys.stderr)
                            raise SystemExit(3)
                target = t0 + seq * dt
                d = target - time.perf_counter()
                if d > 0:
                    time.sleep(d)
    finally:
        for _ in range(20):
            seq += 1
            sp.send(encode_record(seq, time.time_ns(), [(p0, 0, 0, 0, 0)], mode=args.mode))
            time.sleep(1.0 / args.hz)
        sp.close()
    st = status(cmd_sock)
    if st and "pos" in st:
        print(f"结束位置 {st['pos']}（位移 {(float(st['pos'])-p0)/(16384*28/360.0):+.3f}°）",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
