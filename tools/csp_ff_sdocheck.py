# -*- coding: utf-8 -*-
"""CSP 保持位置、持续发 0x6071=500‰，同时 SDO 回读 0x6071。

判定：
  回读 500 -> 驱动器收到了（说明它收到但不执行，是 CiA402 语义问题）
  回读 0   -> 主站没把 tau 转发进 PDO（主站侧可修）
"""
from __future__ import annotations

import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bench_traj import command_cmd, parse_axis  # noqa: E402
from setpoint_send import encode_record, setpoint_socket_path  # noqa: E402


def main():
    dep = sys.argv[1]
    cmd_sock = f"/tmp/emaster-{dep}.sock"
    tau_cmd = 500

    st = parse_axis(command_cmd(cmd_sock, "status"))
    p0 = int(float(st["pos"]))
    print(f"位置 {p0}，SDO 基线 0x6071 = {command_cmd(cmd_sock, 'sdo_read 1 0x6071 0')}")
    print(f"位置 {p0}，SDO 基线 0x6072 = {command_cmd(cmd_sock, 'sdo_read 1 0x6072 0')}")

    sp = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    sp.connect(setpoint_socket_path(dep))

    seq = 0
    dt = 0.005
    t_end = time.perf_counter() + 3.0
    reads = []
    nxt_read = time.perf_counter() + 0.6
    while time.perf_counter() < t_end:
        seq += 1
        sp.send(encode_record(seq, time.time_ns(), [(p0, 0, tau_cmd, 0, 0)], mode=8))
        now = time.perf_counter()
        if now >= nxt_read:
            r = command_cmd(cmd_sock, "sdo_read 1 0x6071 0")
            pos = parse_axis(command_cmd(cmd_sock, "status")).get("pos")
            reads.append((r, pos))
            nxt_read = now + 0.6
        time.sleep(dt)

    # 卸载
    for _ in range(40):
        seq += 1
        sp.send(encode_record(seq, time.time_ns(), [(p0, 0, 0, 0, 0)], mode=8))
        time.sleep(dt)
    sp.close()

    print(f"\n发送期间 0x6071 回读（命令值 {tau_cmd}）：")
    for r, pos in reads:
        print(f"  {r}   位置={pos}")
    print(f"\n停机后 0x6071 = {command_cmd(cmd_sock, 'sdo_read 1 0x6071 0')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
