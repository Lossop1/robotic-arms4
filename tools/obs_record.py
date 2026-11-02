# -*- coding: utf-8 -*-
"""1 kHz 观测口记录器：把主站每拍的实际值 + **目标值** 落成 CSV。

关键：观测帧里有 target_position，这是命令口 status 里拿不到的（那边是瞬时阶梯）。

用法:
  python3 obs_record.py --deployment <ID> --seconds 40 --out /tmp/obs.csv
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

EMASTER = Path(os.environ.get("EMASTER_SRC", "/home/orangepi/ethercat-master"))
sys.path.insert(0, str(EMASTER / "tools"))
import observation_client as oc  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--deployment", required=True)
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--out", default="/tmp/obs.csv")
    ap.add_argument("--poll", type=float, default=0.08)
    args = ap.parse_args()

    sock = oc.connect(f"unix:/tmp/emaster-{args.deployment}-obs.sock", 2.0)
    start = 0
    rows = []
    t_end = time.time() + args.seconds
    while time.time() < t_end:
        first, frames = oc.dump(sock, start, 256)
        for f in frames:
            ax = f["axes"][0]
            rows.append((f["cycle"] * 1e-3,               # 1 kHz -> 秒
                         ax["actual_position"], ax["target_position"],
                         ax["actual_velocity"], ax["actual_torque"],
                         ax["status_word"]))
        if frames:
            start = first + len(frames)
        else:
            time.sleep(args.poll)
    sock.close()

    with open(args.out, "w", encoding="utf-8") as fh:
        fh.write("t_s,pos,target,vel,torque,status\n")
        for r in rows:
            fh.write(f"{r[0]:.4f},{r[1]},{r[2]},{r[3]},{r[4]},{r[5]}\n")
    print(f"[obs] {len(rows)} 帧 -> {args.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
