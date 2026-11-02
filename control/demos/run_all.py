# -*- coding: utf-8 -*-
"""
一键跑完全部 demo + 自检，汇总成 out/report.json 并在终端打印指标表。

    python demos/run_all.py
"""
from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out"
sys.path.insert(0, str(ROOT))


def run_script(path):
    t0 = time.time()
    p = subprocess.run([sys.executable, str(path)], cwd=str(path.parent),
                       capture_output=True, text=True, encoding="utf-8")
    if p.returncode != 0:
        print(p.stdout)
        print(p.stderr)
        raise SystemExit(f"{path.name} 失败")
    print(p.stdout.rstrip())
    print(f"  ({time.time()-t0:.1f}s)\n")
    return p.stdout


def main():
    OUT.mkdir(exist_ok=True)
    print("=" * 78)
    print("R4 四轴机械臂 控制算法自检 + Demo")
    print("=" * 78)

    print("\n【自检】模型 / 逆解 / 轨迹")
    for f in ("tests/test_model.py", "tests/test_ik.py", "tests/test_traj.py"):
        run_script(ROOT / f)

    print("\n【Demo】")
    texts = {}
    for f in ("demos/demo1_servo.py", "demos/demo2_cartesian.py", "demos/demo3_gravity.py"):
        texts[f] = run_script(ROOT / f)

    (OUT / "demo_output.txt").write_text("\n\n".join(texts.values()), encoding="utf-8")
    print(f"最后更新: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"图与日志: {OUT}")


if __name__ == "__main__":
    main()
