# -*- coding: utf-8 -*-
"""逆解自检：
1) 随机位形 FK -> IK -> FK 位置必须复现（严格）
2) 若该位形远离限位，则要求角度也复现（分支不跳）
3) 工作空间边界：合理
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "robot"))
from r4ctl import ik as IK  # noqa: E402
from r4ctl.model import chain  # noqa: E402

ch = chain()
rng = np.random.default_rng(7)

n_test = n_pos_ok = n_ang_ok = n_fail = 0
worst_pos = 0.0
flips = 0
fails = []

for _ in range(5000):
    q = rng.uniform(ch.q_min, ch.q_max)
    if IK.singular_margin(q) > 0.995:
        continue
    p = IK.fk(q)
    res = IK.ik(p, roll=q[3], q_ref=q)
    n_test += 1
    if not res.ok:
        n_fail += 1
        fails.append(("not_ok", np.degrees(q).round(1), res.reason))
        continue
    e_pos = float(np.linalg.norm(IK.fk(res.q) - p))
    worst_pos = max(worst_pos, e_pos)
    if e_pos < 1e-9:
        n_pos_ok += 1
    else:
        fails.append(("pos_err", np.degrees(q).round(1), e_pos))

    d = np.abs(((res.q - q + math.pi) % (2 * math.pi)) - math.pi)
    if np.all(q > ch.q_min + 0.20) and np.all(q < ch.q_max - 0.20):
        if d.max() < 1e-9:
            n_ang_ok += 1
        else:
            flips += 1

print(f"随机位形 {n_test} 个：位置复现 {n_pos_ok}，未通过 {n_fail}，最大位置误差 {worst_pos:.2e} m")
print(f"远离限位的位形中：角度完全复现 {n_ang_ok}，走了等价分支 {flips}")
for f in fails[:5]:
    print("   ", f)

print("\n工作空间边界（目标在 z = J2 高度上）:")
for r_mm in (30, 44, 60, 300, 560, 564, 580):
    p = np.array([r_mm / 1000.0, 0.0, IK.Z_J2])
    res = IK.ik(p, 0.0, q_ref=np.array([0.0, 0.7, 0.0, 0.0]))
    tag = "ok" if res.ok else "X"
    print(f"  r={r_mm:4d} mm [{tag}]  q={np.round(np.degrees(res.q),1)}  {res.reason}")

# 连续性：沿一条水平直线逐点逆解，相邻解跳变应很小
q_prev = np.array([0.0, 0.9, -0.2, 0.0])
max_jump = 0.0
first = True
for s in np.linspace(0, 1, 400):
    p = np.array([0.30 + 0.10 * s, 0.0, 0.30 + 0.05 * s])
    res = IK.ik(p, 0.0, q_ref=q_prev)
    if first:                      # 第一点的跳变来自初始猜测，不算连续性
        first = False
    else:
        max_jump = max(max_jump, float(np.abs(res.q - q_prev).max()))
    q_prev = res.q
print(f"\n直线追踪 400 点：相邻解最大跳变 {np.degrees(max_jump):.3f} deg（应 << 5 deg）")
