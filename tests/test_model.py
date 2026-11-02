# -*- coding: utf-8 -*-
"""模型自检：FK 与 URDF 校验脚本是否一致、重力是否与之前的手算一致、质量矩阵是否正定对称。"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "robot"))
from r4ctl.model import chain, G  # noqa: E402

ch = chain()
print(f"n={ch.n}  links={ch.names}  base_mass={ch.base_mass:.3f} kg")
print("q_min deg:", np.round(np.degrees(ch.q_min), 1))
print("q_max deg:", np.round(np.degrees(ch.q_max), 1))

# 1) 零位末端
p = ch.tool_point(np.zeros(ch.n))
print("零位 tool point =", np.round(p * 1000, 3), "mm  (期望 304 0 530)")

# 2) 各关节轴点（零位，世界 mm）
pp, RR, axes = ch.origin_positions(np.zeros(ch.n))
for i, j in enumerate(ch.movable):
    print(f"  {j['name']}: p={np.round(pp[i+1]*1000,1)} mm  a={axes[i]}")

# 3) 重力力矩（零位）vs 之前的估算法
g0 = ch.gravity(np.zeros(ch.n))
print("零位重力力矩 N·m =", np.round(g0, 3), " (期望 J1~0, J2~31, J3~10)")

# 4) 质量矩阵：对称、正定、对角量级
M = ch.mass_matrix(np.zeros(ch.n))
w = np.linalg.eigvalsh(M)
print("M 对角 =", np.round(np.diag(M), 5), " 特征值>0:", bool((w > 0).all()),
      " 对称误差:", float(np.abs(M - M.T).max()))

# 5) 重力项与势能梯度一致性：-dU/dq
q = np.array([0.3, -0.4, 0.6, 0.2])
eps = 1e-6
num = np.zeros(ch.n)
for i in range(ch.n):
    qp, qm = q.copy(), q.copy()
    qp[i] += eps
    qm[i] -= eps
    Up = sum(ch.link[k + 1]["mass"] * G * (ch.fk(qp)[0][k + 1] @ np.array([0, 0, 1]) +
                                           (ch.fk(qp)[1][k + 1] @ ch.link[k + 1]["com"])[2])
             for k in range(ch.n))
    Um = sum(ch.link[k + 1]["mass"] * G * (ch.fk(qm)[0][k + 1] @ np.array([0, 0, 1]) +
                                           (ch.fk(qm)[1][k + 1] @ ch.link[k + 1]["com"])[2])
             for k in range(ch.n))
    num[i] = (Up - Um) / (2 * eps)
print("dU/dq (数值) =", np.round(num, 4))
print("RNEA g(q)   =", np.round(ch.gravity(q), 4), " 最大偏差:",
      float(np.abs(num - ch.gravity(q)).max()))

# 6) 雅可比与有限差分一致性
q = np.array([0.2, -0.3, 0.5, 0.1])
J = ch.jacobian(q)
Jnum = np.zeros((3, ch.n))
eps = 1e-7
for i in range(ch.n):
    qp, qm = q.copy(), q.copy()
    qp[i] += eps
    qm[i] -= eps
    Jnum[:, i] = (ch.tool_point(qp) - ch.tool_point(qm)) / (2 * eps)
print("雅可比线速度部分最大偏差:", float(np.abs(J[:3] - Jnum).max()))
