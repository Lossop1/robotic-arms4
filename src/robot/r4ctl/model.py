# -*- coding: utf-8 -*-
"""
刚体模型：从 URDF 读链参数，提供 FK / 雅可比 / RNEA / 质量矩阵 / 正动力学。

单位：SI（m, kg, s, rad）。本项目所有连杆坐标系零位与装配系同姿态（纯平移），
URDF 里 rpy 全 0；代码仍按通用 origin(rpy) 处理，换模型不用改。

动力学两套实现：
  rnea(q, qd, qdd)     纯浮点世界系 Newton-Euler（~35 us/次），1 kHz 环里可用
  rnea_np(...)         numpy 版参考实现（~800 us/次），仅用于交叉校验
两者在同一模型上必须一致到 1e-9 量级，tests/test_model.py 会查。

关节坐标系原点都落在关节轴上（URDF 就是这么建的），所以 τ_j = a_j · (关于该轴点的力矩和)。
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from lxml import etree

from . import config as C

G = 9.80665


# --------------------------------------------------------------------- 小工具
def rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rot_axis(axis, ang):
    a = np.asarray(axis, float)
    n = np.linalg.norm(a)
    if n == 0:
        return np.eye(3)
    a = a / n
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


def _mm(A, B):
    """9 元组矩阵乘（行主序）"""
    a0, a1, a2, a3, a4, a5, a6, a7, a8 = A
    b0, b1, b2, b3, b4, b5, b6, b7, b8 = B
    return (a0 * b0 + a1 * b3 + a2 * b6, a0 * b1 + a1 * b4 + a2 * b7, a0 * b2 + a1 * b5 + a2 * b8,
            a3 * b0 + a4 * b3 + a5 * b6, a3 * b1 + a4 * b4 + a5 * b7, a3 * b2 + a4 * b5 + a5 * b8,
            a6 * b0 + a7 * b3 + a8 * b6, a6 * b1 + a7 * b4 + a8 * b7, a6 * b2 + a7 * b5 + a8 * b8)


def _mv(A, v):
    a0, a1, a2, a3, a4, a5, a6, a7, a8 = A
    x, y, z = v
    return (a0 * x + a1 * y + a2 * z, a3 * x + a4 * y + a5 * z, a6 * x + a7 * y + a8 * z)


def _mt(A):
    a0, a1, a2, a3, a4, a5, a6, a7, a8 = A
    return (a0, a3, a6, a1, a4, a7, a2, a5, a8)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _rod(axis, ang):
    """Rodrigues，axis 需已归一化"""
    ux, uy, uz = axis
    c = math.cos(ang)
    s = math.sin(ang)
    k = 1.0 - c
    return (c + ux * ux * k, ux * uy * k - uz * s, ux * uz * k + uy * s,
            uy * ux * k + uz * s, c + uy * uy * k, uy * uz * k - ux * s,
            uz * ux * k - uy * s, uz * uy * k + ux * s, c + uz * uz * k)


IDENT = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)


# --------------------------------------------------------------------- 链
class Chain:
    """串行链：links[0]=base, links[i]=第 i 个运动连杆"""

    def __init__(self, urdf: Path | None = None):
        urdf = Path(urdf or C.URDF)
        robot = etree.parse(str(urdf)).getroot()

        links = {}
        for e in robot.findall("link"):
            ine = e.find("inertial")
            if ine is None:
                links[e.get("name")] = dict(mass=0.0, com=np.zeros(3), I=np.zeros((3, 3)))
                continue
            o = ine.find("origin")
            xyz = np.array([float(v) for v in o.get("xyz").split()]) if o is not None else np.zeros(3)
            rpy = [float(v) for v in o.get("rpy").split()] if o is not None else [0, 0, 0]
            m = [[float(ine.find("inertia").get(k)) for k in ("ixx", "ixy", "ixz")],
                 [float(ine.find("inertia").get("ixy")), float(ine.find("inertia").get("iyy")),
                  float(ine.find("inertia").get("iyz"))],
                 [float(ine.find("inertia").get("ixz")), float(ine.find("inertia").get("iyz")),
                  float(ine.find("inertia").get("izz"))]]
            links[e.get("name")] = dict(mass=float(ine.find("mass").get("value")),
                                        com=rpy_to_R(*rpy) @ xyz,
                                        I=np.array(m, float))

        joints = []
        for e in robot.findall("joint"):
            o = e.find("origin")
            ax = e.find("axis")
            joints.append(dict(
                name=e.get("name"), type=e.get("type"),
                parent=e.find("parent").get("link"), child=e.find("child").get("link"),
                xyz=np.array([float(v) for v in o.get("xyz").split()]) if o is not None else np.zeros(3),
                R0=rpy_to_R(*[float(v) for v in o.get("rpy").split()]) if o is not None else np.eye(3),
                axis=np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None
                else np.array([0, 0, 1.0]),
            ))

        by_parent = {}
        for j in joints:
            by_parent.setdefault(j["parent"], []).append(j)
        roots = [n for n in links if n not in {j["child"] for j in joints}]
        cur = roots[0]
        self.names = [cur]
        self.joints = []
        while cur in by_parent:
            j = by_parent[cur][0]
            self.joints.append(j)
            cur = j["child"]
            self.names.append(cur)

        self.link = [links[n] for n in self.names]
        self.movable = [j for j in self.joints if j["type"] != "fixed"]
        self.n = len(self.movable)
        if any(j["type"] == "fixed" for j in self.joints[: self.n]):
            raise SystemExit("fixed 关节必须都在链尾（当前实现假定如此）")
        self.base_mass = links[self.names[0]]["mass"]

        self.q_min = np.radians([j["q_min"] for j in C.JOINTS[: self.n]])
        self.q_max = np.radians([j["q_max"] for j in C.JOINTS[: self.n]])
        self.v_max = np.array([j["v_max"] for j in C.JOINTS[: self.n]])
        self.a_max = np.array([j["a_max"] for j in C.JOINTS[: self.n]])
        self.j_max = np.array([j["j_max"] for j in C.JOINTS[: self.n]])
        self.parent_idx = [-1]
        for k in range(1, len(self.joints)):
            self.parent_idx.append(k - 1)

        # ---- 预编译成扁平元组，供热路径使用
        self._J = []
        for j in self.joints:
            a = np.asarray(j["axis"], float)
            a = a / max(np.linalg.norm(a), 1e-12)
            R0 = j["R0"].ravel()
            self._J.append((tuple(float(v) for v in R0), tuple(float(v) for v in j["xyz"]),
                            tuple(float(v) for v in a), j["type"] == "fixed"))
        self._L = []
        for i in range(1, self.n + 1):
            L = self.link[i]
            self._L.append((float(L["mass"]), tuple(float(v) for v in L["com"]),
                            tuple(float(v) for v in L["I"].ravel())))

    # ----------------------------------------------------------------- FK
    def fk(self, q):
        """返回 (p, R)：每个关节之后的连杆坐标系（长度 n_all+1，含 base）。"""
        q = np.asarray(q, float)
        ps = [np.zeros(3)]
        Rs = [np.eye(3)]
        p = np.zeros(3)
        R = np.eye(3)
        k = 0
        for j in self.joints:
            Rj = R @ j["R0"]
            p = p + R @ j["xyz"]
            if j["type"] != "fixed":
                R = Rj @ rot_axis(j["axis"], q[k])
                k += 1
            else:
                R = Rj
            ps.append(p.copy())
            Rs.append(R.copy())
        return np.array(ps), np.array(Rs)

    def origin_positions(self, q):
        """(p, R, axes)：axes 为各运动关节在世界系的轴向量。"""
        q = np.asarray(q, float)
        ps, Rs = self.fk(q)
        axes = [Rs[i + 1] @ self.movable[i]["axis"] for i in range(self.n)]
        return ps, Rs, np.array(axes)

    def tool_point(self, q):
        return self.fk(q)[0][-1]

    def tool_frame(self, q):
        p, R = self.fk(q)
        return p[-1], R[-1]

    # ----------------------------------------------------------------- 雅可比
    def jacobian(self, q, point=None):
        p, R, axes = self.origin_positions(q)
        if point is None:
            point = p[-1]
        J = np.zeros((6, self.n))
        for i in range(self.n):
            J[:3, i] = np.cross(axes[i], point - p[i + 1])
            J[3:, i] = axes[i]
        return J

    # ----------------------------------------------------------------- 动力学
    def rnea(self, q, qd=None, qdd=None, gravity=True):
        """世界系 Newton-Euler（纯浮点，~35 us/次）。qdd 全 0 且 qd 为 0 时即 g(q)。"""
        q = q if isinstance(q, (list, tuple)) else q.tolist()
        qd = (np.zeros(self.n) if qd is None else qd)
        qd = qd.tolist() if isinstance(qd, np.ndarray) else list(qd)
        qdd = (np.zeros(self.n) if qdd is None else qdd)
        qdd = qdd.tolist() if isinstance(qdd, np.ndarray) else list(qdd)
        n = self.n
        JJ = self._J
        LL = self._L

        R = IDENT
        p = (0.0, 0.0, 0.0)
        w = (0.0, 0.0, 0.0)
        al = (0.0, 0.0, 0.0)
        ac = (0.0, 0.0, G) if gravity else (0.0, 0.0, 0.0)

        orig = [(0.0, 0.0, 0.0)] * (len(JJ) + 1)
        axw = [(0.0, 0.0, 0.0)] * n
        F = [(0.0, 0.0, 0.0)] * n
        Nt = [(0.0, 0.0, 0.0)] * n
        comw = [(0.0, 0.0, 0.0)] * n

        k = 0
        for i, (R0, xyz, axL, fixed) in enumerate(JJ):
            Rj = _mm(R, R0)
            p_new = _add(p, _mv(R, xyz))
            if fixed:
                R, p = Rj, p_new
                orig[i + 1] = p
                continue
            qi = q[k]
            R = _mm(Rj, _rod(axL, qi))
            aw = _mv(Rj, axL)
            axw[k] = aw
            orig[i + 1] = p_new
            r = _sub(p_new, p)
            # a_i = a + alpha×r + w×(w×r)
            a_i = _add(ac, _add(_cross(al, r), _cross(w, _cross(w, r))))
            w_i = _add(w, _mul(aw, qd[k]))
            al_i = _add(al, _add(_mul(aw, qdd[k]), _cross(w, _mul(aw, qd[k]))))
            m, com, Il = LL[k]
            c = _mv(R, com)
            a_com = _add(a_i, _add(_cross(al_i, c), _cross(w_i, _cross(w_i, c))))
            IW = _mm(_mm(R, Il), _mt(R))
            F[k] = _mul(a_com, m)
            Nt[k] = _add(_mv(IW, al_i), _cross(w_i, _mv(IW, w_i)))
            comw[k] = _add(p_new, c)
            w, al, ac, p = w_i, al_i, a_i, p_new
            k += 1

        tau = [0.0] * n
        for j in range(n):
            pj = orig[j + 1]
            nx = ny = nz = 0.0
            for i in range(j, n):
                rx, ry, rz = comw[i][0] - pj[0], comw[i][1] - pj[1], comw[i][2] - pj[2]
                fx, fy, fz = F[i]
                nx += Nt[i][0] + (ry * fz - rz * fy)
                ny += Nt[i][1] + (rz * fx - rx * fz)
                nz += Nt[i][2] + (rx * fy - ry * fx)
            tau[j] = axw[j][0] * nx + axw[j][1] * ny + axw[j][2] * nz
        return np.array(tau)

    def rnea_np(self, q, qd=None, qdd=None, gravity=True):
        """numpy 参考实现（交叉校验用）"""
        q = np.asarray(q, float)
        qd = np.zeros(self.n) if qd is None else np.asarray(qd, float)
        qdd = np.zeros(self.n) if qdd is None else np.asarray(qdd, float)

        p, R, axes = self.origin_positions(q)
        n = self.n
        omega = np.zeros(3)
        alpha = np.zeros(3)
        a = np.array([0.0, 0.0, G]) if gravity else np.zeros(3)
        p_prev = p[0]
        F = np.zeros((n, 3))
        N = np.zeros((n, 3))
        comW = np.zeros((n, 3))

        for i in range(n):
            r = p[i + 1] - p_prev
            a_i = a + np.cross(alpha, r) + np.cross(omega, np.cross(omega, r))
            omega_i = omega + qd[i] * axes[i]
            alpha_i = alpha + qdd[i] * axes[i] + np.cross(omega, qd[i] * axes[i])
            L = self.link[i + 1]
            R_i = R[i + 1]
            c = R_i @ L["com"]
            a_com = a_i + np.cross(alpha_i, c) + np.cross(omega_i, np.cross(omega_i, c))
            IW = R_i @ L["I"] @ R_i.T
            F[i] = L["mass"] * a_com
            N[i] = IW @ alpha_i + np.cross(omega_i, IW @ omega_i)
            comW[i] = p[i + 1] + c
            omega, alpha, a, p_prev = omega_i, alpha_i, a_i, p[i + 1]

        tau = np.zeros(n)
        for j in range(n):
            pj = p[j + 1]
            n_j = np.zeros(3)
            for i in range(j, n):
                n_j += N[i] + np.cross(comW[i] - pj, F[i])
            tau[j] = axes[j] @ n_j
        return tau

    def gravity(self, q):
        return self.rnea(q)

    def mass_matrix(self, q):
        n = self.n
        z = np.zeros(n)
        M = np.zeros((n, n))
        for j in range(n):
            e = np.zeros(n)
            e[j] = 1.0
            M[:, j] = self.rnea(q, z, e, gravity=False)
        return 0.5 * (M + M.T)

    def bias(self, q, qd):
        return self.rnea(q, qd, np.zeros(self.n))

    def forward_dynamics(self, q, qd, tau):
        M = self.mass_matrix(q)
        h = self.bias(q, qd)
        return np.linalg.solve(M, np.asarray(tau, float) - h)

    # ----------------------------------------------------------------- 惯量估计
    def effective_inertia(self, n_samples=40, seed=0):
        """关节有效惯量 M(q)_ii 在可达位形上的最大值，用于增益整定。"""
        rng = np.random.default_rng(seed)
        lo = self.q_min
        hi = self.q_max
        best = np.zeros(self.n)
        samples = [0.5 * (lo + hi)] + [rng.uniform(lo, hi) for _ in range(n_samples)]
        for q in samples:
            M = self.mass_matrix(q)
            best = np.maximum(best, np.diag(M))
        return best


_CHAIN = None


def chain(urdf: Path | None = None) -> Chain:
    global _CHAIN
    if _CHAIN is None:
        _CHAIN = Chain(urdf)
    return _CHAIN
