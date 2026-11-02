# -*- coding: utf-8 -*-
"""
关节伺服控制律。

四种模式（同一套增益，只是前馈项不同）：
  pd         τ = Kp·e + Kd·ė                                 纯 PD，最容易看到重力下垂
  pd_g       τ = Kp·e + Kd·ė + g(q)                           + 重力前馈
  pd_g_ff    τ = Kp·e + Kd·ė + g(q) + τ_fric(q̇_d)             + 摩擦补偿
  ctc        τ = M(q)·(q̈_d + Kd·ė + Kp·e) + C(q,q̇)q̇ + g(q) + τ_fric
             计算力矩法（逆动力学前馈），跟踪精度最好，代价是每拍要算 M(q) 和 h(q,q̇)

增益按"关节有效惯量 + 目标带宽"整定：
  Kp_i = M_eff_i·ω_i²,  Kd_i = 2·ζ_i·M_eff_i·ω_i
M_eff_i 取可达位形上 M(q)_ii 的最大值（最保守），ω/ζ 见 config.JOINTS。
"""
from __future__ import annotations

import math

import numpy as np

from . import config as C
from .model import chain

TAU_MAX = C.SAFETY["tau_margin"] * C.ACTUATOR["tau_cont"]


class Servo:
    def __init__(self, mode="pd_g_ff", kp=None, kd=None, kp_scale=1.0, kd_scale=1.0):
        self.ch = chain()
        self.mode = mode
        self.M_eff = self.ch.effective_inertia()
        if kp is None or kd is None:
            wn = np.array([2 * math.pi * j["wn_hz"] for j in C.JOINTS[: self.ch.n]])
            zeta = np.array([j["zeta"] for j in C.JOINTS[: self.ch.n]])
            self.kp = self.M_eff * wn ** 2
            self.kd = 2.0 * zeta * self.M_eff * wn
        else:
            self.kp = np.asarray(kp, float)
            self.kd = np.asarray(kd, float)
        self.kp *= kp_scale
        self.kd *= kd_scale
        self.ch = chain()
        self.fc = np.array([f["fc"] for f in C.FRICTION[: self.ch.n]])
        self.fv = np.array([f["fv"] for f in C.FRICTION[: self.ch.n]])
        self.sat_count = 0

    # ----------------------------------------------------------------- 摩擦前馈
    def friction_ff(self, qd_des):
        eps = 1e-3
        return self.fc * np.tanh(qd_des / eps) + self.fv * qd_des

    # ----------------------------------------------------------------- 主控
    def compute(self, q, qd, q_des, qd_des, qdd_des=None, gravity=True):
        q = np.asarray(q, float)
        qd = np.asarray(qd, float)
        q_des = np.asarray(q_des, float)
        qd_des = np.asarray(qd_des, float)
        qdd_des = np.zeros(self.ch.n) if qdd_des is None else np.asarray(qdd_des, float)

        e = q_des - q
        ed = qd_des - qd
        g = self.ch.gravity(q) if gravity else np.zeros(self.ch.n)
        tf = self.friction_ff(qd_des) if C.CTL["friction_ff"] else np.zeros(self.ch.n)

        if self.mode == "pd":
            tau = self.kp * e + self.kd * ed
        elif self.mode == "pd_g":
            tau = self.kp * e + self.kd * ed + g
        elif self.mode == "pd_g_ff":
            tau = self.kp * e + self.kd * ed + g + tf
        elif self.mode == "ctc":
            M = self.ch.mass_matrix(q)
            h = self.ch.bias(q, qd)
            acc = qdd_des + self.kd * ed + self.kp * e
            tau = M @ acc + h + tf
        else:
            raise ValueError(f"未知控制模式 {self.mode}")

        sat = np.abs(tau) > TAU_MAX
        if sat.any():
            self.sat_count += 1
        return np.clip(tau, -TAU_MAX, TAU_MAX)

    # ----------------------------------------------------------------- 诊断
    def bandwidth(self):
        wn = np.sqrt(self.kp / np.maximum(self.M_eff, 1e-12))
        return wn / (2 * math.pi)

    def report(self):
        lines = [f"控制模式: {self.mode}",
                 f"{'关节':6s}{'M_eff(kg·m²)':>14s}{'Kp(N·m/rad)':>14s}{'Kd(N·m·s/rad)':>16s}"
                 f"{'带宽(Hz)':>10s}"]
        for i, j in enumerate(C.JOINTS[: self.ch.n]):
            lines.append(f"{j['name']:6s}{self.M_eff[i]:14.5f}{self.kp[i]:14.1f}"
                         f"{self.kd[i]:16.2f}{self.bandwidth()[i]:10.1f}")
        return "\n".join(lines)
