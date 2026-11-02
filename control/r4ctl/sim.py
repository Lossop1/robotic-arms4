# -*- coding: utf-8 -*-
"""
被控对象仿真：刚体动力学（RNEA）+ 关节摩擦 + 力矩饱和 + 1 拍传输延迟 +
编码器量化 + 速度估计低通。

积分用半隐式欧拉（辛），每个控制周期把 M(q) 冻结、按 substeps 细分，
这样 1 kHz 控制 + 4 kHz 积分在本机上约 0.25 s/仿真秒。
"""
from __future__ import annotations

import math

import numpy as np

from . import config as C
from .model import chain

ENC_RES = 2 * math.pi / (2 ** 19)      # 19 bit 单圈绝对值编码器


class Sim:
    def __init__(self, dt=None, substeps=None, delay_steps=None,
                 friction=True, quantize=True, vel_filter_alpha=0.2):
        self.ch = chain()
        self.dt = 1.0 / C.CTL["rate_hz"] if dt is None else dt
        self.substeps = C.CTL["sub_steps"] if substeps is None else substeps
        self.delay_steps = (max(0, int(round(C.CTL["cmd_delay_ms"] / 1000.0 / self.dt)))
                            if delay_steps is None else delay_steps)
        self.friction = friction
        self.quantize = quantize
        self.alpha = vel_filter_alpha
        self.fc = np.array([f["fc"] for f in C.FRICTION[: self.ch.n]])
        self.fv = np.array([f["fv"] for f in C.FRICTION[: self.ch.n]])

    # ----------------------------------------------------------------- 摩擦
    def _friction_torque(self, qd):
        if not self.friction:
            return np.zeros(self.ch.n)
        return self.fc * np.tanh(qd / 2e-3) + self.fv * qd

    # ----------------------------------------------------------------- 主循环
    def run(self, ref, ctrl, safety=None, T=6.0, q0=None, disturbance=None, label=""):
        ch = self.ch
        n = ch.n
        q = np.zeros(n) if q0 is None else np.asarray(q0, float).copy()
        qd = np.zeros(n)

        steps = int(round(T / self.dt))
        log = dict(t=np.zeros(steps), q=np.zeros((steps, n)), qd=np.zeros((steps, n)),
                   q_des=np.zeros((steps, n)), qd_des=np.zeros((steps, n)),
                   tau=np.zeros((steps, n)), tip=np.zeros((steps, 3)),
                   q_meas=np.zeros((steps, n)))

        q_m, qd_m = q.copy(), qd.copy()
        q_m_prev = q.copy()
        tau_hist = [np.zeros(n) for _ in range(self.delay_steps + 1)]
        hold = q.copy()
        latched = False

        for k in range(steps):
            t = k * self.dt
            q_des, qd_des, qdd_des = ref(t)

            if latched:
                q_des, qd_des, qdd_des = hold, np.zeros(n), np.zeros(n)

            tau = ctrl.compute(q_m, qd_m, q_des, qd_des, qdd_des)

            if safety is not None:
                tau, now_latched, _ = safety.check(q_m, qd_m, q_des, tau, t)
                if now_latched and not latched:
                    latched = True
                    hold = q_m.copy()

            tau_cmd = tau_hist.pop(0)
            tau_hist.append(tau)

            # ---- 被控对象
            M = ch.mass_matrix(q)
            Minv = np.linalg.inv(M)
            h = self.dt / self.substeps
            ext = disturbance(t) if disturbance else np.zeros(n)
            for _ in range(self.substeps):
                tau_L = self._friction_torque(qd)
                qdd = Minv @ (tau_cmd - tau_L + ext - ch.bias(q, qd))
                qd = qd + qdd * h
                q = q + qd * h

            # ---- 测量（编码器量化 + 差分速度低通）
            qd_raw = (q - q_m) / self.dt
            q_m_prev = q_m
            q_m = (np.round(q / ENC_RES) * ENC_RES) if self.quantize else q.copy()
            qd_m = (1 - self.alpha) * qd_raw + self.alpha * qd_m

            log["t"][k] = t
            log["q"][k] = q
            log["qd"][k] = qd
            log["q_meas"][k] = q_m
            log["q_des"][k] = q_des
            log["qd_des"][k] = qd_des
            log["tau"][k] = tau_cmd
            log["tip"][k] = ch.tool_point(q)

        log["label"] = label
        log["latched"] = latched
        return log


# --------------------------------------------------------------------- 指标
def metrics(log, t_from=0.0):
    t = log["t"]
    m = t >= t_from
    e = log["q"][m] - log["q_des"][m]
    return dict(
        rms_err_deg=float(np.degrees(np.sqrt((e ** 2).mean()))),
        max_err_deg=float(np.degrees(np.abs(e).max())),
        final_err_deg=float(np.degrees(np.abs(e[-1]).max())),
        tau_peak=float(np.abs(log["tau"]).max()),
        latched=log["latched"],
    )
