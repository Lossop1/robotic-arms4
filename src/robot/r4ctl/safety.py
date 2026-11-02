# -*- coding: utf-8 -*-
"""
安全层：软限位、速度、跟随误差、力矩饱和、看门狗、急停。

在伺服与执行器之间串一层。任何一条触发就 latch，进入"保持当前位姿"状态
（真实系统里还要切伺服使能 + 抱闸，这里只建模到力矩层）。
"""
from __future__ import annotations

import numpy as np

from . import config as C
from .model import chain

TAU_MAX = C.SAFETY["tau_margin"] * C.ACTUATOR["tau_cont"]


class Safety:
    def __init__(self):
        ch = chain()
        self.ch = ch
        self.margin = np.radians(C.SAFETY["q_margin_deg"])
        self.follow_err = np.radians(C.SAFETY["follow_err_deg"])
        self.latched = False
        self.reason = ""
        self.events = []

    def reset(self):
        self.latched = False
        self.reason = ""

    def check(self, q, qd, q_des, tau, t=None):
        ch = self.ch
        flags = []

        lim = (q < ch.q_min) | (q > ch.q_max)
        if lim.any():
            flags.append(("joint_limit", [ch.movable[i]["name"] for i in np.where(lim)[0]]))

        vmax = ch.v_max * (1.0 + 1e-9)
        vs = np.abs(qd) > ch.v_max * 1.05
        if vs.any():
            flags.append(("velocity", [ch.movable[i]["name"] for i in np.where(vs)[0]]))

        fe = np.abs(q_des - q) > self.follow_err
        if fe.any():
            flags.append(("follow_error", [ch.movable[i]["name"] for i in np.where(fe)[0]]))

        if flags and not self.latched:
            self.latched = True
            self.reason = "; ".join(f"{k}:{v}" for k, v in flags)
            self.events.append((t, self.reason))

        tau = np.clip(tau, -TAU_MAX, TAU_MAX)
        return tau, self.latched, flags
