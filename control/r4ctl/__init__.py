# -*- coding: utf-8 -*-
"""r4ctl 包：R4 四轴机械臂 运动学 / 轨迹 / 伺服 控制栈。"""
from .model import Chain, chain, G  # noqa: F401
from . import config as cfg  # noqa: F401

__all__ = ["Chain", "chain", "G", "cfg"]
