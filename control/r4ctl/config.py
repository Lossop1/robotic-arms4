# -*- coding: utf-8 -*-
"""
R4 四轴机械臂控制参数（单一数据源）

机构来源：urdf/r4_urdf/urdf/r4.urdf（由 R4_Integrated_20261029 装配包生成）
执行器来源：4轴硬件/LZ12028_4轴_交付/说明.md + 03_源文件/config.py
  → 4 × LZ12028CH 关节模组，S1 连续 80 N·m / S2 100 / 峰值 360，
    减速比 28:1，额定输出 80 rpm，空载 105 rpm，48 V，1.0 kW，双绝对值编码器，
    回差 ≤15 arcmin，通讯 CANFD / EtherCAT。

⚠ 行程：包内无 R4 的机械限位数据（LZ12028 交付里那组是另一台臂的），
  这里取"机构能到且不撞"的保守值，上机前必须按实物挡块/干涉实测改写。
"""
from __future__ import annotations

import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / "urdf" / "r4_urdf" / "urdf" / "r4.urdf"

# ===================================================================== 执行器
ACTUATOR = dict(
    model="LZ12028CH",
    ratio=28.0,
    tau_cont=80.0,          # S1 连续 N·m
    tau_s2=100.0,           # S2 短时
    tau_peak=360.0,         # 峰值
    n_rated_rpm=80.0,       # 输出额定转速 -> 8.38 rad/s
    n_noload_rpm=105.0,     # 输出空载 -> 11.0 rad/s
    v_out_max=8.38,         # rad/s，取额定转速
    kt=0.17,                # N·m/A（电机轴）
    i_rated=29.7,
    i_peak=102.0,
    voltage=48.0,
    backlash_arcmin=15.0,
    encoder_bits=19,        # 双绝对值编码器（单圈），折算关节分辨率
    comm="CANFD",
)

# ===================================================================== 关节
# 关节符号与 URDF 一致：所有 link 零位姿态相同，轴向为装配系下的轴向量
#   J1 +Z，J2 +Y，J3 +Y，J4 +X
JOINTS = [
    dict(name="J1", axis=(0, 0, 1), q_min=-170.0, q_max=170.0,
         v_max=1.6, a_max=6.0, j_max=40.0, wn_hz=10.0, zeta=1.0),
    dict(name="J2", axis=(0, 1, 0), q_min=-90.0, q_max=100.0,
         v_max=1.6, a_max=6.0, j_max=40.0, wn_hz=9.0, zeta=1.0),
    dict(name="J3", axis=(0, 1, 0), q_min=-135.0, q_max=100.0,
         v_max=1.6, a_max=8.0, j_max=50.0, wn_hz=11.0, zeta=1.0),
    dict(name="J4", axis=(1, 0, 0), q_min=-180.0, q_max=180.0,
         v_max=3.0, a_max=15.0, j_max=100.0, wn_hz=18.0, zeta=1.0),
]

# ===================================================================== 控制环
CTL = dict(
    rate_hz=1000.0,         # 伺服环 1 kHz
    sub_steps=8,            # 仿真积分细分（8 kHz）
    cmd_delay_ms=1.0,       # 指令到执行器的传输延迟（1 拍）
    gravity_ff=True,        # 重力前馈
    inertia_ff=False,       # 惯性前馈（M(q)·q̈_d），对轨迹跟踪有明显收益但吃模型精度
    friction_ff=True,       # 摩擦补偿
    integral=False,         # 位置环不加积分：前馈够用，积分会引入 windup
    kp_scale=1.0,
    kd_scale=1.0,
)

# 摩擦模型 τ_f = fc·sign(q̇) + fv·q̇ （Stribeck 项默认关掉，待辨识）
FRICTION = [
    dict(fc=1.2, fv=12.0),   # J1  腰转，谐波+密封
    dict(fc=1.5, fv=15.0),   # J2
    dict(fc=1.2, fv=10.0),   # J3
    dict(fc=0.4, fv=2.0),    # J4
]

# ===================================================================== 轨迹
TRAJ = dict(
    profile="scurve",       # "scurve"（加加速度受限）| "trapezoid"（梯形速度）
    jerk_ratio=0.7,         # S 曲线加加速度取 a_max/τ 的经验比例
    sync=True,              # 多轴时间同步（用最慢的关节定总时长）
)

# ===================================================================== 安全
SAFETY = dict(
    q_margin_deg=3.0,       # 软限位距离硬限位的余量
    v_margin=0.95,
    tau_margin=0.90,        # 力矩饱和按 S1 连续值的 90% 发
    watchdog_ms=200.0,
    follow_err_deg=12.0,    # 跟随误差超限 -> 停
)
