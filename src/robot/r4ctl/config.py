# -*- coding: utf-8 -*-
"""
R4 四轴机械臂控制参数 —— **全部由台架实测结果派生**。

单一数据源：data/bench/joint-test-result.json（单关节实测，2026-11-02）
  机构：data/models/r4_urdf/urdf/r4.urdf（由 R4_Integrated 装配数据生成）

硬编码只剩两类，且都标注了"未测"：
  1. 关节行程（包里没有机械限位数据）
  2. 轨迹的 v/a/j 上限（未做节拍试验）
其余执行器参数、摩擦、带宽、增益全部从实测文件读。
"""
from __future__ import annotations

import json
import math
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
URDF = ROOT / "data" / "models" / "r4_urdf" / "urdf" / "r4.urdf"

TEST_PATH = ROOT / "data" / "bench" / "joint-test-result.json"
JOINT_TEST = json.loads(TEST_PATH.read_text(encoding="utf-8"))

# ===================================================================== 执行器
_U = JOINT_TEST["units"]
_R = JOINT_TEST["rated"]
ACTUATOR = dict(
    model=JOINT_TEST["model"],
    ratio=_U["gear_ratio"],
    counts_per_motor_rev=_U["counts_per_motor_rev"],
    counts_per_output_rev=_U["counts_per_output_rev"],
    deg_per_count=_U["deg_per_count"],
    tau_rated=_R["torque_nm_datasheet"],       # 厂家额定 100 N·m（0x6076 读回 10，冲突待标定）
    torque_scale_unresolved=True,
    tau_unit="per-mille of rated",
    tau_max_permille=_R["max_torque_0x6072_permille"],
    v_out_max_deg_s=_R["max_speed_deg_s"],     # 实测 0x6080
    modes=list(_R["modes"]),
    mit_supported=bool(_R["mit_supported"]),
    comm="EtherCAT (CoE, 1 ms DC)",
)

# ===================================================================== 摩擦（实测）
_F = JOINT_TEST["friction"]
TAU_RATED = ACTUATOR["tau_rated"]


def _permille_to_nm(p: float) -> float:
    """per-mille 额定 → N·m"""
    return p / 1000.0 * TAU_RATED


FRICTION_MODEL = dict(
    fc=_permille_to_nm(_F["fc_permille"]),
    fv=_permille_to_nm(_F["fv_permille_per_deg_s"]),   # N·m per (°/s)
    bias=_permille_to_nm(_F["bias_permille"]),
    fc_permille=_F["fc_permille"],
    fv_permille_per_deg_s=_F["fv_permille_per_deg_s"],
    r2=_F["fit_r2"],
    source=str(TEST_PATH.relative_to(ROOT)),
)

# ===================================================================== 带宽 → 增益
# 实测闭环带宽（正弦扫频 −3 dB）
_BW = JOINT_TEST["closed_loop"]["bandwidth_hz"]
BW_MEAS_HZ = sum(_BW) / len(_BW)

# 整定规则：
#   二阶系统 f_3dB ≈ 0.64·f_n  →  f_n = f_3dB / 0.64
#   再留 2 倍相位裕度（未建模：摩擦、齿隙、结构弹性、通讯延迟）→ f_n_design = f_n / 2
DERATE = 2.0
FN_DESIGN_HZ = BW_MEAS_HZ / 0.64 / DERATE
WN_DESIGN = 2 * math.pi * FN_DESIGN_HZ


def kp_for(m_eff: float, zeta: float = 1.0) -> float:
    """Kp = M_eff·ω²（M_eff 由 URDF 算，见 model.Chain.effective_inertia）"""
    return m_eff * WN_DESIGN ** 2


def kd_for(m_eff: float, zeta: float = 1.0) -> float:
    """Kd = 2ζ·M_eff·ω"""
    return 2.0 * zeta * m_eff * WN_DESIGN


# ===================================================================== 关节
# 符号与 URDF 一致：J1 +Z，J2 +Y，J3 +Y，J4 +X
# ⚠ 本臂**无机械限位**，关节可连续旋转。这里的 q_min/q_max 是**软件限位**（防绕线/自碰），
#   不是机械行程；上机前按线缆余量和干涉实测收紧。
JOINTS = [
    dict(name="J1", axis=(0, 0, 1), q_min=-170.0, q_max=170.0,
         v_max=1.6, a_max=6.0, j_max=40.0, zeta=1.0),
    dict(name="J2", axis=(0, 1, 0), q_min=-90.0, q_max=100.0,
         v_max=1.6, a_max=6.0, j_max=40.0, zeta=1.0),
    dict(name="J3", axis=(0, 1, 0), q_min=-135.0, q_max=100.0,
         v_max=1.6, a_max=8.0, j_max=50.0, zeta=1.0),
    dict(name="J4", axis=(1, 0, 0), q_min=-180.0, q_max=180.0,
         v_max=3.0, a_max=15.0, j_max=100.0, zeta=1.0),
]

# ===================================================================== 控制环
CTL = dict(
    rate_hz=1000.0,
    sub_steps=8,
    cmd_delay_ms=1.0,
    gravity_ff=True,
    inertia_ff=False,
    friction_ff=True,          # 摩擦占 26% 额定，前馈是必选项
    integral=False,
    kp_scale=1.0,
    kd_scale=1.0,
    wn_design_hz=FN_DESIGN_HZ,
)

# ===================================================================== 轨迹
# ⚠ v/a/j 上限未做节拍试验，取保守值
TRAJ = dict(profile="scurve", jerk_ratio=0.7, sync=True)

# ===================================================================== 安全
SAFETY = dict(
    q_margin_deg=3.0,
    v_margin=0.95,
    tau_margin=0.90,
    watchdog_ms=200.0,
    follow_err_deg=12.0,
)
