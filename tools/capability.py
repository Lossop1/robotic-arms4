# -*- coding: utf-8 -*-
"""由实测参数 + URDF 算出这台机器的能力上限（与任务无关）。

CSP 定死之后，上限只由硬件决定：
  - 跟踪误差 ≈ 等效时间延迟 × 速度（延迟从实测相位滞后反推）
  - 速度上限：由允许误差反推
  - 载荷上限：由各关节重力静力矩对额定/峰值力矩的裕度定
  - 精度下限：编码器分辨率 + 重复性（实测）

用法: python tools/capability.py
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "robot"))
from r4ctl.model import Chain  # noqa: E402

J = json.loads((ROOT / "data" / "bench" / "joint-test-result.json").read_text(encoding="utf-8"))
TAU_RATED = J["datasheet"]["rated_torque_nm"]
TAU_PEAK = J["datasheet"]["peak_torque_nm"]
V_RATED_DEG_S = J["datasheet"]["rated_speed_deg_s"]
DEG_PER_COUNT = J["units"]["deg_per_count"]
BW = J["closed_loop"]["bandwidth_hz"]
FRF = J["closed_loop"]["frf"]
VALLEY = 500.0          # 参考力臂 500 mm（臂展量级），用于把角度折成末端毫米


def delay_from_phase():
    """等效时间延迟 = 相位滞后 / (360° × f)。取远离共振的 1 Hz 点。"""
    pts = {p["f_hz"]: p["phase_lag_deg"] for p in FRF}
    f = 1.0
    if f in pts:
        return pts[f] / (360.0 * f)
    return float("nan")


def main():
    ch = Chain()
    M_eff = ch.effective_inertia()

    delay = delay_from_phase()
    print("=" * 70)
    print("能力上限（CSP，实测参数 + URDF）")
    print("=" * 70)
    print(f"额定/峰值力矩      {TAU_RATED:.0f} / {TAU_PEAK:.0f} N·m")
    print(f"额定输出转速       {V_RATED_DEG_S:.0f} °/s")
    print(f"闭环带宽（实测）   {BW[0]:.1f}–{BW[1]:.1f} Hz")
    print(f"等效时间延迟       {delay*1000:.0f} ms  ← 由 1 Hz 相位滞后 {FRF[1]['phase_lag_deg']:.0f}° 反推")
    print()

    print("① 跟踪误差 vs 速度（CSP 的本质：误差 = 延迟 × 速度）")
    print(f"   {'速度':>8s}{'误差°':>10s}{'末端mm@500':>12s}")
    for v in (2, 5, 8, 15, 30, 60, 120):
        e = delay * v
        print(f"   {v:6.0f}°/s{e:10.3f}{e / 360.0 * 2 * math.pi * VALLEY:12.2f}")
    print(f"   实测对照：≤8 °/s 时 RMS {J['tracking']['err_rms_mdeg']/1000:.3f}°")
    print()

    print("② 速度上限：允许误差反推")
    print(f"   {'允许误差':>10s}{'可用速度':>12s}")
    for tol_mm in (0.5, 1.0, 2.0, 5.0):
        tol_deg = tol_mm / (2 * math.pi * VALLEY) * 360.0
        print(f"   {tol_mm:8.1f}mm{tol_deg / delay:10.0f}°/s")
    print()

    print("③ 载荷上限：各关节最大重力静力矩（含末端负载）")
    rng = np.random.default_rng(0)
    qs = rng.uniform(-math.pi, math.pi, size=(400, ch.n))
    nominal = np.zeros(ch.n)
    for q in qs:
        nominal = np.maximum(nominal, np.abs(ch.gravity(q)))
    print(f"   空载最大静力矩  " + "  ".join(f"J{i+1}={t:5.1f}" for i, t in enumerate(nominal)))
    print(f"   {'负载':>6s}" + "".join(f"{'J'+str(i+1):>8s}" for i in range(ch.n))
          + f"{'裕度(J1)':>10s}")
    # 末端负载的重力力臂（由 URDF 尺寸：J2 以上臂展约 0.62 m）
    LEVER = {0: 0.0, 1: 0.62, 2: 0.35, 3: 0.10}   # J1 绕竖直轴，重力无力矩
    for m in (0.0, 2.0, 5.0, 10.0, 20.0):
        tot = np.array([nominal[i] + m * 9.81 * LEVER.get(i, 0.0) for i in range(ch.n)])
        marg = TAU_RATED / max(tot.max(), 1e-9)
        worst = int(np.argmax(tot))
        print(f"   {m:4.1f}kg" + "".join(f"{v:8.1f}" for v in tot)
              + f"   {marg:5.2f}x  (最紧 J{worst+1})")
    print()

    print("④ 精度下限（与任务无关的硬件底线）")
    print(f"   编码器 1 count      {DEG_PER_COUNT:.6f}° = {DEG_PER_COUNT/360*2*math.pi*VALLEY*1000:.1f} µm @500mm")
    rep = J["repeatability_mdeg"]["sigma"]
    print(f"   重复定位（实测）    σ {rep[0]:.0f}/{rep[1]:.0f} mdeg = "
          f"{rep[1]/1000/360*2*math.pi*VALLEY:.2f}–{rep[0]/1000/360*2*math.pi*VALLEY:.2f} mm @500mm")
    print(f"   反向误差（实测）    {J['reversal_error_mdeg']:.1f} mdeg")
    print()

    print("⑤ 结构能力")
    print(f"   整机质量            {sum(ch.masses):.2f} kg" if hasattr(ch, 'masses') else "")
    print(f"   有效惯量            " + "  ".join(f"J{i+1}={v:.4f}" for i, v in enumerate(M_eff)))


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    main()
