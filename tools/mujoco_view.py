# -*- coding: utf-8 -*-
"""MuJoCo 交互式可视化：你自己用键盘控制四个关节。

每个关节是一个 position 执行器（模拟驱动器里的 CSP 位置环），
所以你给的是"目标角度"，执行器按 kp/kv 拉过去——和真机的 CSP 一致。

用法:
  python tools/mujoco_view.py            # 打开窗口，键盘控制
  python tools/mujoco_view.py --sine     # 无键盘时看正弦演示
  python tools/mujoco_view.py --headless # 只跑物理，不开窗口（自检）

按键:
  1 2 3 4   选择关节
  A / D     目标角 -5° / +5°（按住可连续）
  W / S     目标角微调 ±1°
  G         重力开关
  R         回到零位
  Q         退出
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MJCF = ROOT / "data" / "models" / "r4_urdf" / "urdf" / "r4_mjcf.xml"


class Keys:
    """Windows 下用 msvcrt 取键；其它平台退化为无输入。"""

    def __init__(self):
        self.ok = False
        try:
            import msvcrt
            self.msvcrt = msvcrt
            self.ok = True
        except ImportError:
            self.msvcrt = None

    def poll(self):
        out = []
        if not self.ok:
            return out
        while self.msvcrt.kbhit():
            ch = self.msvcrt.getwch()
            out.append(ch.lower())
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", default=str(MJCF))
    ap.add_argument("--sine", action="store_true")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--seconds", type=float, default=3.0)
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(args.mjcf)
    data = mujoco.MjData(model)
    n = model.nq
    print(f"关节 {n}，执行器 {model.nu}，总质量 {model.body_mass.sum():.2f} kg")

    target = np.zeros(n)          # 目标角（rad）
    active = 0                    # 当前选中的关节
    step_deg = 5.0

    def apply():
        for i in range(min(n, model.nu)):
            data.ctrl[i] = target[i]

    def info():
        q = np.degrees(data.qpos)
        ee = data.site_xpos if model.nsite else None
        s = "  ".join(f"J{i+1}={q[i]:+7.1f}°" for i in range(n))
        return f"{s}   [选中 J{active+1}]  目标 J{active+1}={math.degrees(target[active]):+.1f}°"

    if args.headless:
        for k in range(int(args.seconds / model.opt.timestep)):
            target[:] = np.radians(10.0) * np.sin(2 * np.pi * 0.5 * k * model.opt.timestep)
            apply()
            mujoco.mj_step(model, data)
        print(f"headless 步进 {int(args.seconds/model.opt.timestep)} 步，末态 {info()}")
        return 0

    import mujoco.viewer as mjv
    keys = Keys()
    if not keys.ok:
        print("提示：当前平台没有 msvcrt，键盘不可用；用 --sine 看演示")

    with mjv.launch_passive(model, data) as viewer:
        viewer.cam.distance = 1.8
        viewer.cam.elevation = -20
        viewer.cam.azimuth = 135
        t0 = time.perf_counter()
        while viewer.is_running():
            for ch in keys.poll():
                if ch == "q":
                    return 0
                elif ch in "1234" and int(ch) <= n:
                    active = int(ch) - 1
                elif ch == "a":
                    target[active] = max(target[active] - math.radians(step_deg), -math.pi)
                elif ch == "d":
                    target[active] = min(target[active] + math.radians(step_deg), math.pi)
                elif ch == "w":
                    target[active] += math.radians(1.0)
                elif ch == "s":
                    target[active] -= math.radians(1.0)
                elif ch == "r":
                    target[:] = 0.0
                elif ch == "g":
                    model.opt.gravity[2] *= -1.0

            if args.sine:
                t = time.perf_counter() - t0
                target[:] = np.radians(15.0) * np.sin(2 * np.pi * 0.4 * t)

            apply()
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)
    return 0


if __name__ == "__main__":
    sys.exit(main())
