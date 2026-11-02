# -*- coding: utf-8 -*-
"""
生成可视化验证动画（GIF）：

  anim_joint.gif      S 曲线关节运动（pd_g_ff）
  anim_cartesian.gif  笛卡尔直线 + 圆弧，红管 = 仿真末端实际走过的轨迹
  anim_gravity.gif    左：断电自由下坠 / 右：纯重力补偿 + 手推（25 N·m）

q(t) 全部来自 control 栈的闭环输出，网格来自 urdf/r4_urdf，位姿由 FK 给出。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from animate import Scene, frames_to_gif  # noqa: E402

from r4ctl import ik as IK  # noqa: E402
from r4ctl.model import chain  # noqa: E402
from r4ctl.servo import Servo  # noqa: E402
from r4ctl.sim import Sim  # noqa: E402
from r4ctl.traj import CartesianMove, JointMove, circle, line  # noqa: E402

OUT = ROOT / "out"
OUT.mkdir(exist_ok=True)
ch = chain()

try:
    FONT = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 20)
    FONT_S = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 16)
except Exception:
    FONT = FONT_S = ImageFont.load_default()


def pick(log, n):
    idx = np.linspace(0, len(log["t"]) - 1, n).astype(int)
    return idx


def caption(img, text, sub=None):
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, img.width, 30], fill=(30, 34, 40))
    d.text((10, 4), text, fill=(255, 255, 255), font=FONT)
    if sub:
        d.rectangle([0, img.height - 26, img.width, img.height], fill=(30, 34, 40))
        d.text((10, img.height - 24), sub, fill=(230, 230, 235), font=FONT_S)
    return img


# ============================================================ 1. 关节运动
def anim_joint(n=44):
    q0 = np.radians([0.0, -45.0, 40.0, 0.0])
    q1 = np.radians([35.0, -15.0, 5.0, 60.0])
    mv = JointMove(q0, q1)

    def ref(t):
        if t <= mv.T:
            return mv.sample(t)
        return mv.sample(mv.T)[0], np.zeros(ch.n), np.zeros(ch.n)

    sim = Sim()
    log = sim.run(ref, Servo("pd_g_ff"), T=mv.T + 0.5, q0=q0)
    sc = Scene()
    frames = []
    for k in pick(log, n):
        sc.set_q(log["q"][k])
        t = log["t"][k]
        e = np.degrees(np.abs(log["q"][k] - log["q_des"][k]).max())
        frames.append(caption(sc.grab(),
                              f"含重力+摩擦前馈的位置伺服    t = {t:4.2f}s",
                              f"最大关节误差 {e:.3f}°   |   给定 q_d 来自 S 曲线时间律"))
    p = frames_to_gif(frames, OUT / "anim_joint.gif", duration=55)
    print("  ", p.name, f"{len(frames)} 帧")
    return p


# ============================================================ 2. 笛卡尔
def anim_cartesian(n=52):
    q0 = np.array([0.0, 0.55, -0.15, 0.0])
    p0 = IK.fk(q0)
    radius = 0.08
    center = p0[:2] - np.array([radius, 0.0])
    mv = CartesianMove(q0, circle(center, radius, p0[2], roll=0.0,
                                  plane="xy", ang0=0.0, ang1=1.6 * math.pi), n=320)

    def ref(t):
        if t <= mv.T:
            return mv.sample(t)
        return mv.sample(mv.T)[0], np.zeros(ch.n), np.zeros(ch.n)

    sim = Sim()
    log = sim.run(ref, Servo("ctc"), T=mv.T + 0.3, q0=q0)
    sc = Scene(cam=(1.25, -1.15, 0.62), look=(0.16, 0.0, 0.33))
    frames, trail = [], []
    for k in pick(log, n):
        tip = sc.set_q(log["q"][k])
        trail.append(tip.copy())
        if len(trail) > 2:
            sc.set_trace(trail)
        t = log["t"][k]
        frames.append(caption(sc.grab(),
                              f"笛卡尔圆（r=80 mm，1.6π）   t = {t:4.2f}s",
                              "红管 = 仿真末端实际轨迹；蓝臂由 FK 驱动，网格来自 URDF"))
    p = frames_to_gif(frames, OUT / "anim_cartesian.gif", duration=55)
    print("  ", p.name, f"{len(frames)} 帧")
    return p


# ============================================================ 3. 重力 vs 断电
def anim_gravity(n=44):
    q0 = np.radians([20.0, -60.0, 55.0, 15.0])

    def hold(t):
        return q0, np.zeros(ch.n), np.zeros(ch.n)

    def push(t):
        d = np.zeros(ch.n)
        if 0.9 <= t < 1.3:
            d[1] = 25.0
        return d

    off = Sim().run(hold, Servo("pd", kp=np.zeros(ch.n), kd=np.zeros(ch.n)), T=1.6, q0=q0)
    gcomp = Sim().run(hold, Servo("pd_g", kp=np.zeros(ch.n), kd=np.zeros(ch.n)),
                      T=1.6, q0=q0, disturbance=push)

    sa, sb = Scene(), Scene()
    frames = []
    for k in pick(off, n):
        sa.set_q(off["q"][k])
        sb.set_q(gcomp["q"][k])
        t = off["t"][k]
        a = caption(sa.grab(), f"断电 τ=0   t={t:4.2f}s",
                    f"J2 = {np.degrees(off['q'][k,1]):7.1f}°   末端位移 "
                    f"{np.linalg.norm(off['tip'][k]-off['tip'][0])*1000:5.0f} mm")
        b = caption(sb.grab(), f"γ 重力补偿 τ=g(q)   t={t:4.2f}s",
                    f"J2 = {np.degrees(gcomp['q'][k,1]):7.1f}°   末端位移 "
                    f"{np.linalg.norm(gcomp['tip'][k]-gcomp['tip'][0])*1000:5.0f} mm"
                    + ("   [手推中]" if 0.9 <= t < 1.3 else ""))
        canvas = Image.new("RGB", (a.width + b.width, max(a.height, b.height)), (255, 255, 255))
        canvas.paste(a, (0, 0))
        canvas.paste(b, (a.width, 0))
        frames.append(canvas)
    p = frames_to_gif(frames, OUT / "anim_gravity.gif", duration=70)
    print("  ", p.name, f"{len(frames)} 帧")
    return p


if __name__ == "__main__":
    print("生成可视化动画：")
    for f in (anim_joint, anim_cartesian, anim_gravity):
        f()
