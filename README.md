# 四轴机械臂 R4 / LZ12028

四轴机械臂从 CAD 到 URDF 到控制栈到台架闭环的一条链，全部结论可复现。

## 详述

三个层次，各自有独立的证据来源：

- **URDF**：从 `data/cad/R4_Integrated_20261029` 的装配数据生成。几何、位姿、关节轴全部来自数据，
  没有手工量取。整体包围盒与 `R4_COMPLETE.STEP` 偏差 0.00 mm。
- **控制栈**：闭式逆解 + S 曲线时间参数化 + 四种伺服律 + 安全层，跑在自写的刚体动力学上。
  逆解回代误差 5.98e-16 m，轨迹限幅越限 1.8e-15，跟踪 RMS 0.017°（CTC）。
- **台架**：香橙派 + EtherCAT 主站（C11 + SOEM，1 ms DC）。单轴已验证：1 从站、WKC 3/3、
  第一段真实闭环运动 +36°/5 s，终点误差 0.014°、跟随误差峰值 0.745°、无故障。

驱动器真值（SDO 读回）：`CyberBeast ISVD90RC-300B-100-70`，16384 counts/电机圈、28:1、
支持 CSP/CSV/CST（**不含 MIT**）、力矩单位 per-mille、额定 0x6076 = 10 N·m。

**已知局限**：动力学是自写的刚体模型，无关节柔性、无齿隙、无接触；未做摩擦辨识；
跟随误差离工业臂水平还有差距（见未决问题）。

## 目录

根下只有元文件、工具配置和五个角色目录，每个角色一个桶。

```
src/robot/r4ctl/        控制栈：模型 / 逆解 / 轨迹 / 伺服 / 安全 / 仿真
src/robot/demos/        三个可跑的验证 demo + run_all
tests/                  模型 / 逆解 / 轨迹三组自检
tools/                  URDF 生成与校验、台架 SSH、ESI 解析、浏览器控制台
data/cad/               SolidWorks 交付包（URDF 的唯一输入）
data/models/r4_urdf/    URDF + 网格（ROS2 / pybullet / MuJoCo 可直接加载）
data/ref/               参考图片与笔记
external/lz12028/       早期 LZ12028 交付压缩包（不入 git）
```

## 安装

```
pip install cadquery trimesh numpy scipy vtk fast-simplification matplotlib Pillow paramiko
```

Python 3.13。运行时不需要 ROS、不需要 SolidWorks。

## 用法

```
python tools/build_urdf.py                  # 重新生成 URDF（约 20 s）
python tools/validate_urdf.py --deep        # 校验并渲染（包围盒 vs STEP）
python src/robot/demos/run_all.py           # 控制栈自检 + 三个 demo
python tools/export_viewer.py               # 重建浏览器控制台的 data.js
```

浏览器控制台：双击 `tools/viewer/index.html`。

台架（香橙派 `orangepi@192.168.124.81`）：

```
python tools/opssh.py "hostname"
python tools/opssh.py --file <脚本>
```

## 命令表

| 命令 | 作用 | 退出码 |
|---|---|---|
| `tools/build_urdf.py` | 生成 URDF 与网格 | 0 成功 / 1 失败 / 2 用法 |
| `tools/validate_urdf.py [--deep]` | 结构、惯量、FK、包围盒校验 + 渲染 | 同上 |
| `tools/export_viewer.py` | 生成浏览器控制台数据 | 同上 |
| `src/robot/demos/run_all.py` | 全量自检与 demo | 同上 |
| `tools/opssh.py <命令>` | 远程执行 | 同上 |
| `tools/esi_dump.py <ESI>` | 解析 ESI 的 PDO 映射 | 同上 |

## 实测结果 → 配置

唯一数据源是 data/bench/joint-test-result.json（单关节实测，2026-11-02），
src/robot/r4ctl/config.py 从它派生，不硬编码执行器参数：

| 派生量 | 来源字段 | 当前值 |
|---|---|---|
| 额定力矩 | 
ated.torque_nm（0x6076 实测） | 10.0 N·m |
| 减速比 / 分辨率 | units.gear_ratio、counts_per_motor_rev | 28:1、0.000785°/count |
| 库仑摩擦 fc | riction.fc_permille | 2.595 N·m = 26% 额定 |
| 粘滞摩擦 fv | riction.fv_permille_per_deg_s | 0.104 N·m/(°/s) |
| 闭环带宽 | closed_loop.bandwidth_hz | 1.2–1.8 Hz |
| 设计 f_n | 带宽 ÷ 0.64 ÷ 2（相位裕度） | 1.17 Hz |
| Kp / Kd | M_eff × ω²、2ζ M_eff ω | J1 89.5 / 24.3；J3 21.1 / 5.7 |

## 机型

**R4 = 用 4 台 LZ12028CH 关节模组搭的四轴臂**，几何来自装配包
data/cad/R4_Integrated_20261029/source/assembly.json（SolidWorks 导出，mm）。
装配里每个关节由 J*_motor（零件 MotorReference）与 J*_hub（零件 P04）组成，
关节轴由 P04 毂的圆柱面法向确定。

**关节无机械限位，可连续旋转** → URDF 中 4 个关节为 continuous；
config.py 里的 q_min/q_max 是**软件限位**（防绕线/自碰），不是机械行程。

执行器参数以厂家规格表为准：行星 28:1、额定 100 N·m、峰值 360 N·m、
额定 80 rpm（输出 480 °/s）、双编码器、无刹车、重 2.27 kg。

## 可视化仿真（MuJoCo）

`ash
python tools/make_mjcf.py          # URDF -> MJCF（含 CSP 位置执行器）
python tools/mujoco_view.py        # 打开窗口，用键盘自己控制
`

按键：1–4 选关节，A/D 转 ±5°，W/S 微调，R 回零，G 重力开关，Q 退出。
每个关节是一个 position 执行器（kp/kv），对应真机的 CSP 位置环；
--headless 只跑物理不开窗口，用于自检。

## 未决问题

1. **额定转矩 10 N·m 与 LZ12028 标称 80 N·m 差 8 倍**：需用 CST 下发已知 per-mille 并读回标定。
2. **跟随误差偏大**：7.2 °/s 下 RMS 0.153°、峰值 0.745°、稳态滞后约 0.15°（缺速度前馈）。
   已做的两次改进（±10°/2 s，200 Hz 设点，1000 Hz 采样）：

   | 轨迹 | 峰值误差 | RMS | 巡航滞后 |
   |---|---|---|---|
   | 驱动器内置梯形（+36°/5 s） | 0.745° | 0.153° | — |
   | S 曲线（本仓库 quintic） | 0.521° | 0.166° | 0.144° |
   | S 曲线 + 0x60FF 速度前馈 | 0.290° | 0.143° | 0.119° |

   **注意**：这组 A/B 两个工况方向相反（+10° vs −10°），重力与摩擦方向相反，
   结果被方向混淆，不能直接当作前馈的净效果。
3. **摩擦已辨识**（空载、≤8 °/s、±15°）：τ = 259.5‰·sign(v) + 10.4‰/(°/s)·v − 9.6‰，
   R² = 0.9945。库仑项约 26% 额定力矩，必须做摩擦前馈。
4. **闭环带宽实测约 1.2–1.8 Hz**（±3° 正弦扫频，1 kHz 观测帧、目标打标记）：
   幅值比 0.5 Hz 1.07、1 Hz 1.12、2 Hz 0.69、相位滞后 6.5°/18°/153°。
   −3 dB 落在 1–2 Hz 之间。4、8 Hz 段样本太少，不作为结论。
   **这个值远低于按模型估的 9–18 Hz**，控制整定必须按 1–2 Hz 重做，之前 0.24° 的
   跟踪误差由此解释。
5. **反向误差（回差的实际意义）1 mdeg**：位置闭环把机械间隙吸收了，输出的换向偏差很小；
   机械间隙本身属于机械验收项，控制侧不必测（曾为此尝试 CST，造成一次失控，已放弃）。
5. **软限位 0x607D = ±2e9**，等于没设，必须在主站或主机侧加。
6. **齿隙未测**：样本 15 arcmin（250 mdeg），需要观测口数据才能分离滞回。
7. **MIT 模式不可用**：0x6502 无 bit5、0x2013 对象不存在。驱动升级由用户决定。

## 维护者

Lossop1 <2107862895@qq.com>

## 贡献

见 [CONTRIBUTING.md](CONTRIBUTING.md)。文档只有本文件一份，新增内容并入对应小节，不再新建 md。

## 许可

未定（待权利人选择），见 `LICENSE` 缺失说明。
