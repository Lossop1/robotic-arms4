# 四轴机械臂（R4 / LZ12028）· 项目总览

本仓库 = **1 台四轴臂的 CAD → URDF → 控制栈 → 台架调试**，一条链。所有结论都带证据来源，
不能复现的数字不写进来。

当前状态：URDF 可用（几何与 CAD 对到 0.00 mm）；控制栈在自写刚体动力学上验证通过；
台架单轴已联网验证，并跑出**第一段真实闭环运动**（+36° / 5 s，WKC 3/3，无故障，
终点误差 18 counts = 0.014°，跟随误差峰值 0.745°、RMS 0.153°——见 §7 第 8 条）。

---

## 1. 目录

```
R4_Integrated_20261029/   SolidWorks 交付包（STEP / assembly.json / 配合日志）——URDF 的唯一输入
4轴硬件/                   早期 LZ12028 四轴交付包（zips，不进 git）
urdf/r4_urdf/             URDF + 网格（可被 ROS2/pybullet/MuJoCo 直接加载）
urdf/tools/               生成与校验 URDF 的脚本（build_urdf.py / validate_urdf.py / make_compare.py）
control/r4ctl/            控制栈：模型 / 逆解 / 轨迹 / 伺服 / 安全 / 仿真
control/demos/            三个可跑的验证 demo（`python demos/run_all.py`）
control/tests/            模型 / 逆解 / 轨迹三组自检
control/viewer/           浏览器交互控制台（双击 index.html）
bench/                    台架工具：opssh.py（SSH 到香橙派）、esi_dump.py（解析 ESI）
images/ 相关知识/           参考资料
```

约定：**文档只有这一份**；临时脚本用完就删或放进上面的目录，不放根目录；
根目录只允许 README.md、.gitignore 和上面这些目录。

## 2. 硬件真值（从驱动器 SDO 读出来的，不是猜的）

| 项 | 值 | 出处 |
|---|---|---|
| 驱动器 | CyberBeast `ISVD90RC-300B-100-70`，vendor `0x000C0B00` / product `0x00080117` / rev 1 | `emaster-scan` |
| 编码器 | 16384 counts/电机圈（0x608F:1） | SDO 读回 |
| 齿轮比 | 28 : 1（0x6091:1/2） | SDO 读回 |
| 分辨率 | 458752 counts/输出圈 → **0.000785°/count** | 计算 |
| 支持模式 | 0x6502 = **896 = 0x380** → 只有 CSP(8)/CSV(9)/CST(10) | SDO 读回 |
| MIT | **不可用**：模式位没有 bit5，且 0x2013（MIT 增益）SDO 读失败 wkc=0 | SDO 读回 |
| 额定转矩 0x6076 | 10000（0.001 N·m）= **10 N·m**（与 LZ12028 标称 80 N·m 矛盾，见 §7） | SDO 读回 |
| 最大转矩 0x6072 | 1000（0.1%）= 额定的 100% | SDO 读回 |
| 力矩单位 | **per-mille（0.1%）**：τ = 0x6077/1000 × 额定 | 手册 + 0x6072 佐证 |
| 最大转速 0x6080 | 841045 counts/s ≈ **110 °/s（输出）** | SDO 读回 |
| 软限位 0x607D | ±2 000 000 000（等于没设） | SDO 读回 |
| 驱动器内部增益 | 速度环 Kp=10 Ki=100；位置环 Kp=2000（单位 0.01） | 主站启动日志 |

香橙派：`orangepi6plus`（aarch64，6.6.89-cix），管理网口 `enp97s0`，EtherCAT `enp49s0`。
主站：`~/ethercat-master`（C11+SOEM，1 ms DC）；台架配置 `~/bench-config-lz` + `~/bench-runtime-lz`。

## 3. URDF

从 `R4_Integrated_20261029` 的装配数据生成，**没有手工量取**：

```
python urdf/tools/build_urdf.py          # 生成（约 17 s）
python urdf/tools/validate_urdf.py --deep  # 校验 + 渲染
```

- 关节轴由 4 个 P04 轮毂的圆柱面反算，不是拿渲染图目测（目测会把 J4 猜成绕 Z）；
- 验证：整体包围盒 vs `R4_COMPLETE.STEP` 偏差 **0.00 mm**；零位 `tool_link` = (304, 0, 530) mm；
- 5 连杆 / 4 转动 + 1 固定，19.34 kg，网格 14.5 MB；
- 关节：J1 轴点 (0,0,110)+Z，J2 (0,0,270)+Y，J3 (0,36,530)+Y，J4 (240,0,530)+X。

## 4. 控制栈

```
python control/demos/run_all.py     # 自检 + 三个 demo，约 30 s
```

四层：轨迹（S 曲线，标量约束折算，多轴同步） → 闭式逆解（4-DOF，无冗余） →
伺服（PD / +重力 / +摩擦 / CTC） → 安全层（限位、跟随误差、力矩钳位、latch）。

已量化结果：

| 项 | 结果 |
|---|---|
| 逆解回代（4978 位形） | 位置误差 5.98e-16 m；连续路径跳变 0.08° |
| 轨迹限幅 | 越限 1.8e-15；同步占用 100% |
| 跟踪 RMS（J2 S 曲线） | pd 0.143° → +重力 0.035° → +摩擦 0.0186° → CTC 0.0173° |
| 静态误差 | 0.289° → 0.009°（重力前馈） |
| 笛卡尔直线/圆 | 末端偏离 234 µm / 435 µm（CTC） |
| 重力补偿 | 3 s 内末端漂 0.1 mm |

**已知局限**：刚体 + 集中质量，无关节柔性、无齿隙、无接触；动力学是自写的（两套实现互比
3.6e-15，但共享同一套建模假设）。所以它是"自洽"，不是"已被独立验证"。

## 5. 台架操作

```bash
python bench/opssh.py --file bench/xxx.sh     # 远程执行
python bench/opssh.py "hostname"             # 单条命令
```

已知可用的部署：`orangepi-current-bench-idle`（1 轴、CSP、不动）。
启动：`sudo ~/ethercat-master/build/apps/master/emaster-master --config-dir ~/bench-runtime-lz --deployment <ID>`
停机：`kill -INT <pid>`（走 CiA402 安全失能 + 恢复 INIT）。
命令口：`/tmp/emaster-<部署ID>.sock`，一行一条命令；`status`、`sdo_read <从站> <索引> <子索引>` 可用。

## 6. 模式选择：CSP / CST / MIT

| 模式 | 谁闭环 | 主机要发 | 适用 |
|---|---|---|---|
| **CSP(8)** | 驱动器（内部 8 kHz 级） | 位置 | 轨迹跟踪、点到点。**本项目默认** |
| **CSV(9)** | 驱动器 | 速度 | 恒速拖动 → 摩擦辨识 |
| **CST(10)** | 驱动器（电流环） | 力矩 | 力矩标定、主机侧阻抗控制 |
| MIT(5) | 驱动器，但增益由主机给 | 位置/速度/力矩/Kp/Kd | 柔顺、力控、示教 |

结论：**先用 CSP + CSV + CST，不要为 MIT 换固件**。理由见 §7。

## 7. 未决问题（明码标价）

1. **额定转矩 10 N·m vs 标称 80 N·m 差 8 倍** —— 必须先标定：CST 下发已知 per-mille，
   读 0x6077 并用已知惯量自由加速反算，把"per-mille ↔ N·m"钉死。**在此之前摩擦力矩只有相对意义。**
2. **MIT 不可用** —— 若要柔顺/力控再议；主机侧用 CST + PD 前馈（1 kHz）已能覆盖 3–6 Hz 带宽需求。
3. **软限位 ±2e9 等于没设** —— 必须在主站/主机侧加软限位，不能依赖驱动器。
4. **跟随误差窗口未配置** —— 上一份 MT 报告里它是 2 个电机圈（等于没保护）。
5. **齿隙 15 arcmin（约 2.2 mm @500 mm）未实测** —— 实机绝对精度的主项。
6. **摩擦未辨识** —— 装配里没有，必须做恒速拖动实验。
7. **R4 臂的力矩预算 vs 实际模组** —— R4 肩部静态最坏约 31 N·m，模组额定按驱动器读回是
   10 N·m（0x6076），若属实则带不动，需与 LZ12028 标称 80 N·m 对齐后再定。
8. **首段运动的跟随误差偏大** —— +36°/5 s（7.2 °/s）下 RMS 0.153°、峰值 0.745°（起点加速度阶跃），
   稳态滞后约 0.15°（与速度成正比，说明缺速度前馈）。工业臂同级速度下应 <0.05°。
   下一步：① 用 S 曲线替代驱动器内置的梯形档；② 在 0x60FF 灌速度前馈，对比误差下降。

## 8. 复现

```
python urdf/tools/build_urdf.py && python urdf/tools/validate_urdf.py --deep
python control/demos/run_all.py
python control/tools/export_viewer.py     # 重建浏览器控制台的 data.js
```

依赖：Python 3.13 + cadquery(OCP) + trimesh + numpy + scipy + vtk + fast-simplification +
matplotlib + Pillow + paramiko；台架侧需要 Linux + 网卡直通。
