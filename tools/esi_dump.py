# -*- coding: utf-8 -*-
"""
台架摸底：把 ESI 的 PDO 映射和电机参数 PDF 里的关键数字抽出来。

用途：确定"能采到什么信号"（PDO 里有什么）和"怎么换算成物理量"（额定电流/转矩常数/减速比）。
"""
from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ESI = Path(r"D:\taili\ethercat_master\docs\product\RMD-X4-36  260920\MT-Device_260424.xml")
PDF = Path(r"D:\taili\ethercat_master\docs\product\RMD-X4-36  260920\RMD-X4-36 电机参数 260902.pdf")

NAMES = {
    "0x6040": "controlword", "0x6041": "statusword",
    "0x6060": "mode_of_op", "0x6061": "mode_display",
    "0x607A": "target_position", "0x6064": "actual_position",
    "0x606C": "actual_velocity", "0x60FF": "target_velocity",
    "0x6071": "target_torque", "0x6074": "torque_demand",
    "0x6077": "actual_torque", "0x6078": "actual_current",
    "0x6072": "max_torque", "0x607F": "max_profile_velocity",
    "0x6065": "following_error_window", "0x6066": "following_error_timeout",
    "0x603F": "error_code", "0x6502": "supported_drive_modes",
    "0x60B0": "position_offset", "0x60B1": "velocity_offset", "0x60B2": "torque_offset",
    "0x60C2": "interpolation_time_period", "0x60F4": "following_error_actual",
    "0x6081": "profile_velocity", "0x6083": "profile_accel", "0x6084": "profile_decel",
    "0x6098": "homing_method", "0x6099": "homing_speeds", "0x609A": "homing_accel",
    "0x6075": "motor_rated_current", "0x6076": "motor_rated_torque",
}


def dump_esi(path=None):
    path = Path(path or ESI)
    root = ET.parse(str(path)).getroot()
    vend = root.findtext(".//Vendor/Id")
    vname = root.findtext(".//Vendor/Name")
    dev = root.find(".//Devices/Device")
    print(f"ESI: {path.name}")
    print(f"     vendor={vend} ({vname})")
    t = dev.find("Type")
    print(f"     type={t.get('ProductCode')} rev={t.get('RevisionNo')} name={t.text}")
    print(f"      Physics={dev.get('Physics')}")
    for tag in ("RxPdo", "TxPdo"):
        print(f"\n== {tag} ==")
        for pdo in root.iter(tag):
            idx = pdo.findtext("Index")
            name = pdo.findtext("Name")
            ents = []
            for e in pdo.findall("Entry"):
                ix = e.findtext("Index")
                si = e.findtext("SubIndex")
                bl = e.findtext("BitLen")
                ents.append(f"{ix}:{si}={NAMES.get(ix, '?')}({bl}b)")
            print(f"  {idx} Sm={pdo.get('Sm')} name={name}")
            for x in ents:
                print(f"      {x}")
    # 对象字典里只看几个关键项的范围/默认值
    print("\n== objects (selected) ==")
    for obj in root.iter("Object"):
        ix = obj.findtext("Index")
        if ix in ("0x6075", "0x6076", "0x6078", "0x6077", "0x6072", "0x6065", "0x6066", "0x6060"):
            print(f"  {ix} {NAMES.get(ix,'?'):22s} "
                  f"dflt={obj.findtext('Info/DefaultData')} "
                  f"name={obj.findtext('Name')}")


def dump_pdf():
    try:
        import fitz
    except Exception as e:
        print("PDF: 无法读", e)
        return
    doc = fitz.open(str(PDF))
    text = "\n".join(page.get_text() for page in doc)
    keys = ["额定", "转矩", "扭矩", "电流", "减速比", "编码器", "转速", "电压", "Kt", "力矩"]
    print(f"\n== PDF {PDF.name} 共 {len(doc)} 页 ==")
    seen = set()
    for line in text.splitlines():
        s = line.strip()
        if not s or s in seen:
            continue
        if any(k in s for k in keys):
            seen.add(s)
            print("  ", s[:120])
            if len(seen) > 60:
                break


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    dump_esi()
    dump_pdf()
