"""把 URDF 渲染图与原始 SolidWorks 预览拼成对照图，并汇总交付物统计。"""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

_ROOT = Path(__file__).resolve().parents[1]
URDF = _ROOT / "data" / "models" / "r4_urdf"
PKG = _ROOT / "data" / "cad" / "R4_Integrated_20261029"

TILES = [
    (PKG / "preview" / "complete.png", "CAD assembly (SolidWorks preview)"),
    (URDF / "preview" / "urdf_zero.png", "URDF reassembled from meshes (zero pose)"),
    (URDF / "preview" / "urdf_demo.png", "URDF driven by FK (j1=25 j2=-35 j3=55 j4=40 deg)"),
]

W, H = 900, 470
PANEL = 34
py = 0
try:
    font = ImageFont.truetype("C:/Windows/Fonts/consola.ttf", 20)
except Exception:
    font = ImageFont.load_default()

sheet = Image.new("RGB", (W * 3, H + PANEL), (250, 250, 252))
d = ImageDraw.Draw(sheet)
for i, (p, title) in enumerate(TILES):
    im = Image.open(p).convert("RGB")
    im.thumbnail((W, H))
    x = i * W + (W - im.width) // 2
    y = PANEL + (H - im.height) // 2
    sheet.paste(im, (x, y))
    d.text((i * W + 10, 8), title, fill=(20, 20, 25), font=font)
    d.line([(i * W, 0), (i * W, H + PANEL)], fill=(180, 185, 195), width=2)
out = URDF / "preview" / "compare_cad_vs_urdf.png"
sheet.save(out)
print("saved", out, sheet.size)

print("\n# 交付物")
tot = 0
for f in sorted(URDF.rglob("*")):
    if f.is_file():
        tot += f.stat().st_size
        print(f"  {f.relative_to(URDF)}  {f.stat().st_size/1024:9.1f} KB")
print(f"  合计 {tot/1e6:.2f} MB")
