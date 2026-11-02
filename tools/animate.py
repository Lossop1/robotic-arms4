# -*- coding: utf-8 -*-
"""
把仿真出来的关节角逐帧喂回 URDF 网格，用 vtk 离屏渲染成 GIF。

这就是"用仿真数据驱动真模型"的可视化验证：q(t) 来自控制栈的闭环输出，
网格来自 urdf/r4_urdf/meshes，位姿由 r4ctl.model 的 FK 给出。
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import vtk
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
URDF_DIR = ROOT / "data" / "models" / "r4_urdf"
sys.path.insert(0, str(ROOT))

from r4ctl.model import chain  # noqa: E402

LINKS = ["base_link", "link1", "link2", "link3", "link4"]
PALETTE = [(0.60, 0.64, 0.70), (0.24, 0.42, 0.28), (0.30, 0.45, 0.62),
           (0.52, 0.56, 0.64), (0.78, 0.48, 0.18)]


def _polydata(tm):
    import trimesh
    m = trimesh.load_mesh(str(tm), process=False)
    V = np.asarray(m.vertices)
    F = np.asarray(m.faces)
    pts = vtk.vtkPoints()
    pts.SetNumberOfPoints(len(V))
    for i, (x, y, z) in enumerate(V):
        pts.SetPoint(i, float(x), float(y), float(z))
    cells = vtk.vtkCellArray()
    for tri in F:
        cells.InsertNextCell(3)
        for k in tri:
            cells.InsertCellPoint(int(k))
    pd = vtk.vtkPolyData()
    pd.SetPoints(pts)
    pd.SetPolys(cells)
    return pd


class Scene:
    def __init__(self, size=(760, 620), cam=(1.15, -1.05, 0.72), look=(0.14, 0.0, 0.30)):
        self.ren = vtk.vtkRenderer()
        self.ren.SetBackground(0.95, 0.95, 0.96)
        self.ren.SetBackground2(0.82, 0.84, 0.88)
        self.ren.GradientBackgroundOn()
        self.actors = {}
        for i, name in enumerate(LINKS):
            pd = _polydata(URDF_DIR / "meshes" / "visual" / f"{name}.stl")
            nrm = vtk.vtkPolyDataNormals()
            nrm.SetInputData(pd)
            nrm.ConsistencyOn()
            nrm.SplittingOff()
            nrm.Update()
            mp = vtk.vtkPolyDataMapper()
            mp.SetInputConnection(nrm.GetOutputPort())
            mp.ScalarVisibilityOff()
            act = vtk.vtkActor()
            act.SetMapper(mp)
            p = act.GetProperty()
            p.SetColor(*PALETTE[i])
            p.SetSpecular(0.25)
            p.SetSpecularPower(24)
            p.SetAmbient(0.30)
            p.SetDiffuse(0.75)
            self.ren.AddActor(act)
            self.actors[name] = act

        # 地面网格 + 基座参考
        self.ground = vtk.vtkPlaneSource()
        self.ground.SetOrigin(-0.5, -0.5, -0.001)
        self.ground.SetPoint1(0.5, -0.5, -0.001)
        self.ground.SetPoint2(-0.5, 0.5, -0.001)
        gm = vtk.vtkPolyDataMapper()
        gm.SetInputConnection(self.ground.GetOutputPort())
        ga = vtk.vtkActor()
        ga.SetMapper(gm)
        ga.GetProperty().SetColor(0.88, 0.88, 0.90)
        ga.GetProperty().SetAmbient(0.6)
        self.ren.AddActor(ga)

        for pos, inten in [((2, -2, 3), 1.0), ((-2, -1.5, 1.5), 0.45)]:
            lt = vtk.vtkLight()
            lt.SetPosition(*pos)
            lt.SetFocalPoint(*look)
            lt.SetIntensity(inten)
            self.ren.AddLight(lt)
        self.ren.SetLightFollowCamera(False)

        cam_ = self.ren.GetActiveCamera()
        cam_.SetPosition(*cam)
        cam_.SetFocalPoint(*look)
        cam_.SetViewUp(0, 0, 1)
        cam_.SetViewAngle(28)
        self.ren.ResetCameraClippingRange()

        self.rw = vtk.vtkRenderWindow()
        self.rw.SetOffScreenRendering(1)
        self.rw.AddRenderer(self.ren)
        self.rw.SetSize(*size)
        self.size = size
        self.trace_pts = []
        self.trace_actor = None

    # -----------------------------------------------------------------
    def set_q(self, q):
        p, R = chain().fk(q)
        for i, name in enumerate(LINKS):
            T = np.eye(4)
            T[:3, :3] = R[i]
            T[:3, 3] = p[i]
            vm = vtk.vtkMatrix4x4()
            for r in range(4):
                for c in range(4):
                    vm.SetElement(r, c, float(T[r, c]))
            # base_link 在 base 系里本来就恒定，但仍按 FK 放置
            self.actors[name].SetUserMatrix(vtk.vtkMatrix4x4().DeepCopy(vm))
        return p[-1]

    def set_trace(self, points):
        if len(points) < 2:
            return
        pts = vtk.vtkPoints()
        for x, y, z in points:
            pts.InsertNextPoint(float(x), float(y), float(z))
        lines = vtk.vtkCellArray()
        lines.InsertNextCell(len(points))
        for i in range(len(points)):
            lines.InsertCellPoint(i)
        pd = vtk.vtkPolyData()
        pd.SetPoints(pts)
        pd.SetLines(lines)
        tube = vtk.vtkTubeFilter()
        tube.SetInputData(pd)
        tube.SetRadius(0.0018)
        tube.SetNumberOfSides(8)
        mp = vtk.vtkPolyDataMapper()
        mp.SetInputConnection(tube.GetOutputPort())
        if self.trace_actor is None:
            self.trace_actor = vtk.vtkActor()
            self.trace_actor.SetMapper(mp)
            self.trace_actor.GetProperty().SetColor(0.85, 0.15, 0.15)
            self.trace_actor.GetProperty().SetAmbient(0.4)
            self.ren.AddActor(self.trace_actor)
        else:
            self.trace_actor.SetMapper(mp)

    def grab(self):
        self.rw.Render()
        w2i = vtk.vtkWindowToImageFilter()
        w2i.SetInput(self.rw)
        w2i.SetInputBufferTypeToRGB()
        w2i.ReadFrontBufferOn()
        w2i.Update()
        img = w2i.GetOutput()
        w, h, _ = img.GetDimensions()
        arr = np.frombuffer(img.GetPointData().GetScalars(), dtype=np.uint8)
        arr = arr.reshape(h, w, 3)[::-1]
        return Image.fromarray(arr)


def frames_to_gif(frames, path, duration=60, loop=0):
    if not frames:
        return None
    frames[0].save(path, save_all=True, append_images=frames[1:],
                   duration=duration, loop=loop, optimize=True)
    return path
