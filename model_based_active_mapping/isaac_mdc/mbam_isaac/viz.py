"""Scene visuals for MBAM result footage (Isaac).

Two things make a top-down tracking video readable, and neither comes for free:

  FovWedge  - the sensor footprint. The policy's whole job is to keep targets
              inside these wedges, so without them the video shows drones
              wandering for no visible reason. The wedge geometry is the exact
              triangle used by `triangle_SDF`: apex at the robot, reaching
              `radius` forward with half-angle `psi`.

  TwoStateMarker - tracked/untracked colouring. Setting `displayColor` on a prim
              that already has a PreviewSurface material bound does nothing (the
              material wins), so instead each marker is a pair of differently
              coloured prims and we toggle visibility. Crude, but it always works.
"""

from __future__ import annotations

import numpy as np


def _yaw_quat(Gf, yaw: float):
    half = float(yaw) / 2.0
    return Gf.Quatf(float(np.cos(half)), Gf.Vec3f(0.0, 0.0, float(np.sin(half))))


class PoseXform:
    """A prim whose translate/orient ops are cached for cheap per-step updates."""

    def __init__(self, stage, path: str):
        from pxr import UsdGeom, Gf

        self._Gf = Gf
        xform = UsdGeom.Xformable(stage.GetPrimAtPath(path))
        ops = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
        self._t = next((o for n, o in ops.items() if "translate" in n), None) or xform.AddTranslateOp()
        self._o = next((o for n, o in ops.items() if "orient" in n), None)
        if self._o is None:
            self._o = xform.AddOrientOp()

    def set(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        self._t.Set((float(x), float(y), float(z)))
        try:
            self._o.Set(_yaw_quat(self._Gf, yaw))
        except Exception:
            from pxr import Gf
            half = float(yaw) / 2.0
            self._o.Set(Gf.Quatd(float(np.cos(half)), Gf.Vec3d(0.0, 0.0, float(np.sin(half)))))


class FovWedge:
    """Flat triangle showing one robot's sensor footprint, following its pose."""

    def __init__(self, stage, path: str, radius: float, psi: float,
                 color=(0.2, 0.7, 1.0), opacity: float = 0.35, altitude: float = 0.05):
        from pxr import UsdGeom, Vt, Gf

        half_width = float(radius) * float(np.tan(psi))
        pts = [Gf.Vec3f(0.0, 0.0, 0.0),
               Gf.Vec3f(float(radius), float(half_width), 0.0),
               Gf.Vec3f(float(radius), float(-half_width), 0.0)]

        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3]))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2]))
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))
        mesh.CreateDisplayOpacityAttr(Vt.FloatArray([float(opacity)]))

        self._altitude = float(altitude)
        self._pose = PoseXform(stage, path)

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        self._pose.set(x, y, self._altitude, yaw)


class TwoStateMarker:
    """A target marker that switches colour by toggling two child prims."""

    def __init__(self, stage, path: str, radius: float,
                 color_off=(0.95, 0.45, 0.05), color_on=(0.1, 0.85, 0.2)):
        import isaaclab.sim as sim_utils
        from pxr import UsdGeom

        UsdGeom.Xform.Define(stage, path)
        self._pose = PoseXform(stage, path)

        for name, color in (("off", color_off), ("on", color_on)):
            cfg = sim_utils.SphereCfg(
                radius=radius,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color),
            )
            cfg.func(f"{path}/{name}", cfg)

        self._on = UsdGeom.Imageable(stage.GetPrimAtPath(f"{path}/on"))
        self._off = UsdGeom.Imageable(stage.GetPrimAtPath(f"{path}/off"))
        self._state = None
        self.set_tracked(False)

    def set_pose(self, x: float, y: float, z: float) -> None:
        self._pose.set(x, y, z, 0.0)

    def set_tracked(self, tracked: bool) -> None:
        if tracked == self._state:
            return
        (self._on if tracked else self._off).MakeVisible()
        (self._off if tracked else self._on).MakeInvisible()
        self._state = tracked
