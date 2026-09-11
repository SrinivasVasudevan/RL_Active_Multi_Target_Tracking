"""Result recording for MBAM runs (Isaac).

Footage is produced from the simulator itself rather than an external screen
recorder, so the videos are reproducible and framed on the action.

Capture goes through a Replicator render product + `rgb` annotator rather than
`capture_viewport_to_file`. That path works with or without a GUI viewport,
hands back a numpy array directly (no PNG round-trip), and is synchronous with
the render, so a frame taken after `world.step(render=True)` is exactly the
state the policy just produced.
"""

from __future__ import annotations

import os
from typing import List, Optional, Tuple

import numpy as np


class OverheadCamera:
    """Top-down camera, re-framed per episode to fit that episode's world size."""

    def __init__(self, prim_path: str = "/World/mbam_cam", height: float = 30.0):
        from pxr import UsdGeom
        import omni.usd

        self._stage = omni.usd.get_context().get_stage()
        self._UsdGeom = UsdGeom
        self._path = prim_path

        cam = UsdGeom.Camera.Define(self._stage, prim_path)
        cam.CreateFocalLengthAttr(18.0)      # wide, so the whole arena fits
        cam.CreateHorizontalApertureAttr(36.0)
        cam.CreateVerticalApertureAttr(24.0)
        cam.CreateClippingRangeAttr((0.1, 10000.0))
        self._cam = cam

        # A freshly Define()d camera has no xform ops yet; add one translate op
        # and reuse it. USD cameras look down -Z, which is already straight down.
        xform = UsdGeom.Xformable(self._stage.GetPrimAtPath(prim_path))
        ops = [o for o in xform.GetOrderedXformOps()
               if o.GetOpType() == UsdGeom.XformOp.TypeTranslate]
        self._translate = ops[0] if ops else xform.AddTranslateOp()
        self._center = None
        self._smoothed_height = None
        self.set_height(height)

    @property
    def path(self) -> str:
        return self._path

    def set_height(self, height: float) -> None:
        self._height = float(height)
        cx, cy = (0.0, 0.0) if self._center is None else (float(self._center[0]), float(self._center[1]))
        self._translate.Set((cx, cy, self._height))

    def frame_env(self, env_size: float, margin: float = 1.25) -> None:
        """Set the height so a square arena of `env_size` metres fills the view."""
        self.set_height(self._height_for(env_size, margin))
        self._center = None
        self._smoothed_height = None

    def _height_for(self, extent: float, margin: float) -> float:
        focal = float(self._cam.GetFocalLengthAttr().Get())
        # vertical aperture is the smaller one, so it is the binding constraint
        aperture = float(self._cam.GetVerticalApertureAttr().Get())
        return max(6.0, (float(extent) * margin) * focal / aperture)

    def frame_points(self, points_xy, margin: float = 1.4, smooth: float = 0.35,
                     min_extent: float = 10.0) -> None:
        """Follow the action: centre on the entities and zoom to contain them.

        Targets drift under a motion bias and the robots chase them, so a camera
        pinned at the origin loses the action off-frame within a few steps. The
        centre and height are exponentially smoothed to keep the motion calm.
        """
        pts = np.asarray(points_xy, dtype=float).reshape(-1, 2)
        if pts.size == 0:
            return
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        centre = (lo + hi) / 2.0
        extent = max(float((hi - lo).max()), float(min_extent))
        height = self._height_for(extent, margin)

        if self._center is None:
            self._center, self._smoothed_height = centre, height
        else:
            a = float(smooth)
            self._center = (1.0 - a) * self._center + a * centre
            self._smoothed_height = (1.0 - a) * self._smoothed_height + a * height

        self._height = float(self._smoothed_height)
        self._translate.Set((float(self._center[0]), float(self._center[1]), self._height))

    def activate(self) -> None:
        """Point the GUI viewport at this camera, when there is one."""
        try:
            from omni.kit.viewport.utility import get_active_viewport
            vp = get_active_viewport()
            if vp is not None:
                vp.set_active_camera(self._path)
        except Exception as exc:  # pragma: no cover
            print(f"[MBAM recorder] could not activate viewport camera: {exc}")


class ViewportRecorder:
    """Grabs one RGB frame per policy step, writes one video per episode."""

    def __init__(self, output_dir: str, fps: int = 5, enabled: bool = True,
                 resolution: Tuple[int, int] = (960, 720), keep_frames: bool = False):
        self.output_dir = output_dir
        self.video_dir = os.path.join(output_dir, "videos")
        self.frames_dir = os.path.join(output_dir, "frames")
        self.fps = fps
        self.enabled = enabled
        self.resolution = resolution
        self.keep_frames = keep_frames

        self._annotator = None
        self._episode: Optional[int] = None
        self._frames: List[np.ndarray] = []

        if enabled:
            os.makedirs(self.video_dir, exist_ok=True)

    # -- setup -------------------------------------------------------------

    def attach(self, camera_path: str) -> None:
        """Create the render product feeding this recorder."""
        if not self.enabled:
            return
        try:
            import omni.replicator.core as rep
            render_product = rep.create.render_product(camera_path, self.resolution)
            self._annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            self._annotator.attach([render_product])
            print(f"[MBAM recorder] attached rgb annotator to {camera_path} at {self.resolution}")
        except Exception as exc:
            print(f"[MBAM recorder] could not attach render product, recording off: {exc}")
            self.enabled = False

    # -- capture -----------------------------------------------------------

    def start_episode(self, episode: int) -> None:
        self._episode = episode
        self._frames = []

    def capture(self) -> None:
        """Grab the current render. Call right after a render step."""
        if not self.enabled or self._annotator is None or self._episode is None:
            return
        try:
            frame = np.asarray(self._annotator.get_data())
            if frame.size == 0:
                return
            self._frames.append(frame[..., :3].copy())  # drop alpha
        except Exception as exc:
            print(f"[MBAM recorder] capture failed, recording off: {exc}")
            self.enabled = False

    # -- assembly ----------------------------------------------------------

    def finish_episode(self) -> Optional[str]:
        """Write this episode's frames to a video. Returns the path."""
        if not self.enabled or self._episode is None or not self._frames:
            return None

        try:
            import cv2
        except ImportError:
            print("[MBAM recorder] opencv missing, cannot assemble video")
            return None

        h, w = self._frames[0].shape[:2]
        out_path = os.path.join(self.video_dir, f"mbam_episode_{self._episode:03d}.mp4")
        writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, h))
        if not writer.isOpened():
            out_path = out_path.replace(".mp4", ".avi")
            writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"MJPG"), self.fps, (w, h))

        for f in self._frames:
            writer.write(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
        writer.release()

        if self.keep_frames:
            ep_dir = os.path.join(self.frames_dir, f"episode_{self._episode:03d}")
            os.makedirs(ep_dir, exist_ok=True)
            for i, f in enumerate(self._frames):
                cv2.imwrite(os.path.join(ep_dir, f"frame_{i:05d}.png"),
                            cv2.cvtColor(f, cv2.COLOR_RGB2BGR))

        n = len(self._frames)
        self._frames = []
        print(f"[MBAM recorder] wrote {out_path} ({n} frames)")
        return out_path
