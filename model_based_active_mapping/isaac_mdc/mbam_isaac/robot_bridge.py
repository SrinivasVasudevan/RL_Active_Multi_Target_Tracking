"""Map the MBAM SE(2) policy onto MDC drones (Isaac).

The policy outputs a body-frame (linear velocity, angular velocity) pair, which
we integrate with the same SE(2) kinematics used in training to get a pose
setpoint per control interval. Two ways to realise that setpoint:

KinematicRobotBridge (control_mode="kinematic")
    Teleports a visual iris prim to the commanded pose. No physics, no PX4.
    Pose tracking is exact, so the achieved trajectory - and therefore every
    reported number - matches the synthetic evaluation. Use this when the point
    is "same policy, different renderer/scene".

OffboardRobotBridge (control_mode="offboard")
    Full MDC stack: Pegasus Multirotor + PX4 backend + OffboardController,
    streaming SET_POSITION_TARGET_LOCAL_NED setpoints. Realistic flight
    dynamics; the drone lags the 1 s setpoints, so tracking numbers will differ
    from the thesis. Use this to measure how much the flight stack costs you.

Both expose the same two calls: `command_pose(x_se2)` and `read_pose()`.
"""

from __future__ import annotations

import os
from typing import Optional

import numpy as np


class KinematicRobotBridge:
    """Visual-only drone prim teleported to the commanded SE(2) pose."""

    def __init__(self, index: int, altitude: float, asset_path: str,
                 root_path: str = "/World/mbam_robots", scale: float = 6.0):
        import isaaclab.sim as sim_utils
        from pxr import UsdGeom
        import omni.usd

        self._stage = omni.usd.get_context().get_stage()
        self._UsdGeom = UsdGeom
        self._path = f"{root_path}/robot_{index}"
        self._altitude = float(altitude)

        # The iris mesh is ~0.5 m across and the overhead camera sits tens of
        # metres up, so scale it up hard or the robots vanish in the footage.
        cfg = sim_utils.UsdFileCfg(usd_path=asset_path, scale=(scale, scale, scale))
        cfg.func(self._path, cfg, translation=(0.0, 0.0, self._altitude))

        # A UsdFileCfg spawn creates translate / orient / scale ops. Reuse the
        # existing ops - appending a RotateZ on top of an orient quaternion would
        # compose two rotations instead of replacing the yaw.
        from pxr import Gf
        self._Gf = Gf
        xform = UsdGeom.Xformable(self._stage.GetPrimAtPath(self._path))
        ops = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
        self._translate = next((o for n, o in ops.items() if "translate" in n), None) or xform.AddTranslateOp()
        self._orient = next((o for n, o in ops.items() if "orient" in n), None)

        self._pose = np.zeros(3, dtype=float)  # x, y, yaw

    def _set_yaw(self, yaw: float) -> None:
        if self._orient is None:
            return
        Gf = self._Gf
        half = float(yaw) / 2.0
        w, z = float(np.cos(half)), float(np.sin(half))
        try:
            self._orient.Set(Gf.Quatf(w, Gf.Vec3f(0.0, 0.0, z)))
        except Exception:
            self._orient.Set(Gf.Quatd(w, Gf.Vec3d(0.0, 0.0, z)))

    def command_pose(self, x_se2) -> None:
        """x_se2: (x, y, yaw) in world ENU metres / radians."""
        x, y, yaw = float(x_se2[0]), float(x_se2[1]), float(x_se2[2])
        self._translate.Set((x, y, self._altitude))
        self._set_yaw(yaw)
        self._pose[:] = (x, y, yaw)

    def read_pose(self) -> np.ndarray:
        """Achieved pose. Exact by construction in kinematic mode."""
        return self._pose.copy()

    def reset(self, x_se2) -> None:
        self.command_pose(x_se2)

    def close(self) -> None:
        pass


class OffboardRobotBridge:
    """Real MDC drone: Pegasus + PX4, driven by position setpoints."""

    def __init__(self, parent_env, index: int, altitude: float, init_pose=(0.0, 0.0, 0.0)):
        from drone.backends.px4_backend import PX4Backend
        from drone.control.offboard_controller import OffboardController
        from drone.drone_controller import DroneController
        from pegasus.simulator.logic import PegasusInterface

        self._altitude = float(altitude)
        pg = PegasusInterface()
        pg._world = parent_env.world

        backend = PX4Backend(pg, vehicle_id=index)
        self.controller = DroneController(
            parent_env=parent_env,
            backend=backend,
            stage_prefix=f"/World/drone_{index}",
            init_pos=(float(init_pose[0]), float(init_pose[1]), self._altitude),
            init_rot=(0.0, 0.0, float(np.rad2deg(init_pose[2]))),
        )
        self.controller.set_controller(
            OffboardController(self.controller.drone,
                               target_pos=(float(init_pose[0]), float(init_pose[1]), self._altitude),
                               target_yaw_deg=float(np.rad2deg(init_pose[2])))
        )

    def command_pose(self, x_se2) -> None:
        ctrl = self.controller.controller
        ctrl.set_target_position((float(x_se2[0]), float(x_se2[1]), self._altitude))
        ctrl.set_target_yaw_deg(float(np.rad2deg(x_se2[2])))

    def read_pose(self) -> np.ndarray:
        """Achieved pose, read back from the Pegasus vehicle state."""
        state = self.controller.drone._state
        pos = np.asarray(state.position, dtype=float)
        # Pegasus reports orientation as a quaternion (x, y, z, w) in ENU.
        q = np.asarray(state.attitude, dtype=float)
        yaw = np.arctan2(2.0 * (q[3] * q[2] + q[0] * q[1]),
                         1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2))
        return np.array([pos[0], pos[1], yaw], dtype=float)

    def reset(self, x_se2) -> None:
        self.command_pose(x_se2)

    def post_init(self) -> None:
        self.controller.post_init()

    def post_step(self) -> None:
        self.controller.post_step(None)

    def close(self) -> None:
        self.controller.close()


def make_bridges(cfg, parent_env, init_poses, asset_path: Optional[str] = None):
    """Build one bridge per robot according to cfg.control_mode."""
    if cfg.control_mode == "kinematic":
        if asset_path is None:
            from utils import get_project_path
            asset_path = os.path.join(get_project_path(), "assets", "drones", "iris.usd")
        return [KinematicRobotBridge(i, cfg.flight_altitude, asset_path, scale=cfg.robot_scale)
                for i in range(cfg.num_robots)]
    if cfg.control_mode == "offboard":
        return [OffboardRobotBridge(parent_env, i, cfg.flight_altitude, init_poses[i])
                for i in range(cfg.num_robots)]
    raise ValueError(f"unknown control_mode {cfg.control_mode!r} (expected 'kinematic' or 'offboard')")
