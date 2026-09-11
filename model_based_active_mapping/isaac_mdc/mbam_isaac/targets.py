"""Kinematic target markers for the MBAM scenario (Isaac).

The thesis targets follow a biased random walk (`landmark_motion_real`), which is
the motion model the policy was trained against. The MDC template's crab/ant
agents move under their own RL policies instead, so using them as targets would
change the target dynamics and break comparability with the thesis. We therefore
drive plain kinematic markers from our own motion model and keep the animals out
of the loop; the crabs remain available as scenery.

Markers are pure USD prims - no physics, no articulation - so a teleport per
control step is exact and cheap.
"""

from __future__ import annotations

from typing import List, Optional

from .viz import TwoStateMarker


class TargetMarkers:
    """`max_num_landmarks` markers, teleported each control step.

    Targets beyond the current episode's landmark count are parked far away
    rather than destroyed, so the stage layout stays constant across episodes.
    """

    def __init__(self, num_markers: int, radius: float = 0.45,
                 root_path: str = "/World/mbam_targets", hide_distance: float = 1000.0,
                 altitude: float = 0.0):
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        self._hide_distance = float(hide_distance)
        self._altitude = float(altitude)
        self._markers: List[TwoStateMarker] = [
            TwoStateMarker(stage, f"{root_path}/target_{i}", radius) for i in range(num_markers)
        ]
        for m in self._markers:
            m.set_pose(self._hide_distance, self._hide_distance, self._altitude)

    def update(self, mu_real, visible_any=None) -> None:
        """Place the active targets and colour them; park the rest.

        Args:
            mu_real: [num_landmarks x 2] world-frame target positions.
            visible_any: optional [num_landmarks] bool, True if any robot sees it.
        """
        n = int(mu_real.shape[0])
        for i, m in enumerate(self._markers):
            if i < n:
                m.set_pose(float(mu_real[i, 0]), float(mu_real[i, 1]), self._altitude)
                if visible_any is not None:
                    m.set_tracked(bool(visible_any[i]))
            else:
                m.set_pose(self._hide_distance, self._hide_distance, self._altitude)
