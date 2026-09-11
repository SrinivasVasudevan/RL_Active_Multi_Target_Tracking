"""MBAM (model-based active mapping) integration for the Multi-Drone-Control Isaac Sim template.

The package is split so that everything policy-related is Isaac-free:

  config.py       - one dataclass holding the whole scenario (2 robots, N targets, FoV, ...)
  episode_sampler.py - episode sampling, bit-identical to MRMT/multi_robot_env.py
  coordinator.py  - the policy/estimator/statistics loop, no Isaac imports
  targets.py      - kinematic target prims (Isaac)
  robot_bridge.py - SE(2) pose -> drone setpoint (Isaac)
  mbam_env.py     - MBAMDroneEnv, ties the coordinator to Isaac (Isaac)

Only the last three touch Isaac, so `coordinator` can be exercised — and checked
against the thesis numbers — on a machine with no Isaac Sim installed.
"""
