import math
import os
from typing import List, Optional

import numpy as np
import rclpy
import torch
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node

from mbam_interfaces.msg import AgentObservation, TargetBeliefs
from central_controller.core.model_based_agent_att_ros import ModelBasedAgentAttRos


class CentralControllerNode(Node):
    def __init__(self):
        super().__init__('central_controller')

        self.declare_parameter('num_robots', 2)
        self.declare_parameter('num_targets', 3)
        self.declare_parameter('max_num_targets', 7)
        self.declare_parameter('params_file', '')
        self.declare_parameter('model_path', '')
        self.declare_parameter('tau', 1.0)
        self.declare_parameter('max_linear_vel', 0.15)
        self.declare_parameter('max_angular_vel', 0.40)
        self.declare_parameter('seed', 42)
        self.declare_parameter('arena_half_size', 5.0)

        self.num_robots = int(self.get_parameter('num_robots').value)
        self.num_targets = int(self.get_parameter('num_targets').value)
        self.max_num_targets = int(self.get_parameter('max_num_targets').value)
        params_file = self.get_parameter('params_file').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.tau = float(self.get_parameter('tau').value)
        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        seed = int(self.get_parameter('seed').value)
        arena_half = float(self.get_parameter('arena_half_size').value)

        torch.manual_seed(seed)
        np.random.seed(seed)

        if not os.path.exists(params_file):
            raise FileNotFoundError(f'params_file not found: {params_file}')
        if not os.path.exists(model_path):
            raise FileNotFoundError(f'model_path not found: {model_path}')

        with open(params_file, 'r', encoding='utf-8') as f:
            params = yaml.safe_load(f)

        a = torch.zeros((2, 2))
        a[0, 0] = params['motion']['A']['_1']
        a[1, 1] = params['motion']['A']['_2']
        b = torch.zeros((2, 2))
        b[0, 0] = params['motion']['B']['_1']
        b[1, 1] = params['motion']['B']['_2']
        w = torch.zeros(2)
        w[0] = params['motion']['W']['_1']
        w[1] = params['motion']['W']['_2']
        v_noise = torch.zeros(2)
        v_noise[0] = params['FoV']['V']['_1']
        v_noise[1] = params['FoV']['V']['_2']

        self.agent = ModelBasedAgentAttRos(
            max_num_landmarks=self.max_num_targets,
            init_info=float(params['init_info']),
            a=a, b=b, w=w,
            radius=float(params['FoV']['radius']),
            psi=torch.tensor([float(params['FoV']['psi'])]),
            kappa=float(params['FoV']['kappa']),
            v_noise=v_noise,
            lr=float(params['lr']),
            num_robots=self.num_robots,
        )
        self.agent.load_policy_state_dict(model_path)
        self.agent.eval_policy()

        # Initialize Kalman state: random positions, high uncertainty (low init_info)
        init_mu = (torch.rand(self.num_targets, 2) - 0.5) * 2.0 * arena_half
        self.agent.reset_estimate_mu(init_mu)
        self.agent.reset_agent_info()

        # Target velocity assumed zero (autonomous, uncontrolled targets)
        self.v = torch.zeros(self.num_targets, 2)

        # Publishers
        self.cmd_pubs = [
            self.create_publisher(Twist, f'/agent_{i}/cmd_vel_desired', 10)
            for i in range(self.num_robots)
        ]
        self.beliefs_pub = self.create_publisher(TargetBeliefs, '/target_beliefs', 10)

        # Per-agent observation buffers
        self.obs_buf: List[Optional[AgentObservation]] = [None] * self.num_robots
        for i in range(self.num_robots):
            self.create_subscription(
                AgentObservation,
                f'/agent_{i}/observation',
                self._make_obs_cb(i),
                20,
            )

        self._first_step = True
        self.timer = self.create_timer(self.tau, self._control_loop)
        self.get_logger().info(
            f'CentralController ready | robots={self.num_robots} targets={self.num_targets} '
            f'tau={self.tau}s max_lin={self.max_linear_vel} max_ang={self.max_angular_vel}'
        )

    def _make_obs_cb(self, idx: int):
        def cb(msg: AgentObservation):
            self.obs_buf[idx] = msg
        return cb

    def _control_loop(self):
        x = self._build_pose_tensor()
        if x is None:
            self.get_logger().warn('Waiting for poses from all agents')
            return

        z_world, visible = self._build_obs_tensors()

        # Step 1: Kalman update with latest observations (skip on very first tick)
        if not self._first_step:
            self.agent.update_info_from_observations(z_world, visible, x)

        # Step 2: Kalman predict + policy forward pass
        actions = self.agent.plan(self.v, x)
        self._first_step = False

        # Step 3: Publish predicted beliefs for agent data association
        self._publish_beliefs()

        # Step 4: Clip + publish velocity commands
        self._publish_actions(actions)

        tracked = int(visible.any(dim=0).sum().item())
        self.get_logger().info(f'Control step | tracked={tracked}/{self.num_targets}')

    def _build_pose_tensor(self) -> Optional[torch.Tensor]:
        poses = []
        for obs in self.obs_buf:
            if obs is None:
                return None
            poses.append([obs.pose_x, obs.pose_y, obs.pose_yaw])
        return torch.tensor(poses, dtype=torch.float32)

    def _build_obs_tensors(self):
        z_world = torch.zeros(self.num_robots, self.num_targets, 2)
        visible = torch.zeros(self.num_robots, self.num_targets, dtype=torch.bool)
        for i, obs in enumerate(self.obs_buf):
            if obs is None:
                continue
            n = min(len(obs.visible), self.num_targets)
            for t in range(n):
                if obs.visible[t]:
                    z_world[i, t, 0] = float(obs.z_world_x[t])
                    z_world[i, t, 1] = float(obs.z_world_y[t])
                    visible[i, t] = True
        return z_world, visible

    def _publish_actions(self, actions: torch.Tensor):
        actions = actions.detach()
        if actions.dim() == 1:
            actions = actions.unsqueeze(0)
        for i in range(self.num_robots):
            if i >= actions.size(0):
                continue
            msg = Twist()
            # Policy already scales: linear in [0,2] m/s, angular in [-pi/6, pi/6]
            # We clip to safe slow speeds for real Limo hardware
            msg.linear.x = float(np.clip(float(actions[i, 0]), 0.0, self.max_linear_vel))
            msg.angular.z = float(np.clip(float(actions[i, 1]), -self.max_angular_vel, self.max_angular_vel))
            self.cmd_pubs[i].publish(msg)

    def _publish_beliefs(self):
        msg = TargetBeliefs()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.max_num_targets = self.num_targets
        mu = self.agent.mu_update.detach()
        msg.mean_x = [float(mu[t, 0]) for t in range(self.num_targets)]
        msg.mean_y = [float(mu[t, 1]) for t in range(self.num_targets)]
        msg.active_mask = [True] * self.num_targets
        self.beliefs_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CentralControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
