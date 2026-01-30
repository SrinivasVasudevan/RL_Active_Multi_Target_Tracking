import torch

from torch import tensor
from torch.optim import SGD, Adam
from models.policy_net import PolicyNet
from models.policy_net_att import PolicyNetAtt
from utilities.utils import landmark_motion, triangle_SDF, get_transformation, phi


class ModelBasedAgent:

    def __init__(self, num_landmarks, init_info, A, B, W, radius, psi, kappa, V, lr, num_robots=2):
        self._init_info = init_info
        self._info = None

        self._num_robots = num_robots
        self._num_landmarks = num_landmarks
        self._A = A
        self._B = B
        self._W = W
        self._psi = psi
        self._radius = radius
        self._kappa = kappa
        self._V = V
        self._inv_V = V ** (-1)

        # input_dim = num_landmarks * 4 + 3
        input_dim = num_landmarks * 4 + 3 * self._num_robots
        self._policy = PolicyNet(input_dim=input_dim, policy_dim=2 * self._num_robots, num_robots=self._num_robots)

        self._policy_optimizer = Adam(self._policy.parameters(), lr=lr)

    def reset_agent_info(self):
        self._info = self._init_info * torch.ones((self._num_landmarks, 2))

    def reset_estimate_mu(self, mu_real):
        self._mu_update = mu_real + torch.normal(mean=torch.zeros(self._num_landmarks, 2), std=torch.sqrt(self._V))  # with the shape of (num_landmarks, 2)

    def eval_policy(self):
        self._policy.eval()

    def train_policy(self):
        self._policy.train()

    def plan(self, v, x):
        self._mu_predict = torch.clip(landmark_motion(self._mu_update, v, self._A, self._B),
                                      min=-tensor([self._num_landmarks, self._num_landmarks]),
                                      max=tensor([self._num_landmarks, self._num_landmarks]))
        self._info = (self._info**(-1) + self._W)**(-1)

        if len(x.size()) == 1:
            x = x[None, :]

        x_ref = x[0]
        q_predict = torch.vstack(((self._mu_predict[:, 0] - x_ref[0]) * torch.cos(x_ref[2]) + (self._mu_predict[:, 1] - x_ref[1]) * torch.sin(x_ref[2]),
                          (x_ref[0] - self._mu_predict[:, 0]) * torch.sin(x_ref[2]) + (self._mu_predict[:, 1] - x_ref[1]) * torch.cos(x_ref[2]))).T

        # net_input = torch.hstack((x, self._info.flatten(), next_mu.flatten()))
        agent_pos_local = torch.zeros(3, device=x.device, dtype=x.dtype)
        other_robots = []
        max_other = min(self._num_robots - 1, x.size(0) - 1)
        for i in range(max_other):
            other_pose = x[i + 1]
            dx = other_pose[0] - x_ref[0]
            dy = other_pose[1] - x_ref[1]
            theta = x_ref[2]
            rel_x = dx * torch.cos(theta) + dy * torch.sin(theta)
            rel_y = -dx * torch.sin(theta) + dy * torch.cos(theta)
            rel_theta = other_pose[2] - theta
            other_robots.append(torch.stack((rel_x, rel_y, rel_theta)))
        if len(other_robots) > 0:
            other_robots_input = torch.cat(other_robots)
        else:
            other_robots_input = torch.zeros(0, device=x.device, dtype=x.dtype)
        missing = (self._num_robots - 1) - max_other
        if missing > 0:
            other_robots_input = torch.cat((other_robots_input, torch.zeros(3 * missing, device=x.device, dtype=x.dtype)))

        net_input = torch.hstack((agent_pos_local, other_robots_input, self._info.flatten(), q_predict.flatten()))
        # net_input = q.flatten()
        action = self._policy.forward(net_input)
        return action

    def update_info_mu(self, mu_real, x):
        if len(x.size()) == 1:
            x = x[None, :]

        num_robots = x.size(0)
        if num_robots == 0:
            return

        dx = mu_real[:, 0].unsqueeze(0) - x[:, 0].unsqueeze(1)
        dy = mu_real[:, 1].unsqueeze(0) - x[:, 1].unsqueeze(1)
        c = torch.cos(x[:, 2]).unsqueeze(1)
        s = torch.sin(x[:, 2]).unsqueeze(1)
        q_real = torch.stack((dx * c + dy * s,
                              -dx * s + dy * c), dim=2)
        sdf_real = triangle_SDF(q_real, self._psi, self._radius).reshape(num_robots, self._num_landmarks)
        visible = (sdf_real <= 0).unsqueeze(-1)

        noise = torch.normal(mean=torch.zeros((num_robots, self._num_landmarks, 2), device=mu_real.device, dtype=mu_real.dtype),
                             std=torch.sqrt(self._V))
        z = mu_real.unsqueeze(0) + noise

        weights = visible.to(mu_real.dtype)
        sum_weights = weights.sum(dim=0)

        info_prior = self._info
        y_prior = info_prior * self._mu_predict
        meas_sum = (weights * z).sum(dim=0)

        info_post = info_prior + sum_weights * self._inv_V
        info_post_safe = info_post.clamp_min(1e-8)
        y_post = y_prior + meas_sum * self._inv_V
        self._mu_update = y_post / info_post_safe

        dx_u = self._mu_update[:, 0].unsqueeze(0) - x[:, 0].unsqueeze(1)
        dy_u = self._mu_update[:, 1].unsqueeze(0) - x[:, 1].unsqueeze(1)
        q_update = torch.stack((dx_u * c + dy_u * s,
                                -dx_u * s + dy_u * c), dim=2)
        sdf_update = triangle_SDF(q_update, self._psi, self._radius).reshape(num_robots, self._num_landmarks)
        weights_info = (1 - phi(sdf_update, self._kappa))
        M_total = weights_info.sum(dim=0).unsqueeze(1) * self._inv_V
        self._info = self._info + M_total
        self._mu_predict = self._mu_update

    # def update_policy(self, debug=False):
    #     self._policy_optimizer.zero_grad()
    #
    #     if debug:
    #         param_list = []
    #         grad_power = 0
    #         for i, p in enumerate(self._policy.parameters()):
    #             param_list.append(p.data.detach().clone())
    #             if p.grad is not None:
    #                 grad_power += (p.grad**2).sum()
    #             else:
    #                 grad_power += 0
    #
    #         print("Gradient power before backward: {}".format(grad_power))
    #
    #     reward = - torch.sum(torch.log(self._info))
    #     reward.backward()
    #     self._policy_optimizer.step()
    #
    #     if debug:
    #         grad_power = 0
    #         total_param_ssd = 0
    #         for i, p in enumerate(self._policy.parameters()):
    #             if p.grad is not None:
    #                 grad_power += (p.grad ** 2).sum()
    #             else:
    #                 grad_power += 0
    #             total_param_ssd += ((param_list[i] - p.data) ** 2).sum()
    #
    #         print("Gradient power after backward: {}".format(grad_power))
    #         print("SSD of weights after applying the gradient: {}".format(total_param_ssd))
    #
    #     return -reward.item()

    def set_policy_grad_to_zero(self):
        self._policy_optimizer.zero_grad()

    def update_policy_grad(self, train=True):
        reward = - torch.sum(torch.log(self._info))
        if train == True:
            reward.backward()
        return -reward.item()

    # def update_policy_grad(self, mu, x):
    #     reward = ((x[:2] - mu)**2).sum()
    #     reward.backward()
    #     return reward.item()

    def policy_step(self, debug=False):
        if debug:
            param_list = []
            for i, p in enumerate(self._policy.parameters()):
                param_list.append(p.data.detach().clone())

        self._policy_optimizer.step()

        if debug:
            total_param_rssd = 0
            grad_power = 0
            for i, p in enumerate(self._policy.parameters()):
                if p.grad is not None:
                    grad_power += (p.grad ** 2).sum()
                else:
                    grad_power += 0
                total_param_rssd += ((param_list[i] - p.data) ** 2).sum().sqrt()

            print("Gradient power after backward: {}".format(grad_power))
            print("RSSD of weights after applying the gradient: {}".format(total_param_rssd))

    def get_policy_state_dict(self):
        return self._policy.state_dict()

    def load_policy_state_dict(self, load_model):
        self._policy.load_state_dict(torch.load(load_model))


class ModelBasedAgentAtt:

    def __init__(self, max_num_landmarks, init_info, A, B, W, radius, psi, kappa, V, lr, num_robots=2):
        self._init_info = init_info
        self._info = None

        self._num_robots = num_robots
        self._max_num_landmarks = max_num_landmarks
        self._A = A
        self._B = B
        self._W = W
        self._psi = psi
        self._radius = radius
        self._kappa = kappa
        self._V = V
        self._inv_V = V ** (-1)

        # input_dim = num_landmarks * 4 + 3
        input_dim = max_num_landmarks * 5 + 3 * self._num_robots
        self._policy = PolicyNetAtt(input_dim=input_dim, policy_dim=2,
                                    num_other_robots=self._num_robots - 1, num_robots=self._num_robots)

        self._policy_optimizer = Adam(self._policy.parameters(), lr=lr)

    def reset_agent_info(self):
        self._info = self._init_info * torch.ones((self._num_landmarks, 2))

    def reset_estimate_mu(self, mu_real):
        self._num_landmarks = mu_real.size()[0]
        self._mu_update = mu_real + torch.normal(mean=torch.zeros(self._num_landmarks, 2), std=torch.sqrt(self._V))  # with the shape of (num_landmarks, 2)
        self._padding = torch.zeros(2 * (self._max_num_landmarks - self._num_landmarks))
        self._mask = torch.tensor([True] * self._num_landmarks + [False] * (self._max_num_landmarks - self._num_landmarks))

    def eval_policy(self):
        self._policy.eval()

    def train_policy(self):
        self._policy.train()

    def plan(self, v, x):
        # self._mu_predict = torch.clip(landmark_motion(self._mu_update, v, self._A, self._B),
        #                               min=-tensor([self._num_landmarks, self._num_landmarks]),
        #                               max=tensor([self._num_landmarks, self._num_landmarks]))
        self._mu_predict = landmark_motion(self._mu_update, v, self._A, self._B)
        self._info = (self._info**(-1) + self._W)**(-1)

        if len(x.size()) == 1:
            x = x[None, :]

        num_robots = x.size(0)
        target_other_len = 3 * (self._num_robots - 1)
        observations = []

        for i in range(num_robots):
            other_robots_rel = []
            for j in range(num_robots):
                if i == j:
                    continue
                dx = x[j, 0] - x[i, 0]
                dy = x[j, 1] - x[i, 1]
                theta = x[i, 2]
                rel_x = dx * torch.cos(theta) + dy * torch.sin(theta)
                rel_y = -dx * torch.sin(theta) + dy * torch.cos(theta)
                rel_theta = x[j, 2] - theta
                other_robots_rel.append(torch.stack([rel_x, rel_y, rel_theta]))

            if len(other_robots_rel) > 0:
                other_robots_input = torch.cat(other_robots_rel)
            else:
                other_robots_input = torch.zeros(0, device=x.device, dtype=x.dtype)

            pad_len = target_other_len - other_robots_input.numel()
            if pad_len > 0:
                other_robots_input = torch.cat((other_robots_input,
                                                torch.zeros(pad_len, device=x.device, dtype=x.dtype)))

            q_predict = torch.vstack(((self._mu_predict[:, 0] - x[i, 0]) * torch.cos(x[i, 2]) + (self._mu_predict[:, 1] - x[i, 1]) * torch.sin(x[i, 2]),
                              (x[i, 0] - self._mu_predict[:, 0]) * torch.sin(x[i, 2]) + (self._mu_predict[:, 1] - x[i, 1]) * torch.cos(x[i, 2]))).T

            agent_pos_local = torch.zeros(3, device=x.device, dtype=x.dtype)
            net_input = torch.hstack((agent_pos_local, other_robots_input, self._info.flatten(),
                                      self._padding, q_predict.flatten(), self._padding, self._mask))
            observations.append(net_input)

        batch_input = torch.stack(observations)
        actions = self._policy.forward(batch_input)
        return actions

    def update_info_mu(self, mu_real, x):
        if len(x.size()) == 1:
            x = x[None, :]

        num_robots = x.size(0)
        if num_robots == 0:
            return

        dx = mu_real[:, 0].unsqueeze(0) - x[:, 0].unsqueeze(1)
        dy = mu_real[:, 1].unsqueeze(0) - x[:, 1].unsqueeze(1)
        c = torch.cos(x[:, 2]).unsqueeze(1)
        s = torch.sin(x[:, 2]).unsqueeze(1)
        q_real = torch.stack((dx * c + dy * s,
                              -dx * s + dy * c), dim=2)
        sdf_real = triangle_SDF(q_real, self._psi, self._radius).reshape(num_robots, self._num_landmarks)
        visible = (sdf_real <= 0).unsqueeze(-1)

        noise = torch.normal(mean=torch.zeros((num_robots, self._num_landmarks, 2), device=mu_real.device, dtype=mu_real.dtype),
                             std=torch.sqrt(self._V))
        z = mu_real.unsqueeze(0) + noise

        weights = visible.to(mu_real.dtype)
        sum_weights = weights.sum(dim=0)

        info_prior = self._info
        y_prior = info_prior * self._mu_predict
        meas_sum = (weights * z).sum(dim=0)

        info_post = info_prior + sum_weights * self._inv_V
        info_post_safe = info_post.clamp_min(1e-8)
        y_post = y_prior + meas_sum * self._inv_V
        self._mu_update = y_post / info_post_safe

        dx_u = self._mu_update[:, 0].unsqueeze(0) - x[:, 0].unsqueeze(1)
        dy_u = self._mu_update[:, 1].unsqueeze(0) - x[:, 1].unsqueeze(1)
        q_update = torch.stack((dx_u * c + dy_u * s,
                                -dx_u * s + dy_u * c), dim=2)
        sdf_update = triangle_SDF(q_update, self._psi, self._radius).reshape(num_robots, self._num_landmarks)
        weights_info = (1 - phi(sdf_update, self._kappa))
        M_total = weights_info.sum(dim=0).unsqueeze(1) * self._inv_V
        self._info = self._info + M_total
        self._mu_predict = self._mu_update

    def set_policy_grad_to_zero(self):
        self._policy_optimizer.zero_grad()

    def update_policy_grad(self, train=True):
        reward = - torch.sum(torch.log(self._info))
        if train == True:
            reward.backward()
        return -reward.item()

    def policy_step(self, debug=False):
        if debug:
            param_list = []
            for i, p in enumerate(self._policy.parameters()):
                param_list.append(p.data.detach().clone())

        self._policy_optimizer.step()

        if debug:
            total_param_rssd = 0
            grad_power = 0
            for i, p in enumerate(self._policy.parameters()):
                if p.grad is not None:
                    grad_power += (p.grad ** 2).sum()
                else:
                    grad_power += 0
                total_param_rssd += ((param_list[i] - p.data) ** 2).sum().sqrt()

            print("Gradient power after backward: {}".format(grad_power))
            print("RSSD of weights after applying the gradient: {}".format(total_param_rssd))

    def get_policy_state_dict(self):
        return self._policy.state_dict()

    def load_policy_state_dict(self, load_model):
        self._policy.load_state_dict(torch.load(load_model))
