import copy

import torch

from torch import nn
from torch.optim import Adam

from models.hra_policy_net import HRAPolicyNet, REWARD_COMPONENTS
from utilities.utils import landmark_motion, triangle_SDF, phi


class ReplayBuffer:
    """Flat circular buffer of per-robot transitions.

    Rewards are team-level scalars (Sec. 3.6), so every robot's transition at step k carries the
    same reward vector; the policy and critics are shared across the team.
    """

    def __init__(self, capacity, obs_dim, action_dim, num_components):
        self._capacity = int(capacity)
        self._obs = torch.zeros((self._capacity, obs_dim))
        self._action = torch.zeros((self._capacity, action_dim))
        self._reward = torch.zeros((self._capacity, num_components))
        self._next_obs = torch.zeros((self._capacity, obs_dim))
        self._done = torch.zeros(self._capacity)
        self._idx = 0
        self._size = 0

    def __len__(self):
        return self._size

    def add(self, obs, action, reward, next_obs, done):
        num = obs.size(0)
        for i in range(num):
            self._obs[self._idx] = obs[i]
            self._action[self._idx] = action[i]
            self._reward[self._idx] = reward
            self._next_obs[self._idx] = next_obs[i]
            self._done[self._idx] = float(done)
            self._idx = (self._idx + 1) % self._capacity
            self._size = min(self._size + 1, self._capacity)

    def sample(self, batch_size):
        idx = torch.randint(0, self._size, (min(batch_size, self._size),))
        return self._obs[idx], self._action[idx], self._reward[idx], self._next_obs[idx], self._done[idx]


class HRAAgent:
    """Decomposed Reward Architecture agent (Sec. 3.8).

    Unlike ModelBasedAgentAtt, which back-propagates the entropy objective through the
    differentiable filter, this agent is a model-free deterministic actor-critic (DDPG-style)
    with one Q-head per reward component. The belief update is therefore run without gradients
    and matches the baseline's filter exactly, so the two methods see identical belief dynamics.
    """

    def __init__(self, max_num_landmarks, init_info, A, B, W, radius, psi, kappa, V, lr, num_robots=2,
                 reward_weights=None, gamma=0.99, critic_lr=None, buffer_capacity=200000,
                 batch_size=256, updates_per_episode=50, warmup_transitions=2000, polyak=0.995,
                 expl_noise=0.2, noise_decay=0.999, min_expl_noise=0.05):
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

        # Scalarization weights of Eq. (3.32). The DRA does NOT use these to train: each Q-head sees
        # its own raw component reward. They are only used to report a single comparable episode
        # score alongside the linear baseline.
        weights = {'exp': 1.0, 'coord': 1.0, 'pers': 1.0}
        if reward_weights is not None:
            weights.update(reward_weights)
        self._log_weights = weights

        self._gamma = float(gamma)
        self._batch_size = int(batch_size)
        self._updates_per_episode = int(updates_per_episode)
        self._warmup_transitions = int(warmup_transitions)
        self._polyak = float(polyak)
        self._expl_noise = float(expl_noise)
        self._noise_decay = float(noise_decay)
        self._min_expl_noise = float(min_expl_noise)
        self._log_epsilon = 1e-6

        self._training = True
        self._num_landmarks = None

        obs_dim = max_num_landmarks * 5 + 3 * num_robots
        self._policy = HRAPolicyNet(max_num_landmarks=max_num_landmarks,
                                    num_other_robots=num_robots - 1,
                                    action_dim=2)
        self._target_q_heads = copy.deepcopy(self._policy.q_heads)
        for p in self._target_q_heads.parameters():
            p.requires_grad_(False)

        self._actor_optimizer = Adam(self._policy.actor_parameters(), lr=lr)
        self._critic_optimizer = Adam(self._policy.critic_parameters(),
                                      lr=lr if critic_lr is None else critic_lr)

        self._buffer = ReplayBuffer(capacity=buffer_capacity, obs_dim=obs_dim, action_dim=2,
                                    num_components=len(REWARD_COMPONENTS))

        # Transition awaiting its next observation, and the episode's reward bookkeeping.
        self._pending = None
        self._episode_component_sums = None
        self._reward_steps = 0
        self._prev_assignment = None
        self._info_prev_posterior = None
        self._last_critic_loss = None
        self._last_actor_loss = None

    # ------------------------------------------------------------------ episode setup

    def reset_estimate_mu(self, mu_real):
        self._num_landmarks = mu_real.size(0)
        self._mu_update = mu_real + torch.normal(mean=torch.zeros(self._num_landmarks, 2),
                                                 std=torch.sqrt(self._V))
        self._padding = torch.zeros(2 * (self._max_num_landmarks - self._num_landmarks))
        self._mask = torch.tensor([True] * self._num_landmarks +
                                  [False] * (self._max_num_landmarks - self._num_landmarks))

    def reset_agent_info(self):
        self._info = self._init_info * torch.ones((self._num_landmarks, 2))
        self._pending = None
        self._episode_component_sums = {name: 0.0 for name in REWARD_COMPONENTS}
        self._reward_steps = 0
        self._prev_assignment = None
        self._info_prev_posterior = None

    def eval_policy(self):
        self._training = False
        self._policy.eval()

    def train_policy(self):
        self._training = True
        self._policy.train()

    # ------------------------------------------------------------------ rollout

    def _build_observations(self, x):
        """Per-robot observation s_k of Eq. (3.23), in the layout the backbone expects."""
        num_robots = x.size(0)
        other_len = 3 * (self._num_robots - 1)
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
                other_robots_rel.append(torch.stack([rel_x, rel_y, x[j, 2] - theta]))

            if len(other_robots_rel) > 0:
                other_robots_input = torch.cat(other_robots_rel)
            else:
                other_robots_input = torch.zeros(0, device=x.device, dtype=x.dtype)
            pad_len = other_len - other_robots_input.numel()
            if pad_len > 0:
                other_robots_input = torch.cat((other_robots_input,
                                                torch.zeros(pad_len, device=x.device, dtype=x.dtype)))

            q_predict = torch.vstack((
                (self._mu_predict[:, 0] - x[i, 0]) * torch.cos(x[i, 2]) +
                (self._mu_predict[:, 1] - x[i, 1]) * torch.sin(x[i, 2]),
                (x[i, 0] - self._mu_predict[:, 0]) * torch.sin(x[i, 2]) +
                (self._mu_predict[:, 1] - x[i, 1]) * torch.cos(x[i, 2]))).T

            agent_pos_local = torch.zeros(3, device=x.device, dtype=x.dtype)
            observations.append(torch.hstack((agent_pos_local, other_robots_input, self._info.flatten(),
                                              self._padding, q_predict.flatten(), self._padding, self._mask)))

        return torch.stack(observations)

    def plan(self, v, x):
        # Snapshot Lambda_{k|k} before the prediction step: the exploration reward (Eq. 3.19) is
        # measured against the previous *posterior*, so that a target nobody observes yields a
        # negative reward once process noise has inflated its covariance.
        self._info_prev_posterior = self._info.detach().clone()

        with torch.no_grad():
            self._mu_predict = landmark_motion(self._mu_update, v, self._A, self._B)
            self._info = (self._info ** (-1) + self._W) ** (-1)

            if len(x.size()) == 1:
                x = x[None, :]
            observations = self._build_observations(x)
            _, raw_action = self._policy.raw_action(observations)

            if self._training:
                raw_action = (raw_action + self._expl_noise * torch.randn_like(raw_action)).clamp(-1.0, 1.0)

            action = self._policy.scale_action(raw_action)

        # The transition opened at the previous step can now be closed: this step's observation is
        # its next_obs.
        self._flush_pending(next_obs=observations, done=False)
        self._pending = {'obs': observations, 'action': raw_action, 'reward': None}

        return action[0] if action.size(0) == 1 else action

    def _flush_pending(self, next_obs, done):
        if self._pending is None or self._pending['reward'] is None:
            return
        if not self._training:
            self._pending = None
            return
        self._buffer.add(self._pending['obs'], self._pending['action'], self._pending['reward'],
                         next_obs, done)
        self._pending = None

    # ------------------------------------------------------------------ belief + rewards

    def update_info_mu(self, mu_real, x):
        if len(x.size()) == 1:
            x = x[None, :]
        num_robots = x.size(0)
        if num_robots == 0:
            return

        with torch.no_grad():
            dx = mu_real[:, 0].unsqueeze(0) - x[:, 0].unsqueeze(1)
            dy = mu_real[:, 1].unsqueeze(0) - x[:, 1].unsqueeze(1)
            c = torch.cos(x[:, 2]).unsqueeze(1)
            s = torch.sin(x[:, 2]).unsqueeze(1)
            q_real = torch.stack((dx * c + dy * s, -dx * s + dy * c), dim=2)
            sdf_real = triangle_SDF(q_real, self._psi, self._radius).reshape(num_robots, self._num_landmarks)
            visible = (sdf_real <= 0)

            noise = torch.normal(mean=torch.zeros((num_robots, self._num_landmarks, 2),
                                                  device=mu_real.device, dtype=mu_real.dtype),
                                 std=torch.sqrt(self._V))
            z = mu_real.unsqueeze(0) + noise

            weights = visible.unsqueeze(-1).to(mu_real.dtype)
            info_prior = self._info
            y_prior = info_prior * self._mu_predict
            meas_sum = (weights * z).sum(dim=0)
            info_post = info_prior + weights.sum(dim=0) * self._inv_V
            y_post = y_prior + meas_sum * self._inv_V
            self._mu_update = y_post / info_post.clamp_min(1e-8)

            dx_u = self._mu_update[:, 0].unsqueeze(0) - x[:, 0].unsqueeze(1)
            dy_u = self._mu_update[:, 1].unsqueeze(0) - x[:, 1].unsqueeze(1)
            q_update = torch.stack((dx_u * c + dy_u * s, -dx_u * s + dy_u * c), dim=2)
            sdf_update = triangle_SDF(q_update, self._psi, self._radius).reshape(num_robots, self._num_landmarks)
            M_total = (1 - phi(sdf_update, self._kappa)).sum(dim=0).unsqueeze(1) * self._inv_V
            self._info = self._info + M_total

            reward = self._compute_component_rewards(visible, x)
            self._mu_predict = self._mu_update

        if self._pending is not None:
            self._pending['reward'] = reward
        for i, name in enumerate(REWARD_COMPONENTS):
            self._episode_component_sums[name] += reward[i].item()
        self._reward_steps += 1

    def _compute_component_rewards(self, visible, x):
        """The three reward signals of Sec. 3.6, as a vector ordered like REWARD_COMPONENTS."""
        num_robots = x.size(0)
        num_landmarks = self._num_landmarks

        # r^exp, Eq. (3.19): mean per-target information gain. The belief is diagonal here, so
        # log det Lambda^m is the sum of the logs of its two diagonal entries.
        logdet_post = torch.log(self._info.clamp_min(self._log_epsilon)).sum()
        logdet_prev = torch.log(self._info_prev_posterior.clamp_min(self._log_epsilon)).sum()
        r_exp = (logdet_post - logdet_prev) / num_landmarks

        # r^coord, Eq. (3.20): redundant co-observation, normalised to [0, 1].
        overlap = (visible.sum(dim=0) - 1).clamp(min=0).to(x.dtype).sum()
        r_coord = overlap / (num_landmarks * max(num_robots - 1, 1))

        # r^pers, Eq. (3.21): fraction of targets whose nearest-robot assignment is unchanged.
        assignment = torch.argmin(torch.cdist(self._mu_update, x[:, :2]), dim=1)
        if self._prev_assignment is None:
            r_pers = torch.zeros((), dtype=x.dtype)
        else:
            r_pers = (assignment == self._prev_assignment).to(x.dtype).sum() / num_landmarks
        self._prev_assignment = assignment

        return torch.stack((r_exp.to(x.dtype), r_coord.to(x.dtype), r_pers.to(x.dtype)))

    # ------------------------------------------------------------------ two-stage training

    def set_policy_grad_to_zero(self):
        # Kept for interface parity with the baseline agents; each optimizer clears its own
        # gradients inside _train_step.
        pass

    def update_policy_grad(self, train=True):
        """Closes the episode and runs the two-stage updates. Returns a scalarized episode score
        (Eq. 3.32) purely for logging and checkpoint selection -- it is never differentiated."""
        if self._pending is not None and self._pending['reward'] is not None:
            self._flush_pending(next_obs=self._pending['obs'], done=True)
        self._pending = None

        if train and self._training and len(self._buffer) >= self._warmup_transitions:
            for _ in range(self._updates_per_episode):
                self._train_step()
            self._expl_noise = max(self._min_expl_noise, self._expl_noise * self._noise_decay)

        steps = max(self._reward_steps, 1)
        means = {name: self._episode_component_sums[name] / steps for name in REWARD_COMPONENTS}
        score = (self._log_weights['exp'] * means['exp']
                 + self._log_weights['pers'] * means['pers']
                 - self._log_weights['coord'] * means['coord'])
        self._episode_component_sums = {name: 0.0 for name in REWARD_COMPONENTS}
        self._reward_steps = 0
        return score

    def _train_step(self):
        obs, action, reward, next_obs, done = self._buffer.sample(self._batch_size)

        # --- Stage 1: critic update. The shared encoder and policy head are held fixed, so the
        # latents are computed without gradient and each Q-head fits its own component's TD target.
        with torch.no_grad():
            h = self._policy.latent(obs)
            next_h = self._policy.latent(next_obs)
            next_action = self._policy.policy_head(next_h)

        critic_loss = 0.0
        for i, name in enumerate(REWARD_COMPONENTS):
            with torch.no_grad():
                target_q = self._target_q_heads[name](next_h, next_action)
                y = reward[:, i] + self._gamma * (1.0 - done) * target_q
            q = self._policy.q_heads[name](h, action)
            critic_loss = critic_loss + nn.functional.mse_loss(q, y)

        self._critic_optimizer.zero_grad()
        critic_loss.backward()
        self._critic_optimizer.step()

        # --- Stage 2: actor update. Freeze the Q-heads so the gradient of Q^DRA flows through them
        # into the policy head and the shared encoder without changing the critics themselves.
        self._policy.q_heads.requires_grad_(False)
        h_actor = self._policy.latent(obs)
        actor_loss = -self._policy.aggregate_q(h_actor, self._policy.policy_head(h_actor)).mean()
        self._actor_optimizer.zero_grad()
        actor_loss.backward()
        self._actor_optimizer.step()
        self._policy.q_heads.requires_grad_(True)

        with torch.no_grad():
            for target_p, p in zip(self._target_q_heads.parameters(), self._policy.q_heads.parameters()):
                target_p.mul_(self._polyak).add_(p, alpha=1.0 - self._polyak)

        self._last_critic_loss = critic_loss.item()
        self._last_actor_loss = actor_loss.item()

    def policy_step(self, debug=False):
        # The baseline accumulates gradients across a batch of episodes and steps here; the DRA
        # already stepped inside update_policy_grad, so this only reports the latest losses.
        if debug and self._last_critic_loss is not None:
            print("Critic loss: {:.4f} | Actor loss: {:.4f} | Exploration noise: {:.3f}".format(
                self._last_critic_loss, self._last_actor_loss, self._expl_noise))

    def latest_losses(self):
        return self._last_critic_loss, self._last_actor_loss

    # ------------------------------------------------------------------ checkpointing

    def get_policy_state_dict(self):
        return self._policy.state_dict()

    def load_policy_state_dict(self, load_model):
        self._policy.load_state_dict(torch.load(load_model))
        self._target_q_heads = copy.deepcopy(self._policy.q_heads)
        for p in self._target_q_heads.parameters():
            p.requires_grad_(False)
