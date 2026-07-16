import torch

from torch import nn

# Reward components of the decomposition (Sec. 3.6). Order fixes the layout of the reward vector
# stored in the replay buffer and of the per-head TD targets.
REWARD_COMPONENTS = ('exp', 'coord', 'pers')


class HRABackbone(nn.Module):
    """Shared masked-attention encoder g_omega, blocks D1-D7 of Fig. 3.4.

    Functionally identical to blocks B1-B7 of the baseline PolicyNetAtt: it is kept as a separate
    module so the baseline stays checkpoint-compatible. The observation layout is the one built by
    the agent's plan(): [robot pose (3 + 3 * num_other_robots), info matrices (2 * max_num_landmarks),
    predicted target means (2 * max_num_landmarks), padding mask (max_num_landmarks)].
    """

    def __init__(self, max_num_landmarks: int, num_other_robots: int = 0):
        super(HRABackbone, self).__init__()

        self.num_landmark = max_num_landmarks
        self._agent_pos_dim = 3 + 3 * num_other_robots

        self.agent_pos_fc1 = nn.Linear(self._agent_pos_dim, 32)
        self.agent_pos_fc2 = nn.Linear(32, 32)
        self.landmark_fc1 = nn.Linear(4, 64)
        self.landmark_fc2 = nn.Linear(64, 32)
        self.fusion_fc = nn.Linear(64, 64)

        self.relu = nn.ReLU()
        self.softmax = nn.Softmax(dim=2)

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        if observation.dim() == 1:
            observation = observation[None, :]
        batch = observation.size(0)

        # D1: robot encoder -> attention query.
        agent_embedding = self.relu(self.agent_pos_fc1(observation[:, :self._agent_pos_dim]))
        agent_embedding = self.relu(self.agent_pos_fc2(agent_embedding))

        # D2: target encoder over the per-target features [mu_predict; info].
        info_vector = observation[:, self._agent_pos_dim: self._agent_pos_dim + 2 * self.num_landmark]
        landmark_pos = observation[:, self._agent_pos_dim + 2 * self.num_landmark: -self.num_landmark]
        landmark_features = torch.cat((landmark_pos.reshape(batch, self.num_landmark, 2),
                                       info_vector.reshape(batch, self.num_landmark, 2)), 2)
        landmark_embedding = self.relu(self.landmark_fc1(landmark_features))
        landmark_embedding = self.relu(self.landmark_fc2(landmark_embedding))

        # D4-D6: masked dot-product attention over the active target slots.
        landmark_embedding_tr = torch.transpose(landmark_embedding, 1, 2)
        mask = observation[:, -self.num_landmark:].unsqueeze(1)
        attention = torch.matmul(agent_embedding.unsqueeze(1), landmark_embedding_tr) / 4
        attention = attention.masked_fill(mask == 0, -1e10)
        att = self.softmax(attention)
        landmark_context = self.relu(torch.matmul(att, landmark_embedding).squeeze(1))

        # D7: shared latent h_k re-used by the policy head and every Q-head.
        return self.relu(self.fusion_fc(torch.cat((agent_embedding, landmark_context), 1)))


class HRAPolicyHead(nn.Module):
    """Deterministic policy head pi_phi, block D12. Maps h_k to a raw action in [-1, 1]^2."""

    def __init__(self, latent_dim: int = 64, action_dim: int = 2):
        super(HRAPolicyHead, self).__init__()
        self.fc1 = nn.Linear(latent_dim, 64)
        self.fc2 = nn.Linear(64, action_dim)
        self.tanh = nn.Tanh()

    def forward(self, h: torch.Tensor) -> torch.Tensor:
        return self.tanh(self.fc2(self.tanh(self.fc1(h))))


class HRAQHead(nn.Module):
    """One critic head per reward component, blocks D8-D10.

    Q_theta_j(h_k, u) estimates the expected discounted return of component j alone (Eq. 3.33).
    The action is consumed in raw [-1, 1] units, before f_scale.
    """

    def __init__(self, latent_dim: int = 64, action_dim: int = 2):
        super(HRAQHead, self).__init__()
        self.fc1 = nn.Linear(latent_dim + action_dim, 64)
        self.fc2 = nn.Linear(64, 1)
        self.relu = nn.ReLU()

    def forward(self, h: torch.Tensor, u: torch.Tensor) -> torch.Tensor:
        return self.fc2(self.relu(self.fc1(torch.cat((h, u), dim=-1)))).squeeze(-1)


class HRAPolicyNet(nn.Module):
    """Decomposed Reward Architecture (Fig. 3.4, blocks D1-D13).

    A shared attention backbone feeds one independent Q-head per reward component and a dedicated
    deterministic policy head. Q-heads are trained on their own component reward only (Stage 1) so
    that no objective's gradient can suppress another's; the policy is then updated through the
    frozen aggregate Q^DRA (Stage 2). See HRAAgent for the training procedure.
    """

    def __init__(self, max_num_landmarks: int, num_other_robots: int = 0, action_dim: int = 2):
        super(HRAPolicyNet, self).__init__()

        self.action_dim = action_dim
        self.backbone = HRABackbone(max_num_landmarks=max_num_landmarks, num_other_robots=num_other_robots)
        self.policy_head = HRAPolicyHead(action_dim=action_dim)
        self.q_heads = nn.ModuleDict({name: HRAQHead(action_dim=action_dim) for name in REWARD_COMPONENTS})

    def actor_parameters(self):
        """Parameters updated in Stage 2: the shared encoder (D1-D7) and the policy head (D12)."""
        return list(self.backbone.parameters()) + list(self.policy_head.parameters())

    def critic_parameters(self):
        """Parameters updated in Stage 1: the Q-heads (D8-D10)."""
        return list(self.q_heads.parameters())

    def latent(self, observation: torch.Tensor) -> torch.Tensor:
        return self.backbone(observation)

    def raw_action(self, observation: torch.Tensor):
        h = self.latent(observation)
        return h, self.policy_head(h)

    def q_values(self, h: torch.Tensor, u: torch.Tensor) -> dict:
        return {name: head(h, u) for name, head in self.q_heads.items()}

    def aggregate_q(self, h: torch.Tensor, u: torch.Tensor) -> torch.Tensor:
        """Q^DRA(u) = sum_j Q_theta_j(h_k, u), the D11 aggregation block (Eq. 3.37)."""
        return torch.stack([head(h, u) for head in self.q_heads.values()], dim=0).sum(dim=0)

    @staticmethod
    def scale_action(raw_action: torch.Tensor) -> torch.Tensor:
        """f_scale: map the raw tanh output to the same velocity ranges the baseline policy uses."""
        scaled_linear = (1 + raw_action[..., 0]) * 1.0
        scaled_angular = raw_action[..., 1] * torch.pi / 6
        return torch.stack((scaled_linear, scaled_angular), dim=-1)

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        """Deterministic control command, so a trained HRA net can be dropped in wherever the
        baseline policy's forward() is used."""
        _, raw = self.raw_action(observation)
        action = self.scale_action(raw)
        return action[0] if action.size(0) == 1 else action
