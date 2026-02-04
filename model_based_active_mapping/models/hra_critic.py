import torch
from torch import nn


class HRACritic(nn.Module):
    def __init__(self, state_dim: int, action_dim: int, num_heads: int = 4, hidden_dim: int = 256):
        super().__init__()
        self._num_heads = num_heads
        input_dim = state_dim + action_dim

        self.backbone = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
        )
        self.heads = nn.ModuleList([nn.Linear(hidden_dim, 1) for _ in range(num_heads)])

    def forward(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        if state.dim() == 1:
            state = state[None, :]
        if action.dim() == 1:
            action = action[None, :]

        x = torch.cat((state, action), dim=-1)
        features = self.backbone(x)
        q_values = [head(features) for head in self.heads]
        return torch.cat(q_values, dim=-1)
