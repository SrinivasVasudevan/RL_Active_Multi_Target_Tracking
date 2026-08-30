"""Decomposed critic heads for the Decomposed Reward Architecture (thesis §3.8).

Implements blocks D8-D11 of Figure 3.4:

    D8  Q-head (exploration)   Q^exp(h, u)
    D9  Q-head (coordination)  Q^coord(h, u)
    D10 Q-head (persistence)   Q^pers(h, u)
    D11 Q aggregation          Q^DRA(h, u) = sum_j Q^j(h, u)          (Eq. 3.37)

Each head is a two-layer MLP over the concatenation [h_k ; u] and outputs a
scalar estimate of the discounted return of *its own* reward component only
(Eq. 3.33). Keeping the heads separate is the whole point: the exploration head
cannot be swamped by the persistence gradient, and vice versa.

Each head carries a slowly-synchronised target copy Q^j_theta_bar, used to form
the per-head TD target of Eq. (3.34).
"""

from __future__ import annotations

import copy
from typing import Dict, Iterable, List

import torch
from torch import nn

# Component order is fixed so state dicts and reward vectors stay aligned.
COMPONENTS: List[str] = ["exp", "coord", "pers"]


class QHead(nn.Module):
    """Two-layer MLP critic head: [h ; u] -> scalar action-value."""

    def __init__(self, latent_dim: int, action_dim: int, hidden_dim: int = 64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(latent_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1),
        )

    def forward(self, latent: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        return self.net(torch.cat((latent, action), dim=-1)).squeeze(-1)


class DecomposedCritic(nn.Module):
    """The three Q-heads (D8-D10) plus their aggregation (D11) and targets."""

    def __init__(self, latent_dim: int, action_dim: int, hidden_dim: int = 64,
                 components: Iterable[str] = COMPONENTS):
        super().__init__()
        self.components = list(components)
        self.heads = nn.ModuleDict({
            name: QHead(latent_dim, action_dim, hidden_dim) for name in self.components
        })
        # Target network: a frozen copy synchronised by polyak_update().
        self.target_heads = copy.deepcopy(self.heads)
        for p in self.target_heads.parameters():
            p.requires_grad_(False)

    def q_values(self, latent: torch.Tensor, action: torch.Tensor,
                 target: bool = False) -> Dict[str, torch.Tensor]:
        """Per-component Q values. Returns {component: [B]}."""
        heads = self.target_heads if target else self.heads
        return {name: heads[name](latent, action) for name in self.components}

    def q_aggregate(self, latent: torch.Tensor, action: torch.Tensor,
                    target: bool = False) -> torch.Tensor:
        """D11: Q^DRA = sum_j Q^j  (Eq. 3.37)."""
        qs = self.q_values(latent, action, target=target)
        return sum(qs[name] for name in self.components)

    @torch.no_grad()
    def polyak_update(self, tau: float = 0.005) -> None:
        """Slowly synchronise the target heads towards the live heads.

        The thesis calls for "a periodically synchronized target network"; a
        Polyak average is the standard continuous form of that and is what DDPG
        (which §3.8.3 names as the analogue) uses.
        """
        for name in self.components:
            live = self.heads[name]
            targ = self.target_heads[name]
            for p_t, p in zip(targ.parameters(), live.parameters()):
                p_t.mul_(1.0 - tau).add_(tau * p)

    @torch.no_grad()
    def hard_update(self) -> None:
        for name in self.components:
            self.target_heads[name].load_state_dict(self.heads[name].state_dict())
