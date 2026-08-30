"""Staged warm-start training for the Decomposed Reward Architecture (thesis 3.8).

The DRA reuses the baseline's shared backbone (D1-D7) and policy head (D12)
verbatim - the thesis says so explicitly - so the only genuinely new parameters
are the three Q-heads. That makes a warm start the natural route:

  Phase 1  Critic fit. Freeze the backbone and the policy entirely and train only
           the Q-heads on per-component TD targets (Eq. 3.34/3.35). The policy
           cannot drift, so this is a pure regression problem: teach each head
           what the *existing* policy's per-component returns look like.

  Phase 2  Two-stage alternation (3.8.3). Stage 1 updates the Q-heads; Stage 2
           updates the policy head (and optionally the encoder) by ascending the
           aggregate Q (Eq. 3.37/3.38) with the heads frozen.

Usage:
    python3 run_hra_training.py --resume ./checkpoints/best_model_seed42_resume1.pth
"""

import argparse
import os
import sys
import time

repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(repo_root)
sys.path.append(os.path.join(repo_root, "MRMT"))

import numpy as np
import torch
import yaml
from torch import tensor

from multi_robot_env import MultiRobotEnv
from agents.model_based_agent import ModelBasedAgentAtt
from models.hra_critic import COMPONENTS, DecomposedCritic
from utilities.utils import triangle_SDF


# ----------------------------------------------------------------------
# Replay buffer
# ----------------------------------------------------------------------
class ReplayBuffer:
    """Flat transition store. Each robot row is one sample; the component
    rewards are team-level and broadcast to every robot in that step."""

    def __init__(self, capacity=200000):
        self.capacity = capacity
        self.obs, self.act, self.rew, self.next_obs, self.done = [], [], [], [], []

    def add_episode(self, records):
        for r in records:
            n = r['obs'].shape[0]
            self.obs.append(r['obs'])
            self.act.append(r['action'])
            self.next_obs.append(r['next_obs'])
            self.rew.append(torch.tensor([[r['rewards'][c] for c in COMPONENTS]] * n, dtype=torch.float32))
            self.done.append(torch.full((n,), float(r['done'])))
        self._trim()

    def _trim(self):
        while len(self.obs) > self.capacity:
            for buf in (self.obs, self.act, self.rew, self.next_obs, self.done):
                buf.pop(0)

    def __len__(self):
        return sum(o.shape[0] for o in self.obs)

    def tensors(self):
        return (torch.cat(self.obs), torch.cat(self.act), torch.cat(self.rew),
                torch.cat(self.next_obs), torch.cat(self.done))


# ----------------------------------------------------------------------
# Environment / agent construction
# ----------------------------------------------------------------------
def build(args, params):
    A = torch.zeros((2, 2)); A[0, 0] = params['motion']['A']['_1']; A[1, 1] = params['motion']['A']['_2']
    B = torch.zeros((2, 2)); B[0, 0] = params['motion']['B']['_1']; B[1, 1] = params['motion']['B']['_2']
    W = torch.zeros(2); W[0] = params['motion']['W']['_1']; W[1] = params['motion']['W']['_2']
    V = torch.zeros(2); V[0] = params['FoV']['V']['_1']; V[1] = params['FoV']['V']['_2']
    psi = tensor([params['FoV']['psi']])

    env = MultiRobotEnv(num_robots=args.num_robots, max_num_landmarks=params['max_num_landmarks'],
                        horizon=params['horizon'], tau=params['tau'], A=A, B=B, V=V, W=W,
                        landmark_motion_scale=params['motion']['landmark_motion_scale'],
                        psi=psi, radius=params['FoV']['radius'],
                        num_clusters=args.num_clusters, clustering_prob=args.clustering_prob)
    agent = ModelBasedAgentAtt(max_num_landmarks=params['max_num_landmarks'], init_info=params['init_info'],
                               A=A, B=B, W=W, radius=params['FoV']['radius'], psi=psi,
                               kappa=params['FoV']['kappa'], V=V, lr=params['lr'], num_robots=args.num_robots)
    return env, agent, psi, params['FoV']['radius']


def rollout(env, agent, collect=True):
    """One episode with the current policy. Returns (records, mean team reward)."""
    mu_real, v, x, done = env.reset()
    num_landmarks = mu_real.size()[0]
    agent.reset_estimate_mu(mu_real)
    agent.reset_agent_info()
    agent.reset_dra_episode()
    while not done:
        with torch.no_grad():
            action = agent.plan(v, x)
        mu_real, v, x, done = env.step(action)
        agent.update_info_mu(mu_real, x)
    objective = agent.update_policy_grad(False) / max(num_landmarks, 1)
    return (agent.end_dra_episode() if collect else []), objective


# ----------------------------------------------------------------------
# Evaluation (same metrics as run_model_based_testing.py)
# ----------------------------------------------------------------------
def fov_mask(mu_real, x, psi, radius):
    if len(x.size()) == 1:
        x = x[None, :]
    masks = []
    for r in range(x.size(0)):
        q = torch.vstack((
            (mu_real[:, 0] - x[r, 0]) * torch.cos(x[r, 2]) + (mu_real[:, 1] - x[r, 1]) * torch.sin(x[r, 2]),
            (x[r, 0] - mu_real[:, 0]) * torch.sin(x[r, 2]) + (mu_real[:, 1] - x[r, 1]) * torch.cos(x[r, 2])
        )).T
        masks.append(triangle_SDF(q, psi, radius) <= 0)
    return torch.stack(masks)


def evaluate(env, agent, psi, radius, trials, seed):
    """Cumulative / overlap / mean-targets, matching the thesis metric definitions."""
    state = torch.get_rng_state()
    torch.manual_seed(seed)
    was_dra, agent._dra_enabled = agent._dra_enabled, False
    agent.eval_policy()

    cums, ovls, tgts = [], [], []
    for _ in range(trials):
        mu_real, v, x, done = env.reset()
        n = mu_real.size()[0]
        agent.reset_estimate_mu(mu_real); agent.reset_agent_info()
        first = [-1] * n; poss = [0] * n; cum = [0] * n; ovl = [0] * n
        step = 0; per_step = []
        while not done:
            with torch.no_grad():
                action = agent.plan(v, x)
            mu_real, v, x, done = env.step(action)
            agent.update_info_mu(mu_real, x)
            m = fov_mask(mu_real, x, psi, radius).cpu().numpy()
            for t in range(n):
                seen = m[:, t].sum()
                if seen > 0 and first[t] == -1:
                    first[t] = step
                if first[t] != -1:
                    poss[t] += 1
                    if seen > 0:
                        cum[t] += 1
                    if seen > 1:
                        ovl[t] += 1
            per_step.append(int((m.sum(axis=0) > 0).sum()))
            step += 1
        denom = sum(poss)
        cums.append(100.0 * sum(cum) / denom if denom else 0.0)
        ovls.append(100.0 * sum(ovl) / denom if denom else 0.0)
        tgts.append(float(np.mean(per_step)) if per_step else 0.0)

    agent.train_policy()
    agent._dra_enabled = was_dra
    torch.set_rng_state(state)
    return float(np.mean(cums)), float(np.mean(ovls)), float(np.mean(tgts))


# ----------------------------------------------------------------------
# DRA updates
# ----------------------------------------------------------------------
def critic_update(policy, critic, opt, batch, gamma, tau):
    """Stage 1: per-head TD regression (Eqs. 3.34, 3.35). Encoder + policy fixed."""
    obs, act, rew, next_obs, done = batch
    with torch.no_grad():
        h = policy.encode(obs)
        h2 = policy.encode(next_obs)
        a2 = policy.act_raw(h2)
        q_next = critic.q_values(h2, a2, target=True)

    loss = 0.0
    per_head = {}
    q_now = critic.q_values(h, act)
    for i, c in enumerate(COMPONENTS):
        y = rew[:, i] + gamma * (1.0 - done) * q_next[c]
        l = torch.nn.functional.mse_loss(q_now[c], y)
        per_head[c] = float(l.detach())
        loss = loss + l

    opt.zero_grad()
    loss.backward()
    opt.step()
    critic.polyak_update(tau)
    return float(loss), per_head


def actor_update(policy, critic, opt, batch, train_encoder):
    """Stage 2: ascend the aggregate Q (Eqs. 3.37, 3.38). Q-heads frozen."""
    obs = batch[0]
    if train_encoder:
        h = policy.encode(obs)
    else:
        with torch.no_grad():
            h = policy.encode(obs)
    a = policy.act_raw(h)
    loss = -critic.q_aggregate(h, a).mean()

    opt.zero_grad()
    for p in critic.parameters():
        p.grad = None
    loss.backward()
    opt.step()
    return float(loss)


def sample(buffer_tensors, batch_size):
    obs, act, rew, next_obs, done = buffer_tensors
    idx = torch.randint(0, obs.shape[0], (min(batch_size, obs.shape[0]),))
    return obs[idx], act[idx], rew[idx], next_obs[idx], done[idx]


# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Staged warm-start DRA training")
    ap.add_argument('--resume', type=str, required=True, help='baseline policy checkpoint to warm-start from')
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--num-robots', type=int, default=2)
    ap.add_argument('--num-clusters', type=int, default=2)
    ap.add_argument('--clustering-prob', type=float, default=0.65)
    ap.add_argument('--gamma', type=float, default=0.95)
    ap.add_argument('--critic-lr', type=float, default=1e-3)
    ap.add_argument('--actor-lr', type=float, default=3e-5)
    ap.add_argument('--polyak', type=float, default=0.005)
    ap.add_argument('--phase1-rounds', type=int, default=60, help='critic-only fitting rounds')
    ap.add_argument('--phase2-rounds', type=int, default=300, help='two-stage alternation rounds')
    ap.add_argument('--episodes-per-round', type=int, default=10)
    ap.add_argument('--critic-steps', type=int, default=40)
    ap.add_argument('--actor-steps', type=int, default=10)
    ap.add_argument('--batch-size', type=int, default=256)
    ap.add_argument('--train-encoder', action='store_true', help='Stage 2 also updates the shared encoder (thesis default)')
    ap.add_argument('--eval-every', type=int, default=25)
    ap.add_argument('--eval-trials', type=int, default=30)
    ap.add_argument('--eval-seed', type=int, default=42)
    ap.add_argument('--out-dir', type=str, default='./checkpoints')
    ap.add_argument('--tag', type=str, default='hra')
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    params = yaml.load(open(os.path.join(repo_root, 'params', 'params_compare.yaml')), Loader=yaml.FullLoader)
    env, agent, psi, radius = build(args, params)

    if not os.path.exists(args.resume):
        raise FileNotFoundError(args.resume)
    agent.load_policy_state_dict(args.resume)
    agent.enable_dra(True)
    policy = agent.policy

    critic = DecomposedCritic(latent_dim=policy.latent_dim, action_dim=2)
    critic_opt = torch.optim.Adam(critic.heads.parameters(), lr=args.critic_lr)
    # Stage 2 trains the policy head (D12); the encoder joins only with --train-encoder.
    head_params = [p for n, p in policy.named_parameters() if n.startswith('action_fc')]
    actor_params = list(policy.parameters()) if args.train_encoder else head_params
    actor_opt = torch.optim.Adam(actor_params, lr=args.actor_lr)

    os.makedirs(args.out_dir, exist_ok=True)
    best_path = os.path.join(args.out_dir, f'best_model_seed{args.seed}_{args.tag}.pth')
    critic_path = os.path.join(args.out_dir, f'critic_seed{args.seed}_{args.tag}.pth')

    print(f'warm-start   : {args.resume}')
    print(f'trainable    : critic {sum(p.numel() for p in critic.heads.parameters())}, '
          f'actor {sum(p.numel() for p in actor_params)} '
          f'({"encoder+head" if args.train_encoder else "policy head only"})')

    buf = ReplayBuffer()
    agent.train_policy()

    base = evaluate(env, agent, psi, radius, args.eval_trials, args.eval_seed)
    print(f'baseline eval: cumulative {base[0]:.2f}%  overlap {base[1]:.2f}%  targets {base[2]:.4f}\n')
    best = base[0]
    torch.save(agent.get_policy_state_dict(), best_path)

    t0 = time.time()

    # ---------------- Phase 1: critic fit, policy frozen ----------------
    print(f'--- Phase 1: fitting Q-heads to the frozen baseline policy ({args.phase1_rounds} rounds) ---')
    for rnd in range(args.phase1_rounds):
        for _ in range(args.episodes_per_round):
            recs, _ = rollout(env, agent)
            buf.add_episode(recs)
        tens = buf.tensors()
        losses = []
        for _ in range(args.critic_steps):
            l, per_head = critic_update(policy, critic, critic_opt, sample(tens, args.batch_size), args.gamma, args.polyak)
            losses.append(l)
        if (rnd + 1) % 10 == 0 or rnd == 0:
            print(f'  round {rnd+1:3d} | buffer {len(buf):6d} | critic loss {np.mean(losses):.5f} | '
                  + ' '.join(f'{c}={per_head[c]:.4f}' for c in COMPONENTS))

    torch.save(critic.state_dict(), critic_path)
    print(f'  critic saved -> {critic_path}\n')

    # ---------------- Phase 2: two-stage alternation ----------------
    print(f'--- Phase 2: two-stage alternation ({args.phase2_rounds} rounds) ---')
    for rnd in range(args.phase2_rounds):
        for _ in range(args.episodes_per_round):
            recs, _ = rollout(env, agent)
            buf.add_episode(recs)
        tens = buf.tensors()

        closs = [critic_update(policy, critic, critic_opt, sample(tens, args.batch_size), args.gamma, args.polyak)[0]
                 for _ in range(args.critic_steps)]
        aloss = [actor_update(policy, critic, actor_opt, sample(tens, args.batch_size), args.train_encoder)
                 for _ in range(args.actor_steps)]

        if (rnd + 1) % args.eval_every == 0:
            cum, ovl, tg = evaluate(env, agent, psi, radius, args.eval_trials, args.eval_seed)
            flag = ''
            if cum > best:
                best = cum
                torch.save(agent.get_policy_state_dict(), best_path)
                torch.save(critic.state_dict(), critic_path)
                flag = '  <-- new best'
            print(f'  round {rnd+1:3d} | critic {np.mean(closs):.5f} | actor {np.mean(aloss):+.4f} | '
                  f'cumulative {cum:.2f}%  overlap {ovl:.2f}%  targets {tg:.4f}{flag}')

    final = evaluate(env, agent, psi, radius, args.eval_trials, args.eval_seed)
    print(f'\nelapsed {(time.time()-t0)/60:.1f} min')
    print(f'baseline : cumulative {base[0]:.2f}%  overlap {base[1]:.2f}%  targets {base[2]:.4f}')
    print(f'final    : cumulative {final[0]:.2f}%  overlap {final[1]:.2f}%  targets {final[2]:.4f}')
    print(f'best     : cumulative {best:.2f}%  -> {best_path}')


if __name__ == '__main__':
    main()
