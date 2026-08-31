"""Hybrid training: analytic exploration gradient + learned coord/pers critics.

Motivation
----------
Two facts about the baseline drive this design:

1. `_episode_reward` is computed entirely from detached tensors, so the
   persistence / coverage / continuity / loss / overlap terms contribute
   **zero gradient**. The baseline policy is trained purely by the analytic
   information gain backpropagated through the differentiable Kalman filter;
   those five terms only affect logging and best-checkpoint selection.

2. A pure DRA replaces that exact analytic gradient with a learned Q^exp, which
   measurably degrades tracking (see branch best_simple_reward_HRA).

So: keep the analytic gradient for exploration, where an exact gradient exists,
and use Q-heads only for coordination and persistence, where none does. This
gives those objectives real gradient influence for the first time, weighted by
lambda:

    L = -( w_info * sum_k dlogdetLambda_k  +  lambda * mean_k [ Q^coord + Q^pers ] )

lambda = 0 recovers the baseline exactly, so the method can only help or be
tuned back to neutral.

Usage:
    python3 run_hybrid_training.py --resume ./checkpoints/best_model_seed42_resume1.pth --lam 1.0
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

from models.hra_critic import DecomposedCritic
from run_hra_training import ReplayBuffer, build, evaluate, sample

HYBRID_COMPONENTS = ["coord", "pers"]


def collect_episode(env, agent, differentiable):
    """One episode. If differentiable, the graph is retained for the surrogate."""
    agent.enable_differentiable_latents(differentiable)
    mu_real, v, x, done = env.reset()
    num_landmarks = mu_real.size()[0]
    agent.reset_estimate_mu(mu_real)
    agent.reset_agent_info()
    agent.reset_dra_episode()
    agent.enable_differentiable_latents(differentiable)

    while not done:
        if differentiable:
            action = agent.plan(v, x)
        else:
            with torch.no_grad():
                action = agent.plan(v, x)
        mu_real, v, x, done = env.step(action)
        agent.update_info_mu(mu_real, x)
    return num_landmarks


def critic_update(policy, critic, opt, batch, gamma, tau, comp_index):
    """TD update for the coord/pers heads only."""
    obs, act, rew, next_obs, done = batch
    with torch.no_grad():
        h = policy.encode(obs)
        h2 = policy.encode(next_obs)
        a2 = policy.act_raw(h2)
        q_next = critic.q_values(h2, a2, target=True)

    loss = 0.0
    q_now = critic.q_values(h, act)
    for c in HYBRID_COMPONENTS:
        y = rew[:, comp_index[c]] + gamma * (1.0 - done) * q_next[c]
        loss = loss + torch.nn.functional.mse_loss(q_now[c], y)

    opt.zero_grad()
    loss.backward()
    opt.step()
    critic.polyak_update(tau)
    return float(loss.detach())


def main():
    from models.hra_critic import COMPONENTS

    ap = argparse.ArgumentParser(description="Hybrid analytic + decomposed-critic training")
    ap.add_argument('--resume', type=str, required=True)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--num-robots', type=int, default=2)
    ap.add_argument('--num-clusters', type=int, default=2)
    ap.add_argument('--clustering-prob', type=float, default=0.65)
    ap.add_argument('--lam', type=float, default=1.0, help='weight on the critic surrogate (0 = baseline)')
    ap.add_argument('--gamma', type=float, default=0.95)
    ap.add_argument('--critic-lr', type=float, default=1e-3)
    ap.add_argument('--policy-lr', type=float, default=3e-5)
    ap.add_argument('--polyak', type=float, default=0.005)
    ap.add_argument('--phase1-rounds', type=int, default=60)
    ap.add_argument('--epochs', type=int, default=120)
    ap.add_argument('--batch-episodes', type=int, default=10)
    ap.add_argument('--critic-steps', type=int, default=30)
    ap.add_argument('--batch-size', type=int, default=256)
    ap.add_argument('--eval-every', type=int, default=10)
    ap.add_argument('--eval-trials', type=int, default=30)
    ap.add_argument('--eval-seed', type=int, default=42)
    ap.add_argument('--out-dir', type=str, default='./checkpoints')
    ap.add_argument('--tag', type=str, default='hybrid')
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    comp_index = {c: i for i, c in enumerate(COMPONENTS)}

    params = yaml.load(open(os.path.join(repo_root, 'params', 'params_compare.yaml')), Loader=yaml.FullLoader)
    env, agent, psi, radius = build(args, params)
    agent.load_policy_state_dict(args.resume)
    agent.enable_dra(True)
    policy = agent.policy

    critic = DecomposedCritic(latent_dim=policy.latent_dim, action_dim=2)
    critic_opt = torch.optim.Adam(critic.heads.parameters(), lr=args.critic_lr)
    policy_opt = torch.optim.Adam(policy.parameters(), lr=args.policy_lr)

    os.makedirs(args.out_dir, exist_ok=True)
    best_path = os.path.join(args.out_dir, f'best_model_seed{args.seed}_{args.tag}.pth')

    print(f'warm-start : {args.resume}')
    print(f'lambda     : {args.lam}   (0 = pure baseline)')
    print(f'critic     : heads for {HYBRID_COMPONENTS}; exploration stays analytic')

    agent.train_policy()
    base = evaluate(env, agent, psi, radius, args.eval_trials, args.eval_seed)
    print(f'baseline   : cumulative {base[0]:.2f}%  overlap {base[1]:.2f}%  targets {base[2]:.4f}\n')
    best = base[0]
    torch.save(agent.get_policy_state_dict(), best_path)

    t0 = time.time()
    buf = ReplayBuffer()

    # ---- Phase 1: fit the coord/pers heads to the frozen baseline policy ----
    print(f'--- Phase 1: fitting coord/pers Q-heads ({args.phase1_rounds} rounds) ---')
    for rnd in range(args.phase1_rounds):
        for _ in range(args.batch_episodes):
            collect_episode(env, agent, differentiable=False)
            buf.add_episode(agent.end_dra_episode())
        tens = buf.tensors()
        losses = [critic_update(policy, critic, critic_opt, sample(tens, args.batch_size),
                                args.gamma, args.polyak, comp_index) for _ in range(args.critic_steps)]
        if (rnd + 1) % 20 == 0 or rnd == 0:
            print(f'  round {rnd+1:3d} | buffer {len(buf):6d} | critic loss {np.mean(losses):.5f}')

    # ---- Phase 2: analytic gradient + lambda * critic surrogate ----
    print(f'\n--- Phase 2: hybrid policy training ({args.epochs} epochs) ---')
    for ep in range(args.epochs):
        policy_opt.zero_grad()
        objs, surrs = [], []

        for _ in range(args.batch_episodes):
            n = collect_episode(env, agent, differentiable=True)

            analytic = agent._info_gain_weight * agent._accumulated_info_gain / max(n, 1)
            surrogate = agent.critic_surrogate(critic, HYBRID_COMPONENTS)

            loss = -analytic
            if surrogate is not None and args.lam != 0.0:
                loss = loss - args.lam * surrogate
                surrs.append(float(surrogate.detach()))
            loss = loss / args.batch_episodes
            loss.backward()
            objs.append(float(analytic.detach()))

            buf.add_episode(agent.end_dra_episode())

        torch.nn.utils.clip_grad_norm_(policy.parameters(), 10.0)
        policy_opt.step()

        tens = buf.tensors()
        for _ in range(args.critic_steps):
            critic_update(policy, critic, critic_opt, sample(tens, args.batch_size),
                          args.gamma, args.polyak, comp_index)

        if (ep + 1) % args.eval_every == 0:
            cum, ovl, tg = evaluate(env, agent, psi, radius, args.eval_trials, args.eval_seed)
            flag = ''
            if cum > best:
                best = cum
                torch.save(agent.get_policy_state_dict(), best_path)
                flag = '  <-- new best'
            print(f'  epoch {ep+1:3d} | analytic {np.mean(objs):8.3f} | surrogate '
                  f'{(np.mean(surrs) if surrs else 0.0):8.3f} | cumulative {cum:.2f}%  '
                  f'overlap {ovl:.2f}%  targets {tg:.4f}{flag}')

    final = evaluate(env, agent, psi, radius, args.eval_trials, args.eval_seed)
    print(f'\nelapsed {(time.time()-t0)/60:.1f} min   (lambda={args.lam})')
    print(f'baseline : cumulative {base[0]:.2f}%  overlap {base[1]:.2f}%  targets {base[2]:.4f}')
    print(f'final    : cumulative {final[0]:.2f}%  overlap {final[1]:.2f}%  targets {final[2]:.4f}')
    print(f'best     : cumulative {best:.2f}%  -> {best_path}')


if __name__ == '__main__':
    main()
