import os
import sys
import yaml
import json
repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(repo_root)
sys.path.append(os.path.join(repo_root, "MRMT"))
import torch
import argparse
import numpy as np
import cv2

from torch import tensor
from envs.simple_env import SimpleEnv, SimpleEnvAtt
from multi_robot_env import MultiRobotEnv
from agents.model_based_agent import ModelBasedAgent, ModelBasedAgentAtt
from utilities.utils import triangle_SDF

parser = argparse.ArgumentParser(description='model-based mapping')
parser.add_argument('--network-type', type=int, default=1, help='by default, it should attention block,'
                                                                'otherwise, it would be MLP')
parser.add_argument('--seed', type=int, default=0)
parser.add_argument('--model-path', type=str, default=None, help='Path to the .pth model checkpoint')
parser.add_argument('--num-robots', type=int, default=2)
parser.add_argument('--num-clusters', type=int, default=2)
parser.add_argument('--clustering-prob', type=float, default=0.65)
args = parser.parse_args()
torch.manual_seed(args.seed)


# ==========================================
# 1. Tracking Statistics Class
# ==========================================
class TrackingStatistics:
    """Tracks per-target, per-robot tracking statistics for analysis."""

    def __init__(self, num_robots, num_landmarks):
        self.num_robots = num_robots
        self.num_landmarks = num_landmarks

        # First step each target was tracked (by any robot), -1 means never tracked
        self.first_tracked_step = [-1] * num_landmarks

        # Per-robot, per-target tracking step counts (after first tracked)
        # robot_tracking[robot_idx][target_idx] = count of steps tracked
        self.robot_tracking = [[0] * num_landmarks for _ in range(num_robots)]

        # Per-target cumulative (any robot tracking after first tracked)
        self.cumulative_tracking = [0] * num_landmarks

        # Per-target overlap count (steps where >1 robot tracks this target, after first tracked)
        self.overlap_counts = [0] * num_landmarks

        # Total steps for each target since first tracked
        self.steps_since_first_tracked = [0] * num_landmarks

        self.current_step = 0

    def update(self, fov_mask):
        """
        Update statistics with current FOV mask.

        Args:
            fov_mask: torch.Tensor of shape [num_robots x num_landmarks], boolean
        """
        if fov_mask is None:
            self.current_step += 1
            return

        fov_mask_np = fov_mask.cpu().numpy() if hasattr(fov_mask, 'cpu') else np.array(fov_mask)

        for target_idx in range(self.num_landmarks):
            # Check if any robot is tracking this target
            robots_tracking = [fov_mask_np[r, target_idx] for r in range(self.num_robots)]
            any_tracking = any(robots_tracking)
            num_robots_tracking = sum(robots_tracking)

            # Update first tracked step if this is the first time
            if any_tracking and self.first_tracked_step[target_idx] == -1:
                self.first_tracked_step[target_idx] = self.current_step

            # Only count stats after the target has been first tracked
            if self.first_tracked_step[target_idx] != -1:
                self.steps_since_first_tracked[target_idx] += 1

                # Update per-robot tracking counts
                for robot_idx in range(self.num_robots):
                    if robots_tracking[robot_idx]:
                        self.robot_tracking[robot_idx][target_idx] += 1

                # Update cumulative (any robot tracking)
                if any_tracking:
                    self.cumulative_tracking[target_idx] += 1

                # Update overlap (>1 robot tracking same target)
                if num_robots_tracking > 1:
                    self.overlap_counts[target_idx] += 1

        self.current_step += 1

    def get_trail_summary(self):
        """
        Return summary dict for this trail.

        Returns:
            dict with per-target and aggregated statistics
        """
        summary = {
            'num_targets': self.num_landmarks,
            'num_robots': self.num_robots,
            'total_steps': self.current_step,
            'per_target': [],
            'totals': {}
        }

        # Aggregate counters
        total_robot_tracking = [0] * self.num_robots
        total_cumulative = 0
        total_overlap = 0
        total_possible_steps = 0

        for target_idx in range(self.num_landmarks):
            first_step = self.first_tracked_step[target_idx]
            steps_possible = self.steps_since_first_tracked[target_idx]

            target_data = {
                'target_idx': target_idx,
                'first_tracked_step': first_step,
                'steps_possible': steps_possible,
                'per_robot': []
            }

            for robot_idx in range(self.num_robots):
                steps_tracked = self.robot_tracking[robot_idx][target_idx]
                pct = (steps_tracked / steps_possible * 100) if steps_possible > 0 else 0.0
                target_data['per_robot'].append({
                    'robot_idx': robot_idx,
                    'steps_tracked': steps_tracked,
                    'percentage': round(pct, 2)
                })
                total_robot_tracking[robot_idx] += steps_tracked

            cumulative_steps = self.cumulative_tracking[target_idx]
            cumulative_pct = (cumulative_steps / steps_possible * 100) if steps_possible > 0 else 0.0
            target_data['cumulative_steps'] = cumulative_steps
            target_data['cumulative_percentage'] = round(cumulative_pct, 2)

            overlap_steps = self.overlap_counts[target_idx]
            overlap_pct = (overlap_steps / steps_possible * 100) if steps_possible > 0 else 0.0
            target_data['overlap_steps'] = overlap_steps
            target_data['overlap_percentage'] = round(overlap_pct, 2)

            summary['per_target'].append(target_data)

            total_cumulative += cumulative_steps
            total_overlap += overlap_steps
            total_possible_steps += steps_possible

        # Compute totals
        summary['totals'] = {
            'total_possible_steps': total_possible_steps,
            'per_robot': []
        }

        for robot_idx in range(self.num_robots):
            pct = (total_robot_tracking[robot_idx] / total_possible_steps * 100) if total_possible_steps > 0 else 0.0
            summary['totals']['per_robot'].append({
                'robot_idx': robot_idx,
                'total_steps_tracked': total_robot_tracking[robot_idx],
                'percentage': round(pct, 2)
            })

        cumulative_pct = (total_cumulative / total_possible_steps * 100) if total_possible_steps > 0 else 0.0
        summary['totals']['cumulative_steps'] = total_cumulative
        summary['totals']['cumulative_percentage'] = round(cumulative_pct, 2)

        overlap_pct = (total_overlap / total_possible_steps * 100) if total_possible_steps > 0 else 0.0
        summary['totals']['overlap_steps'] = total_overlap
        summary['totals']['overlap_percentage'] = round(overlap_pct, 2)

        return summary


def print_trail_summary(summary, episode_num):
    """Print formatted trail summary to console."""
    num_targets = summary['num_targets']
    num_robots = summary['num_robots']

    print(f"\n{'='*80}")
    print(f"=== Trail {episode_num} ({num_targets} targets) ===")
    print(f"{'='*80}")

    # Header
    header = "Target | First Step |"
    for r in range(num_robots):
        header += f" Robot {r} | R{r} % |"
    header += " Cumulative | Cum % | Overlap %"
    print(header)
    print("-" * len(header))

    # Per-target rows
    for target_data in summary['per_target']:
        row = f"  {target_data['target_idx']:3d}  |    {target_data['first_tracked_step']:4d}    |"
        for robot_data in target_data['per_robot']:
            row += f"  {robot_data['steps_tracked']:5d}  | {robot_data['percentage']:5.1f}% |"
        row += f"     {target_data['cumulative_steps']:5d}   | {target_data['cumulative_percentage']:5.1f}% |  {target_data['overlap_percentage']:5.1f}%"
        print(row)

    # Totals row
    print("-" * len(header))
    totals = summary['totals']
    total_row = " Total |     --     |"
    for robot_data in totals['per_robot']:
        total_row += f"  {robot_data['total_steps_tracked']:5d}  | {robot_data['percentage']:5.1f}% |"
    total_row += f"     {totals['cumulative_steps']:5d}   | {totals['cumulative_percentage']:5.1f}% |  {totals['overlap_percentage']:5.1f}%"
    print(total_row)


def aggregate_by_target_count(all_summaries):
    """
    Aggregate trail summaries grouped by number of targets.

    Args:
        all_summaries: List of trail summary dicts

    Returns:
        Dict mapping num_targets -> aggregated stats
    """
    from collections import defaultdict

    grouped = defaultdict(list)
    for summary in all_summaries:
        grouped[summary['num_targets']].append(summary)

    aggregated = {}
    for num_targets, summaries in sorted(grouped.items()):
        num_trails = len(summaries)
        num_robots = summaries[0]['num_robots'] if summaries else 0

        # Average per-robot percentages
        per_robot_pcts = [[] for _ in range(num_robots)]
        cumulative_pcts = []
        overlap_pcts = []

        for s in summaries:
            for r in range(num_robots):
                per_robot_pcts[r].append(s['totals']['per_robot'][r]['percentage'])
            cumulative_pcts.append(s['totals']['cumulative_percentage'])
            overlap_pcts.append(s['totals']['overlap_percentage'])

        aggregated[num_targets] = {
            'num_trails': num_trails,
            'per_robot_avg_pct': [round(np.mean(pcts), 2) for pcts in per_robot_pcts],
            'per_robot_std_pct': [round(np.std(pcts), 2) for pcts in per_robot_pcts],
            'cumulative_avg_pct': round(np.mean(cumulative_pcts), 2),
            'cumulative_std_pct': round(np.std(cumulative_pcts), 2),
            'overlap_avg_pct': round(np.mean(overlap_pcts), 2),
            'overlap_std_pct': round(np.std(overlap_pcts), 2),
        }

    return aggregated


def print_aggregated_summary(aggregated, num_robots):
    """Print aggregated statistics by target count."""
    print(f"\n{'='*80}")
    print("=== Averaged Statistics by Number of Targets ===")
    print(f"{'='*80}")

    header = "Targets | Trails |"
    for r in range(num_robots):
        header += f" Robot {r} Avg % |"
    header += " Cumulative Avg % | Overlap Avg %"
    print(header)
    print("-" * len(header))

    for num_targets, stats in sorted(aggregated.items()):
        row = f"   {num_targets:2d}   |   {stats['num_trails']:2d}   |"
        for r in range(num_robots):
            avg = stats['per_robot_avg_pct'][r]
            std = stats['per_robot_std_pct'][r]
            row += f" {avg:5.1f}±{std:4.1f}% |"
        row += f"    {stats['cumulative_avg_pct']:5.1f}±{stats['cumulative_std_pct']:4.1f}%    |  {stats['overlap_avg_pct']:5.1f}±{stats['overlap_std_pct']:4.1f}%"
        print(row)


def compute_fov_mask(mu_real, x, psi, radius):
    if len(x.size()) == 1:
        x = x[None, :]
    masks = []
    for r in range(x.size(0)):
        q = torch.vstack((
            (mu_real[:, 0] - x[r, 0]) * torch.cos(x[r, 2]) + (mu_real[:, 1] - x[r, 1]) * torch.sin(x[r, 2]),
            (x[r, 0] - mu_real[:, 0]) * torch.sin(x[r, 2]) + (mu_real[:, 1] - x[r, 1]) * torch.cos(x[r, 2])
        )).T
        sdf = triangle_SDF(q, psi, radius)
        masks.append(sdf <= 0)
    return torch.stack(masks)

def get_frame(env, current_fov_mask=None):
    if current_fov_mask is not None:
        try:
            return env.get_current_frame(current_fov_mask=current_fov_mask)
        except TypeError:
            return env.get_current_frame()
    return env.get_current_frame()

def run_model_based_testing(params_filename):
    assert os.path.exists(params_filename)
    with open(os.path.join(params_filename)) as f:
        params = yaml.load(f, Loader=yaml.FullLoader)

    max_num_landmarks = params['max_num_landmarks']
    num_landmarks = params['num_landmarks']
    horizon = params['horizon']
    env_width = params['env_width']
    env_height = params['env_height']
    tau = params['tau']

    A = torch.zeros((2, 2))
    A[0, 0] = params['motion']['A']['_1']
    A[1, 1] = params['motion']['A']['_2']

    B = torch.zeros((2, 2))
    B[0, 0] = params['motion']['B']['_1']
    B[1, 1] = params['motion']['B']['_2']

    W = torch.zeros(2)
    W[0] = params['motion']['W']['_1']
    W[1] = params['motion']['W']['_2']

    landmark_motion_scale = params['motion']['landmark_motion_scale']

    init_info = params['init_info']

    radius = params['FoV']['radius']
    psi = tensor([params['FoV']['psi']])
    kappa = params['FoV']['kappa']

    V = torch.zeros(2)
    V[0] = params['FoV']['V']['_1']
    V[1] = params['FoV']['V']['_2']

    lr = params['lr']
    max_epoch = params['max_epoch']
    batch_size = params['batch_size']
    num_test_trials = params['num_test_trials']

    use_multi = args.num_robots > 1
    if args.network_type == 1:
        if use_multi:
            env = MultiRobotEnv(num_robots=args.num_robots, max_num_landmarks=max_num_landmarks, horizon=horizon, tau=tau,
                                A=A, B=B, V=V, W=W, landmark_motion_scale=landmark_motion_scale, psi=psi, radius=radius,
                                num_clusters=args.num_clusters, clustering_prob=args.clustering_prob)
        else:
            env = SimpleEnvAtt(max_num_landmarks=max_num_landmarks, horizon=horizon, tau=tau,
                               A=A, B=B, V=V, W=W, landmark_motion_scale=landmark_motion_scale, psi=psi, radius=radius)
        agent = ModelBasedAgentAtt(max_num_landmarks=max_num_landmarks, init_info=init_info, A=A, B=B, W=W,
                                   radius=radius, psi=psi, kappa=kappa, V=V, lr=lr, num_robots=args.num_robots)
    else:
        if use_multi:
            env = MultiRobotEnv(num_robots=args.num_robots, max_num_landmarks=max_num_landmarks, horizon=horizon, tau=tau,
                                A=A, B=B, V=V, W=W, landmark_motion_scale=landmark_motion_scale, psi=psi, radius=radius,
                                num_clusters=args.num_clusters, clustering_prob=args.clustering_prob)
        else:
            env = SimpleEnv(num_landmarks=num_landmarks, horizon=horizon, width=env_width, height=env_height, tau=tau,
                            A=A, B=B, V=V, W=W, landmark_motion_scale=landmark_motion_scale, psi=psi, radius=radius)
        agent = ModelBasedAgent(num_landmarks=num_landmarks, init_info=init_info, A=A, B=B, W=W,
                                radius=radius, psi=psi, kappa=kappa, V=V, lr=lr, num_robots=args.num_robots)

    model_path = args.model_path
    if model_path is None:
        seed_path = f'./checkpoints/best_model_seed{args.seed}.pth'
        fallback_path = './checkpoints/best_model.pth'
        model_path = seed_path if os.path.exists(seed_path) else fallback_path

    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model path does not exist: {model_path}")

    agent.load_policy_state_dict(model_path)

    agent.eval_policy()
    results_dir = './test_results'
    videos_dir = os.path.join(results_dir, 'videos')
    os.makedirs(videos_dir, exist_ok=True)
    overall_avg_targets = []
    all_trail_summaries = []
    model_name = os.path.splitext(os.path.basename(model_path))[0]
    for i in range(num_test_trials):
        mu_real, v, x, done = env.reset()
        num_landmarks = mu_real.size()[0]
        agent.reset_estimate_mu(mu_real)
        agent.reset_agent_info()
        stats = TrackingStatistics(num_robots=args.num_robots, num_landmarks=num_landmarks)
        episode_targets_tracked = []
        frames = []
        while not done:
            action = agent.plan(v, x)
            mu_real, v, x, done = env.step(action)
            agent.update_info_mu(mu_real, x)
            current_fov_mask = compute_fov_mask(mu_real, x, psi, radius)
            stats.update(current_fov_mask)
            episode_targets_tracked.append(int(current_fov_mask.any(dim=0).sum().item()))
            frame = get_frame(env, current_fov_mask=current_fov_mask)
            frames.append(frame)

        if frames:
            height, width, layers = frames[0].shape
            video_name = os.path.join(videos_dir, f'{model_name}_seed{args.seed}_episode_{i+1:03d}.avi')
            fourcc = cv2.VideoWriter_fourcc(*'DIVX')
            video = cv2.VideoWriter(video_name, fourcc, 5, (width, height))
            for frame in frames:
                video.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            video.release()

        reward = agent.update_policy_grad(False) / num_landmarks
        print("num_landmark:", num_landmarks, "reward:", reward)

        trail_summary = stats.get_trail_summary()
        trail_summary['episode'] = i + 1
        all_trail_summaries.append(trail_summary)
        print_trail_summary(trail_summary, i + 1)

        avg_targets = np.mean(episode_targets_tracked) if episode_targets_tracked else 0
        overall_avg_targets.append(avg_targets)
        print(f"\n  > Episode Complete. Avg Unique Targets Tracked: {avg_targets:.2f}")

    aggregated = aggregate_by_target_count(all_trail_summaries)
    print_aggregated_summary(aggregated, num_robots=args.num_robots)

    print("\n" + "=" * 80)
    print("TESTING SUMMARY (Legacy)")
    print("=" * 80)
    print(f"Mean Targets Tracked: {np.mean(overall_avg_targets):.4f} +/- {np.std(overall_avg_targets):.4f}")
    print("=" * 80)

    json_results = {
        'model_path': model_path,
        'model_name': model_name,
        'seed': args.seed,
        'network_type': args.network_type,
        'num_robots': args.num_robots,
        'num_test_trials': num_test_trials,
        'trails': all_trail_summaries,
        'aggregated_by_targets': {str(k): v for k, v in aggregated.items()},
        'legacy_summary': {
            'mean_targets_tracked': round(float(np.mean(overall_avg_targets)), 4),
            'std_targets_tracked': round(float(np.std(overall_avg_targets)), 4)
        }
    }

    json_filename = f'{results_dir}/tracking_stats_{model_name}_seed{args.seed}.json'
    with open(json_filename, 'w') as f:
        json.dump(json_results, f, indent=2)
    print(f"\n> JSON results saved to {json_filename}")


if __name__ == '__main__':
    # torch.manual_seed(0)
    # torch.autograd.set_detect_anomaly(True)
    run_model_based_testing(params_filename=os.path.join(os.path.abspath(os.path.join("", os.pardir)),
                                                          "params/params_compare.yaml"))
