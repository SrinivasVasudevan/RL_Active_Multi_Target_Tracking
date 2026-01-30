import os, sys, yaml
repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(repo_root)
sys.path.append(os.path.join(repo_root, "MRMT"))
import argparse
import torch
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import cv2

from torch import tensor
from utilities.utils import triangle_SDF
from envs.simple_env import SimpleEnv, SimpleEnvAtt
from multi_robot_env import MultiRobotEnv
from agents.model_based_agent import ModelBasedAgent, ModelBasedAgentAtt
from torch.utils.tensorboard import SummaryWriter

parser = argparse.ArgumentParser(description='model-based mapping')
parser.add_argument('--network-type', type=int, default=1, help='by default, it should attention block,'
                                                                'otherwise, it would be MLP')
parser.add_argument('--seed', type=int, default=0)
parser.add_argument('--num-robots', type=int, default=2)
parser.add_argument('--num-clusters', type=int, default=2)
parser.add_argument('--clustering-prob', type=float, default=0.65)
args = parser.parse_args()

torch.manual_seed(args.seed)


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

def run_model_based_training(params_filename):
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
    writer = SummaryWriter('./tensorboard/')

    agent.train_policy()
    reward_list = np.empty((max_epoch, batch_size))
    action_list = np.full((max_epoch * batch_size, horizon, args.num_robots, 2), np.nan)
    best_reward = 1.
    for i in range(max_epoch):
        agent.set_policy_grad_to_zero()

        for j in range(batch_size):
            mu_real, v, x, done = env.reset()
            num_landmarks = mu_real.size()[0]
            agent.reset_estimate_mu(mu_real)
            agent.reset_agent_info()
            step = 0
            while not done:
                action = agent.plan(v, x)
                action_np = action.detach().cpu().numpy()
                if action_np.ndim == 1:
                    action_np = action_np[None, :]
                action_list[i * batch_size + j, step, :, :] = action_np
                mu_real, v, x, done = env.step(action)
                agent.update_info_mu(mu_real, x)
                step += 1

            reward_list[i, j] = agent.update_policy_grad() / num_landmarks
            writer.add_scalar('Average Reward', reward_list[i, j], i)
            # reward_list[i, j] = agent.update_policy_grad(mu, x) / num_landmarks

        agent.policy_step(debug=False)

        print('Epoch {} finished!'.format(i + 1))
        mean_reward = np.mean(reward_list[i])
        print('Normalized average reward at epoch {}: {}'.format(i, mean_reward))
        print('Normalized median reward at epoch {}: {}'.format(i, np.median(reward_list[i])))
        if mean_reward > best_reward:
            torch.save(agent.get_policy_state_dict(), './checkpoints/best_model_seed{}.pth'.format(args.seed))
            best_reward = mean_reward
            print("New best model!\n")

        if (i + 1) % 50 == 0:
            print(f"Saving visualization for epoch {i + 1}...")
            os.makedirs('./training_visualizations', exist_ok=True)

            mu_real, v, x, done = env.reset()
            agent.reset_estimate_mu(mu_real)
            agent.reset_agent_info()

            frames = []
            with torch.no_grad():
                step = 0
                while not done and step < 50:
                    action = agent.plan(v, x)
                    mu_real, v, x, done = env.step(action)
                    agent.update_info_mu(mu_real, x)
                    current_mask = compute_fov_mask(mu_real, x, psi, radius)
                    frame = get_frame(env, current_fov_mask=current_mask)
                    frames.append(frame)
                    step += 1

            if frames:
                height, width, layers = frames[0].shape
                video_name = f'./training_visualizations/epoch_{i+1}_seed{args.seed}.avi'
                fourcc = cv2.VideoWriter_fourcc(*'DIVX')
                video = cv2.VideoWriter(video_name, fourcc, 5, (width, height))
                for frame in frames:
                    video.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                video.release()
                print(f"Saved video to {video_name}\n")

    torch.save(agent.get_policy_state_dict(), './checkpoints/model_info_5_moving_landmarks_2.pth')

    os.makedirs('logs', exist_ok=True)
    plt.figure()
    plt.plot(np.mean(reward_list, axis=1), 'b-', label='Average')
    plt.plot(np.mean(reward_list, axis=1) + np.std(reward_list, axis=1), 'b--')
    plt.plot(np.mean(reward_list, axis=1) - np.std(reward_list, axis=1), 'b--')
    plt.plot(np.median(reward_list, axis=1), 'r--', label='Median')
    plt.xlabel("Epoch")
    plt.ylabel("Normalized Reward")
    plt.legend(loc="upper right")
    plt.savefig(os.path.join('logs', f'reward_plot_seed{args.seed}.png'), bbox_inches='tight')
    plt.close()

    plt.figure()
    mean_linear = np.nanmean(action_list[..., 0], axis=(1, 2))
    std_linear = np.nanstd(action_list[..., 0], axis=(1, 2))
    mean_angular = np.nanmean(action_list[..., 1], axis=(1, 2))
    std_angular = np.nanstd(action_list[..., 1], axis=(1, 2))

    plt.plot(mean_linear, 'b-', label='Linear Velocity')
    plt.plot(mean_linear + 5 * std_linear, 'b--')
    plt.plot(mean_linear - 5 * std_linear, 'b--')

    plt.plot(mean_angular, 'r-', label='Angular Velocity')
    plt.plot(mean_angular + 5 * std_angular, 'r--')
    plt.plot(mean_angular - 5 * std_angular, 'r--')

    plt.legend(loc="upper right")
    plt.xlabel("Epoch")
    plt.savefig(os.path.join('logs', f'action_plot_seed{args.seed}.png'), bbox_inches='tight')
    plt.close()

    agent.eval_policy()
    for i in range(num_test_trials):
        mu, v, x, done = env.reset()
        agent.reset_estimate_mu(mu)
        agent.reset_agent_info()
        while not done:
            action = agent.plan(v, x)
            mu_real, v, x, done = env.step(action)
            agent.update_info_mu(mu_real, x)
            if hasattr(env, "render"):
                env.render()


if __name__ == '__main__':
    # torch.manual_seed(0)
    # torch.autograd.set_detect_anomaly(True)
    run_model_based_training(params_filename=os.path.join(os.path.abspath(os.path.join("", os.pardir)),
                                                          "params/params_compare.yaml"))
