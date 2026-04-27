import os
import torch
import cv2
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.patches import Polygon, Circle

from typing import Tuple
from torch import tensor
from utilities.utils import SE2_kinematics, landmark_motion_real, triangle_SDF


class SimpleEnv:

    def __init__(self, num_landmarks, horizon, width, height, tau, A, B, V, W, landmark_motion_scale, psi, radius):
        self._num_landmarks = num_landmarks
        self._horizon = horizon
        self._env_size = tensor([self._num_landmarks*2, self._num_landmarks*2])
        self._tau = tau
        self._A = A
        self._B = B
        self._V = V  # sensor_std ** 2 with the shape of (2, )
        self._W = W  # motion_std ** 2 with the shape of (2, )
        self._landmark_motion_scale = landmark_motion_scale

        self._mu = None
        self._v = None
        self._landmark_motion_bias = None
        self._x = None
        self._step_num = None

        self._psi = psi
        self._radius = radius
        self._episode_id = 0
        self._video_writer = None
        self._video_path = None
        self._video_frame_size = None

    def _reset_video_state(self):
        if self._video_writer is not None:
            self._video_writer.release()
        self._video_writer = None
        self._video_path = None
        self._video_frame_size = None

    def reset(self):
        x = torch.empty(3)
        x[:2] = (torch.rand(2) - 0.5) * self._env_size
        x[2] = (torch.rand(1) * 2 - 1) * torch.pi

        num_clusters = min(2, self._num_landmarks)
        min_robot_dist = 0.3 * torch.min(self._env_size)
        mu = self._generate_clusters(self._num_landmarks, num_clusters, self._env_size, x, min_robot_dist)

        landmark_motion_bias = (torch.rand(2) - 0.5) * 2
        v = (torch.rand((self._num_landmarks, 2)) + landmark_motion_bias - 0.5) * self._landmark_motion_scale

        self._mu_real = mu
        self._v = v
        self._landmark_motion_bias = landmark_motion_bias
        self._x = x
        self._step_num = 0

        self.history_poses = [self._x.detach().numpy().tolist()]
        self.fig = None
        self.ax = None
        self._episode_id += 1
        self._reset_video_state()

        return self._mu_real, v, x, False

    def _generate_clusters(self, num_landmarks, num_clusters, env_size, robot_pose, min_robot_dist):
        centers = torch.zeros((num_clusters, 2))
        env_half = env_size * 0.5
        robot_xy = robot_pose[:2]
        min_center_sep = min_robot_dist * 0.5

        for i in range(num_clusters):
            best_center = None
            best_min_dist = -1.0
            for _ in range(50):
                candidate = (torch.rand(2) - 0.5) * env_size
                d_to_robot = torch.linalg.norm(robot_xy - candidate).item()

                if i > 0:
                    d_to_centers = torch.linalg.norm(centers[:i] - candidate, dim=1)
                    if d_to_centers.min().item() < min_center_sep:
                        continue

                if d_to_robot >= min_robot_dist:
                    best_center = candidate
                    break

                if d_to_robot > best_min_dist:
                    best_min_dist = d_to_robot
                    best_center = candidate

            centers[i] = best_center if best_center is not None else torch.clamp(candidate, -env_half, env_half)

        mu = torch.zeros((num_landmarks, 2))
        points_per_cluster = num_landmarks // num_clusters
        remainder = num_landmarks % num_clusters
        start_idx = 0
        for i in range(num_clusters):
            count = points_per_cluster + (1 if i < remainder else 0)
            cluster_points = centers[i] + torch.randn((count, 2)) * (env_size[0] / 15.0)
            mu[start_idx:start_idx + count] = cluster_points
            start_idx += count

        mu = torch.clamp(mu, -env_half, env_half)
        return mu

    def step(self, action: tensor) -> Tuple[tensor, tensor, tensor, bool]:
        self._x = SE2_kinematics(self._x, action, self._tau)
        # self._x[:2] = torch.clip(self._x[:2], min=torch.zeros(2), max=self._env_size)

        self._mu_real = torch.clip(landmark_motion_real(self._mu_real, self._v, self._A, self._B, self._W),
                                   min=-self._env_size/2, max=self._env_size/2)

        self._v = (torch.rand((self._num_landmarks, 2)) + self._landmark_motion_bias - 0.5) *\
                  self._landmark_motion_scale

        done = False
        self._step_num += 1
        if self._step_num >= self._horizon:
            done = True

        self.history_poses.append(self._x.detach().numpy().tolist())

        return self._mu_real, self._v, self._x, done

    # def render(self):
    #     render_size = 50
    #     arrow_length = 20
    #     canvas = 255 * np.ones((self._env_size[1] * render_size, self._env_size[0] * render_size), dtype=np.uint8)
    #     canvas = cv2.cvtColor(canvas, cv2.COLOR_GRAY2RGB)
    #
    #     # cv2.circle(canvas, (0, int(self._env_size[1] / 4 * render_size)), 10, (255, 0, 0), -1)
    #
    #     for landmark_pos in self._mu:
    #         cv2.circle(canvas, (int(landmark_pos[0] * render_size), int(landmark_pos[1] * render_size)), 10,
    #                    (255, 0, 0), -1)
    #
    #     robot_pose = self._x.detach().numpy()
    #     # robot_pose = np.array([0, 5, np.pi * 0.0])
    #
    #     cv2.circle(canvas, (int(robot_pose[0] * render_size), int(robot_pose[1] * render_size)), 10, (0, 0, 255), -1)
    #     canvas = cv2.arrowedLine(canvas, (int(robot_pose[0] * render_size), int(robot_pose[1] * render_size)),
    #                              (int(robot_pose[0] * render_size + arrow_length * np.cos(robot_pose[2])),
    #                               int(robot_pose[1] * render_size + arrow_length * np.sin(robot_pose[2]))),
    #                              (0, 0, 255), 2, tipLength=0.5)
    #
    #     FoV_corners = np.array([(int(robot_pose[0] * render_size), int(robot_pose[1] * render_size)),
    #                             (int((robot_pose[0] + self._radius *
    #                                   np.cos(robot_pose[2] + self._psi) / np.cos(self._psi)) * render_size),
    #                              int((robot_pose[1] + self._radius *
    #                                   np.sin(robot_pose[2] + self._psi) / np.cos(self._psi)) * render_size)),
    #                             (int((robot_pose[0] + self._radius *
    #                                   np.cos(robot_pose[2] - self._psi) / np.cos(self._psi)) * render_size),
    #                              int((robot_pose[1] + self._radius *
    #                                   np.sin(robot_pose[2] - self._psi) / np.cos(self._psi)) * render_size))])
    #
    #     cv2.polylines(canvas, [FoV_corners], isClosed=True, color=(0, 255, 0), thickness=2)
    #
    #     cv2.namedWindow('map', cv2.WINDOW_GUI_NORMAL)
    #     cv2.imshow('map', canvas)
    #     # cv2.resizeWindow('map', *render_size)
    #     cv2.waitKey(100)

    def _plot(self, legend, title='trajectory'):
        if self.fig is None or self.ax is None:
            self.fig = plt.figure(1)
            self.ax = self.fig.gca()
        self.landmarks = self._mu_real.flatten().detach().numpy().reshape(self._num_landmarks*2, 1)

        # plot agent trajectory
        plt.tick_params(labelsize=15)
        history_poses = np.array(self.history_poses)  # with the shape of (self._num_landmarks, 2)
        self.ax.plot(history_poses[:, 0], history_poses[:, 1], c='black', linewidth=3, label='agent trajectory')

        # plot agent trajectory start & end
        self.ax.scatter(history_poses[0, 0], history_poses[0, 1], marker='>', s=70, c='red', label="start")
        self.ax.scatter(history_poses[-1, 0], history_poses[-1, 1], marker='s', s=70, c='red', label="end")

        self.ax.scatter(history_poses[-1, 0] + np.cos(history_poses[-1, 2])*0.5,
                     history_poses[-1, 1] + np.sin(history_poses[-1, 2])*0.5, marker='o', c='black')

        # plot landmarks
        self.ax.scatter(self.landmarks[list(range(0, self._num_landmarks*2, 2)), :],
                        self.landmarks[list(range(1, self._num_landmarks*2+1, 2)), :], s=50, c='blue', label="landmark")

        # axes
        self.ax.set_xlabel("x", fontdict={'size': 20})
        self.ax.set_ylabel("y", fontdict={'size': 20})

        # title
        # self.ax.set_title(title, fontdict={'size': 16})

        self.ax.set_facecolor('whitesmoke')
        plt.grid(alpha=0.4)
        # legend
        if legend == True:
            self.ax.legend()
            plt.legend(prop={'size': 14})


    def _draw_on_axis_single(self, ax):
        ax.clear()

        if torch.is_tensor(self._x):
            robot_t = self._x.detach().cpu()
            robot = robot_t.numpy()
        else:
            robot = np.array(self._x)
            robot_t = torch.tensor(robot, dtype=torch.float32)

        if torch.is_tensor(self._mu_real):
            landmarks_t = self._mu_real.detach().cpu()
            landmarks = landmarks_t.numpy()
        else:
            landmarks = np.array(self._mu_real)
            landmarks_t = torch.tensor(landmarks, dtype=torch.float32)

        all_x = np.concatenate(([robot[0]], landmarks[:, 0]))
        all_y = np.concatenate(([robot[1]], landmarks[:, 1]))
        min_x, max_x = np.min(all_x), np.max(all_x)
        min_y, max_y = np.min(all_y), np.max(all_y)
        padding = 5.0
        ax.set_xlim(min_x - padding, max_x + padding)
        ax.set_ylim(min_y - padding, max_y + padding)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(f"Step: {self._step_num}", fontsize=12)

        rx, ry, r_theta = float(robot[0]), float(robot[1]), float(robot[2])
        robot_circle = Circle((rx, ry), 0.2, color='black', zorder=5)
        ax.add_patch(robot_circle)
        arrow_len = 0.3
        ax.arrow(rx, ry, arrow_len * np.cos(r_theta), arrow_len * np.sin(r_theta),
                 head_width=0.1, color='white', zorder=6)

        psi_val = self._psi.item() if torch.is_tensor(self._psi) else float(self._psi)
        radius_val = self._radius.item() if torch.is_tensor(self._radius) else float(self._radius)
        angle_left = r_theta + psi_val
        angle_right = r_theta - psi_val

        x_left = rx + radius_val * np.cos(angle_left)
        y_left = ry + radius_val * np.sin(angle_left)
        x_right = rx + radius_val * np.cos(angle_right)
        y_right = ry + radius_val * np.sin(angle_right)

        fov_verts = np.array([[rx, ry], [x_left, y_left], [x_right, y_right]])
        fov_patch = Polygon(fov_verts, closed=True, color='blue', alpha=0.15, zorder=1)
        ax.add_patch(fov_patch)
        ax.plot([rx, x_left], [ry, y_left], color='blue', alpha=0.3, linewidth=1)
        ax.plot([rx, x_right], [ry, y_right], color='blue', alpha=0.3, linewidth=1)

        q = torch.vstack((
            (landmarks_t[:, 0] - robot_t[0]) * torch.cos(robot_t[2]) +
            (landmarks_t[:, 1] - robot_t[1]) * torch.sin(robot_t[2]),
            (robot_t[0] - landmarks_t[:, 0]) * torch.sin(robot_t[2]) +
            (landmarks_t[:, 1] - robot_t[1]) * torch.cos(robot_t[2])
        )).T
        sdf = triangle_SDF(q, self._psi, self._radius)
        is_seen = (sdf <= 0).cpu().numpy()
        colors = ['green' if seen else 'red' for seen in is_seen]
        sizes = [100 if seen else 50 for seen in is_seen]
        ax.scatter(landmarks[:, 0], landmarks[:, 1], c=colors, s=sizes, marker='*', zorder=4)

    def get_current_frame(self):
        fig = Figure(figsize=(10, 10), dpi=100)
        canvas = FigureCanvasAgg(fig)
        ax = fig.add_subplot(111)
        self._draw_on_axis_single(ax)
        canvas.draw()
        data = np.frombuffer(canvas.buffer_rgba(), dtype=np.uint8)
        w, h = fig.get_size_inches() * fig.get_dpi()
        w, h = int(w), int(h)
        data = data.reshape((h, w, 4))
        return data[:, :, :3]

    def render(self, mode='human'):
        frame = self.get_current_frame()

        os.makedirs('test_trails', exist_ok=True)
        if self._video_writer is None:
            height, width, layers = frame.shape
            self._video_frame_size = (width, height)
            self._video_path = os.path.join('test_trails', f'episode_{self._episode_id:04d}.avi')
            fourcc = cv2.VideoWriter_fourcc(*'DIVX')
            self._video_writer = cv2.VideoWriter(self._video_path, fourcc, 5, self._video_frame_size)

        self._video_writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

    def save_plot(self, name='default.png', title='trajectory', legend=False):
        if self.fig is None or self.ax is None:
            self.fig = plt.figure(1)
            self.ax = self.fig.gca()
        self.ax.cla()
        self._plot(legend, title=title)
        self.fig.savefig(name, bbox_inches = 'tight')

    def close (self):
        if self._video_writer is not None:
            self._video_writer.release()
            self._video_writer = None
        plt.close('all')


class SimpleEnvAtt:
    def __init__(self, max_num_landmarks, horizon, tau, A, B, V, W, landmark_motion_scale, psi, radius):
        self._max_num_landmarks = max_num_landmarks

        self._tau = tau
        self._A = A
        self._B = B
        self._V = V  # sensor_std ** 2 with the shape of (2, )
        self._W = W  # motion_std ** 2 with the shape of (2, )
        self._landmark_motion_scale = landmark_motion_scale

        self._mu = None
        self._v = None
        self._landmark_motion_bias = None
        self._x = None
        self._step_num = None

        self._psi = psi
        self._radius = radius
        self._episode_id = 0
        self._video_writer = None
        self._video_path = None
        self._video_frame_size = None

    def _reset_video_state(self):
        if self._video_writer is not None:
            self._video_writer.release()
        self._video_writer = None
        self._video_path = None
        self._video_frame_size = None

    def reset(self):
        self._num_landmarks = torch.randint(3, 8, (1, )).item()
        self._env_size = tensor([self._num_landmarks * 4, self._num_landmarks * 4])
        self._horizon = self._num_landmarks * 5
        x = torch.empty(3)
        x[:2] = (torch.rand(2) - 0.5) * self._env_size * 1.25
        x[2] = (torch.rand(1) * 2 - 1) * torch.pi

        num_clusters = min(2, self._num_landmarks)
        min_robot_dist = 0.3 * torch.min(self._env_size)
        mu = self._generate_clusters(self._num_landmarks, num_clusters, self._env_size, x, min_robot_dist)

        landmark_motion_bias = (torch.rand(2) - 0.5) * 1.6
        v = (torch.rand((self._num_landmarks, 2)) - 0.5) * self._landmark_motion_scale + landmark_motion_bias

        self._mu_real = mu
        self._v = v
        self._landmark_motion_bias = landmark_motion_bias
        self._x = x
        self._step_num = 0

        self.history_poses = [self._x.detach().numpy().tolist()]
        self.fig = None
        self.ax = None
        self._episode_id += 1
        self._reset_video_state()

        return self._mu_real, v, x, False

    def _generate_clusters(self, num_landmarks, num_clusters, env_size, robot_pose, min_robot_dist):
        centers = torch.zeros((num_clusters, 2))
        env_half = env_size * 0.5
        robot_xy = robot_pose[:2]
        min_center_sep = min_robot_dist * 0.5

        for i in range(num_clusters):
            best_center = None
            best_min_dist = -1.0
            for _ in range(50):
                candidate = (torch.rand(2) - 0.5) * env_size
                d_to_robot = torch.linalg.norm(robot_xy - candidate).item()

                if i > 0:
                    d_to_centers = torch.linalg.norm(centers[:i] - candidate, dim=1)
                    if d_to_centers.min().item() < min_center_sep:
                        continue

                if d_to_robot >= min_robot_dist:
                    best_center = candidate
                    break

                if d_to_robot > best_min_dist:
                    best_min_dist = d_to_robot
                    best_center = candidate

            centers[i] = best_center if best_center is not None else torch.clamp(candidate, -env_half, env_half)

        mu = torch.zeros((num_landmarks, 2))
        points_per_cluster = num_landmarks // num_clusters
        remainder = num_landmarks % num_clusters
        start_idx = 0
        for i in range(num_clusters):
            count = points_per_cluster + (1 if i < remainder else 0)
            cluster_points = centers[i] + torch.randn((count, 2)) * (env_size[0] / 15.0)
            mu[start_idx:start_idx + count] = cluster_points
            start_idx += count

        mu = torch.clamp(mu, -env_half, env_half)
        return mu

    def step(self, action: tensor) -> Tuple[tensor, tensor, tensor, bool]:
        self._x = SE2_kinematics(self._x, action, self._tau)
        # self._x[:2] = torch.clip(self._x[:2], min=torch.zeros(2), max=self._env_size)

        # self._mu_real = torch.clip(landmark_motion_real(self._mu_real, self._v, self._A, self._B, self._W),
        #                            min=-self._env_size/2, max=self._env_size/2)
        self._mu_real = landmark_motion_real(self._mu_real, self._v, self._A, self._B, self._W)

        self._v = (torch.rand((self._num_landmarks, 2)) - 0.5) * self._landmark_motion_scale + \
                  self._landmark_motion_bias

        done = False
        self._step_num += 1
        if self._step_num >= self._horizon:
            done = True

        self.history_poses.append(self._x.detach().numpy().tolist())

        return self._mu_real, self._v, self._x, done

    def _plot(self, legend, title='trajectory'):
        if self.fig is None or self.ax is None:
            self.fig = plt.figure(1)
            self.ax = self.fig.gca()
        self.landmarks = self._mu_real.flatten().detach().numpy().reshape(self._num_landmarks*2, 1)

        # plot agent trajectory
        plt.tick_params(labelsize=15)
        history_poses = np.array(self.history_poses)  # with the shape of (self._num_landmarks, 2)
        self.ax.plot(history_poses[:, 0], history_poses[:, 1], c='black', linewidth=3, label='agent trajectory')

        # plot agent trajectory start & end
        self.ax.scatter(history_poses[0, 0], history_poses[0, 1], marker='>', s=70, c='red', label="start")
        self.ax.scatter(history_poses[-1, 0], history_poses[-1, 1], marker='s', s=70, c='red', label="end")

        self.ax.scatter(history_poses[-1, 0] + np.cos(history_poses[-1, 2])*0.5,
                     history_poses[-1, 1] + np.sin(history_poses[-1, 2])*0.5, marker='o', c='black')

        # plot landmarks
        self.ax.scatter(self.landmarks[list(range(0, self._num_landmarks*2, 2)), :],
                        self.landmarks[list(range(1, self._num_landmarks*2+1, 2)), :], s=50, c='blue', label="landmark")

        # axes
        self.ax.set_xlabel("x", fontdict={'size': 20})
        self.ax.set_ylabel("y", fontdict={'size': 20})

        # title
        # self.ax.set_title(title, fontdict={'size': 16})

        self.ax.set_facecolor('whitesmoke')
        plt.grid(alpha=0.4)
        # legend
        if legend == True:
            self.ax.legend()
            plt.legend(prop={'size': 14})


    def _draw_on_axis_single(self, ax):
        ax.clear()

        if torch.is_tensor(self._x):
            robot_t = self._x.detach().cpu()
            robot = robot_t.numpy()
        else:
            robot = np.array(self._x)
            robot_t = torch.tensor(robot, dtype=torch.float32)

        if torch.is_tensor(self._mu_real):
            landmarks_t = self._mu_real.detach().cpu()
            landmarks = landmarks_t.numpy()
        else:
            landmarks = np.array(self._mu_real)
            landmarks_t = torch.tensor(landmarks, dtype=torch.float32)

        all_x = np.concatenate(([robot[0]], landmarks[:, 0]))
        all_y = np.concatenate(([robot[1]], landmarks[:, 1]))
        min_x, max_x = np.min(all_x), np.max(all_x)
        min_y, max_y = np.min(all_y), np.max(all_y)
        padding = 5.0
        ax.set_xlim(min_x - padding, max_x + padding)
        ax.set_ylim(min_y - padding, max_y + padding)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(f"Step: {self._step_num}", fontsize=12)

        rx, ry, r_theta = float(robot[0]), float(robot[1]), float(robot[2])
        robot_circle = Circle((rx, ry), 0.2, color='black', zorder=5)
        ax.add_patch(robot_circle)
        arrow_len = 0.3
        ax.arrow(rx, ry, arrow_len * np.cos(r_theta), arrow_len * np.sin(r_theta),
                 head_width=0.1, color='white', zorder=6)

        psi_val = self._psi.item() if torch.is_tensor(self._psi) else float(self._psi)
        radius_val = self._radius.item() if torch.is_tensor(self._radius) else float(self._radius)
        angle_left = r_theta + psi_val
        angle_right = r_theta - psi_val

        x_left = rx + radius_val * np.cos(angle_left)
        y_left = ry + radius_val * np.sin(angle_left)
        x_right = rx + radius_val * np.cos(angle_right)
        y_right = ry + radius_val * np.sin(angle_right)

        fov_verts = np.array([[rx, ry], [x_left, y_left], [x_right, y_right]])
        fov_patch = Polygon(fov_verts, closed=True, color='blue', alpha=0.15, zorder=1)
        ax.add_patch(fov_patch)
        ax.plot([rx, x_left], [ry, y_left], color='blue', alpha=0.3, linewidth=1)
        ax.plot([rx, x_right], [ry, y_right], color='blue', alpha=0.3, linewidth=1)

        q = torch.vstack((
            (landmarks_t[:, 0] - robot_t[0]) * torch.cos(robot_t[2]) +
            (landmarks_t[:, 1] - robot_t[1]) * torch.sin(robot_t[2]),
            (robot_t[0] - landmarks_t[:, 0]) * torch.sin(robot_t[2]) +
            (landmarks_t[:, 1] - robot_t[1]) * torch.cos(robot_t[2])
        )).T
        sdf = triangle_SDF(q, self._psi, self._radius)
        is_seen = (sdf <= 0).cpu().numpy()
        colors = ['green' if seen else 'red' for seen in is_seen]
        sizes = [100 if seen else 50 for seen in is_seen]
        ax.scatter(landmarks[:, 0], landmarks[:, 1], c=colors, s=sizes, marker='*', zorder=4)

    def get_current_frame(self):
        fig = Figure(figsize=(10, 10), dpi=100)
        canvas = FigureCanvasAgg(fig)
        ax = fig.add_subplot(111)
        self._draw_on_axis_single(ax)
        canvas.draw()
        data = np.frombuffer(canvas.buffer_rgba(), dtype=np.uint8)
        w, h = fig.get_size_inches() * fig.get_dpi()
        w, h = int(w), int(h)
        data = data.reshape((h, w, 4))
        return data[:, :, :3]

    def render(self, mode='human'):
        frame = self.get_current_frame()

        os.makedirs('test_trails', exist_ok=True)
        if self._video_writer is None:
            height, width, layers = frame.shape
            self._video_frame_size = (width, height)
            self._video_path = os.path.join('test_trails', f'episode_{self._episode_id:04d}.avi')
            fourcc = cv2.VideoWriter_fourcc(*'DIVX')
            self._video_writer = cv2.VideoWriter(self._video_path, fourcc, 5, self._video_frame_size)

        self._video_writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

    def save_plot(self, name='default.png', title='trajectory', legend=False):
        if self.fig is None or self.ax is None:
            self.fig = plt.figure(1)
            self.ax = self.fig.gca()
        self.ax.cla()
        self._plot(legend, title=title)
        self.fig.savefig(name, bbox_inches = 'tight')

    def close (self):
        if self._video_writer is not None:
            self._video_writer.release()
            self._video_writer = None
        plt.close('all')
