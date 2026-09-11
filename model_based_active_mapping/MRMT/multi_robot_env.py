import torch
import cv2
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
from typing import Tuple
from torch import tensor
from utilities.utils import SE2_kinematics, landmark_motion_real
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

class MultiRobotEnv:
    def __init__(self, num_robots, max_num_landmarks, horizon, tau, A, B, V, W, landmark_motion_scale, psi, radius, num_clusters=2, clustering_prob=0.65):
        self._num_robots = num_robots
        self._max_num_landmarks = max_num_landmarks
        self._num_clusters = num_clusters
        self._clustering_prob = clustering_prob

        self._tau = tau
        self._A = A
        self._B = B
        self._V = V  # sensor_std ** 2 with the shape of (2, )
        self._W = W  # motion_std ** 2 with the shape of (2, )
        self._landmark_motion_scale = landmark_motion_scale

        self._mu = None
        self._v = None
        self._landmark_motion_bias = None
        self._x = None # Robot poses (num_robots, 3)
        self._step_num = None

        self._psi = psi
        self._radius = radius

    def reset(self):
        # Randomize number of landmarks, respecting max_num_landmarks
        self._num_landmarks = torch.randint(4, min(10, self._max_num_landmarks + 1), (1, )).item()
        # Ensure enough landmarks for clusters
        self._num_landmarks = max(self._num_landmarks, self._num_clusters)
        # Final clamp to max_num_landmarks
        self._num_landmarks = min(self._num_landmarks, self._max_num_landmarks)
        
        self._env_size = tensor([self._num_landmarks * 3.2, self._num_landmarks * 3.2])
        self._horizon = self._num_landmarks * 5
        
        # Set FOV radius to at least 60% of map size
        map_size = self._env_size[0].item()  # Assuming square map
        
        # Initialize Robots
        # Spread them out initially or put them together? Let's put them slightly apart near center
        x = torch.empty(self._num_robots, 3)
        for i in range(self._num_robots):
             x[i, :2] = (torch.rand(2) - 0.5) * self._env_size * 0.5 # Start somewhat central
             x[i, 2] = (torch.rand(1) * 2 - 1) * torch.pi

        # Generate Landmarks (Clustered or Uniform)
        if torch.rand(1).item() < self._clustering_prob:
            min_robot_dist = 0.3 * torch.min(self._env_size)
            mu = self._generate_clusters(self._num_landmarks, self._num_clusters, self._env_size, x, min_robot_dist)
        else:
            mu = (torch.rand((self._num_landmarks, 2)) - 0.5) * self._env_size

        landmark_motion_bias = (torch.rand(2) - 0.5) * 1.6
        v = (torch.rand((self._num_landmarks, 2)) - 0.5) * self._landmark_motion_scale + landmark_motion_bias

        self._mu_real = mu
        self._v = v
        self._landmark_motion_bias = landmark_motion_bias
        self._x = x
        self._step_num = 0

        self.history_poses = [self._x.detach().numpy().tolist()] # List of (num_robots, 3)
        # Don't create matplotlib figures during training - only when rendering/plotting
        self.fig = None
        self.ax = None

        return self._mu_real, v, x, False

    def _generate_clusters(self, num_landmarks, num_clusters, env_size, robot_poses, min_robot_dist):
        # Randomly place clusters while keeping them far from robot start positions.
        centers = torch.zeros((num_clusters, 2))
        env_half = env_size * 0.5
        robot_xy = robot_poses[:, :2]
        min_center_sep = min_robot_dist * 0.5

        for i in range(num_clusters):
            best_center = None
            best_min_dist = -1.0
            for _ in range(50):
                candidate = (torch.rand(2) - 0.5) * env_size
                d_to_robots = torch.linalg.norm(robot_xy - candidate, dim=1)
                min_dist = d_to_robots.min().item()

                if i > 0:
                    d_to_centers = torch.linalg.norm(centers[:i] - candidate, dim=1)
                    if d_to_centers.min().item() < min_center_sep:
                        continue

                if min_dist >= min_robot_dist:
                    best_center = candidate
                    break

                if min_dist > best_min_dist:
                    best_min_dist = min_dist
                    best_center = candidate

            centers[i] = best_center if best_center is not None else torch.clamp(candidate, -env_half, env_half)
        
        mu = torch.zeros((num_landmarks, 2))
        
        points_per_cluster = num_landmarks // num_clusters
        remainder = num_landmarks % num_clusters
        
        start_idx = 0
        for i in range(num_clusters):
            count = points_per_cluster + (1 if i < remainder else 0)
            # Gaussian spread around center (smaller spread for tighter clusters)
            cluster_points = centers[i] + torch.randn((count, 2)) * (env_size[0] / 15.0) 
            mu[start_idx:start_idx+count] = cluster_points
            start_idx += count
            
        return mu

    def step(self, action: tensor) -> Tuple[tensor, tensor, tensor, bool]:
        # action shape: (num_robots, 2)
        
        # Update each robot
        next_x_list = []
        for i in range(self._num_robots):
            next_x_list.append(SE2_kinematics(self._x[i], action[i], self._tau))
        self._x = torch.stack(next_x_list)

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
        self.landmarks = self._mu_real.flatten().detach().numpy().reshape(self._num_landmarks*2, 1)

        # plot agent trajectory
        plt.tick_params(labelsize=15)
        history_poses = np.array(self.history_poses)  # shape (steps, num_robots, 3)
        
        colors = ['black', 'green', 'purple', 'orange'] # Support up to 4 robots for now
        
        for i in range(self._num_robots):
            c = colors[i % len(colors)]
            self.ax.plot(history_poses[:, i, 0], history_poses[:, i, 1], c=c, linewidth=3, label=f'agent {i} trajectory')
            
            # Start & End
            self.ax.scatter(history_poses[0, i, 0], history_poses[0, i, 1], marker='>', s=70, c='red')
            self.ax.scatter(history_poses[-1, i, 0], history_poses[-1, i, 1], marker='s', s=70, c='red')
            
            # Orientation
            self.ax.scatter(history_poses[-1, i, 0] + np.cos(history_poses[-1, i, 2])*0.5,
                         history_poses[-1, i, 1] + np.sin(history_poses[-1, i, 2])*0.5, marker='o', c=c)
            
            # Draw FOV cone for final position
            robot_x = history_poses[-1, i, 0]
            robot_y = history_poses[-1, i, 1]
            robot_theta = history_poses[-1, i, 2]
            
            # FOV is a triangle defined by psi (half angle) and radius
            psi_val = self._psi.item() if isinstance(self._psi, torch.Tensor) else self._psi
            radius_val = self._radius
            
            # Three corners of FOV triangle
            # Center point (robot position)
            fov_x = [robot_x]
            fov_y = [robot_y]
            
            # Left edge
            left_angle = robot_theta + psi_val
            fov_x.append(robot_x + radius_val * np.cos(left_angle))
            fov_y.append(robot_y + radius_val * np.sin(left_angle))
            
            # Right edge
            right_angle = robot_theta - psi_val
            fov_x.append(robot_x + radius_val * np.cos(right_angle))
            fov_y.append(robot_y + radius_val * np.sin(right_angle))
            
            # Close the triangle
            fov_x.append(robot_x)
            fov_y.append(robot_y)
            
            # Plot FOV as filled polygon with transparency
            self.ax.fill(fov_x, fov_y, color=c, alpha=0.15, linestyle='--', linewidth=1, edgecolor=c)

        # plot landmarks
        self.ax.scatter(self.landmarks[list(range(0, self._num_landmarks*2, 2)), :],
                        self.landmarks[list(range(1, self._num_landmarks*2+1, 2)), :], s=50, c='blue', label="landmark")

        # axes
        self.ax.set_xlabel("x", fontdict={'size': 20})
        self.ax.set_ylabel("y", fontdict={'size': 20})

        self.ax.set_facecolor('whitesmoke')
        plt.grid(alpha=0.4)
        # legend
        if legend == True:
            self.ax.legend()
            plt.legend(prop={'size': 14})


    def render(self, mode='human'):
        if self.fig is None:
            self.fig = plt.figure(1)
            self.ax = self.fig.gca()
        self.ax.cla()
        self._plot(True)
        plt.draw()
        plt.pause(0.3)

    def save_plot(self, name='default.png', title='trajectory', legend=False):
        if self.fig is None:
            self.fig = plt.figure(1)
            self.ax = self.fig.gca()
        self.ax.cla()
        self._plot(legend, title=title)
        self.fig.savefig(name, bbox_inches = 'tight')

    def close (self):
        plt.close('all')
    
    def save_episode_animation(self, name_prefix='episode', title='Episode', legend=False):
        """Save each step of the episode as a separate image."""
        import os
        
        # Create directory for episode frames
        base_dir = os.path.dirname(name_prefix) if os.path.dirname(name_prefix) else '.'
        os.makedirs(base_dir, exist_ok=True)
        
        history_poses = np.array(self.history_poses)
        num_steps = len(history_poses)
        
        for step in range(num_steps):
            fig_temp = plt.figure(figsize=(10, 8))
            ax_temp = fig_temp.add_subplot(111)
            
            # Plot trajectories up to current step
            colors = ['black', 'green', 'purple', 'orange']
            
            for i in range(self._num_robots):
                c = colors[i % len(colors)]
                # Trajectory up to current step
                ax_temp.plot(history_poses[:step+1, i, 0], history_poses[:step+1, i, 1], 
                           c=c, linewidth=2, label=f'agent {i} trajectory')
                
                # Current position
                ax_temp.scatter(history_poses[step, i, 0], history_poses[step, i, 1], 
                              marker='o', s=100, c=c, edgecolors='black', linewidths=2)
                
                # Current orientation arrow
                arrow_length = 2.0
                dx = arrow_length * np.cos(history_poses[step, i, 2])
                dy = arrow_length * np.sin(history_poses[step, i, 2])
                ax_temp.arrow(history_poses[step, i, 0], history_poses[step, i, 1], 
                            dx, dy, head_width=1.0, head_length=0.5, fc=c, ec=c)
                
                # Draw FOV cone
                robot_x = history_poses[step, i, 0]
                robot_y = history_poses[step, i, 1]
                robot_theta = history_poses[step, i, 2]
                
                psi_val = self._psi.item() if isinstance(self._psi, torch.Tensor) else self._psi
                radius_val = self._radius
                
                fov_x = [robot_x]
                fov_y = [robot_y]
                
                left_angle = robot_theta + psi_val
                fov_x.append(robot_x + radius_val * np.cos(left_angle))
                fov_y.append(robot_y + radius_val * np.sin(left_angle))
                
                right_angle = robot_theta - psi_val
                fov_x.append(robot_x + radius_val * np.cos(right_angle))
                fov_y.append(robot_y + radius_val * np.sin(right_angle))
                
                fov_x.append(robot_x)
                fov_y.append(robot_y)
                
                ax_temp.fill(fov_x, fov_y, color=c, alpha=0.2, linestyle='--', linewidth=1.5, edgecolor=c)
            
            # Plot landmarks
            landmarks = self._mu_real.flatten().detach().numpy().reshape(self._num_landmarks*2, 1)
            ax_temp.scatter(landmarks[list(range(0, self._num_landmarks*2, 2)), :],
                          landmarks[list(range(1, self._num_landmarks*2+1, 2)), :], 
                          s=80, c='blue', marker='*', label="landmark", edgecolors='black', linewidths=1)
            
            # Formatting
            ax_temp.set_xlabel("x", fontdict={'size': 16})
            ax_temp.set_ylabel("y", fontdict={'size': 16})
            ax_temp.set_title(f"{title} - Step {step+1}/{num_steps}", fontdict={'size': 18})
            ax_temp.set_facecolor('whitesmoke')
            ax_temp.grid(alpha=0.4)
            
            if legend and step == 0:
                ax_temp.legend(fontsize=12)
            
            # Save frame
            frame_filename = f"{name_prefix}_step_{step:03d}.png"
            fig_temp.savefig(frame_filename, bbox_inches='tight', dpi=100)
            plt.close(fig_temp)
        
        print(f"Saved {num_steps} frames to {base_dir}/")

    def _draw_on_axis(self, ax, current_fov_mask=None):
        ax.clear()
        
        # A. Extract State
        if torch.is_tensor(self._x):
            robots = self._x.cpu().detach().numpy()
        else:
            robots = self._x
            
        if torch.is_tensor(self._mu_real):
            landmarks = self._mu_real.cpu().detach().numpy()
        else:
            landmarks = self._mu_real

        # B. Dynamic Camera Logic
        all_x = np.concatenate((robots[:, 0], landmarks[:, 0]))
        all_y = np.concatenate((robots[:, 1], landmarks[:, 1]))
        min_x, max_x = np.min(all_x), np.max(all_x)
        min_y, max_y = np.min(all_y), np.max(all_y)
        padding = 5.0 
        ax.set_xlim(min_x - padding, max_x + padding)
        ax.set_ylim(min_y - padding, max_y + padding)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(f"Step: {self._step_num}", fontsize=12)

        # C. Plot Robots and FOV
        for i in range(self._num_robots):
            rx, ry, r_theta = float(robots[i, 0]), float(robots[i, 1]), float(robots[i, 2])
            
            # Robot Body
            robot_circle = Circle((rx, ry), 0.2, color='black', zorder=5)
            ax.add_patch(robot_circle)
            
            # Heading Arrow
            arrow_len = 0.3
            ax.arrow(rx, ry, arrow_len * np.cos(r_theta), arrow_len * np.sin(r_theta), 
                     head_width=0.1, color='white', zorder=6)

            # FOV Triangle
            angle_left = r_theta + float(self._psi)
            angle_right = r_theta - float(self._psi)
            
            x_left = rx + float(self._radius) * np.cos(angle_left)
            y_left = ry + float(self._radius) * np.sin(angle_left)
            
            x_right = rx + float(self._radius) * np.cos(angle_right)
            y_right = ry + float(self._radius) * np.sin(angle_right)
            
            fov_verts = np.array([[rx, ry], [x_left, y_left], [x_right, y_right]])
            fov_patch = Polygon(fov_verts, closed=True, color='blue', alpha=0.15, zorder=1)
            ax.add_patch(fov_patch)
            ax.plot([rx, x_left], [ry, y_left], color='blue', alpha=0.3, linewidth=1)
            ax.plot([rx, x_right], [ry, y_right], color='blue', alpha=0.3, linewidth=1)

        # D. Plot Landmarks (Red/Green)
        if current_fov_mask is not None:
            if torch.is_tensor(current_fov_mask):
                mask_np = current_fov_mask.cpu().detach().numpy()
            else:
                mask_np = current_fov_mask
            
            # Check for size mismatch before plotting (Handling dynamic num_landmarks)
            num_actual = landmarks.shape[0]
            if mask_np.shape[1] >= num_actual:
                mask_slice = mask_np[:, :num_actual]
                is_seen = np.any(mask_slice, axis=0)
                colors = ['green' if seen else 'red' for seen in is_seen]
                sizes = [100 if seen else 50 for seen in is_seen]
                ax.scatter(landmarks[:, 0], landmarks[:, 1], c=colors, s=sizes, marker='*', zorder=4)
            else:
                # Fallback if mask size doesn't match (shouldn't happen with correct updates)
                ax.scatter(landmarks[:, 0], landmarks[:, 1], c='red', marker='*', zorder=4)
        else:
            ax.scatter(landmarks[:, 0], landmarks[:, 1], c='red', marker='*', zorder=4)

    # 2. Live Render (Used by your Training Loop Plot)
    def render_live(self, ax, current_fov_mask=None):
        self._draw_on_axis(ax, current_fov_mask)

    # 3. New Frame Getter (Used for Video Saving)
    def get_current_frame(self, current_fov_mask=None):
        # 1. Create a pure Figure object (not attached to pyplot interface)
        fig = Figure(figsize=(10, 10), dpi=100)
        
        # 2. Attach the Agg canvas (Headless backend)
        canvas = FigureCanvasAgg(fig)
        
        # 3. Add subplot and draw
        ax = fig.add_subplot(111)
        self._draw_on_axis(ax, current_fov_mask)
        
        # 4. Render to buffer
        canvas.draw()
        
        # 5. Extract RGBA buffer
        data = np.frombuffer(canvas.buffer_rgba(), dtype=np.uint8)
        
        # 6. Reshape and Convert
        w, h = fig.get_size_inches() * fig.get_dpi()
        w, h = int(w), int(h)
        data = data.reshape((h, w, 4))
        
        # Drop Alpha channel (RGBA -> RGB)
        data = data[:, :, :3]
        
        return data
