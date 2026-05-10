from collections import defaultdict
from typing import Dict, List

import numpy as np


class TrackingStatistics:
    def __init__(self, num_robots: int, num_landmarks: int):
        self.num_robots = num_robots
        self.num_landmarks = num_landmarks
        self.first_tracked_step = [-1] * num_landmarks
        self.robot_tracking = [[0] * num_landmarks for _ in range(num_robots)]
        self.cumulative_tracking = [0] * num_landmarks
        self.overlap_counts = [0] * num_landmarks
        self.steps_since_first_tracked = [0] * num_landmarks
        self.current_step = 0

    def update(self, fov_mask):
        if fov_mask is None:
            self.current_step += 1
            return

        fov_mask_np = fov_mask.cpu().numpy() if hasattr(fov_mask, "cpu") else np.array(fov_mask)

        for target_idx in range(self.num_landmarks):
            robots_tracking = [fov_mask_np[r, target_idx] for r in range(self.num_robots)]
            any_tracking = any(robots_tracking)
            num_robots_tracking = sum(robots_tracking)

            if any_tracking and self.first_tracked_step[target_idx] == -1:
                self.first_tracked_step[target_idx] = self.current_step

            if self.first_tracked_step[target_idx] != -1:
                self.steps_since_first_tracked[target_idx] += 1
                for robot_idx in range(self.num_robots):
                    if robots_tracking[robot_idx]:
                        self.robot_tracking[robot_idx][target_idx] += 1
                if any_tracking:
                    self.cumulative_tracking[target_idx] += 1
                if num_robots_tracking > 1:
                    self.overlap_counts[target_idx] += 1

        self.current_step += 1

    def get_trail_summary(self) -> Dict:
        summary = {
            "num_targets": self.num_landmarks,
            "num_robots": self.num_robots,
            "total_steps": self.current_step,
            "per_target": [],
            "totals": {},
        }

        total_robot_tracking = [0] * self.num_robots
        total_cumulative = 0
        total_overlap = 0
        total_possible_steps = 0

        for target_idx in range(self.num_landmarks):
            first_step = self.first_tracked_step[target_idx]
            steps_possible = self.steps_since_first_tracked[target_idx]

            target_data = {
                "target_idx": target_idx,
                "first_tracked_step": first_step,
                "steps_possible": steps_possible,
                "per_robot": [],
            }

            for robot_idx in range(self.num_robots):
                steps_tracked = self.robot_tracking[robot_idx][target_idx]
                pct = (steps_tracked / steps_possible * 100) if steps_possible > 0 else 0.0
                target_data["per_robot"].append(
                    {
                        "robot_idx": robot_idx,
                        "steps_tracked": steps_tracked,
                        "percentage": round(pct, 2),
                    }
                )
                total_robot_tracking[robot_idx] += steps_tracked

            cumulative_steps = self.cumulative_tracking[target_idx]
            cumulative_pct = (cumulative_steps / steps_possible * 100) if steps_possible > 0 else 0.0
            target_data["cumulative_steps"] = cumulative_steps
            target_data["cumulative_percentage"] = round(cumulative_pct, 2)

            overlap_steps = self.overlap_counts[target_idx]
            overlap_pct = (overlap_steps / steps_possible * 100) if steps_possible > 0 else 0.0
            target_data["overlap_steps"] = overlap_steps
            target_data["overlap_percentage"] = round(overlap_pct, 2)

            summary["per_target"].append(target_data)
            total_cumulative += cumulative_steps
            total_overlap += overlap_steps
            total_possible_steps += steps_possible

        summary["totals"] = {"total_possible_steps": total_possible_steps, "per_robot": []}

        for robot_idx in range(self.num_robots):
            pct = (
                (total_robot_tracking[robot_idx] / total_possible_steps * 100)
                if total_possible_steps > 0
                else 0.0
            )
            summary["totals"]["per_robot"].append(
                {
                    "robot_idx": robot_idx,
                    "total_steps_tracked": total_robot_tracking[robot_idx],
                    "percentage": round(pct, 2),
                }
            )

        cumulative_pct = (total_cumulative / total_possible_steps * 100) if total_possible_steps > 0 else 0.0
        summary["totals"]["cumulative_steps"] = total_cumulative
        summary["totals"]["cumulative_percentage"] = round(cumulative_pct, 2)

        overlap_pct = (total_overlap / total_possible_steps * 100) if total_possible_steps > 0 else 0.0
        summary["totals"]["overlap_steps"] = total_overlap
        summary["totals"]["overlap_percentage"] = round(overlap_pct, 2)

        return summary


def aggregate_by_target_count(all_summaries: List[Dict]) -> Dict[int, Dict]:
    grouped = defaultdict(list)
    for summary in all_summaries:
        grouped[summary["num_targets"]].append(summary)

    aggregated = {}
    for num_targets, summaries in sorted(grouped.items()):
        num_trails = len(summaries)
        num_robots = summaries[0]["num_robots"] if summaries else 0

        per_robot_pcts = [[] for _ in range(num_robots)]
        cumulative_pcts = []
        overlap_pcts = []

        for summary in summaries:
            for robot_idx in range(num_robots):
                per_robot_pcts[robot_idx].append(
                    summary["totals"]["per_robot"][robot_idx]["percentage"]
                )
            cumulative_pcts.append(summary["totals"]["cumulative_percentage"])
            overlap_pcts.append(summary["totals"]["overlap_percentage"])

        aggregated[num_targets] = {
            "num_trails": num_trails,
            "per_robot_avg_pct": [round(np.mean(pcts), 2) for pcts in per_robot_pcts],
            "per_robot_std_pct": [round(np.std(pcts), 2) for pcts in per_robot_pcts],
            "cumulative_avg_pct": round(np.mean(cumulative_pcts), 2),
            "cumulative_std_pct": round(np.std(cumulative_pcts), 2),
            "overlap_avg_pct": round(np.mean(overlap_pcts), 2),
            "overlap_std_pct": round(np.std(overlap_pcts), 2),
        }

    return aggregated
