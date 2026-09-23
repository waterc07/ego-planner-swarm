#!/usr/bin/env python3
"""轨迹约束分析：解析导数 vs 数值微分（第 3 项）。

执行端 traj_server 的约定（源码 traj_server.cpp:63-64 + uniform_bspline.cpp:99-123）：
  vel(t) = BSpline(order-1, 控制点 Qi, 结点 u[1:-1]).evaluateDeBoorT(t)
  Qi = p * (P[i+1] - P[i]) / (u[i+p+1] - u[i+1])
因此"真实执行的速度"是**解析导数**，不是有限差分。本脚本两者都算并对比。

输出：
  A. 每条已发布轨迹的解析导数的采样最大值 + 控制点凸包上界（覆盖完整时间区间）；
  B. 同一轨迹的数值微分（400 点）结果，用于量化此前测量偏高多少；
  C. 与障碍盒的最小距离（完整 AABB，步长足以覆盖采样间隔）；
  D. 轨迹的有效执行区间与时间参数（start_time、knots、duration）。
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from traj_utils.msg import Bspline
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2


def de_boor(knots, ctrl, degree, t):
    n = ctrl.shape[0]
    if n <= degree:
        return ctrl[-1]
    k = int(np.searchsorted(knots, t, side="right") - 1)
    k = max(degree, min(k, n - 1))
    d = [ctrl[j].copy() for j in range(k - degree, k + 1)]
    for r in range(1, degree + 1):
        for j in range(degree, r - 1, -1):
            i = k - degree + j
            denom = knots[i + degree - r + 1] - knots[i]
            alpha = 0.0 if abs(denom) < 1e-12 else (t - knots[i]) / denom
            d[j] = (1 - alpha) * d[j - 1] + alpha * d[j]
    return d[degree]


def derivative_curve(knots, ctrl, p):
    """按执行端公式构造导数曲线：(控制点, 结点, order)。"""
    n = ctrl.shape[0] - 1
    new_ctrl = np.zeros((n, 3))
    for i in range(n):
        denom = knots[i + p + 1] - knots[i + 1]
        new_ctrl[i] = p * (ctrl[i + 1] - ctrl[i]) / (denom if abs(denom) > 1e-12 else 1e-12)
    return new_ctrl, knots[1:-1], p - 1


def eval_curve(knots, ctrl, p, samples=2000):
    t0, t1 = knots[p], knots[-p - 1]
    ts = np.linspace(t0, t1, samples)
    pts = np.array([de_boor(knots, ctrl, p, float(t)) for t in ts])
    return ts, pts


def analyze(msg: Bspline, obstacle_lo, obstacle_hi, inflation):
    knots = np.asarray(msg.knots, dtype=float)
    ctrl = np.array([[q.x, q.y, q.z] for q in msg.pos_pts], dtype=float)
    p = int(msg.order)
    out = {"traj_id": int(msg.traj_id), "order": p, "ctrl_pts": int(ctrl.shape[0]),
           "knots": int(knots.size), "start_time": float(msg.start_time.sec) + msg.start_time.nanosec * 1e-9}
    if ctrl.shape[0] < p + 1 or knots.size < 2 * (p + 1):
        out["error"] = "控制点/结点不足"
        return out
    ts, pts = eval_curve(knots, ctrl, p)
    out["endpoint"] = pts[-1].tolist()
    out["duration_s"] = float(knots[-p - 1] - knots[p])

    # A. 解析导数（与执行端一致）
    v_ctrl, v_knots, v_p = derivative_curve(knots, ctrl, p)
    tsv, vs = eval_curve(v_knots, v_ctrl, v_p)
    vmag = np.linalg.norm(vs, axis=1)
    a_ctrl, a_knots, a_p = derivative_curve(v_knots, v_ctrl, v_p)
    tsa, acc = eval_curve(a_knots, a_ctrl, a_p)
    amag = np.linalg.norm(acc, axis=1)
    out["velocity_bound"] = float(np.linalg.norm(v_ctrl,axis=1).max())
    out["acceleration_bound"] = float(np.linalg.norm(a_ctrl,axis=1).max())
    out["analytic_max_vel"] = float(vmag.max())
    out["analytic_max_acc"] = float(amag.max())

    # B. 数值微分（用于对比）
    dt = ts[1] - ts[0]
    d1 = np.gradient(pts, dt, axis=0)
    d2 = np.gradient(d1, dt, axis=0)
    out["fd_max_vel_2000"] = float(np.linalg.norm(d1, axis=1).max())
    out["fd_max_acc_2000"] = float(np.linalg.norm(d2, axis=1).max())
    ts400 = np.linspace(ts[0], ts[-1], 400)
    p400 = np.array([de_boor(knots, ctrl, p, float(t)) for t in ts400])
    dt4 = ts400[1] - ts400[0]
    out["fd_max_vel_400"] = float(np.linalg.norm(np.gradient(p400, dt4, axis=0), axis=1).max())
    d2_400 = np.gradient(np.gradient(p400, dt4, axis=0), dt4, axis=0)
    out["fd_max_acc_400"] = float(np.linalg.norm(d2_400, axis=1).max())

    # C. 障碍距离（完整 AABB）
    below = np.maximum(obstacle_lo - pts, 0.0)
    above = np.maximum(pts - obstacle_hi, 0.0)
    dist = np.linalg.norm(below + above, axis=1)
    out["min_dist_to_obstacle_box"] = float(dist.min())
    # Distance-to-AABB is 1-Lipschitz. Bound unsampled clearance by
    # subtracting the maximum travel from nearest sample over half a timestep.
    out["clearance_lower_bound"] = float(dist.min() - out["velocity_bound"] * dt / 2)
    # Synthetic background wall is a plane at x=3 m, not empty space.
    out["wall_clearance_lower_bound"] = float(3.0 - pts[:, 0].max() -
                                               out["velocity_bound"] * dt / 2)
    out["min_dist_within_inflation"] = min(out["clearance_lower_bound"],
                                          out["wall_clearance_lower_bound"]) < inflation
    out["finite"] = bool(np.isfinite(pts).all() and np.isfinite(vs).all()
                         and np.isfinite(acc).all())
    out["samples"] = int(pts.shape[0])
    return out


class Collector(Node):
    def __init__(self):
        super().__init__("bb_traj_analysis")
        self.msgs = []
        self.commands = []
        self.events = []
        self.occupancy_counts = []
        self.create_subscription(PointCloud2, "/boom_birds/ego/grid_map/occupancy",
                                 lambda m:self.occupancy_counts.append(m.width*m.height), 10)
        self.create_subscription(PositionCommand, "/boom_birds/ego/position_cmd", self._command, 100)
        self.create_subscription(Bspline, "/boom_birds/ego/planning/bspline", self._cb, 100)

    def _command(self, m):
        self.commands.append(m)
        self.events.append((time.monotonic(), "command"))

    def _cb(self, m):
        self.msgs.append(m)
        self.events.append((time.monotonic(), "invalidate" if not m.pos_pts and m.order == 0 else "accept"))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--ready-file")
    ap.add_argument("--expect-gated", action="store_true")
    ap.add_argument("--expect-rejected", action="store_true")
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--goal-y", type=float, default=1.2)
    ap.add_argument("--obstacle-x", type=float, default=1.8)
    ap.add_argument("--obstacle-y", type=float, default=0.0)
    ap.add_argument("--obstacle-z", type=float, default=1.5)
    ap.add_argument("--obstacle-half-y", type=float, default=0.6)
    ap.add_argument("--obstacle-half-z", type=float, default=0.4)
    ap.add_argument("--inflation", type=float, default=0.15)
    ap.add_argument("--max-vel", type=float, default=1.0)
    ap.add_argument("--max-acc", type=float, default=1.0)
    args = ap.parse_args()

    rclpy.init()
    node = Collector()
    if args.ready_file:
        Path(args.ready_file).write_text("subscriptions created\n")
    end = time.monotonic() + args.duration
    while time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)

    lo = np.array([args.obstacle_x - 0.05, args.obstacle_y - args.obstacle_half_y, args.obstacle_z - args.obstacle_half_z])
    hi = np.array([args.obstacle_x + 0.05, args.obstacle_y + args.obstacle_half_y, args.obstacle_z + args.obstacle_half_z])
    invalidations = sum(not m.pos_pts and m.order == 0 for m in node.msgs)
    report = {"bspline_count": len(node.msgs), "invalidations": invalidations,
              "measurement": "analytic derivatives sampled; hull bounds certify full interval",
              "trajectories": []}
    for m in node.msgs:
        if not m.pos_pts and m.order == 0: continue
        report["trajectories"].append(analyze(m, lo, hi, args.inflation))

    trajs = report["trajectories"]
    if trajs:
        report["worst_analytic_vel"] = max(t.get("analytic_max_vel", 0.0) for t in trajs)
        report["worst_analytic_acc"] = max(t.get("analytic_max_acc", 0.0) for t in trajs)
        report["worst_fd_vel_400"] = max(t.get("fd_max_vel_400", 0.0) for t in trajs)
        report["worst_fd_acc_400"] = max(t.get("fd_max_acc_400", 0.0) for t in trajs)
        report["worst_min_dist"] = min(t.get("min_dist_to_obstacle_box", 1e9) for t in trajs)
        report["limits"] = {"max_vel": args.max_vel, "max_acc": args.max_acc, "inflation": args.inflation}
        report["analytic_violations"] = {
            "vel": sum(1 for t in trajs if t.get("velocity_bound", float("inf")) > args.max_vel * 1.05 + 1e-9),
            "acc": sum(1 for t in trajs if t.get("acceleration_bound", float("inf")) > args.max_acc * 1.05 + 1e-9),
            "collision": sum(1 for t in trajs if t.get("min_dist_within_inflation")),
        }
    report["ok"] = bool(
        trajs
        and all(t.get("finite", False) and "error" not in t for t in trajs)
        and report.get("analytic_violations", {}).get("vel", 1) == 0
        and report.get("analytic_violations", {}).get("acc", 1) == 0
        and report["analytic_violations"].get("collision", 1) == 0
    )
    report["position_commands"] = len(node.commands)
    report["max_occupied_voxels"] = max(node.occupancy_counts, default=0)
    moving = [t for t in trajs if t.get("analytic_max_vel", 0) > 0.05]
    report["moving_trajectories"] = len(moving)
    report["minimum_goal_error_m"] = min(
        (float(np.linalg.norm(np.array(t["endpoint"]) - [2.5, args.goal_y, 1.2])) for t in moving),
        default=None)
    report["ok"] = bool(report["ok"] and moving and
                        report["minimum_goal_error_m"] < 0.3 and report["max_occupied_voxels"] > 0)
    invalid_since = None
    stale_commands = 0
    rejection_windows = []
    for stamp, event in node.events:
        if event == "invalidate":
            if invalid_since is None:
                invalid_since = stamp
        elif event == "accept":
            if invalid_since is not None:
                rejection_windows.append(stamp - invalid_since)
            invalid_since = None
        elif invalid_since is not None and stamp - invalid_since > 0.2:
            stale_commands += 1
    if invalid_since is not None:
        rejection_windows.append(end - invalid_since)
    report["commands_after_invalidation_grace_0p2s"] = stale_commands
    report["rejection_windows_s"] = rejection_windows
    report["observation_duration_s"] = args.duration
    report["ok"] = report["ok"] and stale_commands == 0
    if args.expect_gated or args.expect_rejected:
        report["ok"] = bool(invalidations > 0 and not trajs and not node.commands and
                            report["max_occupied_voxels"] > 0)
        report["scope"] = "readiness gate closed" if args.expect_gated else "occupied goal rejected"
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(report, fh, ensure_ascii=False, indent=2)
    print(json.dumps({k: v for k, v in report.items() if k != "trajectories"}, ensure_ascii=False, indent=2))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
