#!/usr/bin/env python3
"""Boom_Birds 集成断言：读 EGO 的**实际输出**验证地图与轨迹（v2，收紧判据）。

相对 v1 的修正（对应审查意见）：
- 检查**本次运行收到的每一条** B 样条，而不是只查最后一条；任一条不满足即整相位失败。
- 碰撞检查对障碍 AABB 做完整欧氏距离（不再排除"障碍面之前"的点），
  因此前侧膨胀区内的轨迹点会被判定为碰撞。
- 速度/加速度上界按配置值收紧（默认 1.05 倍，仅覆盖数值微分误差），不再是 1.35/2.0 倍。
- nopath 相位不再直接 ok=True：它报告实际行为，并要求"没有新轨迹且指令为零速悬停"
  才算通过；否则标注为未通过（本轮无法构造真正的封闭场景，故如实记录）。

用法（先启动脱机链路与 EGO，再运行本脚本）：
    python3 ego_map_traj_checks.py --phase static|motion|invalid|goal|nopath \
        --out /tmp/bb/run/checks_<phase>.json
"""

from __future__ import annotations

import argparse
import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from traj_utils.msg import Bspline


OCC_TOPIC = "/boom_birds/ego/grid_map/occupancy"
OCC_INF_TOPIC = "/boom_birds/ego/grid_map/occupancy_inflate"
BSPLINE_TOPIC = "/boom_birds/ego/planning/bspline"
CMD_TOPIC = "/boom_birds/ego/position_cmd"


def de_boor(knots: np.ndarray, ctrl: np.ndarray, degree: int, t: float) -> np.ndarray:
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


def sample_bspline(msg: Bspline, samples: int = 400):
    knots = np.asarray(msg.knots, dtype=float)
    ctrl = np.array([[p.x, p.y, p.z] for p in msg.pos_pts], dtype=float)
    if ctrl.shape[0] < msg.order + 1 or knots.size < 2 * (msg.order + 1):
        return None, None
    t0, t1 = knots[msg.order], knots[-msg.order - 1]
    if not np.isfinite(t0) or not np.isfinite(t1) or t1 <= t0:
        return None, None
    ts = np.linspace(t0, t1, samples)
    pts = np.array([de_boor(knots, ctrl, msg.order, float(t)) for t in ts])
    return ts, pts


def vel_acc_limits(ts, pts):
    dt = ts[1] - ts[0]
    d1 = np.gradient(pts, dt, axis=0)
    d2 = np.gradient(d1, dt, axis=0)
    return float(np.linalg.norm(d1, axis=1).max()), float(np.linalg.norm(d2, axis=1).max())


def point_aabb_distance(pts: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """点到轴对齐盒的欧氏距离（盒内为 0）。不做任何"面之前/之后"的排除。"""
    below = np.maximum(lo - pts, 0.0)
    above = np.maximum(pts - hi, 0.0)
    return np.linalg.norm(below + above, axis=1)


def cloud_to_np(msg: PointCloud2) -> np.ndarray:
    pts = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if not pts:
        return np.zeros((0, 3))
    arr = np.array(pts, dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")])
    return np.stack([arr["x"], arr["y"], arr["z"]], axis=1)


def occupied_voxels(msg: PointCloud2, res: float = 0.1) -> set:
    pts = cloud_to_np(msg)
    if pts.shape[0] == 0:
        return set()
    idx = np.rint(pts / res).astype(np.int64)
    return {tuple(int(v) for v in row) for row in idx}


class Checks(Node):
    def __init__(self, args):
        super().__init__("bb_integration_checks")
        self.args = args
        self.occ = []
        self.occ_inf = []
        self.bsplines = []
        self.cmds = []
        self.create_subscription(PointCloud2, OCC_TOPIC, lambda m: self.occ.append((self.now(), m)), 10)
        self.create_subscription(PointCloud2, OCC_INF_TOPIC, lambda m: self.occ_inf.append((self.now(), m)), 10)
        self.create_subscription(Bspline, BSPLINE_TOPIC, self.on_bspline, 10)
        self.create_subscription(PositionCommand, CMD_TOPIC, lambda m: self.cmds.append((self.now(), m)), 10)
        self.pub_depth = self.create_publisher(Image, "/boom_birds/depth/image", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/boom_birds/vio/camera_pose", 10)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_bspline(self, msg: Bspline):
        self.bsplines.append((self.now(), msg))

    def publish_depth(self, depth_m: np.ndarray, frame="cam0_rect"):
        m = Image()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.height, m.width = depth_m.shape
        m.encoding = "32FC1"
        m.is_bigendian = 0
        m.step = m.width * 4
        m.data = np.asarray(depth_m, dtype=np.float32).tobytes()
        self.pub_depth.publish(m)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase", required=True, choices=["static", "motion", "invalid", "goal", "nopath"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--goal", default="3.0,0.0,1.2")
    ap.add_argument("--obstacle-x", type=float, default=1.8)
    ap.add_argument("--obstacle-y", type=float, default=0.0)
    ap.add_argument("--obstacle-z", type=float, default=1.5)
    ap.add_argument("--obstacle-half-y", type=float, default=0.6)
    ap.add_argument("--obstacle-half-z", type=float, default=0.4)
    ap.add_argument("--inflation", type=float, default=0.15)
    ap.add_argument("--max-vel", type=float, default=1.0)
    ap.add_argument("--max-acc", type=float, default=1.0)
    ap.add_argument("--limit-tolerance", type=float, default=1.05, help="仅覆盖数值微分误差")
    args = ap.parse_args()

    rclpy.init()
    node = Checks(args)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    report = {"phase": args.phase, "notes": []}
    lo = np.array([args.obstacle_x - 0.05, args.obstacle_y - args.obstacle_half_y, args.obstacle_z - args.obstacle_half_z])
    hi = np.array([args.obstacle_x + 0.05, args.obstacle_y + args.obstacle_half_y, args.obstacle_z + args.obstacle_half_z])

    def spin(seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            ex.spin_once(timeout_sec=0.05)

    spin(3.0)

    if args.phase == "invalid":
        samples = [m for (_, m) in node.occ if m.width > 0]
        before = occupied_voxels(samples[-1]) if samples else set()
        report["voxels_before"] = len(before)
        invalid = np.full((240, 320), np.nan, dtype=np.float32)
        for _ in range(60):
            node.publish_depth(invalid)
            spin(0.05)
        spin(3.0)
        samples2 = [m for (_, m) in node.occ if m.width > 0]
        after = occupied_voxels(samples2[-1]) if samples2 else set()
        report["voxels_after"] = len(after)
        report["cleared"] = len(before - after)
        report["camera_origin_occupied"] = any(
            abs(v[0]) <= 1 and abs(v[1]) <= 1 and abs(v[2] - 15) <= 1 for v in after
        )
        if not before:
            report["notes"].append("注入前未采到占据样本，本相位无效（需先让地图建立）")
            ok = False
        else:
            ok = report["cleared"] == 0 and not report["camera_origin_occupied"]

    elif args.phase in ("static", "motion"):
        samples = [m for (_, m) in node.occ if m.width > 0]
        nonempty = [occupied_voxels(m) for m in samples if occupied_voxels(m)]
        report["occupancy_samples"] = len(samples)
        report["voxel_counts_nonempty"] = [len(v) for v in nonempty[:5]]
        if not nonempty:
            report["notes"].append("未收到非空占据样本")
            ok = False
        else:
            last = nonempty[-1]
            pts = np.array([[v[0] * 0.1, v[1] * 0.1, v[2] * 0.1] for v in last])
            near = pts[np.abs(pts[:, 0] - args.obstacle_x) < 0.35]
            report["obstacle_like_voxels"] = int(near.shape[0])
            report["obstacle_x_median"] = float(np.median(near[:, 0])) if near.shape[0] else None
            if args.phase == "motion" and len(nonempty) >= 2:
                a, b = nonempty[0], nonempty[-1]
                a_obs = {v for v in a if abs(v[0] * 0.1 - args.obstacle_x) < 0.35}
                b_obs = {v for v in b if abs(v[0] * 0.1 - args.obstacle_x) < 0.35}
                report["obstacle_voxels_first"] = len(a_obs)
                report["obstacle_voxels_last"] = len(b_obs)
                report["obstacle_lost"] = len(a_obs - b_obs)
                report["all_symmetric_difference"] = len(a ^ b)
                ok = len(a_obs) > 0 and len(a_obs - b_obs) == 0
            else:
                ok = report["obstacle_like_voxels"] > 0

    elif args.phase == "goal":
        gx, gy, gz = (float(v) for v in args.goal.split(","))
        spin(args.duration)
        report["bspline_count"] = len(node.bsplines)
        report["position_cmd_count"] = len(node.cmds)
        if not node.bsplines:
            report["notes"].append("未收到 planning/bspline")
            ok = False
        else:
            traj_reports = []
            ok = True
            for idx, (_, msg) in enumerate(node.bsplines):
                ts, pts = sample_bspline(msg)
                entry = {"index": idx, "traj_id": int(msg.traj_id)}
                if pts is None:
                    entry["error"] = "控制点/knots 不足或时间区间非法"
                    traj_reports.append(entry)
                    ok = False
                    continue
                entry["samples"] = int(pts.shape[0])
                entry["finite"] = bool(np.isfinite(pts).all())
                vmax, amax = vel_acc_limits(ts, pts)
                entry["max_vel"] = vmax
                entry["max_acc"] = amax
                dist = point_aabb_distance(pts, lo, hi)
                entry["min_dist_to_obstacle_box"] = float(dist.min())
                entry["min_dist_within_inflation"] = bool((dist < args.inflation).any())
                if dist.min() < args.inflation:
                    k = int(np.argmin(dist))
                    entry["closest_point"] = [float(v) for v in pts[k]]
                    entry["closest_ts"] = float(ts[k])
                    entry["inside_box"] = bool(np.all(pts[k] >= lo) and np.all(pts[k] <= hi))
                    entry["obstacle_box"] = {"lo": lo.tolist(), "hi": hi.tolist()}
                entry["end_to_goal_m"] = float(np.linalg.norm(pts[-1] - np.array([gx, gy, gz])))
                entry["start_to_goal_m"] = float(np.linalg.norm(pts[0] - np.array([gx, gy, gz])))
                entry["ok"] = bool(
                    entry["finite"]
                    and vmax <= args.max_vel * args.limit_tolerance
                    and amax <= args.max_acc * args.limit_tolerance
                    and not entry["min_dist_within_inflation"]
                )
                ok = ok and entry["ok"]
                traj_reports.append(entry)
            report["trajectories"] = traj_reports
            report["worst_max_vel"] = max(t.get("max_vel", 0.0) for t in traj_reports)
            report["worst_max_acc"] = max(t.get("max_acc", 0.0) for t in traj_reports)
            report["worst_min_dist"] = min(t.get("min_dist_to_obstacle_box", 0.0) for t in traj_reports)
            report["notes"].append(
                "逐条检查全部轨迹；局部轨迹终点不要求等于全局目标（end_to_goal 仅记录）"
            )

    else:  # nopath
        spin(args.duration)
        report["bspline_count"] = len(node.bsplines)
        report["position_cmd_count"] = len(node.cmds)
        if node.cmds:
            last = node.cmds[-1][1]
            report["last_cmd"] = {
                "trajectory_flag": int(last.trajectory_flag),
                "position": [last.position.x, last.position.y, last.position.z],
                "velocity": [last.velocity.x, last.velocity.y, last.velocity.z],
            }
        report["notes"].append(
            "本轮未构造真正的封闭场景：判据为【没有新轨迹 且 指令为零速悬停】才通过；"
            "若仍持续产生新轨迹或非零速度指令，则如实标为未通过"
        )
        v_zero = False
        if node.cmds:
            v = node.cmds[-1][1].velocity
            v_zero = abs(v.x) < 1e-6 and abs(v.y) < 1e-6 and abs(v.z) < 1e-6
        ok = report["bspline_count"] == 0 and v_zero

    report["ok"] = bool(ok)
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(report, fh, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
