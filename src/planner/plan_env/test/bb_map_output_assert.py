#!/usr/bin/env python3
"""Boom_Birds EGO grid_map 输出断言脚本（task-2 F，TEST-ONLY，不连任何真实设备）。

用途：对**真实运行的** ego_planner_node（boom_birds_offline.launch.py）发布的
      grid_map/occupancy 与 grid_map/occupancy_inflate 做解析比对，断言：

  A 障碍体素集合与解析预期一致（合成深度按相机内参逐像素渲染，预期体素由同一几何解析给出）
  B 相机沿世界 y 往复移动 10 帧后，静态障碍的占据体素集合对称差 <= 1 voxel
  C 全无效深度帧不改变占据集合（无效观测不改变占据）
  D 相机原点附近无虚假占据
  E 占据点云是膨胀点云的子集（inflate 覆盖 occ，且严格更大）

用法（先启动 launch，再本脚本；两者可分开两个终端）：
    source companion/ros2_ws/tools/activate_python_env.sh
    ros2 launch ego_planner boom_birds_offline.launch.py
    python3 src/planner/plan_env/test/bb_map_output_assert.py --out-json /tmp/bb/ego/map_assert.json

合成场景（与 boom_birds_nav/config/synthetic_scene.yaml 同源，按本脚本需要做了两点明确调整）：
  * 障碍盒中心 (1.85, 0.0, 1.5)，半长 (0.1, 0.65, 0.4)：面/边落在体素中心，避免 0.1m 网格边界抖动
  * 远墙 x=5.5 m：大于 max_ray_length(4.5) 与 depth_filter_maxdist(5.0)，因此在建图中恒为无效
    （不会被清图/占据），使"静态障碍占集合"与相机移动解耦
  * 相机 y 往复幅度默认 0.5 m：保证障碍整面始终在 320x240 视场内（否则对称差断言无意义）
"""

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2

# ---- 与 grid_map 参数一致的常量（boom_birds_offline.launch.py）----
FX = FY = 172.5980149526744          # 与 /boom_birds/depth/camera_info 同源
CX = 156.8623504638672
CY = 116.15113067626953
IMG_W, IMG_H = 320, 240
RES = 0.1
MAP_XY, MAP_Z = 20.0, 5.0
GROUND = -0.01
ORIGIN = np.array([-MAP_XY / 2.0, -MAP_XY / 2.0, GROUND])
MAX_RAY = 4.5
MAX_DEPTH = 5.0
SKIP = 2
TRUNC_H = 2.8                        # visualization_truncate_height
CAM_Z = 1.5
Q_CAM2WORLD = np.array([[0.0, 0.0, 1.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0]])
QUAT = (-0.5, 0.5, -0.5, 0.5)        # (x, y, z, w)，与 nav 侧实测相机姿态一致

OBST_LO = np.array([1.75, -0.65, 1.10])
OBST_HI = np.array([1.95, 0.65, 1.90])
WALL_X = 5.5

PASS = 0
FAIL = 0
ART = {}


def check(ok, name, detail=""):
    global PASS, FAIL
    if ok:
        PASS += 1
        print("  [PASS] %s :: %s" % (name, detail), flush=True)
    else:
        FAIL += 1
        print("  [FAIL] %s :: %s" % (name, detail), flush=True)
    ART.setdefault("checks", []).append({"name": name, "pass": bool(ok), "detail": detail})


def voxel_of(points):
    """points: (N,3) world -> 体素 id 集合 (x,y,z) 整数元组"""
    if len(points) == 0:
        return set()
    idx = np.floor((np.asarray(points, dtype=float) - ORIGIN[None, :]) / RES).astype(np.int64)
    return set(map(tuple, idx.tolist()))


def parse_cloud(msg):
    """手工解析 PointCloud2 的 x/y/z（float32），不依赖 sensor_msgs_py"""
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3))
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    step = msg.point_step
    usable = (len(raw) // step) * step
    rec = raw[:usable].reshape(-1, step)
    xyz = rec[:, :12].copy().view(np.float32).reshape(-1, 3).astype(float)
    return xyz


def render_depth(cam_pos):
    """按内参逐像素渲染合成深度（32FC1，米；无回波/超量程 -> 0.0）"""
    u = np.arange(0, IMG_W, SKIP)
    v = np.arange(0, IMG_H, SKIP)
    uu, vv = np.meshgrid(u, v)
    d_cam = np.stack([(uu - CX) / FX, (vv - CY) / FY, np.ones_like(uu, dtype=float)], axis=0)
    D = np.tensordot(Q_CAM2WORLD, d_cam, axes=([1], [0]))  # 单位光轴深度对应的世界方向

    t_lo = np.full(D.shape[1:], -np.inf)
    t_hi = np.full(D.shape[1:], np.inf)
    for i in range(3):
        Di = D[i]
        Oi = cam_pos[i]
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (OBST_LO[i] - Oi) / Di
            t2 = (OBST_HI[i] - Oi) / Di
        lo = np.minimum(t1, t2)
        hi = np.maximum(t1, t2)
        parallel = np.abs(Di) < 1e-12
        inside = (Oi >= OBST_LO[i]) & (Oi <= OBST_HI[i])
        lo = np.where(parallel, np.where(inside, -np.inf, np.inf), lo)
        hi = np.where(parallel, np.where(inside, np.inf, -np.inf), hi)
        t_lo = np.maximum(t_lo, lo)
        t_hi = np.minimum(t_hi, hi)

    hit_box = t_hi >= np.maximum(t_lo, 0.0)
    z_box = np.where(hit_box, np.maximum(t_lo, 0.0), np.inf)

    Dx = D[0]
    with np.errstate(divide="ignore", invalid="ignore"):
        z_wall = (WALL_X - cam_pos[0]) / Dx
    z_wall = np.where((Dx > 1e-9) & (z_wall > 0.0), z_wall, np.inf)

    z = np.minimum(z_box, z_wall)
    z = np.where(np.isfinite(z) & (z <= MAX_DEPTH), z, 0.0)

    img = np.zeros((IMG_H, IMG_W), dtype=np.float32)
    img[vv, uu] = z.astype(np.float32)
    return img, D, z


def expected_occupied(cam_pos):
    """解析预期占据体素：光轴深度 <= MAX_DEPTH 且欧氏距离 <= max_ray_length 的命中点"""
    _, D, z = render_depth(cam_pos)
    valid = (z > 0.0) & (z <= MAX_DEPTH)
    dist = np.linalg.norm(z[None, :, :] * D, axis=0)
    pts = []
    zz = z[valid & (dist <= MAX_RAY)]
    DD = D[:, valid & (dist <= MAX_RAY)]
    if zz.size:
        p = cam_pos[:, None] + zz[None, :] * DD
        pts = p.T
    vx = voxel_of(pts)
    # 与 publishMap 相同的可视化高度过滤（体素中心 z <= TRUNC_H）
    keep = set()
    for (x, y, zz_) in vx:
        cz = ORIGIN[2] + (zz_ + 0.5) * RES
        if cz <= TRUNC_H:
            keep.add((x, y, zz_))
    return keep


class MapAssert(Node):
    def __init__(self, args):
        super().__init__("bb_map_output_assert")
        self.args = args
        qos_pub = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                            history=HistoryPolicy.KEEP_LAST)
        qos_sub = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                            history=HistoryPolicy.KEEP_LAST)
        self.pub_depth = self.create_publisher(Image, args.depth_topic, qos_pub)
        self.pub_pose = self.create_publisher(PoseStamped, args.pose_topic, qos_pub)
        self.occ = None
        self.infl = None
        self.create_subscription(PointCloud2, args.map_topic, self.on_occ, qos_sub)
        self.create_subscription(PointCloud2, args.inflate_topic, self.on_infl, qos_sub)

    def on_occ(self, msg):
        self.occ = voxel_of(parse_cloud(msg))

    def on_infl(self, msg):
        self.infl = voxel_of(parse_cloud(msg))

    def publish_frame(self, depth_img, y_off, settle=0.25):
        img = Image()
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = "global"
        img.height, img.width = depth_img.shape[0], depth_img.shape[1]
        img.encoding = "32FC1"
        img.is_bigendian = 0
        img.step = img.width * 4
        img.data = depth_img.astype(np.float32).tobytes()
        self.pub_depth.publish(img)

        pose = PoseStamped()
        pose.header.stamp = img.header.stamp
        pose.header.frame_id = "global"
        pose.pose.position.x = 0.0
        pose.pose.position.y = float(y_off)
        pose.pose.position.z = CAM_Z
        pose.pose.orientation.x, pose.pose.orientation.y = QUAT[0], QUAT[1]
        pose.pose.orientation.z, pose.pose.orientation.w = QUAT[2], QUAT[3]
        self.pub_pose.publish(pose)
        rclpy.spin_once(self, timeout_sec=settle)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--depth-topic", default="/boom_birds/depth/image")
    ap.add_argument("--pose-topic", default="/boom_birds/vio/camera_pose")
    ap.add_argument("--map-topic", default="/grid_map/occupancy")
    ap.add_argument("--inflate-topic", default="/grid_map/occupancy_inflate")
    ap.add_argument("--settle-frames", type=int, default=12, help="建立障碍所需帧数")
    ap.add_argument("--move-frames", type=int, default=10, help="移动相机的帧数")
    ap.add_argument("--move-amplitude", type=float, default=0.5, help="相机 y 往复幅度（米）")
    ap.add_argument("--missing-tolerance", type=float, default=0.05, help="解析预期缺失比例上限")
    ap.add_argument("--out-json", default="")
    args = ap.parse_args()

    rclpy.init()
    node = MapAssert(args)
    cam0 = np.array([0.0, 0.0, CAM_Z])
    art = {"scene": {"obstacle_lo": OBST_LO.tolist(), "obstacle_hi": OBST_HI.tolist(),
                     "wall_x": WALL_X, "cam_z": CAM_Z, "max_ray_length": MAX_RAY,
                     "skip_pixel": SKIP, "resolution": RES}}

    try:
        # ---------- 建图 ----------
        img, _, _ = render_depth(cam0)
        for i in range(args.settle_frames):
            node.publish_frame(img, 0.0)
        for _ in range(10):  # 等 vis 定时器至少发布一次
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.occ is None or len(node.occ) == 0:
            check(False, "A/占据点云已发布", "grid_map/occupancy 无数据（节点未运行或未收到深度？）")
            raise SystemExit(2)
        occ0 = set(node.occ)
        infl0 = set(node.infl or set())
        exp0 = expected_occupied(cam0)
        art["occ_before"] = sorted(occ0)
        art["expected"] = sorted(exp0)

        # A 解析一致
        近邻 = lambda a, b: any(abs(a[0] - b[0]) <= 1 and abs(a[1] - b[1]) <= 1 and abs(a[2] - b[2]) <= 1
                                for b in b)
        missing = [v for v in exp0 if not 近邻(v, occ0)]
        spurious = [v for v in occ0 if not 近邻(v, exp0)]
        miss_ratio = len(missing) / max(1, len(exp0))
        check(len(exp0) > 0, "A/解析预期非空", "expected=%d voxels" % len(exp0))
        check(miss_ratio <= args.missing_tolerance, "A/障碍体素与解析预期一致",
              "expected=%d observed=%d missing=%d (%.1f%%) spurious=%d"
              % (len(exp0), len(occ0), len(missing), 100.0 * miss_ratio, len(spurious)))
        check(len(spurious) == 0, "A/无解析外虚假占据", "spurious=%d" % len(spurious))

        # E 膨胀覆盖
        check(occ0.issubset(infl0), "E/占据点云是膨胀点云子集",
              "occ=%d inflate=%d" % (len(occ0), len(infl0)))
        check(len(infl0) > len(occ0), "E/膨胀点云严格更大",
              "occ=%d inflate=%d" % (len(occ0), len(infl0)))

        # D 相机原点无虚假占据
        cam_vox = np.floor((cam0 - ORIGIN) / RES).astype(int)
        near = [v for v in occ0
                if abs(v[0] - cam_vox[0]) <= 3 and abs(v[1] - cam_vox[1]) <= 3 and abs(v[2] - cam_vox[2]) <= 3]
        check(len(near) == 0, "D/相机 0.35m 内无虚假占据", "near_cam=%d" % len(near))

        # B 移动相机 10 帧后对称差
        offsets = args.move_amplitude * np.sin(2 * np.pi * np.arange(args.move_frames) / max(1, args.move_frames))
        for y in offsets:
            img_m, _, _ = render_depth(np.array([0.0, float(y), CAM_Z]))
            node.publish_frame(img_m, float(y))
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        occ1 = set(node.occ)
        art["occ_after_move"] = sorted(occ1)
        sym = occ0.symmetric_difference(occ1)
        check(len(sym) <= 1, "B/移动相机 10 帧后静态障碍对称差 <= 1 voxel",
              "before=%d after=%d symdiff=%d %s" % (len(occ0), len(occ1), len(sym), sorted(sym)[:5]))

        # C 无效观测不改变占据
        blank = np.zeros((IMG_H, IMG_W), dtype=np.float32)
        for i in range(4):
            node.publish_frame(blank, 0.0)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        occ2 = set(node.occ)
        art["occ_after_invalid"] = sorted(occ2)
        check(occ2 == occ1, "C/无效观测不改变占据",
              "before=%d after=%d symdiff=%d" % (len(occ1), len(occ2), len(occ1 ^ occ2)))

        # 二次确认 D（移动后相机原点仍无虚假占据）
        cam_vox2 = np.floor((np.array([0.0, float(offsets[-1]), CAM_Z]) - ORIGIN) / RES).astype(int)
        near2 = [v for v in occ2
                 if abs(v[0] - cam_vox2[0]) <= 3 and abs(v[1] - cam_vox2[1]) <= 3 and abs(v[2] - cam_vox2[2]) <= 3]
        check(len(near2) == 0, "D/移动后相机附近无虚假占据", "near_cam=%d" % len(near2))
    finally:
        art["pass"] = PASS
        art["fail"] = FAIL
        if args.out_json:
            with open(args.out_json, "w", encoding="utf-8") as fh:
                json.dump(art, fh, ensure_ascii=False, indent=1)
            print("artifact -> %s" % args.out_json, flush=True)
        print("\nBB_MAP_ASSERT SUMMARY: pass=%d fail=%d" % (PASS, FAIL), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
