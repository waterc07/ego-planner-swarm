#!/usr/bin/env python3
"""WSL synthetic motion-reset regression; owns only the launches it starts."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from traj_utils.msg import Bspline
from quadrotor_msgs.msg import PositionCommand
from std_srvs.srv import Trigger


class Watch(Node):
    def __init__(self):
        super().__init__("bb_reset_review")
        self.maps = []
        self.trajectories = []
        self.commands = []
        self.create_subscription(PointCloud2, "/boom_birds/ego/grid_map/occupancy",
                                 self.on_map, 10)
        self.create_subscription(Bspline, "/boom_birds/ego/planning/bspline",
                                 self.on_trajectory, 100)
        self.create_subscription(PositionCommand, "/boom_birds/ego/position_cmd",
                                 self.on_command, 100)

    def on_map(self, msg):
        pts = list(point_cloud2.read_points(msg, field_names=("x","y","z"),
                                            skip_nans=True))
        voxels = {tuple(int(round(float(c)/0.1)) for c in p) for p in pts}
        self.maps.append((time.monotonic(), voxels))

    def on_trajectory(self, msg):
        if msg.order == 3 and len(msg.pos_pts) >= 4:
            self.trajectories.append(time.monotonic())

    def on_command(self, msg):
        v = msg.velocity
        self.commands.append((time.monotonic(),
                              abs(v.x) + abs(v.y) + abs(v.z)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    args = ap.parse_args()
    out = Path(args.out_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    calib = out/"synthetic.npz"
    subprocess.run([sys.executable, "-m", "boom_birds_nav.synthetic",
                    "--write-calibration", str(calib)], check=True)
    os.environ.setdefault("ROS_DOMAIN_ID", "182")
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init()
    watcher = Watch()
    children = []
    logs = []
    result = {}
    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(watcher, timeout_sec=0.05)
    def start(name, command):
        f = (out/(name+".log")).open("w")
        logs.append(f)
        p = subprocess.Popen(command, stdout=f, stderr=subprocess.STDOUT,
                             start_new_session=True)
        children.append(p)
        return p
    def stop(p):
        if p.poll() is not None:
            return
        try: os.killpg(p.pid, signal.SIGTERM)
        except ProcessLookupError: return
        try: p.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait(timeout=4)
    def source(name, offset):
        return start(name, ["ros2","launch","boom_birds_nav","synthetic_layer2.launch.py",
            "synth_calibration:="+str(calib), "pose_timeout_s:=0.5",
            "origin_offset_m:="+str(offset)])
    def planner(name, goal_x):
        return start(name, ["ros2","launch","ego_planner","boom_birds_offline.launch.py",
            "goal_x:="+str(goal_x), "goal_y:=1.2"])
    def wait_for(predicate, timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if predicate(): return True
            rclpy.spin_once(watcher, timeout_sec=0.05)
        return bool(predicate())
    try:
        src1=source("old_source",0)
        ego1=planner("old_planner",2.5)
        result["moving_before_reset"]=wait_for(
            lambda: bool(watcher.trajectories and watcher.maps and
                         any(v>0.01 for _,v in watcher.commands)),45)
        assert result["moving_before_reset"], "old-frame trajectory never executed"
        spin(2)
        old_map=watcher.maps[-1][1]
        assert old_map, "old map empty"
        result["old_occupied_voxels"]=len(old_map)
        client=watcher.create_client(Trigger,"/boom_birds_pose_adapter/reset")
        assert client.wait_for_service(timeout_sec=5), "reset service missing"
        future=client.call_async(Trigger.Request())
        assert wait_for(lambda: future.done(),5) and future.result().success
        result["adapter_reset"]=True
        stop(ego1)
        stop(src1)
        spin(0.5)  # drain any already-queued messages
        old_commands=len(watcher.commands)
        old_trajectories=len(watcher.trajectories)
        old_maps=len(watcher.maps)
        spin(5)
        result["commands_after_stop"]=len(watcher.commands)-old_commands
        result["trajectories_after_stop"]=len(watcher.trajectories)-old_trajectories
        result["maps_after_stop"]=len(watcher.maps)-old_maps
        assert not any((result[k] for k in ("commands_after_stop",
                    "trajectories_after_stop","maps_after_stop"))), "old frame still active"
        src2=source("new_source",3.0)
        ego2=planner("new_planner",5.5)
        stamp=time.monotonic()
        result["new_frame_ready"]=wait_for(
            lambda: any(t>stamp and vox for t,vox in watcher.maps) and
                    any(t>stamp for t in watcher.trajectories) and
                    any(t>stamp and v>0.01 for t,v in watcher.commands),45)
        assert result["new_frame_ready"], "new frame did not map and plan"
        new_map=next(vox for t,vox in reversed(watcher.maps) if t>stamp and vox)
        result["new_occupied_voxels"]=len(new_map)
        result["old_new_voxel_overlap"]=len(old_map & new_map)
        result["old_map_min_x_m"]=min(v[0] for v in old_map)*0.1
        result["new_map_min_x_m"]=min(v[0] for v in new_map)*0.1
        result["min_x_shift_m"]=result["new_map_min_x_m"]-result["old_map_min_x_m"]
        assert result["old_new_voxel_overlap"]==0, "old frame map reused"
        assert abs(result["min_x_shift_m"]-3.0)<0.3, "depth/pose reset geometry inconsistent"
        result["ok"]=True
    except Exception as error:
        result["ok"]=False
        result["error"]=str(error)
    finally:
        for p in reversed(children): stop(p)
        for f in logs: f.close()
        watcher.destroy_node()
        rclpy.shutdown()
        (out/"result.json").write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps(result,indent=2))
    return 0 if result["ok"] else 1


if __name__=="__main__":
    raise SystemExit(main())
