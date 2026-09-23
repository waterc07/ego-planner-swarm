#!/usr/bin/env python3
"""Offline regression: rejected/malformed trajectories invalidate old commands.
Run under an isolated ROS_DOMAIN_ID with the reviewed install sourced.
Only owns and terminates the traj_server process it starts.
"""
import json
import os
import signal
import subprocess
import time
import argparse
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from traj_utils.msg import Bspline
from quadrotor_msgs.msg import PositionCommand

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--rejection-duration", type=float, default=60.0)
    args=ap.parse_args()
    out=Path(args.out); out.parent.mkdir(parents=True,exist_ok=True)
    log=open(out.with_suffix(".log"),"w")
    proc=subprocess.Popen(["ros2","run","ego_planner","traj_server","--ros-args",
        "-r","planning/bspline:=/review/bspline","-r","/position_cmd:=/review/cmd"],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init()
    node=Node("review_executor_test")
    commands=[]
    node.create_subscription(PositionCommand,"/review/cmd",lambda m:commands.append(m),10)
    pub=node.create_publisher(Bspline,"/review/bspline",10)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=0.02)
    def valid():
        m=Bspline();m.order=3;m.traj_id=1;m.start_time=node.get_clock().now().to_msg()
        m.pos_pts=[Point(x=i*0.3,y=0.,z=1.) for i in range(6)]
        m.knots=[float(i-3) for i in range(10)]
        return m
    result={}
    try:
        spin(2)
        assert pub.get_subscription_count()>0,"executor did not subscribe"
        pub.publish(valid());spin(0.8)
        result["moving_before_rejection"]=any(abs(m.velocity.x)>0.01 for m in commands)
        assert result["moving_before_rejection"]
        # Sustained rejection, not just one packet; allow transport drain first.
        pub.publish(Bspline());spin(0.3)
        before=len(commands)
        result["rejection_duration_s"]=args.rejection_duration
        for _ in range(max(1, int(args.rejection_duration * 10))):
            pub.publish(Bspline());spin(0.1)
        result["commands_during_rejection"]=len(commands)-before
        assert len(commands)==before,"old trajectory still executing"
        pub.publish(valid());spin(0.6)
        result["resumed_on_valid"]=len(commands)>before
        assert result["resumed_on_valid"]
        malformed=valid();malformed.knots[4]=float("nan")
        pub.publish(malformed);spin(0.3);before=len(commands);spin(0.6)
        result["malformed_stops_commands"]=len(commands)==before
        assert result["malformed_stops_commands"]
        result["ok"]=True
    except Exception as exc:
        result["ok"]=False;result["error"]=str(exc)
    finally:
        node.destroy_node();rclpy.shutdown()
        os.killpg(proc.pid,signal.SIGTERM)
        try: proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        log.close()
        out.write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps(result,indent=2))
    return 0 if result["ok"] else 1

if __name__=="__main__":raise SystemExit(main())
