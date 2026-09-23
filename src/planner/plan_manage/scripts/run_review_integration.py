#!/usr/bin/env python3
"""Reproducible WSL-only synthetic run. Source persistent install first.
Uses ROS_DOMAIN_ID from environment (default 174); no global pkill.
"""
import argparse,json,os,re,signal,subprocess,sys,time
from pathlib import Path

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--out-dir",required=True)
    ap.add_argument("--duration",type=float,default=40)
    ap.add_argument("--goal-y",type=float,default=1.2)
    ap.add_argument("--map-inflation",type=float,default=0.15)
    ap.add_argument("--expect-gated",action="store_true")
    ap.add_argument("--expect-rejected",action="store_true",help="Goal inside the synthetic background wall")
    args=ap.parse_args()
    out=Path(args.out_dir).resolve();out.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault("ROS_DOMAIN_ID","174")
    os.environ["ROS_LOCALHOST_ONLY"]="1"
    here=Path(__file__).resolve().parent
    calib=out/"synthetic.npz"
    ready=out/"collector.ready"
    if ready.exists(): ready.unlink()
    subprocess.run([sys.executable,"-m","boom_birds_nav.synthetic",
                    "--write-calibration",str(calib)],check=True)
    children=[]; logs=[]
    def start(name,cmd):
        log=open(out/(name+".log"),"w");logs.append(log)
        p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(p);return p
    try:
        check=start("trajectory_check",[sys.executable,str(here/"review_traj_checks.py"),
              "--out",str(out/"trajectories.json"),"--ready-file",str(ready),"--duration",str(args.duration),"--goal-y",str(args.goal_y),"--inflation",str(args.map_inflation)] + (["--expect-gated"] if args.expect_gated else ["--expect-rejected"] if args.expect_rejected else []))
        deadline=time.monotonic()+30
        while not ready.exists():
            if check.poll() is not None or time.monotonic()>deadline:
                raise RuntimeError("collector failed to become ready; see trajectory_check.log")
            time.sleep(0.1)
        start("source",["ros2","launch","boom_birds_nav","synthetic_layer2.launch.py",
                       "pose_timeout_s:=0.5","synth_calibration:="+str(calib)])
        time.sleep(2)
        start("planner",["ros2","launch","ego_planner","boom_birds_offline.launch.py"] + (["ready_min_fusion_updates:=1000000"] if args.expect_gated else ["goal_x:=3.0"] if args.expect_rejected else ["goal_y:="+str(args.goal_y),"obstacles_inflation:="+str(args.map_inflation)]))
        rc=check.wait(timeout=args.duration+90)
    finally:
        for p in reversed(children):
            try:os.killpg(p.pid,signal.SIGTERM)
            except ProcessLookupError:pass
        for p in reversed(children):
            try:p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGKILL);p.wait()
        for log in logs:log.close()
    report=json.loads((out/"trajectories.json").read_text())
    planner_log=(out/"planner.log").read_text(errors="replace")
    attempts=[float(x) for x in re.findall(r"\[([0-9]+\.[0-9]+)\].*BB_REPLAN_ATTEMPT",planner_log)]
    report["planner_attempts"]=len(attempts)
    report["planner_attempt_rate_hz"]=len(attempts)/args.duration
    report["minimum_attempt_interval_s"]=min(
        (b-a for a,b in zip(attempts,attempts[1:])),default=None)
    if args.expect_rejected:
        report["retry_backoff_ok"]=len(attempts)>=2 and report["minimum_attempt_interval_s"]>=0.49
        report["ok"]=report["ok"] and report["retry_backoff_ok"]
    if args.expect_gated:
        report["ok"]=report["ok"] and not attempts
    (out/"trajectories.json").write_text(json.dumps(report,indent=2)+"\n")
    rc=0 if rc==0 and report["ok"] else 1
    print(json.dumps({k:v for k,v in report.items() if k!="trajectories"},indent=2))
    return rc
if __name__=="__main__":raise SystemExit(main())
