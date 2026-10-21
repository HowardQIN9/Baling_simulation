#!/usr/bin/env python3
"""Run isolated, reproducible RGB-D trials. Use the project's .venv Python."""
import argparse
import json
from pathlib import Path
import sys
import time
from datetime import datetime

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from bale_autonomy.scene import ROOT, make_world, scenario
from bale_autonomy.transport import Session
from bale_autonomy.recording import Recorder
from bale_autonomy.evaluation import replay
from bale_autonomy.approach import run_approach
from bale_autonomy.batch import run_suite, make_report


def collect(args):
    cfg = scenario(args.seed, args.split, loader_lift=args.lift)
    if args.target:
        cfg["target"] = args.target
    directory = ROOT / "datasets" / args.name
    if directory.exists():
        raise SystemExit(f"Refusing to overwrite existing trial: {directory}")
    world, metadata = make_world(directory, cfg)
    session = Session(world, metadata, wall_timeout=args.wall_timeout)
    recorder = Recorder(directory, ROOT / "datasets")
    start = None
    complete = False
    shutdown = None
    try:
        while True:
            session.check()
            state = session.state()
            lift_position = state["loader"].get("lift", [0])[0]
            valve = .5 if lift_position < args.lift - .025 else 0
            session.command(lift=valve)
            for pair in session.pairs():
                if pair["stamp"] < 1.0:
                    continue
                if start is None:
                    start = pair["stamp"]
                recorder.save(pair)
                if pair["stamp"] - start >= args.duration:
                    print(f"Recorded {recorder.count} paired frames to {directory}", flush=True)
                    complete = True
                    return
            time.sleep(.02)
    finally:
        shutdown = session.close()
        recorder.close()
        (directory / "collection.json").write_text(json.dumps(dict(
            complete=complete, frames=recorder.count, shutdown_status=shutdown,
            partition=session.partition), indent=2))
        print(f"Gazebo shutdown status: {shutdown}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("collect", help="Record a stationary multi-camera trial")
    p.add_argument("--name", default="pilot")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--split", choices=["train", "validation", "test"], default="train")
    p.add_argument("--duration", type=float, default=4)
    p.add_argument("--lift", type=float, default=0)
    p.add_argument("--wall-timeout", type=float, default=120)
    p.add_argument("--target", nargs=3, type=float, metavar=("X", "Y", "YAW"))
    p = sub.add_parser("approach", aliases=["demo","pick-place","stack"], help="Approach, pick/place, or collect three bales into a stack")
    p.add_argument("--name")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--split", choices=["train", "validation", "test"], default="validation")
    p.add_argument("--target", nargs=3, type=float, metavar=("X", "Y", "YAW"))
    p.add_argument("--wall-timeout", type=float, default=240)
    p.add_argument("--sim-timeout", type=float, default=90)
    p.add_argument("--sensor-loss-after", type=float, help="Inject loss of visual measurements after this many simulation seconds")
    p.add_argument("--bale",type=int,choices=[1,2,3],default=1,
                   help="Select center (1), left (2), or right (3) bale in the demo layout")
    p.add_argument("--pick-place",action="store_true",help="Continue after approach to grip, carry and place")
    p.add_argument("--drop",nargs=3,type=float,default=[0,-6,0],metavar=("X","Y","YAW"),
                   help="Ground placement pose; default is right of the initial tractor position")
    p.add_argument("--stop-after-pickup",action="store_true",help="Test grip and elevated-load verification only")
    p.add_argument("--stack",action="store_true",help="Collect all three bales into a vertical stack at --drop")
    p.add_argument("--gui", action="store_true",help="Open Gazebo attached to this trial")
    p.add_argument("--show-path", action="store_true",help="Draw planned path, driven trail and stopping pose")
    p.add_argument("--keep-open", action="store_true",help="Keep the GUI open after stopping until closed or Ctrl-C")
    p.add_argument("--wait-for-start",action="store_true",help="Wait for Enter in this terminal before driving")
    p.add_argument("--start-delay",type=float,default=0,help="Wait this many seconds before observing/planning")
    p = sub.add_parser("evaluate", help="Replay recorded data and score separately against labels")
    p.add_argument("directory")
    p.add_argument("--noise", type=float, default=0)
    p.add_argument("--dropout", type=float, default=0)
    p.add_argument("--tag", default="clean")
    p = sub.add_parser("suite", help="Collect and replay a compact train/validation/test suite")
    p.add_argument("--prefix", default="baseline_v1")
    p = sub.add_parser("report", help="Create a report and visual artifacts from existing trials")
    p.add_argument("--prefix", default="baseline_v1")
    args = parser.parse_args()
    if args.command in ("approach","demo","pick-place","stack"):
        args.stack = args.stack or args.command == "stack"
        if args.stack:
            args.pick_place = True
            args.bale = 1
            args.target = args.target or [8.5,0,0]
            args.wall_timeout = max(args.wall_timeout,2400)
            args.sim_timeout = max(args.sim_timeout,1200)
            if args.stop_after_pickup:
                parser.error("--stack cannot be combined with --stop-after-pickup")
        if args.command == "pick-place":
            args.pick_place = True
            args.target = args.target or [8.5,0,0]
        if args.pick_place:
            args.wall_timeout = max(args.wall_timeout,600)
            args.sim_timeout = max(args.sim_timeout,240)
        if args.command == "demo":
            args.gui = args.show_path = args.keep_open = True
            args.wait_for_start = args.start_delay == 0 and sys.stdin.isatty()
            args.target = args.target or [12,.8,.12]
        args.name = args.name or ("demo_"+datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                                  if args.command in ("demo","pick-place","stack") else "approach_00")
        if args.wait_for_start and not sys.stdin.isatty():
            parser.error("--wait-for-start needs an interactive terminal")
    if hasattr(args, "prefix") and (Path(args.prefix).name != args.prefix or args.prefix in (".", "..", "")):
        parser.error("--prefix must be a single directory name")
    if hasattr(args, "name") and (not args.name or Path(args.name).name != args.name or args.name in (".", "..")):
        parser.error("--name must be a single directory name")
    for attr in ("duration", "wall_timeout", "sim_timeout"):
        if hasattr(args, attr) and getattr(args,attr) <= 0:
            parser.error(f"--{attr.replace('_','-')} must be positive")
    if hasattr(args, "lift") and not 0 <= args.lift <= 1.2:
        parser.error("--lift must be between 0 and 1.2 rad")
    if args.command == "collect":
        collect(args)
    elif args.command in ("approach","demo","pick-place","stack"):
        result = run_approach(args)
        if not (result.get("safety_stop_pass") if args.sensor_loss_after is not None else result["acceptance_pass"]):
            raise SystemExit(1)
    elif args.command == "evaluate":
        print(json.dumps(replay(args.directory, args.noise, args.dropout, tag=args.tag), indent=2))
    elif args.command == "suite":
        run_suite(args.prefix)
    elif args.command == "report":
        make_report(args.prefix)


if __name__ == "__main__":
    main()
