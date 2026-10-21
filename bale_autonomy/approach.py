"""Sensor-guided approach runner. Ground truth is used only after control."""
import json
import math
from pathlib import Path
import time

import numpy as np

from .perception import TargetTracker, detect, to_world, yaw_of, wrap_axis
from .planning import (Follower, connector, footprint, integrate, obstacle_polygons, plan,
                       polygon_gap, pregrasp, valid_pose, wrap)
from .recording import Recorder
from .scene import ROOT, make_world, scenario
from .transport import Session, decode_image
from .visualization import LivePath
from .manipulation import PickPlace, payload_observation, LOADED_FRONT
from .stacking import StackPickPlace, STACK_ORDER, support_surface


def run_approach(args):
    stacking = getattr(args,"stack",False)
    cfg = scenario(args.seed, args.split)
    cfg["target_bale"] = args.bale
    region = {1:"forward",2:"left",3:"right"}[args.bale]
    cfg["target_region"] = region
    if args.bale != 1 or args.pick_place:
        cfg["cameras"] = ["front","right","left","mast"]
    if args.target:
        cfg["target"] = args.target
    cfg["visual_loss_after_s"] = args.sensor_loss_after
    cfg["task"] = "stack" if stacking else "pick_place" if args.pick_place else "approach"
    cfg["drop_pose"] = args.drop if args.pick_place else None
    directory = ROOT / "datasets" / args.name
    if directory.exists():
        raise FileExistsError(f"Refusing to overwrite {directory}")
    world, metadata = make_world(directory, cfg)
    session = Session(world, metadata, wall_timeout=args.wall_timeout)
    recorder = Recorder(directory, ROOT / "datasets", rate_hz=2)
    tracker = TargetTracker(initial_region=region)
    trace = (directory / "trajectory.jsonl").open("w")
    labels = (directory / "trajectory_ground_truth.jsonl").open("w")
    phase, reason = "observe", ""
    follower = None
    observed = []
    goal, path = None, None
    planned_target = None
    start = None
    last_trace = -1.
    stopped_since = None
    odom_wall = time.monotonic()
    last_odom_stamp = None
    last_refinement = -1.
    refinements = 0
    minimum_clearance = math.inf
    result = dict(success=False)
    visual = None
    cycle = None
    stage = 0
    stage_start = None
    completed = []
    mission_drop = list(args.drop)
    return_follower = None
    return_obstacles = []
    try:
        if args.show_path or args.gui:
            visual = LivePath(session.environment,directory)
            if args.pick_place:
                from .planning import rectangle
                corners = rectangle(*args.drop,1.4,-1.4,.85)
                visual.create("autonomy_drop_zone",list(corners)+[corners[0]],
                              (.05,.85,.95),width=.12,height=.03)
        if args.gui:
            session.open_gui()
            print("Blue = planned path | Green = driven estimate | Yellow = stopping pose",flush=True)
            if args.pick_place:
                print(f"Cyan = drop zone at {tuple(args.drop[:2])}; magenta = source bale",flush=True)
        if args.wait_for_start:
            input("Arrange the Gazebo view, then press Enter here to start self-driving...")
        if args.start_delay > 0:
            print(f"Starting in {args.start_delay:g} seconds...",flush=True)
            until = time.monotonic()+args.start_delay
            while time.monotonic() < until:
                session.check()
                session.command()
                time.sleep(.05)
        while True:
            session.check()
            state = session.state()
            if state["odometry"] is None:
                session.command()
                time.sleep(.02)
                continue
            now, odom = state["odometry"]
            if last_odom_stamp != now:
                last_odom_stamp, odom_wall = now, time.monotonic()
            if time.monotonic() - odom_wall > 2:
                raise RuntimeError("Odometry stale for 2 wall seconds")
            pose = (*odom["position"][:2], yaw_of(odom))
            if visual:
                visual.update_trail(pose)
            if start is None:
                start = now
                stage_start = now
            if now-start > args.sim_timeout:
                raise TimeoutError(f"Approach exceeded {args.sim_timeout} simulation seconds")
            for pair in session.pairs():
                rgb = decode_image(pair["rgb_message"], "rgb")
                depth = decode_image(pair["depth_message"], "depth")
                detections = detect(rgb, depth, metadata["cameras"][pair["camera"]],
                                    pair["odometry"]["quaternion_xyzw"],
                                    # A turning target can be close in body x
                                    # while still several meters off to a side.
                                    # Preserve it through terminal approach with
                                    # the same self-mask used during gripping.
                                    minimum_forward=3.0 if cycle or phase in
                                    ("approach","settle") else 5.25)
                load = payload_observation(rgb,depth,metadata["cameras"][pair["camera"]],
                    pair["odometry"]["quaternion_xyzw"],maximum_forward=(
                        10.5 if stacking else (8.5 if cycle.phase in
                        ("withdraw","verify_placement","done") else 6.6))) if cycle else None
                if cycle:
                    cycle.observe(load,pair["stamp"])
                    if stacking and cycle.level and cycle.phase in ("transport","lower"):
                        surface = support_surface(rgb,depth,metadata["cameras"][pair["camera"]],
                                                  pair["odometry"],cycle.drop,cycle.level)
                        cycle.observe_surface(surface,pair["stamp"])
                candidates = [to_world(d, pair["odometry"]) for d in detections]
                if args.sensor_loss_after is not None and pair["stamp"]-start >= args.sensor_loss_after:
                    candidates = []  # deliberate loss at the measurement interface
                tracker.update(candidates, pair["stamp"], pair["camera"])
                if phase == "observe":
                    for candidate in candidates:
                        if not any(np.linalg.norm(np.array(candidate["position_world"][:2])-
                                                   old["position_world"][:2]) < .8 for old in observed):
                            observed.append(candidate)
                recorder.save(pair, rgb, depth, estimate=dict(
                    detections=candidates, target=tracker.target,payload=load,
                    task_phase=cycle.phase if cycle else phase))
            target = tracker.target
            speed = yaw_rate = 0.
            if phase == "observe" and now-stage_start > 2.5 and target and target["observations"] >= 6:
                if target["position_std_m"] > .20 or target["yaw_std_rad"] > .15:
                    raise RuntimeError("Target uncertainty exceeds approach limits")
                session.command()
                goal = pregrasp(target, pose)
                planned_target = np.array(target["position_world"][:2])
                obstacles = obstacle_polygons(observed)
                path, info = plan(pose, goal, obstacles, target=target,
                                  cameras=metadata["cameras"])
                follower = Follower(path)
                if visual:
                    visual.set_plan(path,goal)
                    visual.set_target(target)
                (directory / (f"plan_{stage+1}.json" if stacking else "plan.json")).write_text(json.dumps(dict(
                    start=pose, goal=goal, path=path, planner=info,
                    observed_obstacles=observed), indent=2))
                phase = "approach"
                print(f"Planned {len(path)} poses: {info}; goal={goal}", flush=True)
                # Re-read odometry after planning; the vehicle remained stopped.
                continue
            if phase == "approach":
                if target is None or now-target["last_seen"] > 1.0:
                    reason = "target stale; commanding stop"
                    if now-(target["last_seen"] if target else start) > 4:
                        raise RuntimeError("Target lost for more than 4 simulation seconds")
                elif np.linalg.norm(np.array(target["position_world"][:2])-planned_target) > .5:
                    # Replan from the measured pose when the fused camera
                    # estimate improves during approach; the tracker already
                    # rejects single-frame innovations above one meter.
                    session.command()
                    goal = pregrasp(target,pose)
                    path,info = plan(pose,goal,obstacles,target=target,
                                     cameras=metadata["cameras"])
                    follower = Follower(path)
                    planned_target = np.array(target["position_world"][:2])
                    refinements += 1
                    if visual:
                        visual.set_plan(path,goal)
                        visual.set_target(target)
                    (directory/f"plan_{stage+1}_refined_{refinements}.json").write_text(
                        json.dumps(dict(start=pose,goal=goal,path=path,planner=info,
                                        target=target),indent=2))
                    continue
                else:
                    # Refine the terminal approach from fresh camera-relative geometry.
                    # Small odometry drift should not determine the final stopping pose.
                    remaining = math.hypot(pose[0]-goal[0], pose[1]-goal[1])
                    if .7 < remaining < 2.5 and now-last_refinement > .5 and (
                            follower.section == len(follower.sections)-1 and
                            follower.sections[-1][2] > 0):
                        live_goal = pregrasp(target, pose)
                        refined = connector(pose, live_goal, 1, obstacles)
                        if refined:
                            follower = Follower(refined)
                            goal = live_goal
                            refinements += 1
                            if visual:
                                visual.set_plan(refined,goal)
                        last_refinement = now
                    speed, yaw_rate, arrived = follower.command(pose, odom["linear"][0])
                    if arrived:
                        speed = yaw_rate = 0.
                        phase = "settle"
                        stopped_since = now
                    future = integrate(pose, speed*.7, yaw_rate/speed if speed else 0)
                    safety_obstacles = obstacles
                    if target is not None:
                        # The target is intentionally approached; keep every
                        # other bale in the safety check but do not treat the
                        # pre-grasp target itself as a collision obstacle.
                        target_xy = np.array(target["position_world"][:2])
                        safety_obstacles = [p for p in obstacles
                            if np.linalg.norm(p.mean(axis=0)-target_xy)>1.]
                    if not valid_pose(future, safety_obstacles):
                        raise RuntimeError("Predicted footprint collision; stopped")
            if phase == "settle" and now-stopped_since > 1.2:
                if args.pick_place:
                    if stacking:
                        cycle_directory = directory/f"cycle_{stage+1}"
                        cycle_directory.mkdir()
                        cycle = StackPickPlace(target,observed,mission_drop,cycle_directory,visual,level=stage)
                    else:
                        cycle = PickPlace(target,observed,args.drop,directory,visual,args.stop_after_pickup)
                    phase = "manipulation"
                else:
                    result.update(success=True, final_odometry=pose, goal=goal,
                        odometry_goal_error_m=math.hypot(pose[0]-goal[0], pose[1]-goal[1]),
                        odometry_heading_error_deg=math.degrees(abs(wrap(pose[2]-goal[2]))),
                        simulated_duration_s=now-start)
                    break
            command = dict(speed=speed,yaw_rate=yaw_rate,lift=0.,tilt=0.,grip=0.)
            if phase == "manipulation":
                command = cycle.step(now,pose,odom,state["loader"],target)
                if cycle.finished:
                    if stacking:
                        completed.append(dict(bale=STACK_ORDER[stage],level=stage,
                            pickup_verified=cycle.pickup_verified,
                            placed_estimate=cycle.placed.tolist(),duration_s=now-stage_start))
                        (directory/"stack_progress.json").write_text(json.dumps(completed,indent=2))
                        print(f"Stack: {len(completed)}/3 bales placed",flush=True)
                        if stage==0:
                            mission_drop[:2] = cycle.placed[:2].tolist()
                        source = np.array(cycle.target["position_world"][:2])
                        if observed:
                            selected = min(range(len(observed)),key=lambda i: np.linalg.norm(
                                np.array(observed[i]["position_world"][:2])-source))
                            observed.pop(selected)
                        observed = [o for o in observed if np.linalg.norm(
                            np.array(o["position_world"][:2])-np.array(mission_drop[:2]))>1]
                        observed.append(dict(position_world=[*mission_drop[:2],.45],yaw_world=mission_drop[2]))
                        phase = "reset_loader"
                        stopped_since = now
                        command = dict(speed=0.,yaw_rate=0.,lift=0.,tilt=0.,grip=0.)
                    else:
                        result.update(success=True,pickup_verified=cycle.pickup_verified,
                            drop_pose=args.drop,simulated_duration_s=now-start)
                        break
            if stacking and phase=="reset_loader":
                from .manipulation import valve_for_speed
                q,qd = state["loader"]["lift"][:2]
                t,td = state["loader"]["tilt"][:2]
                lift_speed = max(-.055,-1.2*q) if q>.01 else 0.
                tilt_speed = np.clip(-qd+1.2*(-q-t)-.35*(qd+td),-.16,.16)
                command["lift"] = valve_for_speed("lift",lift_speed)
                command["tilt"] = valve_for_speed("tilt",tilt_speed)
                # valve_for_speed has a small-command deadband for routine
                # movement; explicitly overcome it during the final reset.
                if q>.01 and command["lift"]==0.:
                    command["lift"] = -.06
                if abs(t)>.035 and command["tilt"]==0.:
                    command["tilt"] = float(np.sign(-q-t)*.06)
                if q<.025 and abs(t)<.045 and now-stopped_since>5:
                    cycle = None
                    if stage==2:
                        phase = "settle_stack"
                        stopped_since = now
                    else:
                        session.command()
                        return_obstacles = obstacle_polygons(observed)
                        return_goal = (0.,0.,0.)
                        return_waypoint = (-10.5,-6.,0.)
                        points,info = plan(pose,return_waypoint,return_obstacles,timeout=30)
                        return_follower = Follower(points,heading_tolerance=math.radians(8))
                        if visual:
                            visual.set_plan(points,return_waypoint)
                        (directory/f"return_plan_{stage+1}.json").write_text(json.dumps(dict(
                            path=points,planner=info,waypoint=return_waypoint,goal=return_goal,
                            obstacles=[p.tolist() for p in return_obstacles]),indent=2))
                        phase = "return_home"
                        return_stopped_since = None
                if now-stopped_since>35:
                    raise RuntimeError("Loader did not reset after placement")
            if stacking and phase in ("return_home","return_final"):
                v,w,arrived = return_follower.command(pose,odom["linear"][0])
                scale = min(1.,.22/max(abs(v),1e-6))
                v,w = v*scale,w*scale
                command.update(speed=v,yaw_rate=w)
                if arrived:
                    command.update(speed=0.,yaw_rate=0.)
                    if phase=="return_home":
                        return_stopped_since = now
                        phase = "return_settle"
                    else:
                        return_stopped_since = now
                        phase = "return_done_settle"
            if stacking and phase=="return_settle":
                command.update(speed=0.,yaw_rate=0.)
                if abs(odom["linear"][0])<.03 and now-return_stopped_since>1.:
                    points,info = plan(pose,return_goal,return_obstacles,timeout=30)
                    return_follower = Follower(points,heading_tolerance=math.radians(8))
                    if visual:
                        visual.set_plan(points,return_goal)
                    (directory/f"return_final_plan_{stage+1}.json").write_text(json.dumps(
                        dict(path=points,planner=info,start=pose,goal=return_goal),indent=2))
                    phase = "return_final"
            if stacking and phase=="return_done_settle":
                command.update(speed=0.,yaw_rate=0.)
                if abs(odom["linear"][0])<.03 and now-return_stopped_since>1.:
                        stage += 1
                        tracker = TargetTracker(initial_region={1:"forward",2:"left",3:"right"}[STACK_ORDER[stage]])
                        phase = "observe"
                        stage_start = now
                        follower = None
                        print(f"Stack: acquiring source bale {STACK_ORDER[stage]}",flush=True)
            if stacking and phase=="settle_stack" and now-stopped_since>8:
                result.update(success=True,pickup_verified=all(c["pickup_verified"] for c in completed),
                    cycles=completed,drop_pose=args.drop,simulated_duration_s=now-start)
                break
            session.command(**command)
            if now-last_trace >= .1:
                trace.write(json.dumps(dict(stamp=now, phase=cycle.phase if cycle and phase=="manipulation" else phase,
                    stage=stage,target_bale=STACK_ORDER[stage] if stacking else args.bale,odometry=odom,
                    target=target, payload=cycle.payload if cycle else None,loader=state["loader"],
                    load_evidence=cycle.load_evidence(now,state["loader"]) if cycle else None,
                    command=dict(speed_m_s=command["speed"],yaw_rate_rad_s=command["yaw_rate"],
                                 lift_valve=command["lift"],tilt_valve=command["tilt"],grip_valve=command["grip"])))+"\n")
                # Labels are logged separately, never read by planning/control.
                truth = state["poses"]
                if truth:
                    labels.write(json.dumps(dict(stamp=now, pose_stamp=truth[0], poses=truth[1]))+"\n")
                last_trace = now
            time.sleep(.02)
    except KeyboardInterrupt:
        result.update(success=False,failure="Interrupted by operator")
    except Exception as exc:
        result.update(success=False, failure=f"{type(exc).__name__}: {exc}")
        print(result["failure"], flush=True)
    finally:
        session.command()
        final_state = session.state()
        if visual:
            visual.close()
            result["path_visuals"] = dict(plan_revisions=visual.revision,trail_segments=visual.trail_count)
        if args.keep_open and session.gui is not None and result.get("failure") != "Interrupted by operator":
            print("Driving finished. Inspect the path; close Gazebo or press Ctrl-C to exit.",flush=True)
            try:
                while session.gui.poll() is None and session.server.poll() is None:
                    session.command()
                    time.sleep(.1)
            except KeyboardInterrupt:
                pass
        result.update(shutdown_status=session.close(), frames=recorder.count,
                      partition=session.partition, terminal_refinements=refinements)
        recorder.close()
        trace.close()
        labels.close()
        # Evaluate final true pose only after the controller has finished.
        if final_state["poses"] and goal and not args.pick_place:
            truth = final_state["poses"][1]
            tractor, bale = truth.get("simple_tractor"), truth.get(f"bale_{args.bale}")
            if tractor and bale:
                actual = (*tractor["position"][:2], yaw_of(tractor))
                true_goal = pregrasp(dict(position_world=bale["position"],
                                          yaw_world=yaw_of(bale)), actual)
                result.update(final_ground_truth=actual, true_goal=true_goal,
                    true_goal_error_m=math.hypot(actual[0]-true_goal[0],actual[1]-true_goal[1]),
                    true_heading_error_deg=math.degrees(abs(wrap(actual[2]-true_goal[2]))))
        # Score conservative clearance over the entire executed trajectory.
        control_rows = list(map(json.loads,(directory/"trajectory.jsonl").read_text().splitlines()))
        truth_rows = list(map(json.loads,(directory/"trajectory_ground_truth.jsonl").read_text().splitlines()))
        for row in truth_rows:
            poses = row["poses"]
            tractor = poses.get("simple_tractor")
            if not tractor:
                continue
            body = footprint((*tractor["position"][:2], yaw_of(tractor)),
                             front_extent=LOADED_FRONT if args.pick_place else 5.2)
            for name, bale in poses.items():
                if name.startswith("bale_"):
                    if stacking:
                        control = min(control_rows,key=lambda r:abs(r["stamp"]-row["stamp"]))
                        if name == f"bale_{control['target_bale']}":
                            continue
                        if math.dist(bale["position"][:2],args.drop[:2])<1.:
                            # Raised implement is allowed over the support stack;
                            # score chassis clearance independently in these phases.
                            scored_body = footprint((*tractor["position"][:2],yaw_of(tractor)),front_extent=2.)
                        else:
                            scored_body = body
                    else:
                        scored_body = body
                    if args.pick_place and not stacking and name == f"bale_{args.bale}":
                        continue  # intended contact with the selected payload
                    poly = obstacle_polygons([dict(position_world=bale["position"],
                                                   yaw_world=yaw_of(bale))], margin=0)[0]
                    minimum_clearance = min(minimum_clearance, polygon_gap(scored_body, poly))
        result["min_separating_axis_gap_m"] = minimum_clearance if math.isfinite(minimum_clearance) else None
        result["acceptance_pass"] = bool(result["success"] and
            result.get("true_goal_error_m", math.inf) < .30 and
            result.get("true_heading_error_deg", math.inf) < 8 and minimum_clearance > .20)
        result["target_bale"] = args.bale
        if args.pick_place:
            result["task"] = "pickup_only" if args.stop_after_pickup else "pick_place"
            result["pickup_verified"] = bool(cycle and cycle.pickup_verified)
            truth = final_state["poses"][1] if final_state["poses"] else {}
            bale = truth.get(f"bale_{args.bale}")
            if bale:
                result["final_bale_position"] = bale["position"]
                if not args.stop_after_pickup:
                    result["drop_error_m"] = math.dist(bale["position"][:2],args.drop[:2])
                    result["drop_yaw_error_deg"] = math.degrees(abs(wrap_axis(yaw_of(bale)-args.drop[2])))
            payload_ok = (bale and bale["position"][2]>.75) if args.stop_after_pickup else (
                bale and result["drop_error_m"]<.6 and abs(bale["position"][2]-.45)<.15)
            result["acceptance_pass"] = bool(result["success"] and result["pickup_verified"] and
                payload_ok and minimum_clearance>.20)
        if stacking:
            from scipy.spatial.transform import Rotation
            result.update(task="three_bale_stack",cycles=completed,drop_pose=args.drop,
                          pickup_verified=len(completed)==3 and all(c["pickup_verified"] for c in completed))
            truth = final_state["poses"][1] if final_state["poses"] else {}
            layers = []
            for level,index in enumerate(STACK_ORDER):
                bale = truth.get(f"bale_{index}")
                if bale:
                    rpy = Rotation.from_quat(bale["quaternion_xyzw"]).as_euler("xyz")
                    layers.append(dict(bale=index,level=level,position=bale["position"],
                        drop_error_m=math.dist(bale["position"][:2],args.drop[:2]),
                        height_error_m=abs(bale["position"][2]-(.45+.9*level)),
                        roll_pitch_deg=np.degrees(rpy[:2]).tolist(),
                        yaw_error_deg=math.degrees(abs(wrap_axis(yaw_of(bale)-args.drop[2])))))
            result["layers"] = layers
            # Include alignment between neighboring layers, beyond map tolerance.
            aligned = len(layers)==3 and all(math.dist(a["position"][:2],b["position"][:2])<.30
                                              for a,b in zip(layers,layers[1:]))
            result["acceptance_pass"] = bool(result["success"] and len(completed)==3 and aligned
                and all(c["pickup_verified"] for c in completed)
                and all(l["drop_error_m"]<.6 and l["height_error_m"]<.12
                        and max(map(abs,l["roll_pitch_deg"]))<8 and l["yaw_error_deg"]<8 for l in layers)
                and minimum_clearance>.20)
        if args.sensor_loss_after is not None:
            records = list(map(json.loads,(directory / "trajectory.jsonl").read_text().splitlines()))
            late = [r for r in records if r["stamp"]-start > args.sensor_loss_after+1.3]
            result["safety_stop_pass"] = bool(late and all(abs(r["command"]["speed_m_s"])<1e-9
                and abs(r["command"]["yaw_rate_rad_s"])<1e-9 for r in late) and
                "Target lost" in result.get("failure",""))
        (directory / "approach.json").write_text(json.dumps(result, indent=2))
        if args.pick_place:
            (directory / "pick_place.json").write_text(json.dumps(result,indent=2))
        if stacking:
            (directory / "stack.json").write_text(json.dumps(result,indent=2))
    print(json.dumps(result, indent=2), flush=True)
    return result
