"""Small repeatable collection suite and report generation, without overwrites."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np

from .evaluation import replay
from .scene import ROOT


def run_suite(prefix):
    cases = [
        ("train_green", 2, "train", 0, 5),
        ("train_lift", 3, "train", .45, 8),
        ("validation_dry", 22, "validation", 0, 5),
        ("test_green", 302, "test", 0, 5),
        ("test_dry_lift", 303, "test", .65, 10),
        ("test_lowcontrast", 304, "test", .25, 6),
    ]
    summaries = []
    for suffix, seed, split, lift, duration in cases:
        name = prefix + "_" + suffix
        cmd = [sys.executable, str(ROOT / "scripts/bale_autonomy.py"), "collect",
               "--name", name, "--seed", str(seed), "--split", split,
               "--lift", str(lift), "--duration", str(duration), "--wall-timeout", "150"]
        result = subprocess.run(cmd, cwd=ROOT, timeout=170)
        directory = ROOT / "datasets" / name
        if result.returncode:
            summaries.append(dict(trial=name, failure=f"collection exit {result.returncode}"))
            continue
        for tag, noise, dropout, cameras in [
            ("clean", 0, 0, None), ("noisy", .02, .15, None),
            ("front_only", 0, 0, ["front"]), ("right_only", 0, 0, ["right"])]:
            summaries.append(dict(replay(directory, noise, dropout, tag=tag, cameras=cameras), tag=tag))
        print(f"Evaluated {name}", flush=True)
    out = ROOT / "runs" / prefix
    out.mkdir(parents=True, exist_ok=True)
    (out / "suite.json").write_text(json.dumps(summaries, indent=2))
    make_report(prefix)


def make_report(prefix):
    out = ROOT / "runs" / prefix
    out.mkdir(parents=True, exist_ok=True)
    trials = sorted((ROOT / "datasets").glob(prefix+"*"))
    rows, previews, approach_rows = [], [], []
    for trial in trials:
        path = trial / "evaluation_clean.json"
        if path.exists():
            data = json.loads(path.read_text())
            rows.append(data)
            img = cv2.imread(str(trial / "preview_clean.jpg"))
            if img is not None:
                # Label contact sheet rather than relying on file ordering.
                canvas = np.full((img.shape[0]+24,img.shape[1],3), 245, np.uint8)
                canvas[24:] = img
                cv2.putText(canvas, trial.name, (5,16), cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),1)
                previews.append(canvas)
            draw_errors(trial, out / (trial.name + "_errors.png"))
        approach = trial / "approach.json"
        if approach.exists():
            approach_rows.append(dict(trial=trial.name, **json.loads(approach.read_text())))
            draw_trajectory(trial, out / (trial.name + "_trajectory.png"))
    text = ["# RGB-D bale approach experiment report", "",
            "Generated from recorded simulation trials; these are baseline results, not real-world validation.", "",
            "## Perception", "", "| Trial | Split | Frames | Time with valid pose | Position mean / p95 (m) | Yaw mean (deg) |",
            "|---|---|---:|---:|---:|---:|"]
    for r in rows:
        p, y = r["single_position_m"], r["single_yaw_deg"]
        text.append(f"| {r['trial']} | {r['split']} | {r['frames']} | {r.get('time_with_valid_pose_rate',r['detection_rate']):.0%} | " +
                    (f"{p['mean']:.3f} / {p['p95']:.3f} | {y['mean']:.2f} |" if p else "n/a | n/a |"))
    text += ["", "Time with valid pose counts timestamps with at least one camera estimate within 0.30m / 8 degrees of the target labels. Pose errors include wrong associations, not just successful detections.",
             "", "## Sensor and noise ablations", "",
             "| Trial | Front-only valid time | Right-only valid time | Combined valid time | Noisy valid time | Noisy single / fused yaw mean (deg) |",
             "|---|---:|---:|---:|---:|---:|"]
    for trial in trials:
        variants = {}
        for tag in ("front_only","right_only","clean","noisy"):
            path = trial / f"evaluation_{tag}.json"
            if path.exists():
                variants[tag] = json.loads(path.read_text())
        if len(variants)==4:
            rates = [variants[tag].get("time_with_valid_pose_rate",0) for tag in ("front_only","right_only","clean","noisy")]
            noisy = variants["noisy"]
            sy,fy = noisy["single_yaw_deg"],noisy["fused_yaw_deg"]
            text.append("| " + trial.name + " | " + " | ".join(f"{r:.0%}" for r in rates) +
                        " | " + (f"{sy['mean']:.2f} / {fy['mean']:.2f}" if sy and fy else "n/a") + " |")
    text += ["", "Noise replay adds 0.02m Gaussian depth noise and removes 15% of depth pixels (seed 123). It is a controlled perturbation, not a model of a particular commercial sensor.",
             "", "## Autonomous approach", "", "| Trial | Accepted | True goal error (m) | Heading error (deg) | Minimum SAT gap (m) |",
             "|---|---|---:|---:|---:|"]
    failures = []
    for r in approach_rows:
        def num(key):
            value = r.get(key)
            return f"{value:.3f}" if value is not None else "n/a"
        text.append(f"| {r['trial']} | {r['acceptance_pass']} | {num('true_goal_error_m')} | "
                    f"{num('true_heading_error_deg')} | {num('min_separating_axis_gap_m')} |")
        if r.get("failure"):
            failures.append(f"- {r['trial']}: {r['failure']}" +
                            (f" (injected-loss stop passed: {r['safety_stop_pass']})" if "safety_stop_pass" in r else ""))
    text += ["", *failures]
    text += ["", "Acceptance: true pre-grasp error <0.30 m, heading error <8 degrees, and conservative SAT gap >0.20 m.",
             "SAT gap is a separating-axis measure, not an exact Euclidean clearance. Footprint includes the lowered open grip.",
             "", "## Limits", "", "- Known bale size, upright boxes and flat terrain; color-assisted geometric detection requires a visible top face.",
             "- Current trials use wheel-speed / ideal IMU heading integration with a rear-axle frame correction; early trials used wheel-only odometry. No GNSS or real camera validation yet.",
             "- Multi-view/temporal smoothing is heuristic; reported uncertainty is not calibrated covariance.",
             "- Planning uses observed bales and a known field boundary; this is not general obstacle detection.",
             "- Gazebo allocator crashes during teardown are recorded separately in each trial's collection/approach JSON.",
             "", "See each trial's metadata.json, frames.jsonl, ground_truth.jsonl and evaluation_*.json for reproduction and ablations."]
    (out / "report.md").write_text("\n".join(text)+"\n")
    if previews:
        width = max(p.shape[1] for p in previews)
        padded = [cv2.copyMakeBorder(p,0,0,0,width-p.shape[1],cv2.BORDER_CONSTANT,value=(245,245,245))
                  for p in previews]
        cv2.imwrite(str(out / "camera_contact_sheet.jpg"), cv2.vconcat(padded))
    print(f"Report: {out / 'report.md'}", flush=True)
    return out


def draw_trajectory(trial, output):
    plan_path = trial / "plan.json"
    trajectory_path = trial / "trajectory_ground_truth.jsonl"
    if not plan_path.exists() or not trajectory_path.exists():
        return
    plan_data = json.loads(plan_path.read_text())
    path = np.array(plan_data["path"])
    records = list(map(json.loads,trajectory_path.read_text().splitlines()))
    true = [r["poses"]["simple_tractor"]["position"][:2] for r in records
            if "simple_tractor" in r["poses"]]
    plt = plotting()
    from matplotlib.patches import Polygon
    fig,ax = plt.subplots(figsize=(9,6))
    ax.plot(path[:,0],path[:,1],"--",label="Planned model origin")
    if true:
        xy = np.array(true)
        ax.plot(xy[:,0],xy[:,1],label="Actual model origin")
    for d in plan_data["observed_obstacles"]:
        from .planning import rectangle
        corners = rectangle(*d["position_world"][:2],d["yaw_world"],1.2,-1.2,.6)
        ax.add_patch(Polygon(corners,facecolor="tan",edgecolor="saddlebrown",alpha=.6))
    ax.scatter(*plan_data["goal"][:2],marker="*",s=100,color="red",label="Initial pre-grasp goal")
    from .planning import footprint
    if records and "simple_tractor" in records[-1]["poses"]:
        from .perception import yaw_of
        p = records[-1]["poses"]["simple_tractor"]
        ax.add_patch(Polygon(footprint((*p["position"][:2],yaw_of(p))),fill=False,
                             edgecolor="green",linestyle=":",label="Final conservative footprint"))
    ax.set(xlabel="World x (m)",ylabel="World y (m)",title=trial.name)
    ax.set_aspect("equal",adjustable="datalim")
    ax.grid(alpha=.25)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output,dpi=160)
    plt.close(fig)


def plotting():
    os.environ.setdefault("MPLCONFIGDIR",str(ROOT / "runs" / ".matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def draw_errors(trial, output):
    path = trial / "estimates_clean.jsonl"
    if not path.exists():
        return
    rows = list(map(json.loads,path.read_text().splitlines()))
    if not rows:
        return
    plt = plotting()
    fig,axes = plt.subplots(2,1,figsize=(9,6),sharex=True)
    for name in sorted(set(r["camera"] for r in rows)):
        selected = [r for r in rows if r["camera"]==name and "position_error_m" in r]
        axes[0].plot([r["stamp"] for r in selected],[r["position_error_m"] for r in selected],".",label=name)
        axes[1].plot([r["stamp"] for r in selected],[r["yaw_error_deg"] for r in selected],".",label=name)
    selected = [r for r in rows if "fused_position_error_m" in r]
    if selected:
        axes[0].plot([r["stamp"] for r in selected],[r["fused_position_error_m"] for r in selected],label="Fused")
        axes[1].plot([r["stamp"] for r in selected],[r["fused_yaw_error_deg"] for r in selected],label="Fused")
    axes[0].axhline(.3,color="red",linestyle=":",label="Acceptance threshold")
    axes[1].axhline(8,color="red",linestyle=":")
    axes[0].set(ylabel="Position error (m)",title=trial.name)
    axes[1].set(ylabel="Axis yaw error (deg)",xlabel="Simulation time (s)")
    for ax in axes:
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output,dpi=160)
    plt.close(fig)


def draw_pick_place(trial,output):
    """Standalone trajectory, payload-height and actuator-evidence figure."""
    trial = Path(trial)
    rows = list(map(json.loads,(trial/"trajectory.jsonl").read_text().splitlines()))
    truth = list(map(json.loads,(trial/"trajectory_ground_truth.jsonl").read_text().splitlines()))
    result = json.loads((trial/"approach.json").read_text())
    name = f"bale_{result['target_bale']}"
    paired = [(r,t) for r,t in zip(rows,truth)
              if name in t["poses"] and "simple_tractor" in t["poses"]
              and all(key in r.get("loader",{})
                      for key in ("lift","tilt","grip_left","grip_right"))]
    stamps = np.array([r["stamp"] for r,t in paired])
    tractor = np.array([t["poses"]["simple_tractor"]["position"] for r,t in paired])
    bale = np.array([t["poses"][name]["position"] for r,t in paired])
    plt = plotting()
    fig,axes = plt.subplots(2,2,figsize=(14,9))
    ax = axes[0,0]
    ax.plot(tractor[:,0],tractor[:,1],label="Tractor reference",color="tab:green")
    ax.plot(bale[:,0],bale[:,1],"--",label="Bale",color="tab:orange")
    for p in sorted(trial.glob("manipulation_plan_*.json")):
        path = np.array(json.loads(p.read_text())["path"])
        ax.plot(path[:,0],path[:,1],":",color="tab:blue",alpha=.6)
    ax.scatter(0,0,marker="o",label="Tractor start",color="black")
    ax.scatter(*result["drop_pose"][:2],marker="*",s=140,color="tab:cyan",label="Requested drop")
    ax.scatter(*bale[-1,:2],marker="x",s=80,color="tab:red",label="Placed bale")
    ax.set(xlabel="World x (m)",ylabel="World y (m)",title="Actual trajectories and planned routes")
    ax.set_aspect("equal",adjustable="datalim")
    ax.legend(fontsize=8)
    ax = axes[0,1]
    ax.plot(stamps,bale[:,2],label="True bale center")
    measured = [r for r,t in paired if r.get("payload")]
    ax.plot([r["stamp"] for r in measured],[r["payload"]["position_model"][2] for r in measured],
            label="RGB-D height (level frame)",alpha=.7)
    ax.axhline(.45,color="black",linestyle=":",label="Bale resting on ground")
    events = json.loads((trial/"pick_place_events.json").read_text())
    for e in events:
        if e["phase"] in ("clamp","lift_verify","transport","release","withdraw"):
            ax.axvline(e["stamp"],color="grey",alpha=.3)
            ax.text(e["stamp"],1.65,e["phase"],rotation=60,fontsize=7)
    ax.set(xlabel="Simulation time (s)",ylabel="Height (m)",ylim=(0,2),title="Lift, carry and ground release")
    ax.legend(fontsize=8,loc="upper left")
    ax = axes[1,0]
    pressure = ax.twinx()
    for key,color,label in [("grip_left","tab:blue","Left"),("grip_right","tab:orange","Right")]:
        ax.plot(stamps,[r["loader"][key][0] for r,t in paired],color=color,label=label+" jaw closure")
        pressure.plot(stamps,[r["loader"][key][4] for r,t in paired],"--",color=color,alpha=.5)
    ax.set(xlabel="Simulation time (s)",ylabel="Jaw closure (m)",title="Jaw feedback; dashed lines are equivalent pressure")
    pressure.set_ylabel("Equivalent jaw pressure (bar)")
    ax.legend(fontsize=8)
    ax = axes[1,1]
    ax.plot(stamps,[r["command"]["speed_m_s"] for r,t in paired],label="Speed (m/s)")
    ax.plot(stamps,[r["loader"]["lift"][0] for r,t in paired],label="Lift angle (rad)")
    ax.plot(stamps,[r["loader"]["tilt"][0] for r,t in paired],label="Tilt angle (rad)")
    ax.set(xlabel="Simulation time (s)",title="Conventional control commands and loader position")
    ax.legend(fontsize=8)
    for ax in axes.flat:
        ax.grid(alpha=.2)
    fig.suptitle(f"{trial.name} — drop error {result['drop_error_m']:.3f} m")
    fig.tight_layout()
    fig.savefig(output,dpi=160)
    plt.close(fig)


def draw_stack(trial,output):
    """Export measured world trajectories and layer heights for a complete mission."""
    trial = Path(trial)
    rows = list(map(json.loads,(trial/"trajectory_ground_truth.jsonl").read_text().splitlines()))
    result = json.loads((trial/"stack.json").read_text())
    plt = plotting()
    fig,axes = plt.subplots(1,3,figsize=(17,5))
    for index,color in ((1,"tab:orange"),(2,"tab:blue"),(3,"tab:purple")):
        records = [r for r in rows if f"bale_{index}" in r["poses"]]
        xyz = np.array([r["poses"][f"bale_{index}"]["position"] for r in records])
        stamps = [r["stamp"] for r in records]
        axes[0].plot(xyz[:,0],xyz[:,1],color=color,label=f"Bale {index}")
        axes[0].scatter(*xyz[-1,:2],color=color,marker="x",s=70)
        axes[1].plot(stamps,xyz[:,2],color=color,label=f"Bale {index}")
    tractor = np.array([r["poses"]["simple_tractor"]["position"] for r in rows
                        if "simple_tractor" in r["poses"]])
    axes[0].plot(tractor[:,0],tractor[:,1],color="tab:green",alpha=.6,label="Tractor")
    axes[0].scatter(*result["drop_pose"][:2],marker="*",s=150,color="black",label="Drop zone")
    axes[0].set(xlabel="World x (m)",ylabel="World y (m)",title="Actual mission trajectories")
    axes[0].set_aspect("equal",adjustable="datalim")
    for z in (.45,1.35,2.25):
        axes[1].axhline(z,color="grey",linestyle=":",alpha=.5)
    axes[1].set(xlabel="Simulation time (s)",ylabel="Bale center height (m)",title="Pickup, low carry and three layers")
    from matplotlib.patches import Rectangle
    for layer,color in zip(result["layers"],("tab:orange","tab:blue","tab:purple")):
        x,y,z = layer["position"]
        axes[2].add_patch(Rectangle((y-.6,z-.45),1.2,.9,color=color,alpha=.65))
        axes[2].text(y,z,f"Bale {layer['bale']}",ha="center",va="center")
    axes[2].axhline(0,color="black")
    dy = result["drop_pose"][1]
    axes[2].set(xlim=(dy-1.1,dy+1.1),ylim=(-.05,3.05),xlabel="World y (m)",
                ylabel="Height (m)",title="Final stack, front projection")
    axes[2].set_aspect("equal")
    for ax in axes:
        ax.grid(alpha=.2)
    axes[0].legend(fontsize=8)
    axes[1].legend(fontsize=8)
    fig.suptitle(f"{trial.name}: {'PASS' if result['acceptance_pass'] else 'INCOMPLETE / FAIL'}")
    fig.tight_layout()
    fig.savefig(output,dpi=160)
    plt.close(fig)
