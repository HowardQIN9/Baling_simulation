"""Offline replay; simulator labels are opened only here for scoring."""
import json
import math
from pathlib import Path

import cv2
import numpy as np

from .perception import TargetTracker, annotate, detect, to_world, wrap_axis, yaw_of


def replay(directory, noise_std=0, dropout=0, seed=123, tag="clean", cameras=None):
    directory = Path(directory)
    metadata = json.loads((directory / "metadata.json").read_text())
    labels = {r["id"]: r for r in map(json.loads,
              (directory / "ground_truth.jsonl").read_text().splitlines())}
    frames = list(map(json.loads, (directory / "frames.jsonl").read_text().splitlines()))
    if cameras:
        frames = [f for f in frames if f["camera"] in cameras]
    tracker = TargetTracker(initial_region=metadata["scenario"].get("target_region","forward"))
    rng = np.random.default_rng(seed)
    rows, position_errors, yaw_errors = [], [], []
    fused_position, fused_yaw, per_camera = [], [], {}
    previews = {}
    for frame in frames:
        rgb = cv2.cvtColor(cv2.imread(str(directory / frame["rgb"])), cv2.COLOR_BGR2RGB)
        depth = np.load(directory / frame["depth"])["depth_m"].copy()
        if noise_std:
            finite = np.isfinite(depth)
            depth[finite] += rng.normal(0, noise_std, finite.sum())
        if dropout:
            depth[rng.random(depth.shape) < dropout] = np.nan
        detections = detect(rgb, depth, metadata["cameras"][frame["camera"]],
                            frame["odometry"]["quaternion_xyzw"])
        candidates = [to_world(d, frame["odometry"]) for d in detections]
        updated = tracker.update(candidates, frame["stamp"], frame["camera"])
        truth = labels[frame["id"]]["poses"].get(f"bale_{metadata['scenario'].get('target_bale',1)}")
        row = dict(id=frame["id"], camera=frame["camera"], stamp=frame["stamp"],
                   detections=candidates, fused=updated)
        # Score the forward selected candidate; do not choose a candidate by GT proximity.
        if candidates and truth:
            chosen = candidates[0] if tracker.target is None else min(candidates,
                key=lambda c: np.linalg.norm(np.array(c["position_world"][:2]) -
                                             tracker.target["position_world"][:2]))
            error = float(np.linalg.norm(np.array(chosen["position_world"][:2]) - truth["position"][:2]))
            yaw_error = abs(wrap_axis(chosen["yaw_world"]-yaw_of(truth)))
            position_errors.append(error)
            yaw_errors.append(math.degrees(yaw_error))
            per_camera.setdefault(frame["camera"], []).append(error)
            row.update(position_error_m=error, yaw_error_deg=math.degrees(yaw_error))
        if updated and truth:
            pe = float(np.linalg.norm(np.array(updated["position_world"][:2])-truth["position"][:2]))
            ye = math.degrees(abs(wrap_axis(updated["yaw_world"]-yaw_of(truth))))
            fused_position.append(pe)
            fused_yaw.append(ye)
            row.update(fused_position_error_m=pe,fused_yaw_error_deg=ye)
        rows.append(row)
        if frame["camera"] not in previews and detections:
            previews[frame["camera"]] = annotate(rgb, detections)
    def metrics(values):
        return dict(mean=float(np.mean(values)), p95=float(np.percentile(values, 95)),
                    max=float(max(values))) if values else None
    report = dict(trial=directory.name, split=metadata["scenario"]["split"], frames=len(frames),
        detected_frames=len(position_errors), detection_rate=len(position_errors)/max(1,len(frames)),
        single_position_m=metrics(position_errors), single_yaw_deg=metrics(yaw_errors),
        fused_position_m=metrics(fused_position), fused_yaw_deg=metrics(fused_yaw),
        camera_position_m={k:metrics(v) for k,v in per_camera.items()},
        noise_std_m=noise_std, missing_depth_fraction=dropout,
        max_rgb_depth_skew_s=max((abs(f["depth_offset_s"]) for f in frames), default=None),
        association_rejects=tracker.rejects)
    report["pose_within_tolerance_rate"] = sum(p < .3 and y < 8 for p,y in
        zip(position_errors,yaw_errors))/max(1,len(frames))
    report["camera_detection_rate"] = {name:len(per_camera.get(name,[]))/max(1,
        sum(f["camera"]==name for f in frames)) for name in sorted(set(f["camera"] for f in frames))}
    valid_times = {round(row["stamp"],6) for row in rows if
        row.get("position_error_m",1e9)<.3 and row.get("yaw_error_deg",1e9)<8}
    report["time_with_valid_pose_rate"] = len(valid_times)/max(1,len(set(round(f["stamp"],6) for f in frames)))
    (directory / f"evaluation_{tag}.json").write_text(json.dumps(report, indent=2))
    (directory / f"estimates_{tag}.jsonl").write_text("".join(json.dumps(r)+"\n" for r in rows))
    if previews:
        cv2.imwrite(str(directory / f"preview_{tag}.jpg"), cv2.hconcat(list(previews.values())))
    return report
