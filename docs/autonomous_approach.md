# RGB-D data and autonomous pre-grasp approach

This baseline records reproducible native Gazebo trials, estimates upright square
bales from RGB/depth, and drives to a pre-grasp stopping pose. It leaves the
normal world and manual controls unchanged. Automatic gripping is a later stage.

## Setup and quick start

Build the hydraulic plugin as described in the main README, then:

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements-autonomy.txt
.venv/bin/python tests/test_autonomy.py

# A separate headless simulation, including both cameras and the IMU:
.venv/bin/python scripts/bale_autonomy.py approach --name demo_01 --target 12 0 0

# Record stationary views while lifting the loader (angle in radians):
.venv/bin/python scripts/bale_autonomy.py collect --name capture_01 --seed 3 --lift .45 --duration 8
.venv/bin/python scripts/bale_autonomy.py evaluate datasets/capture_01

# Six short scenarios with reserved test seeds and four replay variants each:
.venv/bin/python scripts/bale_autonomy.py suite --prefix experiment_01
.venv/bin/python scripts/bale_autonomy.py report --prefix experiment_01
```

Use a new name/prefix for a new run. Existing trial folders are never overwritten.
Names cannot contain path separators. No ROS bridge is required. Each trial
starts Gazebo in its own randomly generated `GZ_PARTITION` and shuts down only
that process group. It does not command an existing manual simulation.

## Watch the live Gazebo demo

From an interactive terminal with working graphical display access:

```bash
.venv/bin/python scripts/bale_autonomy.py demo
```

This opens a dedicated Gazebo window with an elevated overview camera and a
tested left-offset approach. Arrange the view, then press **Enter in the launch
terminal** to start driving. **Blue** is the current planned path, **green** is
the driven route from the state estimate, and the **yellow** ring/arrow marks
the stopping pose and heading. The path updates during terminal refinement.
Ground visuals have no collisions and sit below the estimator's height gate.
They represent the tractor reference point, not the front of the grip.

The GUI stays open after the tractor stops. Close that window or press Ctrl-C
in the terminal to finish. Each demo gets a new timestamped folder. Do not run
the manual teleop against this demo. The runner sets its own partition for both
the server and GUI; opening the ordinary world separately will show a different
simulation.

For automatic start, use `demo --start-delay 10`. Existing `approach` commands
can also use `--gui --show-path --wait-for-start --keep-open`. A headless drawing
check can use `approach --show-path`. GUI diagnostics are saved in `gui.log`;
remote/headless terminals need an X/Wayland display to open the window.

To approach a different bale in this fixed three-bale layout, use
`demo --bale 2` for the left-hand bale or `demo --bale 3` for the right-hand
bale. This selects a different observed object; it does not move the center
bale to a new test position. A magenta ground outline identifies the selected
target. These runs add a matching left-mast RGB-D pair to retain the wider view.
Initial association uses the chosen left/center/right region; pose and tracking
continue to use RGB-D measurements. Scoring uses the selected bale's labels.

On the development machine, the runner selects NVIDIA's EGL vendor library when
available; the Ogre2 renderer was verified on an RTX 3090. A restricted sandbox
may need permission to access graphics devices and local Transport networking.
The runner does not alter device permissions or install GPU drivers. An existing
Gazebo allocator failure can recur on shutdown, requiring process-group cleanup;
the recorded shutdown status is separate from runtime task acceptance.

## Cameras and frames

Both views are generic ideal pinhole RGB-D pairs, not a calibrated model of a
commercial camera. Each is 320×240 at 10 Hz, with 0.1–30 m clipping. RGB and
depth have identical intrinsics and extrinsics within each pair. Camera poses
in the tractor model frame are:

| View | Position x/y/z (m) | Roll/pitch/yaw (rad) | Horizontal FOV |
|---|---|---|---|
| Front | 2.05 / 0 / 2.25 | 0 / 0.20 / 0 | 60° |
| Right mast | 1.80 / −1.65 / 3.25 | 0 / 0.32 / 0 | 90° |

The second mount is a provisional visibility experiment; the real mounting
bracket, vehicle width and sensor placement still need validation. It helps
when the hydraulic assembly occludes the front camera, but does not guarantee
visibility through the entire loader motion range or from all approach angles.

Camera axes follow Gazebo: +X forward, +Y left, +Z up. Depth is axial forward
distance in meters, not Euclidean range. Intrinsics are derived from the
generated SDF (`fx=fy=width/(2*tan(hfov/2))`). Raw depth uses the Gazebo
`R_FLOAT32` encoding and is stored as float32 without display normalization.
Invalid/infinite samples remain invalid; image row stride is respected.

The IMU is aligned with the chassis at 100 Hz and is ideal in this baseline.
Wheel-speed / IMU-heading integration estimates the tractor pose, starting at
the known initial pose (0,0,0). The rear axle is 1.321 m behind the model origin.
Both integration and Ackermann planning account for that offset. No simulator
pose topic enters localization, perception, planning or control. IMU roll and
pitch level the measured cloud before height filtering; yaw maps estimates
into the initial world frame. GNSS, heading drift, wheel slip correction and a
calibrated probabilistic localization filter remain future work.

The detection field `position_model` is in the explicitly labeled
`tractor_level` frame: model origin and heading, with roll/pitch removed. Camera
extrinsics remain in the physical model/body frame.

## Dataset layout and synchronization

```text
datasets/<trial>/
  metadata.json                 scenario, calibration, frames, source hashes
  source/                       source snapshot for new trials
  world.sdf                     generated scene, including exact randomization
  images/<camera>_<time>.jpg     RGB, JPEG quality 92
  images/<camera>_<time>.npz     float32 depth_m array
  frames.jsonl                  synchronized observations and estimated state
  ground_truth.jsonl            separate image-time evaluation labels
  server.log                    simulator diagnostics
  collection.json               completion, frame count, shutdown status
  evaluation_<variant>.json     aggregate pose scores
  estimates_<variant>.jsonl     detections, fused track, per-frame errors
  plan.json                     initial path and observed obstacle map
  trajectory.jsonl              approach state and commands
  trajectory_ground_truth.jsonl separate trajectory labels
  approach.json                 task result and stopping/clearance scores
```

RGB/depth pairing uses simulation timestamps and accepts at most 5 ms skew.
Pose and odometry samples must be within 60 ms of exposure; offsets are recorded.
Frames carry wheel odometry, integrated state, IMU, and loader state. Loader
arrays retain their own simulation timestamp (element 6); they are snapshots,
not interpolated states at exposure. Commands are logged against the most
recent odometry simulation timestamp. The recorder samples each camera at 2 Hz
by default while the live estimator consumes camera updates at up to 10 Hz.
Queues have fixed bounds. Labels are read only by evaluation and final scoring.

The dataset budget is the smaller of **20 decimal GB** and **40% of filesystem
capacity**, and recording stops before free space falls below 5 GB. A worst-case
frame allowance is checked before writes. Existing records are never deleted to
make space. `datasets/`, `runs/`, `.venv/`, and build outputs are Git-ignored.
The cap is a maximum, not a collection target: short baseline trials need much
less space. Run trials sequentially when collecting near the storage limit.

## Perception and fusion

The baseline uses a broad straw-color cue and height/geometry filtering to
find candidate bales. It fits a rectangle to visible top-face depth points,
using the known 2.4×1.2×0.9 m bale dimensions as a consistency check. It rejects
substantially truncated tops rather than inventing a complete bale center.
Orientation is the long axis, modulo 180°, which is sufficient to choose an
approach end. There is no learned detector or simulator segmentation input.

At initialization the target is selected from forward candidates, using range
and lateral offset. Subsequent association uses world-space position/yaw gates
and does not drop a tracked bale merely because it moves to the side in an image.
RGB/depth supplies each geometric measurement; both views and repeated
observations feed a static-target exponential smoother. Uncertainty floors and
innovation rejection prevent overconfident updates. Reported standard deviations
are heuristic quality measures, not calibrated covariance, and correlated RGB-D
measurements are not advertised as independent sensors.

Replay compares clean data, 2 cm Gaussian depth noise with 15% missing pixels,
front-only input, and right-only input. These are controlled perturbations, not
a detailed real RGB-D noise model. Design trials and held-out seeds are listed
in `batch.py`; the detector has no training phase. Do not use held-out results
for tuning while continuing to call the same cases held out.

## Approach and safety behavior

The target generates two equivalent long-axis goals, 7.15 m from the bale
center in the tractor model frame; the nearer end is selected. This deliberately
leaves clearance for the lowered open grip and the bale's near face. It is a
pre-grasp stop, not a gripping/insertion position.

Planning first tries a curvature-checked Hermite connection, then a bounded
forward/reverse Hybrid A* search. Motion uses rear-axle bicycle kinematics with
a 0.19 m⁻¹ planning curvature limit. Collision checks use a conservative model
footprint spanning x=−2.3 to +5.2 m and y=±1.55 m, observed bale rectangles,
and a known ±14.5 m field boundary. Steering tire envelopes are included in
the width. The horizontal visibility check requires one camera to cover the
target corners, with a margin. It does not raycast the loader or unknown objects.

The follower limits cruise speed to 0.45 m/s, slows further during tight turns,
stops at direction changes, and refines the terminal connection using fresh
camera measurements. Steering reserve allows tracking corrections beyond the
planning curvature. Visual measurements older than 1 simulation second command
zero speed/turn; loss exceeding 4 seconds aborts. Stale odometry, a target jump,
predicted footprint collision, planning failure or a trial timeout also stops
the run. Cleanup publishes stop and neutral commands before terminating the
isolated simulation. This is experimental simulation software, not a certified
real-machine safety system.

Inject measurement loss to test stopping:

```bash
.venv/bin/python scripts/bale_autonomy.py approach --name loss_01 --target 12 0 0 --sensor-loss-after 6
```

The command succeeds only if the expected loss abort occurs and late logged
commands remain zero. The result is labeled `safety_stop_pass`; it is not counted
as a successful approach.

Task acceptance is evaluated against separate simulator labels: pre-grasp
position error <0.30 m, heading error <8°, and minimum conservative separating-axis
gap >0.20 m throughout the logged trajectory. A SAT gap proves separation at a
sample; it is not exact Euclidean clearance or continuous collision detection.
Perception reports also include timestamps with at least one valid target pose,
so a blocked front camera does not count as failure if the other view remains
usable. Reports and standalone plots are generated under `runs/<prefix>/`.

## Limits and the next extension

The current model assumes known-size, upright box bales on flat ground, a
stationary target, known initial tractor pose and ideal IMU heading. Ground
appearance and sunlight vary, with inexpensive visual soil patches. There are
no realistic straw meshes, field slopes, GNSS, general obstacle segmentation,
trained appearance model or real tractor validation. Large maneuvers can still
lose the target or fail planning/tracking; such trials must be reported as
failures even when the stop behavior works correctly.

Next work should improve view-aware maneuvering and wheel-slip localization,
then use the same observation/label interfaces in Unreal. Adding LiDAR or
photorealistic assets should follow a demonstrated sensing limitation. Full
pickup should follow repeatable approach success and grip-relative alignment.
