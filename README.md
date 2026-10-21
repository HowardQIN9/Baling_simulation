# Autonomous Bale Handling with a Tractor

## Current development status — 2026-10-21

The latest work is a separate [four-spear handling prototype](spear_handling/README.md)
that reuses the tractor and L115-inspired loader. Its default bales have solid
collision bodies with a local yielding penetration/support model around the
shafts. There are no pre-cut entry channels.

**A complete pickup, stack, withdrawal, two-bale lift and transport sequence has
passed twice with the material model.** Matched comparisons reproduce pushing during
incorrect insertion and bale following during withdrawal without downward
preload. Laterally offset, modestly angled entry also passes; shallow engagement
fails the pickup checks. These are provisional physics experiments, not a
calibrated straw model or camera-guided spear autonomy.

See the [material model and limitations](docs/spear_material_model.md),
[recorded material results](docs/material_results.json), and
[validation plot](docs/assets/spear_material_validation.png). Historical channel
trials remain archived separately. Gazebo's allocator/teardown fault remains
unresolved; task results and shutdown status are recorded separately. The
existing jaw-grip simulation and autonomy runner remain available.

Build and run the experimental first test from this directory:

```bash
cmake -S spear_handling -B spear_handling/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build spear_handling/build -j 2
python3 spear_handling/validate.py --case A
```

Use system Python with the Gazebo transport/message bindings. The test creates
an isolated Gazebo partition and writes artifacts under `runs/spear_*`.
Parameters are in `spear_handling/config.json`. Use `--case sequence` for
the two-bale mission, or `python3 spear_handling/check_friction.py` for the
independent contact-pair fixtures.

## Running the Current Scene

Requirements: Gazebo Harmonic (`gz sim` 8), including the Ogre2 rendering engine,
Gazebo development libraries, CMake, and a C++17 compiler.

Build the loader actuator plugin once (and rebuild after changing its C++ code):

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j 2
```

From the project directory, launch the simulation:

```bash
bash scripts/run_sim.sh
```

For a server without a display:

```bash
bash scripts/run_sim.sh -s --headless-rendering
```

In another terminal, drive the tractor and operate the loader:

```bash
python3 scripts/teleop_loader.py
```

The scene includes three dynamic square-bale collision targets. Their placeholder
dimensions are 2.4 × 1.2 × 0.9 m; their centers are at (7, 0, 0.45),
(10, 3, 0.45), and (11, -3, 0.45) m. The latter two are rotated ±30°.
Each bale has a box collision matching its visual dimensions. The tractor body
and wheels also have collision geometry, so contact is resolved by the physics
engine. Bales use `static=false` with explicit mass and inertia, so they can slide
and tip under contact forces. Drive slowly toward `bale_1` to try pushing it.
The rigid-box approximation does not model straw compression or fork penetration.
Restart the world after editing its SDF so the new collisions are loaded.

### Loader and Hydraulic Controls

The world includes an L115-inspired raised twin-arm loader with a broad-pad
side-clamping bale grip. It is a sliding-jaw prototype inspired by the Case IH
heavy-duty bale grip description, not a measured replica. There are no fork tines
in the default world.
The configurable actuator model uses provisional 86 L/min shared pump flow,
200 bar equivalent lift/tilt pressure limits, and 100 bar limits for each grip jaw. It is a
force-limited equivalent actuator, not a detailed fluid-circuit simulation or
a calibrated reproduction of the L115. See
[loader geometry, assumptions, controls, and tests](docs/loader_hydraulics.md).

Use **U/J** to increase/decrease the lift valve, **I/K** for tilt, and **N** to
neutralize all valves and hold. **P** closes the grip; **O** opens it. **W/S/A/D** drive; **Space** stops driving and
neutralizes the loader. Commands latch until changed. Do not run a second
driving teleop simultaneously. Restart using `bash scripts/run_sim.sh` so Gazebo
can locate the built actuator plugin.

The grip opens to a 1.9 m internal gap and closes to 0.5 m, using two 0.7 m
jaw strokes. Align the tractor with the long axis of the center bale so the
jaws straddle its 1.2 m width, close with **P**, neutralize with **N**, then raise
with **U**. Press **N** to hold, and **O** to release after lowering. Commands
latch until changed. The grip is tested using contact and friction, with no
invisible attachment joint. Perception and automatic pre-grasp approach are
available in the separate experimental autonomy runner, including contact-based
pickup and ground placement in bounded test cases. The former fork remains only
as a separate test fixture.

To try gripping without positioning the tractor first, close the normal world
and run `python3 scripts/run_grip_demo.py`. This launches a temporary copy with
the center bale on the ground between the open jaws; it does not attach the bale
or change the saved main world. Use the same `scripts/teleop_loader.py` controls.

### RGB-D Recording and Autonomous Approach

The experimental autonomy runner launches an isolated simulation with front and
side-mast RGB-D views plus an ideal IMU. It records synchronized observations,
estimates the bale pose, plans an Ackermann-compatible approach, and stops before
gripping. Ground truth is recorded separately for evaluation.

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements-autonomy.txt
.venv/bin/python scripts/bale_autonomy.py approach --name demo_01 --target 12 0 0
```

No second simulation or teleop terminal is needed for that command. The generated
world leaves the manual scene unchanged. Use a new `--name` for each trial.

To watch it drive with the path drawn in Gazebo:

```bash
.venv/bin/python scripts/bale_autonomy.py demo
```

Press Enter in the terminal after the Gazebo window opens. Blue shows the planned
path, green the driven estimate, and yellow the stopping pose. The window stays
open afterward; close it or press Ctrl-C to finish.

For contact-based pickup and ground placement to the tractor's right:

```bash
.venv/bin/python scripts/bale_autonomy.py pick-place --gui --show-path --keep-open --wait-for-start
```

The cyan drop zone is at `(0, -6)` m relative to the original tractor starting
frame. See [pickup, load verification and ground placement](docs/pick_place.md).

To collect the center and both side bales and stack them at that same drop zone:

```bash
.venv/bin/python scripts/bale_autonomy.py stack --gui --show-path --keep-open --wait-for-start
```

See [three-bale stacking](docs/stacking.md) for the layer sequence and run details.

See [recording, fusion, planning, evaluation and limitations](docs/autonomous_approach.md)
and [the first delivery's measured results](docs/autonomy_results.md).

### Contact Friction

See [the rigid-body physics baseline](docs/physics_parameters.md) for reference
masses, link mass distribution, inertia calculations, limitations, and contact
test instructions.

The relevant collision elements explicitly set both friction directions equally:

| Surface | `mu` | `mu2` |
| --- | --- | --- |
| Ground | 0.55 | 0.55 |
| All four tractor wheels | 1.0 | 1.0 |
| All three bales | 0.4 | 0.4 |

These are approximate simulation parameters, not experimentally validated
material properties. They are per-surface settings; the physics engine determines
the friction for each contacting pair. Bales are dynamic rigid bodies; automatic
approach, alignment, and task execution remain future work.

Support was checked against installed Gazebo Sim 8.15.0, gz-physics DART plugin
7.8.0, DART 6.13.2, and SDFormat 14.9.0. Despite the `ode` tag name, the DART
backend reads both `mu` and `mu2` and forwards them to DART's primary and secondary
friction coefficients for DART >= 6.10. See the
[matching gz-physics 7.8.0 implementation](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.8.0/dartsim/src/SDFFeatures.cc#L826-L835).

### RGB and Depth Cameras

Two separate sensors are attached directly to the chassis. Both share the same
ideal viewpoint at (2.05, 0, 2.25) m in the tractor model frame and point forward
with a 0.20 rad downward pitch. The chassis-relative sensor pose is
`2.05 0 1.20 0 0.20 0`. Gazebo camera axes are +X forward, +Y left, +Z up.

| Setting | Both cameras |
| --- | --- |
| Resolution | 640 × 480 pixels |
| Horizontal field of view | 60° |
| Requested update rate | 15 Hz in simulation time |
| Near / far clip | 0.1 / 30 m |

| Sensor | Gazebo image topic |
| --- | --- |
| RGB | `/tractor/camera/rgb` |
| Depth | `/tractor/camera/depth` |

Matching poses and image geometry provide ideal geometric registration: the same
pixel corresponds to the same viewing direction. This is a simplified baseline
for an RGB-D device. Independent streams must still be matched using message
simulation timestamps; equal update rates alone do not prove synchronization.
If real cameras have different poses or intrinsics, depth reprojection and
extrinsic calibration will be needed.

Inspect the published topics while the simulation is running:

```bash
gz topic -l
gz topic -i -t /tractor/camera/rgb
gz topic -i -t /tractor/camera/depth
```

Use Gazebo's Image Display GUI plugin to inspect the RGB topic. Depth is a
floating-point image in meters and may require normalization for display.
RGB is color imagery; a depth image must not be interpreted as RGB bytes.
This scene uses native Gazebo Transport; ROS bridging is not configured.

If the GUI topic dropdown does not switch the image, open the provided viewers
in separate terminals while the simulation is running:

```bash
gz gui -c worlds/rgb_camera.config
```

```bash
gz gui -c worlds/depth_camera.config
```

Each viewer runs in a separate GUI process and subscribes to a fixed topic,
without a topic dropdown. RGB should show straw-colored bales and green ground;
the depth viewer shows distances as grayscale.

The rendering sensor setup follows the
[Gazebo Harmonic Sensors system documentation](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1Sensors.html).
The numbered sections below describe the broader project plan.

## 1. Project Overview

This project aims to develop an autonomous tractor system for handling square bales.

The target system should be able to detect a bale, approach it, align the tractor and bale fork, pick up the bale, transport it to another location, and unload it.

The system will first be developed in simulation and later validated on a real tractor.

The long-term goal is to build a simulation-to-real autonomy pipeline for agricultural material handling by combining perception, vehicle control, hydraulic control, operator demonstrations, and learning-based methods.

## 2. Why This Project?

Bale handling is a repetitive and labor-intensive agricultural task.

Although specialized bale-handling equipment exists, tractors equipped with loaders and bale forks are still widely used because they are flexible and can perform multiple tasks including bale collection, transport, loading, unloading, and storage.

Bale handling is also a useful robotics problem because it combines several important components of agricultural autonomy:

- Autonomous navigation
- Object detection and localization
- Vehicle alignment
- Loader and hydraulic control
- Task planning
- Interaction between large machinery and agricultural materials

The goal is therefore not simply to make a tractor drive autonomously.

The broader research question is:

> How can simulation, operator demonstrations, perception, and real tractor control be combined to develop autonomous bale-handling behaviors?

## 3. Target Task

The final tractor should perform the following sequence:

1. Detect bale
2. Estimate bale position and orientation
3. Plan an approach path
4. Drive toward the bale
5. Align the tractor and fork with the bale
6. Insert the fork
7. Lift the bale
8. Transport the bale
9. Navigate to the unloading location
10. Align with the target position
11. Lower the bale
12. Release the bale

The eventual goal is to perform this process with minimal operator intervention.

## 4. System Components

The project consists of four major components:

- Simulation
- Perception
- Tractor and loader control
- Real tractor interface

## 5. Simulation

Gazebo is used to build the tractor and bale-handling environment.

The purpose of the simulation is to provide a safe and repeatable environment for developing the system before running experiments on the real tractor.

The simulation can be used for:

- Controller development
- Sensor testing
- Perception development
- Autonomous behavior testing
- Operator demonstrations
- Data generation
- Learning-based policy training
- Sim-to-real experiments

The simulation does not need to reproduce every physical detail of the tractor.

It should reproduce the properties that are important for autonomous behavior, including:

- Tractor geometry
- Steering kinematics
- Vehicle motion
- Sensor placement
- Bale geometry
- Loader and fork geometry
- Actuator limits

## 6. Current Tractor Simulation

A basic tractor model has already been created in Gazebo.

Current capabilities include:

- Ubuntu 22.04 Gazebo environment
- Remote server simulation
- Basic tractor SDF model
- Ackermann steering
- `cmd_vel` control
- Keyboard teleoperation
- Forward and backward motion
- Steering control

Current approximate tractor parameters are:

| Parameter | Value |
| --- | --- |
| Wheelbase | 2.642 meters |
| Wheel separation | 2.00 meters |
| Kingpin width | 1.90 meters |
| Wheel radius | 0.87 meters |
| Steering limit | Approximately 0.6 radians |

These values will later be updated using measurements from the real tractor.

## 7. Tractor Geometry

The next step is to improve the tractor model using real measurements.

Important vehicle measurements include:

- Wheelbase
- Front track width
- Rear track width
- Tire radius
- Tractor width
- Tractor length
- Tractor height
- Steering geometry

Important loader measurements include:

- Loader pivot position
- Loader arm length
- Fork location
- Fork length
- Fork spacing
- Maximum lift height
- Loader joint limits

Sensor measurements will include:

- Sensor position relative to the tractor
- Sensor height
- Yaw
- Pitch
- Roll

For this project, correct kinematic geometry is more important than detailed visual appearance.

## 8. Sensor Simulation

The next major development step is adding sensors to Gazebo.

The initial plan is to install approximately three simulated cameras around the tractor.

Possible locations include:

- A front-facing camera for navigation and bale detection
- A side camera
- A camera near the loader or fork for close-range alignment

Planned sensors include:

- RGB cameras
- RGB-D cameras
- Depth sensing
- 3D point clouds
- Simulated RTK-GNSS

Initial sensor objectives are:

- Publish RGB images
- Publish depth images
- Generate point clouds
- Verify coordinate frames
- Detect the bale
- Estimate the bale location
- Compare different sensing configurations

One possible research question is whether RGB-D sensing, 3D point clouds, or multi-camera fusion provides the most useful information for bale handling.

## 9. Bale Environment

The first manipulation target will be a large square bale.

Square bales are preferred initially because they have a clear orientation and relatively simple geometry.

This makes it easier to evaluate:

- Bale position
- Bale orientation
- Tractor alignment
- Fork alignment
- Pickup accuracy

The simulation should eventually include:

- Single bales
- Multiple bales
- Different bale positions
- Different bale orientations
- Field environments
- Unloading or storage locations

Round bales may be added later.

## 10. Loader and Bale Fork

The tractor must eventually manipulate the bale instead of only driving toward it.

A simplified loader model will therefore be added to Gazebo.

The loader will initially include two main motions:

- Loader lift
- Attachment or fork tilt

The model should include:

- Loader arms
- Fork attachment
- Joint limits
- Joint velocity limits
- Collision geometry
- Bale contact

The first version does not need a detailed hydraulic physics model.

A simplified actuator model is sufficient for early autonomy development.

## 11. Real Tractor Platform

The simulation will eventually be connected to a real tractor platform.

Potential equipment includes:

- Case IH Maxxum 150
- CNH L115 loader
- AccuGuide
- ISOBLUE
- CAN bus access
- RTK-GNSS
- RGB or RGB-D cameras

CAN bus communication may provide access to tractor information and control signals such as:

- Vehicle speed
- Steering
- Engine states
- Vehicle states
- Loader commands
- Hydraulic commands

The exact CAN signals that are available and controllable still need to be verified experimentally.

## 12. Development Roadmap

The planned development sequence is:

1. Basic tractor simulation
2. Accurate tractor geometry
3. Sensor simulation
4. Bale environment
5. Loader and fork model
6. Vehicle and loader kinematics
7. Real tractor CAN logging
8. Simulation and real-world data alignment
9. Perception
10. Operator demonstration collection
11. Imitation learning
12. Autonomous bale handling
13. Sim-to-real validation

## 13. Operator Demonstrations

Human operators can be used to demonstrate how the tractor should approach and manipulate a bale.

Demonstrations may initially be collected using keyboard or joystick teleoperation in simulation.

Later, real tractor operator data may also be collected through CAN bus and sensors.

Recorded data may include:

- Timestamp
- Tractor pose
- Tractor velocity
- Steering angle
- Camera images
- Depth images
- Point clouds
- Bale pose
- Loader joint state
- Operator steering command
- Operator speed command
- Hydraulic commands

These demonstrations can be used both for system analysis and learning-based control.

## 14. Simulation and Real Data Alignment

Simulation and real-world data should eventually use a common data representation.

Important elements include:

- Timestamps
- Coordinate frames
- Control commands
- Vehicle states
- Sensor observations
- Loader states

For example, the simulation may provide:

- Camera images
- Simulated tractor pose
- Steering commands
- Loader commands

The real tractor may provide:

- Camera images
- RTK position
- CAN steering information
- CAN hydraulic information

Both should be converted into a common format.

This will allow the same perception, training, and analysis pipeline to work with both simulated and real data.

## 15. Perception

The tractor needs to understand where the bale is relative to the tractor.

Perception tasks may include:

- Bale detection
- Bale segmentation
- Bale position estimation
- Bale orientation estimation
- Distance estimation
- Fork-to-bale alignment estimation
- Obstacle detection

The perception system may use:

- RGB images
- RGB-D images
- 3D point clouds
- Sensor fusion

The first target should be reliable bale localization and relative pose estimation rather than a very complicated perception system.

## 16. Imitation Learning

One possible approach is to use imitation learning to learn some tractor behaviors from operator demonstrations.

Possible inputs include:

- Camera images
- Depth
- Point clouds
- Tractor state
- Bale relative pose

Possible outputs include:

- Steering
- Vehicle speed
- Loader lift command
- Fork tilt command

Possible learning methods include:

- Behavioral cloning
- Sequence models
- Transformer-based policies
- Diffusion policies

The first learning problem should remain simple.

A reasonable first milestone is to learn bale approach and alignment before attempting the entire pickup and transport task.

## 17. Autonomous Behavior

The complete system does not need to use end-to-end learning.

A hybrid architecture is likely more practical.

The full task can be divided into states such as:

- Search
- Detect bale
- Approach
- Align
- Insert fork
- Lift
- Transport
- Align for unloading
- Lower bale
- Release bale

Different components may use different methods.

For example:

- Bale detection can use computer vision
- Navigation can use classical path planning
- Approach can use feedback control
- Pickup can initially use a scripted controller
- Transport can use autonomous navigation
- Fine alignment may later use imitation learning

This allows the system to remain understandable and easier to debug.

## 18. Sim-to-Real Strategy

A major purpose of simulation is to reduce the amount of expensive and time-consuming real tractor experimentation.

The general pipeline is:

1. Develop perception in simulation
2. Develop control in simulation
3. Generate demonstrations
4. Train policies
5. Test robustness
6. Deploy to the real tractor
7. Collect real-world data
8. Recalibrate or fine-tune the system

Possible sim-to-real techniques include:

- Sensor noise
- Geometry randomization
- Lighting variation
- Bale position randomization
- Terrain variation
- Control latency
- Actuator noise

## 19. Research Questions

Potential research questions include:

### RQ1: What sensing configuration is sufficient for reliable bale localization and alignment?

Possible comparison:

- RGB-D versus 3D point cloud versus multi-camera sensing

### RQ2: Can operator demonstrations be used to learn robust bale approach and alignment behaviors?

### RQ3: How much real-world data is required if the system is initially developed in simulation?

### RQ4: Which parts of autonomous bale handling should be learned, and which parts are better handled using classical control and state machines?

## 20. Evaluation Metrics

Possible navigation metrics include:

- Distance to bale
- Heading error
- Trajectory error
- Travel distance
- Task completion time

Possible alignment metrics include:

- Lateral alignment error
- Yaw alignment error
- Fork insertion error

Possible manipulation metrics include:

- Pickup success rate
- Drop-off success rate
- Bale displacement
- Failed contact events

Possible learning metrics include:

- Success rate
- Number of demonstrations required
- Sim-to-real performance degradation
- Amount of real-world fine-tuning data required

## 21. Current Milestones

### Milestone 1: Basic Tractor Simulation

Completed:

- [x] Tractor model
- [x] Gazebo simulation
- [x] Basic steering
- [x] `cmd_vel` control
- [x] Keyboard teleoperation

### Milestone 2: Add Sensors

To do:

- [x] Front RGB camera
- [x] Complementary side camera in autonomy trials
- [x] RGB-D camera
- [x] Depth-to-point-cloud processing inside pose estimation
- [ ] RTK-GNSS simulation
- [x] Camera/model/rear-axle frame checks for the flat-ground baseline

### Milestone 3: Build Bale Environment

To do:

- [x] Square bale model
- [x] Multiple bale poses
- [ ] Field scenario
- [ ] Storage or drop-off location

### Milestone 4: Add Loader and Fork

To do:

- [x] Loader arm
- [x] Bale grip attachment (fork retained as a test fixture)
- [x] Loader joints
- [x] Joint control
- [x] Bale collision and contact

### Milestone 5: Build Data Pipeline

To do:

- [x] Record simulation data
- [x] Define versioned observation/label dataset structure
- [ ] Record CAN data
- [ ] Record real sensor data
- [ ] Align simulation and real-world data

### Milestone 6: Autonomous Bale Approach

Implemented for the bounded simulated baseline; see the measured results and
remaining visibility, localization and maneuvering limitations in the autonomy docs.

- [x] Geometric RGB-D bale detection
- [x] Bale pose estimation and multi-view/temporal tracking
- [x] Curvature-constrained approach trajectory
- [x] Sensor-guided terminal pre-grasp alignment
- [x] Automatic stopping and measurement-loss abort

### Milestone 7: Autonomous Pickup

Implemented in the rigid-bale simulation baseline:

- [x] Grip alignment and positioning around the bale
- [x] Contact-only jaw closure
- [x] Loader lift with attachment leveling
- [x] Pickup verification from jaw feedback and RGB-D motion

### Milestone 8: Full Task

To do:

- [x] Pickup in bounded simulated cases
- [x] Loaded transport to a ground drop zone
- [x] Ground placement, release and withdrawal
- [x] Three-bale stacking in one simulated run; see measured alignment limits in [stacking results](docs/stacking.md)
- [ ] Sim-to-real testing

## 22. Immediate Next Steps

The current priority is the separate spear-handling physics project:

1. Correct parent/link world transforms and measure penetration in bale-local coordinates.
2. Diagnose zero contact callbacks and validate each friction interaction experimentally.
3. Strengthen phase gates and acceptance checks, then rerun normal insertion and lifting.
4. Test shallow/misaligned insertion before stacking, withdrawal, two-bale lifting, and dynamic stability.
5. Integrate perception and planning after the physical handling model is validated.

The older jaw autonomy baseline still needs better stack alignment, visibility,
and broader scenario coverage; see [its recorded results](docs/stacking.md).

The current experimental autonomous demonstration is:

> A simulated tractor uses onboard RGB-D, wheel/IMU odometry, jaw feedback and hydraulic encoders to pick three rigid bales and stack them. One recorded run completed all three placements; its adjacent layer centers were 0.330 m and 0.345 m apart, slightly above the current 0.30 m nominal alignment limit.

Imitation learning and full bale manipulation should come after this basic perception and control pipeline is working.

## 23. Long-Term Goal

The final system should demonstrate a real agricultural tractor autonomously detecting, approaching, picking up, transporting, and unloading a bale.

The perception and control system should be developed primarily in simulation and later validated on the real tractor.

The broader objective is to investigate how simulation, operator demonstrations, perception, and real agricultural machinery interfaces can reduce the cost and difficulty of developing autonomous agricultural material-handling systems.
