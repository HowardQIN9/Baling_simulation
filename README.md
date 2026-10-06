# Autonomous Bale Handling with a Tractor

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

- [ ] Front RGB camera
- [ ] Additional cameras
- [ ] RGB-D camera
- [ ] Point cloud
- [ ] RTK-GNSS simulation
- [ ] Coordinate frame validation

### Milestone 3: Build Bale Environment

To do:

- [ ] Square bale model
- [ ] Multiple bale poses
- [ ] Field scenario
- [ ] Storage or drop-off location

### Milestone 4: Add Loader and Fork

To do:

- [ ] Loader arm
- [ ] Fork attachment
- [ ] Loader joints
- [ ] Joint control
- [ ] Bale collision and contact

### Milestone 5: Build Data Pipeline

To do:

- [ ] Record simulation data
- [ ] Define common dataset structure
- [ ] Record CAN data
- [ ] Record real sensor data
- [ ] Align simulation and real-world data

### Milestone 6: Autonomous Bale Approach

To do:

- [ ] Bale detection
- [ ] Bale localization
- [ ] Approach trajectory
- [ ] Fine alignment
- [ ] Automatic stopping

### Milestone 7: Autonomous Pickup

To do:

- [ ] Fork alignment
- [ ] Fork insertion
- [ ] Loader lift
- [ ] Pickup validation

### Milestone 8: Full Task

To do:

- [ ] Pickup
- [ ] Transport
- [ ] Drop-off
- [ ] Sim-to-real testing

## 22. Immediate Next Steps

The current priority is:

1. Add cameras to Gazebo.
2. Add RGB-D and point-cloud sensing.
3. Add a square bale model.
4. Test whether the sensors can detect and localize the bale.
5. Measure the real tractor geometry.
6. Refine the tractor model.
7. Add the loader and bale fork.

The first useful autonomous demonstration should therefore be:

> A simulated tractor observes a bale using onboard sensors, estimates the relative bale position, and autonomously approaches and aligns with the bale.

Imitation learning and full bale manipulation should come after this basic perception and control pipeline is working.

## 23. Long-Term Goal

The final system should demonstrate a real agricultural tractor autonomously detecting, approaching, picking up, transporting, and unloading a bale.

The perception and control system should be developed primarily in simulation and later validated on the real tractor.

The broader objective is to investigate how simulation, operator demonstrations, perception, and real agricultural machinery interfaces can reduce the cost and difficulty of developing autonomous agricultural material-handling systems.
