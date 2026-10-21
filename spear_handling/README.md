# Tractor bale spear project

This is a separate development project for the requested four-spear attachment
and two-bale handling sequence. It reuses the existing tractor and L115-inspired
loader in `../worlds/simple_tractor.sdf`; it does not replace the jaw-grip demo.

## Current status — 2026-10-20

The C++ plugins build, the world generator and test runner exist, and trial A
has executed. **A failed acceptance:** the bale rose about 0.508 m, but the
insertion target was not reached according to faulty frame-mixed feedback and
all custom contact counters remained zero. Pair-specific friction is unverified.
B–F and optional insertion resistance have not been tested.

The local evidence is `runs/spear_20261020_214150_A/report.json` and its trajectory.
The report records `passed=false`, `insertion_reached=false`, `bale_raised=true`,
and shutdown `-9`; the server logged an allocator error. Raw runs are git-ignored.
See [the comprehensive handoff](../HANDOFF.md) for measurements and debugging priorities.

From the repository root:

```bash
cmake -S spear_handling -B spear_handling/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build spear_handling/build -j 2
python3 spear_handling/validate.py --case A
```

Use system Python with Gazebo bindings. The runner uses an isolated partition
and simulator truth for experimental control. `--case B`, `--case C`, and
`--case sequence` exist but are unvalidated. Fix coordinate-frame handling,
contact callbacks, and stage acceptance gates before attempting the sequence.

## Inspection completed

- Installed Gazebo Sim: 8.15.0; gz-physics: 7.8.0; DART: 6.13.2;
  SDFormat: 14.9.0.
- Reusable loader chain: `chassis -> loader_mount -> loader_boom ->
  attachment_carrier`. Keep `loader_lift_joint` and `attachment_tilt_joint`.
- Reuse `../src/LoaderHydraulics.cc` and `../src/HydraulicModel.hh`, their
  independent valve topics, joint feedback, timeouts, shared flow allocation,
  force limits, and PID servo with anti-windup.
- Existing lift limits: 0–1.44 rad, 0.20 rad/s; tilt limits: -0.65–0.85 rad,
  0.35 rad/s. Preserve the source effort limits and hydraulic configuration.
  The existing pressure/cylinder values are documented engineering assumptions,
  not measured L115 specifications.
- Existing fork fixture: `../tests/fixtures/square_bale_fork.sdf`. Its two
  1.3 m tines support a prepositioned bale; it does not validate penetration.
- Existing production bales are 2.4 x 1.2 x 0.9 m, 525 kg solid boxes. They
  need a separate configurable geometry for this project's field estimates.
- Existing recording, isolated Gazebo partitions and hydraulic test utilities
  can be reused. Jaw-grip pickup success does not validate spear pickup.

## Design and implementation notes

1. Generate a separate world from the baseline tractor. Replace the jaw
   attachment with a four-spear carrier and remove its unused jaw actuation.
   Recalculate attachment mass and inertia. Preserve tractor motion, loader
   joints, hydraulic limits and existing command interfaces.
2. Generate independent dynamic bales from configurable dimensions and mass.
   Start with length 2.4384 m, width 1.2192 m, height 0.9144 m and mass 200 kg.
   Compute uniform-box inertia analytically from those values, independent of
   the collision-channel void volume. These are preliminary field estimates;
   450 lb is approximately 204 kg, so 200 kg remains a rounded initial value.
3. Mount four spears across a 1.9 m frame, at lateral positions approximately
   -0.85, -0.2833, 0.2833 and 0.85 m. Initial exposed length: 1.0 m; shaft
   diameter: 0.04 m. Current collision and visual shafts are blunt cylinders; a tapered tip remains future work. These are adjustable engineering
   starting values, not manufacturer dimensions or structural ratings.
4. Approach the 8 x 3 ft long side of an upright bale. The four spears run
   across the tractor's lateral axis and penetrate the 1.2192 m bale width.
   A 60% insertion target is then 0.7315 m. If the bale is rolled onto its
   other long face, insertion thickness becomes 0.9144 m and the same target
   becomes 0.5486 m. Compute penetration from geometry rather than a fixed
   distance or an automatic successful-grasp threshold.
5. First evaluate four hidden, open-ended internal channels built from compound
   convex collisions on one rigid bale link. Retain full visual geometry and
   exterior ground/stack contact surfaces. Channel ceilings can supply upward support through actual collision contact;
   floors and walls constrain motion. No bale attachment
   joint is used in this first prototype.
6. Empty channels have no straw insertion resistance. Evaluate that baseline
   separately, then add a configurable, dissipative axial resistance model if
   needed. Apply equal and opposite forces to spear and bale at the engagement
   points, with depth/velocity dependence and force caps. It must not supply
   arbitrary upward lifting force. If channels cannot provide stable support,
   document and separately evaluate a bounded compliant engagement model with
   force/moment limits and release on withdrawal; never weld the upper bale.
7. Tying-band visuals are implemented. Straw texture remains future work.
   Visual changes do not alter collision geometry or inertial properties.

## Friction and major risks

- Configure bale-ground, bale-spear and bale-bale interactions independently.
  Per-surface SDF coefficients alone must not be presented as arbitrary
  independently selectable pair coefficients. Inspect DART's coefficient
  combination and validate the installed contact-property customization path
  before choosing how to implement pair-specific settings. Installed Gazebo
  headers expose contact-surface customization and gz-physics contact callbacks;
  trial A produced zero callback counts, so the overrides remain unverified.
- Channels impose artificial alignment and clearance tolerances, and their
  walls can create unrealistic mechanical retention. Sweep channel clearance,
  spear angle and timestep before treating successful lifting as meaningful.
- Thin spears may cause missed contacts, jitter or solver sensitivity. Start
  from the existing 1 ms step, and check convergence rather than hiding failures
  with large friction coefficients.
- Lowering and downward tilt increase ground friction only if actual contact
  produces additional normal load. Log contact forces and bale motion.
- Tractor drive currently uses wheel-velocity commands. It is not a calibrated
  engine/drivetrain traction model; successful insertion cannot establish real
  tractor pushing capability.
- Upper-bale stability must emerge from gravity and bale-bale contact. It stays
  a separate rigid body through placement, withdrawal, two-bale lifting and
  acceleration/deceleration.
- Exact insertion resistance, steel/straw friction, bale density distribution,
  real spear dimensions and hydraulic capacity require field calibration.

## Incremental validation

Run manual or scripted physical controls first, before adapting autonomous
perception and planning to long-side pickup. Record insertion depth, alignment,
joint state/effort, contact evidence and both bales' poses and velocities.
Simulator object poses are permitted as test measurements, not as evidence of
sensor-based autonomy.

| Scenario | Required observation | Status |
| --- | --- | --- |
| A: aligned, roughly 60% insertion | Single-bale pickup, hold, withdrawal | Executed; failed acceptance; withdrawal not tested |
| B: shallow insertion | Reduced support, slip or rotation where physically warranted | Not run |
| C: lateral/yaw/height error | Collision resistance and stability sensitivity | Not run |
| D: stack and withdraw | Bale A rests independently on Bale B | Not run |
| E: lift the lower bale | Both lift; upper bale has no attachment joint | Not run |
| F: forward/reverse acceleration | Measure sliding, tipping and any loss of load | Not run |

Repeat critical cases with different integration steps and contact/friction
settings. Report failures as well as successes. No spear scenario is currently
claimed to work. Optional axial resistance is implemented but disabled and
untested. The full handoff records additional configuration and validation gaps.

## References inspected

- Local loader documentation: `../docs/loader_hydraulics.md`.
- Local physics assumptions: `../docs/physics_parameters.md`.
- Gazebo DART SDF adapter matching the installed release:
  https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.8.0/dartsim/src/SDFFeatures.cc
- DART contact implementation matching the installed release:
  https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/ContactConstraint.cpp
