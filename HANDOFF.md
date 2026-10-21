# Project handoff — 2026-10-20

## Publication update

After this handoff was written, the user requested README updates and upload to
GitHub. The root and spear-project READMEs now reflect trial A's failed acceptance
and link here. Statements below about README staleness describe the handoff-time
snapshot; those documentation discrepancies were subsequently corrected. No
simulation fixes or new physical validation were performed for publication.

## Session state and scope

Workspace: `/home/zhihao/bale_sim`. Work is **paused at the user's request**.
The final requested action was this handoff, with no implementation edits.
This document was checked against the current source, generated SDF, and saved
run artifacts. No additional simulation was launched for this handoff.

The latest priority is a separate **tractor-mounted four-spear bale-handling
prototype** in `spear_handling/`. It is a subproject in this repository, not a
separate repository or a self-contained copy: it depends on the parent tractor
world and hydraulic sources. Preserve the older jaw-grip autonomy implementation.

The first spear trial completed in the background around the interruption.
Its final report is present and **fails acceptance**. A targeted process check
found no matching `python3 spear_handling/validate.py` / spear `gz sim` process;
this is not a claim that every unrelated Gazebo process has been stopped.

## Goals and user constraints

- Keep the existing tractor; this is not a telehandler.
- Reuse the existing L115-inspired loader joint chain and hydraulic controller.
- Model large square bales as independent dynamic rigid bodies: preliminary
  field estimates 2.4384 m long × 1.2192 m wide × 0.9144 m high, 200 kg.
  The user's 450 lb estimate is approximately 204 kg; 200 kg is rounded.
- Four spears across an initial 1.9 m frame, entering the long 8 × 3 ft face.
  Default penetration thickness is 1.2192 m. A rolled bale would require a
  different orientation/thickness; the current generator implements upright bales.
- Reproduce pickup A, place A on B, withdraw, insert into B, lift and transport
  both. The upper bale must remain supported through collision and friction,
  with no fixed attachment between the bales.
- Use rigid collision channels first; document approximations and failures.
  Detailed straw deformation/fibers, reinforcement learning, and neural models
  are not part of the present prototype.
- Independently configure bale–ground, bale–spear, and bale–bale friction.
  Runtime effectiveness must be measured, not inferred from configured values.
- Validate normal insertion, shallow insertion, misalignment, stacking,
  two-bale lifting, and acceleration/braking/reversing (A–F).
- User permits autonomous work and GPU use, but asked not to remove dependencies.
  Earlier data-storage budget: 20 GB or 40% of available total, whichever is
  the applicable smaller limit. The new spear logger does not yet enforce it.
- User requested modest visual realism; later Unreal testing remains possible.

## Architecture and reusable components

### Existing tractor and hydraulics

`worlds/simple_tractor.sdf` remains the source world. The loader chain is
`chassis -> loader_mount -> loader_boom -> attachment_carrier`.
The new generator retains `loader_lift_joint` and `attachment_tilt_joint` and
removes the jaw links, jaw joints, and jaw-specific controller entries only
from the generated copy.

| Joint | Position limits | Velocity limit | SDF effort limit |
| --- | --- | --- | --- |
| Lift | 0 to 1.44 rad | 0.20 rad/s | 50,000 Nm |
| Tilt | -0.65 to 0.85 rad | 0.35 rad/s | 15,000 Nm |

`src/LoaderHydraulics.cc` and `src/HydraulicModel.hh` provide the reused hydraulic
model: shared flow allocation, valve/spool behavior, command timeout, PID
actuation with effort/pressure limiting, and holding behavior. The existing
86 L/min pump and 200 bar relief configuration are engineering assumptions,
not verified L115 manufacturer specifications. Actual hydraulic effort can
be below the SDF maximum.

- Lift command: `/tractor/loader/lift/valve` (`gz.msgs.Double`).
- Tilt command: `/tractor/loader/tilt/valve` (`gz.msgs.Double`).
- Feedback: `/tractor/loader/{lift,tilt}/state` (`Double_V`):
  `[q, qdot, reference, effort, equivalent_pressure_bar, flow_L_per_min, sim_time]`.
- Drive: `/model/simple_tractor/cmd_vel` (`Twist`), existing Ackermann system.
  Source world limits speed to ±2 m/s and acceleration to ±1 m/s².
  Wheel-velocity control is not a calibrated drivetrain/traction model.

Previously inspected environment: Gazebo Sim 8.15.0, gz-physics 7.8.0,
DART 6.13.2, SDFormat 14.9.0. The current CMake explicitly requires gz-sim8,
gz-plugin2/register, gz-transport13, and gz-physics7. The successful build and
actual simulation establish that these interfaces are available; exact package
versions were not re-queried during this handoff.

### New project files

| File | Actual implementation |
| --- | --- |
| `spear_handling/CMakeLists.txt` | Separate C++17 build. Compiles the existing hydraulic source into `libBaleLoaderHydraulics.so` and the new `libSpearContact.so`. |
| `spear_handling/config.json` | Dimensions, mass, spear geometry, channel geometry, pair-friction requests, optional resistance, timestep, and test settings. |
| `spear_handling/generate.py` | Parses the source world, removes cameras/Sensors for physics tests, replaces jaws with four cylindrical spears, creates two dynamic bales, adds contact plugin. |
| `spear_handling/src/SpearContact.cc` | Attempts pair-specific contact friction; optional equal/opposite axial insertion drag; publishes contact/drag telemetry. |
| `spear_handling/validate.py` | Isolated Gazebo runner with cases `A`, `B`, `C`, `sequence`, optional GUI, truth-based control, trajectory and report output. |
| `spear_handling/README.md` | Original design/inspection notes. Its “Not run” table is now stale for A; use the evidence in this handoff. |
| `spear_handling/.gitignore` | Ignores local build and Python cache files. |

The generator embeds absolute plugin library paths. Regenerate worlds after
moving the repository. It writes under `runs/` by default and does not overwrite
the production world.

### Geometry and physical approximation

Bale local axes: X = penetration width, Y = long bale axis, Z = height.
Each bale has a single `bale_link`, mass 200 kg, full-box analytic inertia,
and **seven box collision components**: lower and upper slabs plus five webs
between the four channels. Generated bales have no joints. The collision voids
do not subtract from the configured mass or uniform-box inertia.

Default channels are 0.10 m wide × 0.14 m high, centered 0.36 m above the bale
bottom, and open through both X faces. Top/bottom exterior surfaces remain
intact. During lifting, the spear's upper surface can bear against the channel
ceiling. Channel walls may create artificial retention and alignment sensitivity.
Empty channels do not reproduce real straw penetration resistance.

Default spear centers across Y are -0.85, -0.283333, +0.283333, +0.85 m.
Shafts are 1.0 m long and 0.04 m diameter, rooted at carrier local X=0.10,
Z=0.15 m. The frame mass is assumed 95 kg; shaft mass uses assumed steel
density 7850 kg/m³. Composite inertia includes the center-of-mass shift and
parallel-axis terms. These are configurable engineering starting values.
Current shafts are blunt cylinders in both visual and collision geometry;
the README's proposed tapered tip has not been implemented.

Bale visuals are solid straw-colored boxes with thin dark visual ties.
There is no straw texture yet. Ties have no collision or mass contribution.
Bale A starts centered at (4.8096, 0, 0.4592) m; B at (8.5, 0, 0.4592) m.
The slight initial gap lets both settle under gravity.

### Contact plugin

The plugin classifies collision entities by scoped names / spear collision
prefixes, creates `EnableContactSurfaceCustomization`, and connects to
`CollectContactSurfaceProperties`. It requests both primary and secondary
friction coefficients of 0.4 (ground), 0.3 (spear), and 0.4 (other bale).
**These overrides are not validated: all callback counters were zero in A.**
Do not claim that the observed lifting used these coefficients.

Optional resistance is disabled by default. If enabled, each finite spear
segment is clipped against the bale's visual box. At the overlap midpoint,
axial drag opposes relative axial velocity and applies equal/opposite forces
at the same world point to carrier and bale. Magnitude is capped at 200 N per
spear, using 100 N/m overlap and 30 N/(m/s) velocity coefficients. It supplies
no explicit grasp joint or success threshold. For tilted spears the axial
force can have a vertical component. This code has not been runtime validated.
It does not model static straw retention at zero velocity.

`/spear/contact/state` is published at approximately 20 Hz:
`[sim_time, ground_callback_count, spear_callback_count, bale_callback_count,
 summed_drag_N, requested_ground_mu, requested_spear_mu, requested_bale_mu]`.
Counters are cumulative callback counts, not measured contact forces/impulses.
No normal-force measurement has been implemented.

### Validation controller

The runner uses system `python3` with `gz.transport13` / `gz.msgs10`, a unique
`GZ_PARTITION`, and `gz sim --physics-engine gz-physics-dartsim-plugin`.
It records config, generated world, server log, JSONL trajectory, and JSON report
under timestamped `runs/spear_*` directories. It uses simulator poses for
feedback: this is a physics experiment, not sensor-based autonomous handling.

Outer joint control is proportional, with gain 8, valve cap 0.65, 0.003 rad
error dead zone, and minimum nonzero valve command 0.09. The reused hydraulic
plugin contains the inner PID. The outer runner is not itself a tuned PID.
Stages have fixed deadlines and mostly hard-coded lift angles/speeds.
The intended D/E/F sequence is coded but has never executed.

## Completed work and actual results

1. Inspected the original tractor, L115-inspired hydraulics, existing fork
   fixture, and installed Gazebo contact APIs.
2. Created the separate configurable spear project and generated world.
3. Fixed a missing `gz/sim/components/Link.hh` include in the contact plugin.
4. Successfully ran `cmake --build spear_handling/build -j 2`; both shared
   libraries built. No implementation changes followed that successful build
   except creation of the Python generator/runner in the same work period.
5. Launched one real headless DART trial A after a sandbox networking failure.
   Ground settling, insertion motion, lifting, and holding were recorded.
6. Preserved all older uncommitted autonomy work; no commit or push was made.

### Spear test evidence

- `runs/spear_20261020_214140_A/`: generated config/world only. Launch attempt
  failed with `error in getifaddrs: Unknown error -1` inside the sandbox.
- `runs/spear_20261020_214150_A/`: actual run, 637 valid trajectory samples,
  `report.json`, config/world, and server log. Report: `passed=false`,
  `insertion_reached=false`, `bale_raised=true`, `server_exit=-9`.

| Stage | Sim time | Bale A center X | Bale A center Z | Reported depth |
| --- | ---: | ---: | ---: | ---: |
| Settled | 2.1 s | 4.8096 m | 0.457199 m | -0.239324 m |
| Insertion timeout | 20.1 s | 6.225048 m | 0.457200 m | -1.654305 m |
| Stopped | 21.1 s | 6.238023 m | 0.457200 m | -1.667282 m |
| Lift | 29.1 s | 6.447612 m | 0.811556 m | -1.680491 m |
| Hold | 32.1 s | 6.466475 m | 0.965653 m | -1.676696 m |

The bale rose about 0.508 m above its settled center height, but this does
**not** validate pickup at 60% insertion. The tractor drove about 2.696 m and
pushed the bale substantially. The measured depth is inconsistent with the
moving tractor and cannot be treated as physical penetration evidence.
All three custom contact counters remained zero throughout the saved trial.

The server log contains `malloc_consolidate(): unaligned fastbin chunk detected`.
The runner eventually used its SIGKILL fallback (`-9`). A similar teardown
symptom is recorded in older autonomy runs; the new plugin has not been
exonerated as a contributor to this run's shutdown problem.

| Requested scenario | Verified status |
| --- | --- |
| A: normal insertion/lift | Executed; acceptance failed. Some physical lifting observed. |
| B: shallow insertion | Code exists; not run. |
| C: misalignment | Lateral-offset case exists; not run. No angular/height sweep yet. |
| D: stack and withdraw | Sequence code exists; not run. |
| E: lift both bales | Sequence code exists; not run. |
| F: dynamic stability | Basic forward/brake/reverse stages exist; not run. |
| Independent pair friction | API attempted; runtime behavior unresolved. |
| Optional insertion resistance | Implemented, disabled, not tested. |

## Known issues and debugging priorities

1. **Pose frame mismatch in `validate.py` is strongly indicated.** The callback
   treats `attachment_carrier` pose from scene Pose_V as world coordinates.
   At insertion end, tractor X is 2.68327 m while carrier X remains 2.85723 m,
   its original model-relative location. `depth()` mixes that link pose with
   a world bale pose. Compose parent-model and link transforms, or publish
   explicit world poses. Verify frame semantics and IDs instead of relying on
   name-only selection. The current code also ignores bale orientation when
   computing depth and uses only a centerline spear rather than all four.
2. **Contact customization has no observed callbacks.** Check actual scoped
   name formatting, classification, event timing, component discovery, and DART
   feature support. `scopedName()` is called without an explicit delimiter while
   matching `::` strings; verify its default. Inspect collision entity IDs in
   callback arguments. Do not fix this by arbitrarily raising friction.
3. **Acceptance checks are too weak for a credible deliverable.** B/C set
   `passed=true` merely when stages finish; this means experiment execution,
   not desired physical failure behavior. Stack checks largely use vertical
   center separation, without adequate lateral overlap, tilt, velocity,
   withdrawal, sustained-support, or force-transfer criteria.
4. **Phase timeout does not reliably gate subsequent actions.** A continues
   to lift after insertion misses the target. Lower-bale insertion and stack
   alignment likewise need explicit success/failure checks before progressing.
5. **Config coverage is incomplete.** `test.forward_tilt` and
   `test.transport_speed` are present but not read by the runner. B's X=8.5 m,
   stage timings/angles, frame depth/height, and several motion settings are
   hard-coded. Input validation is only a few Python asserts; invalid count,
   mass, dimensions, channel/frame fit, friction, or force limits need handling.
6. **Controller stability needs work.** The trace shows appreciable tilt speed
   during hold. No repeatability or timestep/channel-clearance convergence has
   been established. Fix measurements before tuning outer-loop gains.
7. **Lifecycle/logging robustness:** callbacks update shared dictionaries
   without synchronized snapshots; frame IDs/timestamps are not preserved.
   Startup/Popen failures before the try block and external termination can
   leave incomplete records. Logs are buffered. Dataset storage guard is absent.
8. **Physical credibility limits:** channels constrain alignment and may act
   like mechanical sockets; straw compliance, insertion resistance, density
   variation, spear taper, and real actuator capacity remain uncalibrated.
   Frame–bale contact is not given a dedicated pair classification. No test
   yet isolates support from shafts versus frame contact.
9. `spear_handling/README.md` describes future features and pre-test status;
   update it after the next validated implementation, not by assuming success.

## Existing autonomy project: preserve separately

The older implementation is in `bale_autonomy/`, exposed through
`scripts/bale_autonomy.py`, with `tests/test_autonomy.py` and documentation in
`docs/autonomous_approach.md`, `docs/pick_place.md`, and `docs/stacking.md`.
Modules include scene generation, transport, recording, perception,
localization, planning, approach control, manipulation, stacking, and visualization.
It uses the original jaw attachment and 525 kg box bales, not this spear model.
Earlier work covers RGB-D collection/fusion, geometric planning, pickup and
right-side ground placement, and three-bale stacking. It is a conventional
control/perception baseline; GPU rendering is distinct from learned inference.

The saved `datasets/stack_check_66/stack.json` was checked during this handoff:
three cycles and verified pickups, `success=true`, but `acceptance_pass=false`.
Final drop-center errors: 0.249764, 0.083097, 0.425961 m. Neighbor offsets are
about 0.330 and 0.345 m, exceeding the documented 0.30 m threshold. Duration
768.52 sim seconds, 6044 frames, shutdown -9. Thus a physical three-layer run
exists, but it did not pass all acceptance criteria. Do not present it as a
validated spear-handling result. Older `docs/pick_place_results.json` contains
two accepted jaw ground-placement runs; these were read, not rerun here.

## Files and working-tree preservation

Current tracked modifications already present before this handoff:
`.gitignore`, `README.md`.

Untracked work present before this handoff:
`bale_autonomy/`, `docs/assets/`, `docs/autonomous_approach.md`,
`docs/autonomy_results.json`, `docs/autonomy_results.md`, `docs/pick_place.md`,
`docs/pick_place_results.json`, `docs/stacking.md`, `requirements-autonomy.txt`,
`scripts/bale_autonomy.py`, `spear_handling/`, `tests/test_autonomy.py`,
`worlds/autonomy_gui.config`.

Root ignore rules cover `/build/`, `/.venv/`, `/datasets/`, `/runs/`, Python
caches and bytecode. Spear build output is separately ignored. Raw evidence
will not be included in a normal git commit: retain it locally and later copy
small reviewed summaries into versioned documentation. Do not reset or clean
this working tree. This final documentation task adds only `HANDOFF.md`.

## Recommended resumption order

1. Read the actual A report and inspect parent/link coordinate frames first.
   Correct world transforms and compute per-spear segment overlap in bale-local
   coordinates, including yaw/pitch/roll. Test the geometry independently.
2. Diagnose zero contact callbacks with a minimal friction experiment. Verify
   each intended pair coefficient through measured sliding/deceleration, then
   confirm actual spear contact evidence. Add force measurement if needed.
3. Harden phase gates and acceptance/report semantics. Keep geometry/support
   measurements distinct from actuator commands and configured parameters.
4. Rerun A from rest with correct depth feedback, roughly 60% insertion,
   controlled lift, hold, and withdrawal. Measure bale displacement and tilt.
5. Run B/C and clearance/timestep variations. Report unexpected shallow support
   honestly; do not encode a universal insertion-success switch.
6. Execute D, then E, then F incrementally, checking independent upper-bale
   support after full withdrawal and during transport. Add lateral/tilt/stability
   checks and record real failures as well as successes.
7. Evaluate optional resistance separately, including reaction-force balance
   and dissipative behavior. Calibrate coefficients from field evidence later.
8. Improve visuals, configuration validation, storage/lifecycle handling, and
   reproducible documentation. Integrate perception/planning only after the
   physical handling model has credible validation.

Commands for a future resumed session (not run during this handoff):

```bash
cd /home/zhihao/bale_sim
cmake -S spear_handling -B spear_handling/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build spear_handling/build -j 2
python3 spear_handling/generate.py --output runs/spear/world.sdf
python3 spear_handling/validate.py --case A
# Only after A and its measurement/contact issues are resolved:
python3 spear_handling/validate.py --case B
python3 spear_handling/validate.py --case C
python3 spear_handling/validate.py --case sequence
```

Use system Python for the Gazebo bindings. `--gui` is supported by the runner
but has not been tested for this project. Gazebo transport failed inside the
sandbox; use the tool's escalation flow when required. The specific
`python3 spear_handling/validate.py` prefix was approved in this session.
Do not broadly kill Gazebo instances: use the runner's isolated partition and
its own process group, preserving unrelated user simulations.
