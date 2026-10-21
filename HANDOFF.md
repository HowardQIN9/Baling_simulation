# Project handoff — updated 2026-10-21

## Current material-model development — 2026-10-21

This section supersedes the channel-model status and older documentation-only
snapshots below. The user subsequently authorized implementation of the discussed
bale behavior. This material-model update, preceding spear fixes, validation summaries and
documentation are included together in the GitHub publication requested by the
user. Old channel results are archival evidence only.

### Goal and operator evidence

Retain the tractor, L115-inspired loader and four-speared implement, replacing
artificial entry channels with a physically motivated local penetration/support
interaction. Operator videos and explanations describe easy penetration without
large visible deformation, slightly lowering the boom to avoid unloading/pushing
the bale, carrying at modest angles, and pressing down during placement so
support friction lets the shafts withdraw. A trailer deck acts as a support like
ground. Review and exact local video identifiers are documented in
`docs/field_video_review.md`; video files remain in Downloads.

### Architecture and current implementation

- The default config selects `interaction.mode = material`. Each 200 kg bale
  has one full box collision, with no entry channels. Overall shape/inertia stay
  rigid; this is a reduced-order local material model, not a deformable straw
  mesh. The original channel setup remains in `config_channels.json`.
- `EmbeddedSpear.hh` implements 32 sampled material cells per shaft passage.
  Virgin cells initialize at actual entry positions. Covered length determines
  transverse spring/damping bearing and bounded yielding. Axial bristle friction
  is capped by confinement plus bearing load; cutting resistance acts only on
  virgin advancement. Withdrawal progressively removes support; full exit clears
  the passage. No weld or insertion-percentage gate exists in the material law.
- `SpearContact.cc` discovers links, publishes explicit world poses, overrides
  native pair friction and integrates per-bale/per-shaft material states. It
  applies balanced implement/bale forces at the same world point using
  `AddWorldWrench`, including moments about each link origin. Verified code
  semantics differ from the COM-relative position accepted by AddWorldForce;
  the older optional drag path was corrected accordingly.
- Native shaft/bale contact alone is filtered (categories/masks 4/5 and 2/3).
  Ground, frame and bale/bale collisions remain native. Material trials record
  zero native shaft/bale callbacks, while material forces carry the payload.
- `geometry.py` computes transformed finite-shaft overlap, full-body entry
  clearance, and whole-bale clearance/tilt. Legacy channel diagnostics remain
  available but do not gate material entry.
- `validate.py` orchestrates isolated Gazebo trials with synchronized telemetry,
  command connection/setup checks, phase deadlines and physical acceptance
  guards. It uses simulator truth and material force telemetry, without camera
  perception or a real-machine interface. Raw runs retain config, generated
  world, source/binary hashes, telemetry, report and shutdown status.
- Insertion partially engages, lowers until downward material force exceeds
  600 N, then follows the downward shaft slope. Holding height while driving
  horizontally can raise an existing passage and unload/push the bale. Placement
  presses until at least 300 N downward force, then holds achieved height during
  withdrawal. Omission switches support matched comparisons at unchanged mu.
- Tilt feedforward compensates boom velocity during lifting. Current carry angle
  is +0.06 rad; entry angle is -0.03 rad. Combined-load lift is slower and checks
  stack alignment/tilt throughout. Travel checks lower-bale bottom/engagement,
  both tilts and stack separation. These are experimental controller settings,
  not calibrated limits for real bales.

### Verified evidence and failures

Selected raw directories under `runs/` (git-ignored):

| Trial | Result |
| --- | --- |
| `spear_20261021_014929_sequence_64a585` | Full material sequence passed: pickup A, stack on B, withdraw fully, enter B, lift both, forward/brake/reverse/hold. About 149 sim seconds. Withdrawal moved A by 0 m at saved precision. Upper bale receives no direct material support during travel. Shutdown -9. |
| `spear_20261021_114531_sequence_fc4cbc` | Independent repeat passed in 148.8 sim seconds with identical source/binary hashes to the prior full success. Withdrawal moved A by 0 m at saved precision; preload was -416.4 N. Shutdown -9. |
| `spear_20261021_015514_A_2f53e4` | Same final material parameters, insertion correction omitted: push exceeded 0.15 m; lift blocked. |
| `spear_20261021_015235_release_d1e180` | Same final material parameters, release preload omitted: upper bale followed by 0.1534 m and withdrawal stopped. Initial upward spear force was about 1,630 N. |
| `spear_20261021_015613_C_74069e` | Lateral offset +0.12 m and entry angle -0.06 rad passed pickup/retention. This demonstrates entry without channel alignment, not arbitrary-angle capability. |
| `spear_20261021_114355_B_edd1a2` | Final shallow target 0.24384 m reached; lift/retention failed. Center rose 0.2261 m and overlap fell to about 0.067 m. No special shallow-failure rule is imposed in the material law. |
| `spear_friction_20261021_014535_42319f` | All nine native pair-friction fixture checks passed independently of material calibration. Shutdowns -9/-11/-9. |

Earlier weak material parameters supported one bale but failed combined loading
(`014046_sequence_6915dd`). Increasing stiffness/capacity alone still tripped the
tilt guard (`014422_sequence_db73a5`); tilt tracking and slower lift were also
needed. Failed trials remain in the results record. Two concurrent development
runs (`014605_C_588ab5`, `014606_A_559fa8`) showed the lift encoder staying at its
lower stop despite commands; serial repeats passed. Cause remains unresolved.
Run controlled physics trials serially and retain startup positioning checks.

Reviewed numerical evidence is `docs/material_results.json`; comparison curves
are `docs/assets/spear_material_validation.png`. Historical channel results stay
in `docs/spear_results.json` and `docs/spear_channels_baseline.md`. A fresh clone
has summaries/images, not the ignored raw traces. The report distinguishes
physical task success from process shutdown failure.

### Files modified or added

Current material work modifies `spear_handling/config.json`, `generate.py`,
`src/SpearContact.cc`, `validate.py`, `CMakeLists.txt`, `README.md`, root README
and this handoff. It adds `src/EmbeddedSpear.hh`, `config_channels.json`,
`tests/test_embedded_spear.cc`, material documentation/results/plot and field
video review/image. `geometry.py` and `tests/test_spear_geometry.py` were added
in the preceding resumed spear work and extended here. `check_friction.py` and
historical channel result/image additions remain from that work. Existing jaw
control/autonomy and the parent hydraulic implementation were not changed by
this material-model work. Inspect `git status --short` for the full dirty tree.

### Tests and reproduction

Build and final automated regression checks passed:

- `cmake --build spear_handling/build -j 2`: passed.
- `ctest --test-dir spear_handling/build --output-on-failure`: 1/1 material test.
- `python3 tests/test_spear_geometry.py`: 13 tests.
- `.venv/bin/python tests/test_autonomy.py`: 25 tests.
- `ctest --test-dir build --output-on-failure`: 1/1 hydraulic test (existing binary).
- Native friction fixture regression: 9/9 checks.
- `git diff --check`: passed.

See `spear_handling/README.md` for build and individual/comparison commands.
Run the current full demo with system Python/Gazebo bindings:

```bash
python3 spear_handling/validate.py --case sequence --gui --start-delay 15 --keep-open
```

This starts a new simulation; it does not replay a recorded physics state. The
runner saves the report before waiting for the GUI to close. Preserve unrelated
Gazebo processes; use isolated partitions and runner-owned process groups.

### Technical limitations and next steps

1. Material parameters are provisional: stiffness 100,000 N/m², bearing limit
   5,000 N/m, damping 3,000 N·s/m², shaft mu 0.3 and native ground/bale mu 0.4.
   Obtain measured insertion, withdrawal and transverse load data before claiming
   field calibration. Videos establish qualitative behavior, not these values.
2. Run sample-count and timestep convergence, explicit force stability and fuller
   energy checks. The published uncapped viscous-power proxy is not a total
   energy/conservation diagnostic. Persistent passage damage, density variation,
   twine and visible deformation remain unmodeled.
3. Broaden masses, yaw/height/entry/carry angles, support friction and travel
   disturbances. Two nominal sequences and selected perturbations do not establish
   reliability or a universal minimum insertion percentage. Native contact-force
   instrumentation would verify support-load estimates directly.
4. The longstanding allocator/teardown failure persists and was reproduced with
   native Physics alone. Keep its diagnosis separate from task physics; no
   dependency/driver removals were performed. Also investigate intermittent
   missing actuator responses observed during concurrent trials.
5. Add a storage budget guard and reduce hard-coded sequence geometry. Integrate
   perception/planning after material behavior has stronger physical validation;
   no real tractor/CAN integration is completed.


## Latest development — 2026-10-21 (supersedes the archived snapshot below)

Work resumed on the spear subproject after the documentation-only handoff.
The current working tree includes implementation changes; the earlier statements
that only this file changed apply exclusively to that prior documentation pass.
No commit or push has been made for the resumed work.

### Completed and verified

- Fixed contact classification by explicitly requesting `::` scoped names.
  Corrected entity discovery so a fixture without a carrier does not repeatedly
  append bale links. The plugin now publishes timestamped explicit world poses
  on `/spear/world_poses`, including entity IDs, using Gazebo's world transforms.
- Added `spear_handling/geometry.py`: quaternion transforms, finite shaft/box
  intersection, all four signed depths and channel clearances in bale axes,
  whole-bale bottom height and tilt. Simulator truth remains experimental
  feedback; this does not introduce camera-based spear autonomy.
- Reworked `validate.py` with locked telemetry snapshots, stale/synchronization
  checks, required phase gates, neutral/stop commands on transitions/errors,
  damped outer joint feedback, final measurements and source/binary hashes.
  Geometry, mass, friction, channel fit and reachable insertion settings are
  validated before world generation. Generated worlds retain the original
  tractor, hydraulic limits, independent bales and default resistance-disabled
  contact model. No old jaw implementation was edited.
- Pickup requires reached insertion and channel clearance, actual spear-contact
  callbacks during lift, center rise >0.25 m, and last-second retention checks:
  overlap >0.10 m, whole-bale bottom >0.10 m, tilt <8°, height variation <0.03 m.
  Insertion aborts when bale XY displacement exceeds 0.15 m. Stage deadlines
  block subsequent actions rather than simply advancing to lift.
- A successful aligned A trial was followed by **two successful complete
  sequences**: A pickup, placement on B, full withdrawal, lower-bale insertion,
  lifting both, forward/brake/reverse travel and hold. The latest run uses live
  continuous load/support checks during travel. B/C are no longer marked
  successful merely because the script executes.
- Added `check_friction.py`: three fixtures permute 0.1/0.2/0.6 independently
  over all three contact pairs. All nine checks passed; measured sliding mu
  matches 0.100/0.200 and the 0.600 cases stay stationary. All fixture surfaces
  are configured at 0.900, so motion verifies effective pair overrides rather
  than just requested settings. These slab fixtures validate the API/backend,
  not real straw/steel coefficients or thin-shaft numerical convergence.

### Recorded evidence

| Trial directory under `runs/` | Outcome |
| --- | --- |
| `spear_20261020_215301_A_3caa4c` | A passed: stop depth 0.7197 m against target 0.73152 m; center rise 0.4781 m; insertion did not move the bale. |
| `spear_20261020_215509_sequence_a0fe0a` | First full sequence passed in 152.65 sim s; separate shutdown -11. |
| `spear_20261021_004244_sequence_17c406` | Final full sequence passed in 152.75 sim s with continuous travel checks; separate shutdown -9. |
| `spear_20261021_004348_B_3c1b1c` | 20% insertion reached, pickup rejected: about 30° tilt and only 0.243 m center rise. Nonzero exit is the reported pickup failure. |
| `spear_20261021_004325_C_68c69e` | 12 cm lateral offset: push guard aborted at 5.75 sim s after about 0.153 m displacement, before any lift stage. |
| `spear_friction_20261021_004236_8ff9eb` | All nine pair-friction motion checks passed; shutdown -9 for each fixture. |
| `spear_friction_20261021_004408_8f5051` | Fixed-iteration teardown comparison: both native Physics-only and custom-plugin fixtures logged the allocator error and required forced cleanup. |

Versioned numerical evidence is in `docs/spear_results.json`; a trajectory,
height/alignment/tilt plot is `docs/assets/spear_sequence.png`. Original failed A
and intermediate B/C runs are preserved rather than reclassified. Raw data is
ignored by Git; a fresh clone will have the summaries, not the trajectories.
The latest sequence's source snapshot and binary hashes allow the tested code
to be distinguished from subsequent edits.

### Files changed and tests

Resumed work edits `spear_handling/src/SpearContact.cc`, `generate.py`,
`validate.py`, and its `README.md`; adds `geometry.py`, `check_friction.py`,
`tests/test_spear_geometry.py`, `docs/spear_results.json` and
`docs/assets/spear_sequence.png`; updates the root README and this handoff.
Changes from the preceding documentation-only pass to this handoff are retained.

- `cmake --build spear_handling/build -j 2`: passed; both libraries built.
- `python3 tests/test_spear_geometry.py`: 8 tests passed (including invalid
  configuration, rotated/translated frames, finite shafts, channel misses and
  whole-bale clearance versus center height).
- `.venv/bin/python tests/test_autonomy.py`: 25 tests passed.
- `ctest --test-dir build --output-on-failure`: 1/1 passed using existing binary.
- Live trials and friction experiments above completed; B/C deliberately report
  physical pickup failure. No GUI trial or optional resistance trial was run.

### Current remaining work

1. The allocator/teardown failure remains unresolved. It is reproduced without
   either project plugin, so do not attribute it solely to spear handling or
   hide shutdown status behind task success. Diagnose the Gazebo/runtime stack
   separately; no dependency removals or driver changes were performed.
2. Sweep timestep, channel clearance, yaw/height/lateral offsets, mass and
   friction around the successful baseline. Two complete runs are limited
   evidence, not a reliability estimate. Default transport is only 0.15 m/s.
3. Strengthen sequence support criteria with contact-force transfer, all-body
   clearance/tilt/velocity checks, longer holds and deliberate loss tests.
   Current travel checks use center-height/separation and lateral support
   thresholds; pickup has the stronger whole-bale checks described above.
4. Sequence geometry/timing and B's initial X=8.5 m remain largely hard-coded.
   Source snapshots contain the exact parameters. During sequence logs,
   `engagement`/`depth` refers to A even in lower-bale stages; lower insertion
   control correctly calls `depth('bale_2')`. The summary recomputes B's overlap
   independently from the saved world poses.
5. The spear logger still lacks the autonomy recorder's storage budget guard.
   Optional resistance, force measurement, taper, real straw deformation and
   real-machine calibration remain open. Integrate sensor perception/planning
   only after broader physical validation.

Run from the project root with system Python and native Gazebo bindings:

```bash
cmake --build spear_handling/build -j 2
python3 spear_handling/validate.py --case sequence
python3 spear_handling/check_friction.py
python3 spear_handling/check_friction.py --check-teardown
python3 tests/test_spear_geometry.py
```

Gazebo Transport needs the tool escalation flow in this sandbox (interface
enumeration fails inside it). Keep unrelated simulations intact; runners use
isolated partitions and terminate only their own process groups.

## Archived pre-resumption handoff — 2026-10-20

The remainder records the previous state and its evidence. Statements there
about failed-A-only status, zero counters, unrun scenarios and suggested fixes
are historical and are superseded by the development record above.

## Snapshot and scope

Workspace: `/home/zhihao/bale_sim`; branch: `main`; inspected HEAD:
`ed38763e097bc3c24a4a63863468270ef276d8c3` (`Add autonomy baseline and spear
prototype with verified status and handoff`). Origin is
`https://github.com/HowardQIN9/Baling_simulation.git`. The working tree was clean
at the start of this documentation pass. Remote synchronization was not checked.

This pass updates **only `HANDOFF.md`**, as requested. Claims were checked against
source code, installed Gazebo headers, Git history, generated SDF, and saved
results. Lightweight checks were rerun (see below); no Gazebo simulation was
launched, implementation file edited, dependency installed, commit made, or
push performed during this pass.

The latest development priority recorded by the existing project documents is
a separate **tractor-mounted four-spear bale-handling prototype** in
`spear_handling/`. It depends on the parent tractor world and hydraulic sources;
it is not a separate repository or a self-contained copy. Preserve the jaw-grip
simulation and autonomy baseline alongside it.

Current evidence: two jaw-grip ground placements passed; one three-layer jaw
stack physically completed but failed the alignment threshold; spear trial A
failed acceptance despite raising the bale. No successful two-bale spear
sequence is established. The root and spear READMEs already report that failure.

## Goals and recorded design constraints

The long-term goal is simulation-to-real autonomous tractor bale handling:
detect and localize a bale, approach and align, pick it up, carry it, and place
or stack it. Real tractor geometry, CAN interfaces, GNSS, operator demonstrations
and learned policies remain future integration work. The current implementation
is a conventional geometric perception/planning/control baseline. The spear
subproject narrows the immediate work to validating physical engagement first.

The following constraints are preserved from the prior handoff and current
project documentation:

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
- Preserve installed dependencies. The autonomy recorder limits its dataset tree
  to the smaller of 20 GB and 40% of total filesystem capacity and leaves at
  least 5 GB free. The new spear logger does not yet enforce that policy.
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
gz-plugin2/register, gz-transport13, and gz-physics7. The recorded build and
actual simulation establish that these interfaces were available; exact package
versions were not re-queried during this handoff.

### New project files

| File | Actual implementation |
| --- | --- |
| `spear_handling/CMakeLists.txt` | Separate C++17 build. Compiles the existing hydraulic source into `libBaleLoaderHydraulics.so` and the new `libSpearContact.so`. |
| `spear_handling/config.json` | Dimensions, mass, spear geometry, channel geometry, pair-friction requests, optional resistance, timestep, and test settings. |
| `spear_handling/generate.py` | Parses the source world, removes cameras/Sensors for physics tests, replaces jaws with four cylindrical spears, creates two dynamic bales, adds contact plugin. |
| `spear_handling/src/SpearContact.cc` | Attempts pair-specific contact friction; optional equal/opposite axial insertion drag; publishes contact/drag telemetry. |
| `spear_handling/validate.py` | Isolated Gazebo runner with cases `A`, `B`, `C`, `sequence`, optional GUI, truth-based control, trajectory and report output. |
| `spear_handling/README.md` | Current failed-A status, setup, design notes, limitations and A–F test plan. |
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
4. The prior development handoff records `cmake --build spear_handling/build -j 2`
   succeeding for both shared libraries. Both binaries and the executed A
   artifacts remain available locally; the build was not repeated here.
5. Recorded one real headless DART trial A; the previous handoff records an
   initial launch failure under sandbox networking restrictions.
   Ground settling, insertion motion, lifting, and holding were recorded.
6. Preserved the jaw-grip autonomy implementation alongside the spear prototype.
   Git HEAD now includes both in commit `ed38763`; older statements that this
   work is uncommitted are superseded by the current repository state.

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
2. **Recognized contact-pair counters remain zero.** Check actual scoped
   name formatting, classification, event timing, component discovery, and DART
   feature support. A concrete source/header mismatch is verified:
   `scopedName(e,ecm)` defaults to `/` in installed
   `/usr/include/gz/sim8/gz/sim/Util.hh:68`, while the plugin searches for
   `::bale_` and `::ground::`. Those classification branches do not match
   slash-delimited names; the spear-name prefix branch is separate. This is a
   strong explanation for zero recognized pair counts, but callback delivery
   and runtime friction still require a focused test after repair. Inspect
   collision entity IDs in callback arguments; do not mask it by raising friction.
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
9. **Documentation mixes development stages.** Both READMEs now report failed
   spear A correctly, but older roadmap text in the root README and the first
   approach report predates implemented pickup/stacking. Use source and the
   task-specific result files for current capabilities. Preserve failed results
   when updating reports after future validation.

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

### Jaw-grip architecture and data flow

| Component | Role and key assumptions |
| --- | --- |
| `worlds/simple_tractor.sdf` | Manual Ackermann tractor, three 525 kg box bales, side-clamping grip, front RGB/depth sensors, loader limits and plugin configuration. |
| `scripts/run_sim.sh`, `scripts/teleop_loader.py`, `scripts/run_grip_demo.py` | Plugin-aware manual launch, drive/valve keyboard controls, and temporary prepositioned grip fixture. `worlds/teleop_wasd.py` is the older drive-only utility. |
| `bale_autonomy/scene.py` | Seeded scene generation, camera calibration, ideal 100 Hz IMU, source snapshots and hashes; generated worlds leave the manual SDF untouched. |
| `transport.py`, `localization.py` | Native `gz.transport13` / `gz.msgs10`, bounded locked queues, isolated partition/process groups; planar integration of wheel speed and ideal IMU heading, corrected for rear-axle offset. |
| `perception.py` | Straw-color mask, calibrated depth projection, known-size visible top rectangle, axis yaw modulo pi; target association and temporal/multi-view smoothing. |
| `planning.py` | Forward/reverse SE(2) motion primitives and Hermite connectors with curvature limits, rectangular collision envelopes and known field bounds; pure-pursuit following. |
| `approach.py` | Observe, plan, follow, refine, stop/abort; orchestrates manipulation and stacking cycles and scores recorded truth after control. |
| `manipulation.py` | Fine alignment, jaw closure evidence, lift/retention checks, loaded transport, lowering, release and withdrawal. |
| `stacking.py` | Three source cycles, measured support-top geometry, bounded lift/tilt reach calculation, raised insertion and withdrawal. |
| `recording.py`, `evaluation.py`, `batch.py` | Portable observations and separate labels, disk guard, offline replay/noise ablations and suite reports. |
| `visualization.py` | Collision-free live path, driven trail, goal/source/drop-zone drawings. |

The ordinary generated approach scene defaults to front/right cameras. Side-bale
selection or pick-place/stack missions enable front/right/left/mast views.
Autonomy cameras use 320 × 240 pixels at a requested 10 Hz; the manual world's
front sensors use 640 × 480 at 15 Hz. Do not mix their calibration. Camera axes
are +X forward, +Y left, +Z up; depth is axial-forward meters. The tractor's
initial frame is the task world frame; `(0,-6)` is to its right.

RGB and depth are paired within 5 ms, with odometry and available truth poses
matched within 60 ms. The recorder defaults to 2 Hz **per camera**, writing JPEG
RGB, compressed float32 depth NPZ, `frames.jsonl` and separate
`ground_truth.jsonl`. It records timing offsets, calibration and source hashes.
Controller traces are `trajectory.jsonl`, with labels in
`trajectory_ground_truth.jsonl`; task results are `approach.json`,
`pick_place.json` and/or `stack.json`. Manipulation plans and phase evidence are
saved separately, with per-cycle directories for stacking. Source snapshots do
not constitute a complete environment lockfile; only the hydraulic binary hash
is retained, not a copied binary.

Truth travels through the transport/recording layer for labeling and final
scoring; the jaw control path estimates targets from RGB-D and wheel/IMU state.
The spear runner instead reads simulator poses directly for control. Both use
physical contact; the spear's optional drag model additionally applies explicit
equal/opposite forces. GPU rendering is separate from inference: no neural
model, reinforcement learning, ROS bridge or real CAN control is implemented.

### Historical approach and placement acceptance

`docs/autonomy_results.json` and `runs/delivery_v1/report.md` retain three accepted
approaches: nominal 0.111 m / 0.410°, left 0.087 m / 2.429°, right 0.088 m /
0.214° true stopping error / heading error. The larger maneuver failed after
losing the target; the deliberate sensor-loss case passed its separate stop
check. Approach acceptance in `approach.py` is success plus error <0.30 m,
heading <8°, and minimum SAT gap >0.20 m. SAT gap is a separating-axis measure,
not exact Euclidean clearance. This small trial set is not a general success rate.

The two accepted raw `datasets/pick_place_*/pick_place.json` files agree with
`docs/pick_place_results.json`: errors 0.240721 m and 0.106115 m, 813 and 867
recorded camera observations, and shutdown -9 for both. Ground-placement scoring
requires successful completion, verified pickup, center error <0.6 m, height
within 0.15 m of 0.45 m and unrelated-bale SAT gap >0.20 m. Yaw error is reported
but is **not** an acceptance condition for this task.

Stack scoring additionally requires all three cycles, layer height errors
<0.12 m, neighboring centers <0.30 m apart, and roll/pitch/axis-yaw errors <8°.
The recorded neighbor offsets were recomputed as 0.330060 m and 0.345184 m;
`success=true` describes mission completion, while `acceptance_pass=false`
records the failed criteria. Do not relax thresholds to reclassify that run.

### Decisions to retain and technical limits

- Generate isolated worlds and use unique trial names; autonomy commands refuse
  to overwrite an existing trial. Keep the manual scene and spear experiment
  separate. Regenerate after relocation because plugin paths are absolute.
- Keep simulator labels separate from jaw perception/control and from acceptance
  reporting. Distinguish task success, final acceptance and shutdown status.
- Preserve contact/friction gripping and independent bale bodies. Channel voids
  are an explicit spear approximation with artificial alignment/retention risks.
- Hydraulic feedback represents a reduced-order force-limited actuator, not a
  fluid-circuit simulation or measured real-loader pressure/weight. Neutral
  commands retain holding control; command timeout neutralizes after 0.5 s.
- Geometry/control use known bale size, flat terrain, ideal calibration/heading
  and bounded rectangular envelopes. No general obstacle perception, GNSS/slip
  correction, calibrated uncertainty or deformable straw model is established.
  The larger failed approach is a concrete visibility/tracking limitation.
- High stacking approaches the existing -0.65 rad tilt limit; ground/support
  checks use geometric height proxies rather than measured contact force.
  Sampled clearance checks do not prove continuous collision freedom, and the
  source tractor does not rely on enabled self-collision to reject bad geometry.

## Files changed and repository preservation

This documentation pass changes only `HANDOFF.md`; the initial tracked and
untracked status was empty. No implementation changes were made for this request.
The prior handoff's list of uncommitted files is obsolete: HEAD `ed38763`
contains 37 changed files (6,608 insertions, 40 deletions) covering:

- `bale_autonomy/`: all 14 package files, including scene, transport, recording,
  perception, localization, planning, approach, manipulation, stacking,
  visualization, evaluation and batch reporting.
- `scripts/bale_autonomy.py`, `requirements-autonomy.txt`,
  `tests/test_autonomy.py`, and `worlds/autonomy_gui.config`.
- `spear_handling/`: CMake, config, generator, validator, contact plugin,
  README and local ignore file.
- Root `.gitignore`, `README.md`, `HANDOFF.md`; autonomy/pick-place/stacking
  documentation, numerical result JSON, and four images in `docs/assets/`.

Earlier commits provide the baseline: `6f5e896` added hydraulic and grip control,
CMake build, `src/LoaderHydraulics.cc`, `src/HydraulicModel.hh`, loader tests,
teleoperation/demo scripts and source-world loader geometry; `883dd8a` added
rigid-body bale collisions/friction; `ebaf3c8` added cameras. Use
`git show --stat <commit>` for exact per-file history. A clean tree at entry does
not identify which historical changes belong to any particular conversation.

Root ignore rules cover `/build/`, `/.venv/`, `/datasets/`, `/runs/`, Python
caches and bytecode. Spear build output is separately ignored. Raw evidence is
local and will not accompany a normal clone; preserve/back it up separately.
Versioned result summaries are available under `docs/`. Avoid resetting or
cleaning away retained experiment data.

## Validation during this documentation pass

| Command | Result | Scope |
| --- | --- | --- |
| `.venv/bin/python tests/test_autonomy.py` | PASS: 25 tests, 0.162 s | Current Python source; analytic geometry, messages, recording and controller failure paths. |
| `ctest --test-dir build --output-on-failure` | PASS: 1/1 `hydraulic_model` | Existing built C++ test executable; no rebuild in this pass. |
| `.venv/bin/python tests/check_loader_clearance.py` | PASS: 876,876 envelope pairs; minimum SAT gap 0.053 m | Sampled original jaw-loader/wheel geometry, not spear geometry or continuous-motion proof. |

The Python tests cover row stride/encoding, invalid depth, roll/pitch and world
transforms, target association, rear-axle/wheel–IMU integration, curvature and
collision checks, source/calibration recording, storage budget preservation,
empty-clamp/end-stop rejection, payload rotation and overhang, stack reach,
support visibility and a one-frame payload jump. They do not validate spear
insertion, contact callbacks, friction overrides, or the full physical sequence.
The older approach delivery document's count of 13 tests is historical; the
current suite contains 25.

The existing spear build products were found under `spear_handling/build/` and
the main hydraulic library under `build/`. The prior handoff records a successful
spear build, but neither C++ project was rebuilt here. Live hydraulic/Gazebo
checks and historical full-task runs were not rerun. Saved results are evidence
of the recorded trials, not a new repeatability claim for current source.

## Setup and baseline commands

Use Gazebo Harmonic / Sim 8, Ogre2 for RGB-D rendering, CMake and a C++17
compiler. The main build finds `gz-sim8`, `gz-plugin2/register` and
`gz-transport13`; the spear build additionally finds `gz-physics7`. Python
requires system Gazebo message/transport bindings. The existing `.venv` is
available; a fresh environment needs `--system-site-packages` and
`requirements-autonomy.txt` (NumPy, OpenCV headless, SciPy, Matplotlib).
The requirements specify ranges rather than exact dependency pins.

For a resumed session, from the project root (commands below were not executed
in this documentation pass):

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j 2
# Manual world; operate from another terminal with scripts/teleop_loader.py:
bash scripts/run_sim.sh
# Or start one isolated, self-contained autonomy mission:
.venv/bin/python scripts/bale_autonomy.py approach --name resume_approach_01 --target 12 0 0
.venv/bin/python scripts/bale_autonomy.py pick-place --name resume_placement_01
.venv/bin/python scripts/bale_autonomy.py stack --name resume_stack_01
```

Run one mission at a time with fresh names; no second teleop is needed for
isolated autonomy. `demo` opens the GUI with path overlays; other mission commands
accept `--gui --show-path --keep-open --wait-for-start` in an interactive terminal.
`collect`, `evaluate`, `suite` and `report` are also supported by the CLI.
For headless manual launch use `bash scripts/run_sim.sh -s --headless-rendering`.
Native Gazebo Transport is used throughout. `Session` selects the NVIDIA EGL
vendor JSON when present; rendering still depends on the runtime GPU/EGL setup.

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
9. For the preserved jaw baseline, fix visibility/tracking on the failed larger
   maneuver, examine localization drift and stack center offsets, then repeat
   placements and stacking across more seeds. Retain the original acceptance
   limits and separately investigate the recurring allocator/teardown failure.
   Real tractor measurement, CAN capability verification and sensor calibration
   follow simulation evidence; no real-machine integration is completed.

Commands for a future resumed session (not run during this documentation pass):

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
sandbox in an earlier attempt according to the prior handoff; use the tool's
escalation flow if the runtime fails under current sandbox permissions. Do not
assume an earlier approval transfers to every future environment.
Do not broadly kill Gazebo instances: use the runner's isolated partition and
its own process group, preserving unrelated user simulations.
