# Tractor spear handling

The default experiment uses **solid bales with no pre-cut channels**. A local
material model supplies penetration resistance, yielding shaft support and
withdrawal friction while the overall bale shape remains rigid.

The controller applies the operator's technique: engage, lower the boom slightly,
continue entry along the shaft direction, lift with a modest carrying angle,
and press the placed bale down before withdrawing. Surface friction coefficients
stay constant; physical force transfer changes the support load.

The original tractor, L115-inspired hydraulic limits and independent bale bodies
are retained. The runner uses simulator truth for physics experiments. Material
parameters are provisional; this is not calibrated real-straw simulation or
camera-guided spear autonomy.

## Run

From the repository root, using system Python with Gazebo bindings:

```bash
cmake -S spear_handling -B spear_handling/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build spear_handling/build -j 2
python3 spear_handling/validate.py --case A
python3 spear_handling/validate.py --case sequence --gui --start-delay 15 --keep-open
```

The GUI command allows 15 seconds to arrange the view and saves results before
leaving the window open. Close it when finished. Run validation trials one at a
time: simultaneous development trials showed missing actuator responses, and
startup now checks command connections and actuator positioning before driving.

Additional cases and comparisons:

```bash
# Laterally shifted entry; no hole alignment is required:
python3 spear_handling/validate.py --case C
# Shallow entry; outcome is determined by forces and checked after lifting:
python3 spear_handling/validate.py --case B
# Stop after stacking and withdrawal:
python3 spear_handling/validate.py --case release
# Deliberately omit the operator corrections (may fail; nonzero exit expected):
python3 spear_handling/validate.py --case A --no-insertion-preload
python3 spear_handling/validate.py --case release --no-release-preload
# Historical pre-cut-channel model:
python3 spear_handling/validate.py --case A --config spear_handling/config_channels.json
```

Configure mass, dimensions, shaft geometry, material stiffness/capacity, friction,
entry angle, carrying angle and preload targets in `config.json`. There is no
single calibrated maximum angle or universal penetration percentage for success.

## Evidence and implementation

- [Material model, force laws, telemetry and limitations](../docs/spear_material_model.md)
- [Material trial results](../docs/material_results.json)
- [Operator videos and interpretation](../docs/field_video_review.md)
- [Historical channel baseline](../docs/spear_channels_baseline.md)
- [Project handoff](../HANDOFF.md)

Each run writes its config, world, telemetry, result and source/binary hashes
under a unique `runs/spear_*` directory. Raw runs are ignored by Git. A passed
physical task and a clean process shutdown are separate outcomes. The existing
Gazebo allocator/teardown fault remains unresolved and also reproduces without
project plugins.

## Checks

```bash
ctest --test-dir spear_handling/build --output-on-failure
python3 tests/test_spear_geometry.py
python3 spear_handling/check_friction.py
```

The C++ material checks cover virgin/off-channel/angled entry, length-dependent
bearing, downward load transfer, yielding, axial resistance, release and balanced
point-force reactions. Python checks cover frame geometry, whole-bale clearance,
collision filtering, legacy generation, parameter validation and tilt tracking.
The friction fixtures verify native contact overrides independently of material
force calibration. Broader force measurements and timestep/sample convergence
remain future work.
