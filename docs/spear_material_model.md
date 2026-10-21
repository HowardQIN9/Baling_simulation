# Spear penetration and support model

## Purpose

The default spear experiment now uses solid rigid bales with a local yielding
material interaction around each embedded shaft. It implements the handling
behavior discussed with the operator and recorded in
[field_video_review.md](field_video_review.md): relatively easy entry, a small
boom-down adjustment to maintain support friction, lifting at modest angles,
and extra downward loading before withdrawing from a placed bale.

This is a reduced-order interaction model. Bale shape, mass and inertia remain
rigid; individual straws, visual deformation, damage and binding twine are not
simulated. Parameters are provisional engineering values, not measured material
properties or a quantitative fit to the videos.

## Geometry and contact

- Each bale has one full-box collision: there are no pre-existing entry channels.
- Native collisions remain for bale/ground, bale/bale, attachment frame/bale and
  spear/ground. Only native spear/bale contact is filtered in material mode.
- Shaft and bale categories are 4 and 2, with collision masks 5 and 3. Both sides
  exclude the native shaft/bale pair under DART's category/mask filtering.
- All shaft/material forces use actual finite overlap with the bale volume.
  Each shaft creates its own passage where it enters, including laterally
  shifted and modestly angled entries.
- The original channel baseline is available using
  `--config spear_handling/config_channels.json`; historical channel results
  must not be reported as material-model validation.

The generator is `spear_handling/generate.py`. The installed DART backend's
[collision-filter handling](https://raw.githubusercontent.com/gazebosim/gz-physics/gz-physics7/dartsim/src/SDFFeatures.cc)
was inspected, and live material trials report zero native shaft/bale callbacks
while material forces support the payload. Frame and ground collisions remain
physical rather than being replaced with synthetic forces.

## Material interaction

`src/EmbeddedSpear.hh` is independent of the Gazebo entity manager. Each passage
uses a configurable set of material cells distributed through the bale along
the initial entry ray (32 by default).

A cell becomes active as the finite shaft covers its interval. Virgin material
starts with no transverse spring strain at the shaft's actual location. The
cell remembers that passage in bale coordinates. Subsequent transverse motion
produces spring/damping bearing forces, supporting the load or transmitting a
downward push. The force is proportional to covered shaft length and bounded
by a bearing capacity per meter. A plastic return step limits stored strain and
allows the passage to yield under excessive displacement.

Axial resistance uses a bristle spring/damper with a Coulomb cap based on
configured confinement and local transverse bearing load. Cutting resistance
is added only while the tip advances into previously unentered material. This
separates relatively easy axial entry from transverse load support. Static
resistance can retain an embedded payload, while sliding and withdrawal are
allowed when the cap is exceeded. No universal penetration-percentage success
threshold or fixed attachment is used in the material law.

As the shaft withdraws, covered material length and available support decrease.
Full withdrawal removes all forces and resets the passage. Passage damage and
persistent holes after complete withdrawal are not retained in this version.
The finite-cell quadrature and simplified plasticity require convergence and
energy-accounting study before use as a high-fidelity material solver.

`src/SpearContact.cc` transforms the local forces into world coordinates and
applies an equal/opposite reaction to the implement **at the same world point**.
It accumulates each body's force and moment about its link origin and calls
`Link::AddWorldWrench`. This follows the inspected Gazebo
[link wrench implementation](https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/src/Link.cc).
The older optional drag path was also corrected to avoid treating an offset
from the link origin as an offset from the implement's center of mass.

## Boom motion and surface friction

Ground/deck and bale/bale friction coefficients remain constant. Extra resistance
comes from increased normal reaction when the embedded spears press down.

For a nearly stationary supported bale, the approximate vertical balance is
`support normal ≈ bale weight − vertical spear force on bale`.
A negative (downward) spear force therefore increases support reaction; an
upward force unloads it. This balance is an estimate, not a measured native
contact force. Turning moments, acceleration and multiple support contacts
must also be considered.

During first-bale insertion, the runner establishes partial engagement, lowers
the boom until a measured downward-force target is reached, and then follows
an entry trajectory aligned with the shaft direction. With downward-pointing
spears, merely driving horizontally while holding boom height makes an existing
passage rise; it can unload and push the bale. The compensating boom motion
handles this geometry without requiring perfectly level spears. The same
technique is used for lower-bale insertion in the two-bale sequence.

After placement, the runner lowers until the upper bale receives a downward
preload. It then holds the achieved boom position while withdrawing. Native
contact transfers the upper bale's weight and added load to the lower bale.
If the upper bale follows the spears by more than 0.15 m, withdrawal stops and
reports failure. `--no-insertion-preload` and `--no-release-preload` expose the
uncorrected motions as controlled comparisons; neither changes friction values.

During lifting, tilt control includes feedforward for boom velocity so the shaft
can maintain its requested world angle while the boom moves. Merely tracking
the changing tilt target produced excessive lag and tripped the 8-degree load
tilt guard in an intermediate two-bale trial, even after strengthening the
material parameters. The final sequence uses a modest +0.06 rad carrying angle,
a reduced combined-load lift command, and continuous alignment/tilt guards
during the lift as well as travel. These are controller choices, not measured
limits on real bale handling.

The lift/tilt joints, hydraulic controller, pressure limits and tractor platform
are reused. Boom targets and force targets are provisional. The runner still
uses simulator truth and force telemetry for physics experiments, not camera
perception or a real tractor interface.

## Parameters and telemetry

Default material values in `spear_handling/config.json`:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| Transverse stiffness | 100,000 N/m² | Force per shaft displacement per embedded length |
| Transverse damping | 3,000 N·s/m² | Damping per embedded length |
| Bearing limit | 5,000 N/m | Bounded transverse force per embedded length |
| Axial stiffness | 10,000 N/m² | Bristle stiffness per embedded length |
| Axial damping | 200 N·s/m² | Bristle damping per embedded length |
| Confinement | 100 N/m | Unresolved radial load per embedded length |
| Cutting resistance | 20 N/shaft | Resistance during virgin advancement |
| Spear friction | 0.3 | Axial Coulomb factor |
| Ground / bale friction | 0.4 / 0.4 | Native contact pair overrides |

The first weaker setting (50,000 N/m² and 2,400 N/m) supported one bale but
failed the subsequent two-bale load/moment. That failed run is preserved. The
higher current setting was chosen to accommodate the combined load and moment
with less local compliance; it is not a measured calibration. Shallow or
poorly oriented engagement can still yield or tip because capacity and moment
support remain finite.

`/spear/material/bale_1` and `/spear/material/bale_2` publish `Double_V`:

`[sim_time, link_entity_id, summed_overlap_m, Fx, Fy, Fz, Tx, Ty, Tz,
active_cells, yielded_cells_this_step, summed_absolute_axial_force_N,
uncapped_viscous_power_proxy_W]`.

Forces and torques are on the bale in world coordinates; moments are about the
bale link origin. Overlap is summed over all shafts. The final power field is
a damping diagnostic, not a complete dissipated-energy or conservation score.
The existing `/spear/contact/state` format is preserved. Its shaft-contact
counter is normally zero in material mode because that native pair is filtered;
material force evidence is used for pickup verification instead.

Reports, generated SDF, config, traces and source/binary hashes are saved in
unique `runs/spear_*` folders. Small reviewed results are in
[material_results.json](material_results.json). Raw data remains git-ignored.
The existing Gazebo allocator/teardown problem is recorded separately from task
outcome; successful motion does not establish clean process shutdown.

## Recorded behavior

The final parameter set was exercised in the following controlled cases. These
are selected experiments, not a statistical reliability study. Raw run IDs and
all preserved development failures are in `material_results.json`.

| Case | Observed outcome |
| --- | --- |
| Full sequence | Passed twice with identical source/binary hashes: pickup, stack, full withdrawal, lower-bale entry, two-bale lift and slow forward/reverse travel. Maximum logged travel tilt was 4.97°; upper bale received zero direct material force during travel. |
| No insertion correction | Bale moved 0.152 m; push guard blocked lifting. |
| No release preload | Upper bale followed the spears 0.153 m; withdrawal stopped. |
| Lateral +0.12 m, entry -0.06 rad | Pickup and retention passed without channel alignment. |
| Shallow 0.24384 m target | Target reached, but pickup failed: rise 0.226 m, about 26.9° final tilt and 0.067 m overlap. |

The omission comparisons use the same final material and friction parameters as
the nominal sequence. They demonstrate the modeled load-transfer mechanism;
they do not establish that all real bales require the same preload or trajectory.

## Commands

```bash
cmake --build spear_handling/build -j 2
ctest --test-dir spear_handling/build --output-on-failure
python3 tests/test_spear_geometry.py
python3 spear_handling/validate.py --case sequence
python3 spear_handling/validate.py --case sequence --gui --start-delay 15 --keep-open
python3 spear_handling/validate.py --case C
python3 spear_handling/validate.py --case A --no-insertion-preload
python3 spear_handling/validate.py --case release --no-release-preload
python3 spear_handling/validate.py --case A --config spear_handling/config_channels.json
```

Further work includes force/withdrawal calibration, sampled-length/timestep
convergence, repeatability across masses and angles, persistent material damage,
contact-force measurement, stronger energy diagnostics and a storage guard.
