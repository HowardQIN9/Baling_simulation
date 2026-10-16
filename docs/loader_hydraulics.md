# L115-inspired loader and hydraulic-equivalent actuation

## Scope and references

The tractor now has a fixed loader mount, a raised twin-arm boom, and a
broad-pad side-clamping bale grip mounted on its attachment carrier. Lift and tilt are independently actuated revolute joints. Their forces
are applied through a custom Gazebo Harmonic system plugin using DART dynamics.
The two physical boom arms are one rigid link; the carrier is another. The
previous two-tine fork is retained only as an explicit supported-load test fixture
in `tests/fixtures/square_bale_fork.sdf`. The grip has two prismatic jaw joints
operated by one additional hydraulic valve command; it is independent of tilt.
This is a sliding-jaw simplification, not a reproduction of Case IH's actual jaw
linkage or certified capacity.

This is a reduced-order simulation baseline, not a calibrated L115 CAD model or
a fluid-circuit simulation. Cylinders are represented mathematically, not as
separate visible moving piston links. This avoids closed kinematic loops in DART.

[Case IH's L115 page](https://www.caseih.com/en-us/unitedstates/products/loaders-attachments/l11-series-loaders/l115)
lists compatibility with Maxxum 150, a maximum height of 3.81 m, lift capacity up
to 2,360 kg, and boom breakout rated at 5,975 lbf (about 26.6 kN). Those are product
ratings, not verified simulation performance. No rated-capacity claim is made for
this simplified geometry, attachment carrier, or actuator model.

The same page lists a bale spear, deluxe square-bale fork, heavy-duty bale grip,
and bale splitter. It describes the heavy-duty grip as suited for smaller square
bales. That description does not establish a suitable jaw opening or load rating
for our 2.4 × 1.2 × 0.9 m, 525 kg bales. Our jaw geometry and force settings
are independent engineering assumptions for this simulation, not manufacturer
specifications.

[Case IH's Maxxum hydraulics description](https://www.caseih.com/en-us/unitedstates/products/tractors/maxxum-series)
lists pump options around 86, 121, and 150 L/min depending on configuration. The
user's installed pump is unknown; 86 L/min is the provisional default.

The plugin uses the interfaces described in
[Gazebo's system-plugin documentation](https://gazebosim.org/api/sim/8/createsystemplugins.html).

## Parameters

Except for the published reference ratings above, these are **engineering
assumptions**, not verified L115 geometry or cylinder specifications.

| Parameter | Baseline |
| --- | --- |
| Shared available pump flow | 86 L/min |
| Equivalent pressure limit | 200 bar (20 MPa) |
| Lift cylinders | 2 × 70 mm bore / 40 mm rod |
| Tilt cylinders | 2 × 50 mm bore / 30 mm rod |
| Lift / tilt effective cylinder moment arm | 0.32 / 0.18 m, constant |
| Lift / tilt branch flow limits | 60 / 45 L/min |
| Boom / tilt speed limits | 0.20 / 0.35 rad/s |
| Valve response time constant | 0.12 s |
| Command deadband | ±0.05 |
| Command timeout | 0.5 s, wall-clock time |
| Boom pivot in tractor model frame | (0.4, 0, 1.55) m |
| Pivot-to-carrier distance | 2.8 m |
| Raised-arm bend, initial tractor coordinates | (2.3, ±0.7, 1.70) m |
| Carrier hinge, initial tractor coordinates | (2.8572, 0, 0.2076) m |
| Initial boom angle to horizontal | −0.50 rad |
| Lift joint range | 0 to 1.44 rad relative to initial pose |
| Fork tilt joint range | −0.65 to +0.85 rad relative to boom |
| Optional test-fork tine length / center spacing | 1.3 / 1.0 m |
| Loader mount / boom / carrier with grip rail mass | 140 / 300 / 140 kg |
| Each grip jaw mass | 60 kg |
| Optional test fork mass, replacing carrier | 150 kg |
| Loader and carrier contact friction | `mu = mu2 = 0.6`, unvalidated |

The default 700 kg assembly is added to the existing 5,500 kg tractor, giving
6,200 kg unloaded total (6,090 kg in the fork-only test fixture variant). Link inertia includes each rotated box's own inertia and its offset from
the link center of mass using the parallel-axis theorem. This mass split, center
of mass, and rigid geometry require real measurements for validation.

The boom attachment hinge height is approximately
`1.55 + 2.8 sin(-0.50 + q_lift)` m relative to the tractor model. At the upper
limit it is about 3.81 m. Matching that one dimension does not reproduce the
manufacturer's full loader reach envelope or its height measurement convention.
Carrier orientation changes with both boom and tilt; automatic self-leveling is not
implemented. An attachment may exceed the hinge height when tilted.

## Broad-pad grip prototype

The grip is inspired by the broad-pressure-arm description on the L115 page.
Its sliding jaws are deliberately simpler than a manufacturer's pivoting or
linked-arm mechanism. The following dimensions and forces are simulation
assumptions, sized for the current bale rather than copied from the website:

| Parameter | Value |
| --- | --- |
| Internal jaw opening | 1.90 m fully open; 0.50 m fully closed |
| Each jaw stroke | 0.70 m inward |
| Each broad contact pad | 2.0 m long × 0.60 m high × 0.10 m thick |
| Each jaw cylinder | 40 mm bore / 25 mm rod; direct 1:1 travel |
| Each jaw pressure cap | 100 bar; also bounded by shared 200 bar system limit |
| Maximum inward force per jaw | About 12.57 kN |
| Jaw speed / branch flow limit | 0.12 m/s / 15 L/min per jaw |
| Pad friction | `mu = mu2 = 0.6`; original bale friction remains 0.4 |
| Jaw mass / added rail mass | 60 kg per jaw / 50 kg rail |

One `/tractor/loader/grip/valve` command operates both jaw actuators. Positive
closes, negative opens; a neutral command retains the existing holding force
through the equivalent hydraulic servo. Opening clears stored holding effort so
the grip can release promptly. Each jaw has its own feedback and force limit;
they share pump flow with lift and tilt. This is not a detailed flow-divider or
single-cylinder linkage simulation. For these direct prismatic joints the
plugin's stroke/joint-coordinate ratio (`moment_arm` parameter) is 1, yielding
linear speed and force instead of angular speed and torque.

Holding depends on the sum of upward friction forces at the pads exceeding the
bale's weight; it is not guaranteed by closure alone. The prototype passed a
525 kg contact-only test, but no real attachment capacity or crop-damage claim
follows from that. The bales remain undeformable boxes.

For an easy manual test, close any simulation using this world's topics and run:

```bash
python3 scripts/run_grip_demo.py
```

This keeps the full cameras and starts a ground-resting bale already aligned
between the open jaws. It changes only a temporary copy of the world. In another
terminal run `python3 scripts/teleop_loader.py`, press P to close, wait about five
seconds, then N to hold the grip. Use U to raise and N to stop raising. Use J to
lower, N to stop lowering, then O to open and release. Space stops driving and
neutralizes all loader valves; it does not intentionally release a clamped bale.
The demo is prealigned, not an autonomous pickup behavior. In the main world,
align with the bale's long axis so its 1.2 m width fits between the 1.9 m jaws.
A bale turned sideways with 2.4 m across the opening will not fit.

## Actuator behavior

Valve inputs are normalized values from −1 to +1. Positive lift raises the boom;
positive tilt rolls back the attachment carrier. Nonfinite commands become neutral. Each valve
request passes through a first-order response before calculating demanded flow.

```text
Cap-end area = π bore² / 4
Rod-end area = π (bore² − rod²) / 4
Angular speed request = Q / (cylinder_count × area × moment_arm)
Equivalent torque limit = pressure_limit × cylinder_count × area × moment_arm
```

Commands are limited by branch flow and joint speed; simultaneous demands are
scaled proportionally if their sum exceeds the shared available pump flow.
Speed limits can dominate before the 86 L/min supply is exhausted. No extra
speed is implied by selecting a larger pump when those limits already dominate.
Extension and retraction use different cylinder areas.

Flow integrates a bounded joint-position reference. A force-based PID tracks it,
clamping torque by the equivalent pressure limit, with anti-windup and a maximum
reference tracking lag of 0.12 rad under overload. Returning a valve to zero
captures the current joint position and retains the last applied torque for a
smooth transition to holding; the valve response then finishes decaying. Once
neutral, the reference is frozen for simplified load-holding. The neutral
controller can sag during settling and cannot hold a load above its force limit.
Unrefreshed commands return to neutral after 0.5 s; the valve response then decays.

This **does not solve chamber pressures, oil compressibility, leakage, valve
orifice flow, pump power, actual cylinder stroke, or linkage-dependent leverage**.
Reported pressure is inferred from applied torque, not a simulated sensor or
fluid state. Reported flow is allocated command flow, not measured cylinder flow;
stall losses and relief-valve return flow are not modeled. A neutral holding
servo stands in for hydraulic load-holding hardware.

## Build and launch

On this server the required Gazebo development libraries and Python bindings
are already installed. Required build tools are CMake and a C++17 compiler;
packages include `libgz-sim8-dev`, `libgz-plugin2-dev`, and `libgz-transport13-dev`.
Teleoperation and integration tests use `python3-gz-transport13` and
`python3-gz-msgs10` bindings with the system `python3`.

From the project root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j 2
bash scripts/run_sim.sh
```

The launcher adds the local `build` directory to `GZ_SIM_SYSTEM_PLUGIN_PATH`.
Use it when launching this world so the actuator plugin is found. For a server:

```bash
bash scripts/run_sim.sh -s --headless-rendering
```

Alternatively, after building, use the original launch command with the plugin
path supplied explicitly:

```bash
GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build" gz sim -r worlds/simple_tractor.sdf
```

## Controls

Run in another terminal and keep that terminal focused:

```bash
python3 scripts/teleop_loader.py
```

| Keys | Action |
| --- | --- |
| W / S | Increase / decrease driving speed |
| A / D | Increase left / right turn command |
| C | Center steering |
| U / J | Increase / decrease lift valve by 0.25 |
| I / K | Increase / decrease tilt valve by 0.25 |
| O / P | Open / close the grip; commands latch |
| N | Neutral all loader valves; hold position and clamp force |
| Space | Stop driving and neutral all loader valves |
| Q / Ctrl-C | Send stop and neutral commands, then exit |

Settings latch: releasing a key does not neutralize the command. After raising
with U, use N to hold; press J from neutral to lower. This controller refreshes
commands at 20 Hz. Do not run the older driving controller simultaneously.
The hydraulic timeout applies only to loader valves; the existing driving
controller has no added command timeout.

Teleoperation waits up to five seconds for subscribers on the lift, tilt, and
grip command topics before accepting controls. If the actuator plugin is absent,
it exits with launch instructions rather than silently changing command values.
During operation the status line reports `hydraulics=CONNECTED` or `DISCONNECTED`.
This indicates topic connections, not proof of joint motion or an unpaused world.

The existing RGB/depth camera pair remains at its original mounting pose. The
loader or an attachment may enter or obstruct its view as they move.

## Topics

| Topic | Message | Meaning |
| --- | --- | --- |
| `/tractor/loader/lift/valve` | `gz.msgs.Double` | −1 to +1 lower/raise |
| `/tractor/loader/tilt/valve` | `gz.msgs.Double` | −1 to +1 dump/rollback |
| `/tractor/loader/grip/valve` | `gz.msgs.Double` | −1 opens both jaws, +1 closes both |
| `/tractor/loader/grip_left/state`, `/tractor/loader/grip_right/state` | `gz.msgs.Double_V` | Jaw diagnostics |
| `/tractor/loader/lift/state` | `gz.msgs.Double_V` | Lift diagnostics at about 20 Hz simulation time |
| `/tractor/loader/tilt/state` | `gz.msgs.Double_V` | Tilt diagnostics at about 20 Hz simulation time |
| `/tractor/loader/joint_state` | `gz.msgs.Model` | Gazebo joint positions and velocities |

Lift/tilt diagnostics arrays contain:

```text
[joint_angle_rad, joint_velocity_rad_s, reference_angle_rad,
 applied_torque_Nm, equivalent_pressure_bar, allocated_flow_L_min, simulation_time_s]
```

Jaw diagnostic arrays have the same layout, but positions/references are in
meters, velocity is m/s, and applied effort is force in N. Each jaw is limited
to 100 bar by default, whereas lift/tilt retain the 200 bar system limit.

For example:

```bash
gz topic -e -t /tractor/loader/lift/state
```

## Collision and pickup limitations

Boom, mount, carrier, and grip jaws have box collisions; the optional fork fixture does too. Model self-collision
remains disabled, as in the original tractor: tractor/loader links do not collide
with each other. Ground and bale contact remain enabled. A conservative oriented-box tire-envelope check samples the full lift range
and steering limits, plus carrier tilt, independently of self-collision. The
revised arms remain above the tires and drop ahead of their steering envelope.
The mount/pivot is shifted 0.4 m forward to clear the cab. These are simulation
clearance corrections, not measured L115 dimensions. Unsampled poses, flexible
tires, and actual mounting interference still require validation.

The grip holds using jaw contact and friction; no fixed joint attaches the bale.
The jaws do not model bale deformation, crop damage, or a validated pressure
distribution. A wide pad is geometric contact area, not proof of uniform stress. The bales are solid boxes, so
tines in the optional test fixture cannot spear through them. The initial tines
are above the ground, and automatic ground-level insertion/pickup is not solved.
The loaded integration test explicitly prepositions a bale across the tines to
isolate supported lifting from insertion. Full pickup needs a suitable insertion
model or bale support/clearance arrangement; this version does not attach a bale
with an invisible fixed joint.

## Verification commands

```bash
gz sdf -k worlds/simple_tractor.sdf
python3 tests/check_loader_clearance.py
ctest --test-dir build --output-on-failure
python3 tests/check_loader_sim.py
python3 tests/check_loader_sim.py --loaded
python3 tests/check_loader_sim.py --grip
```

The simulation checks run temporary sensor-free copies in unique Gazebo
Transport partitions, leaving the main world and the user's running simulation
untouched. They exercise raising/lowering, tilt/dump, neutral holding, stale
command timeout, and pressure/joint bounds. The loaded variant explicitly replaces the default carrier with the fork test
fixture, starts a 525 kg bale across it, and checks that it rises. It does not
change the production world or select the final project attachment. Gazebo's previously observed
allocator error on process shutdown may still occur and is reported separately
from runtime motion checks.

The corrected raised-arm/grip geometry passes 876,876 tire-envelope/loader box-pair
checks: 73 lift samples, 25 steering samples per front wheel, rear wheel
envelopes, and 16 carrier tilt samples. The minimum separating-axis gap is about
0.053 m, exceeding the check's 0.02 m threshold at all sampled positions. These
are geometric samples, not a continuous swept-volume proof or a field clearance
measurement. The check includes the carrier, rail and both jaws at open/mid/closed strokes;
it does not cover other attachments.

On the installed Gazebo 8.15 / DART 6.13 setup, compilation, the hydraulic math
checks, and both unloaded and supported-load simulation checks passed. In the
loaded case the center of the 525 kg bale rose from approximately 0.620 m after
settling to 2.178 m after a three-second full lift command with the revised arms. This was a
prepositioned support test, not a ground pickup or a rated-capacity test.
The existing allocator error recurred during shutdown and the temporary server
required forced cleanup after the runtime checks completed.

With the grip installed, hydraulic math tests and the contact-only `--grip`
integration test passed. Each jaw contacted the 1.2 m bale at about 0.35 m inward
travel and reached the 100 bar / 12.57 kN force cap. The bale center rose from
0.450 m on the ground to about 1.055 m after raising, stayed off the ground
during forward transport, and returned to about 0.450 m after opening. No fixed
attachment joint or force applied directly to the bale was used. The same run
also passed lift/tilt, neutral, timeout, and pressure/joint-bound checks.
