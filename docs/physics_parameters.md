# Rigid-body physics baseline

## Collisions and friction

Bales use centered 2.4 × 1.2 × 0.9 m box collisions matching their visuals.
Tractor chassis, hood, and cab retain box collisions; wheels retain cylinder
collisions. The ground remains a static box. All three bales are now dynamic.
These simple shapes support contact, sliding, and tipping. They do not model
soil deformation, bale compression, tire tread, or a fork penetrating a bale.

Both friction directions are equal: ground 0.55, each wheel 1.0, each bale 0.4.
These are per-surface simulation parameters, not experimentally validated
material properties or a direct specification of the contacting pair's friction.

## Reference masses and assumptions

Each bale is 525 kg, matching the approximate hay-bale mass and dimensions
published by [KK-Heuhandel](https://kk-heuhandel.de/eng/products-hd-bales-and-large-bales/).
This is a hay baseline, not a universal straw-bale mass. Its volume is 2.592 m³
and implied density is about 202.55 kg/m³. Actual crop, moisture, and compaction
will change the mass.

The tractor totals 5,500 kg, using a provisional Maxxum-family baseline from a
[CNH-published Maxxum T3 brochure](https://www.stevensonandtaylor.co.nz/wp-content/uploads/Case-IH-Maxxum-T3-WEB-MAY23-1.pdf).
That brochure covers the 110/125/140 family; this value is a proxy, not a verified
weight for the project's specific Maxxum 150. Replace it with the real tractor's
weight and axle loads when available. No loader, fork, or additional ballast mass
is included. The following link mass distribution is an engineering assumption,
not a manufacturer-provided component breakdown:

| Link | Mass (kg) |
| --- | ---: |
| Chassis (lumped chassis, drivetrain, and remaining equipment) | 3,480 |
| Hood (lumped engine and front equipment) | 700 |
| Cab | 500 |
| Each front wheel | 150 |
| Each rear wheel | 250 |
| Each steering hub | 10 |
| **Total tractor** | **5,500** |

## Inertia

Inertia is calculated, not measured. Each link's center of mass is at its link
origin. All products of inertia are zero in these symmetric primitive frames.
The body's combined center of mass follows from the link masses and poses.

For a uniform box with mass `m` and side lengths `x`, `y`, `z`:

```text
Ixx = m (y² + z²) / 12
Iyy = m (x² + z²) / 12
Izz = m (x² + y²) / 12
```

Each bale therefore has `(Ixx, Iyy, Izz) = (98.4375, 287.4375, 315)` kg·m².
Chassis, hood, and cab use their existing collision-box dimensions.

For a uniform cylinder of radius `r`, length `L`, and local Z cylinder axis:

```text
Ixx = Iyy = m (3r² + L²) / 12
Izz = m r² / 2
```

Wheel inertia is expressed in the wheel link's local frame. The link's existing
90° roll rotates the cylinder axis into the tractor's axle direction. Steering
hubs use the inertia of a nominal 0.1 m cube with mass 10 kg, without adding
collision geometry to those hubs.

## Trying contact

Restart Gazebo after edits:

```bash
gz sim -r worlds/simple_tractor.sdf
```

In another terminal:

```bash
python3 worlds/teleop_wasd.py
```

Approach the center bale slowly, initially at 0.3–0.5 m/s, and press Space to stop.
The bale should move under sustained contact rather than behave as an anchored
obstacle. Pushing is a physics check, not the intended fork-based transfer method.
The Ackermann controller commands wheel velocity; finite engine torque, tire
compliance, and a validated drivetrain are not modeled. Successful pushing alone
does not establish realistic tractor pulling or pushing capacity.

## Validation performed

The updated world passes `gz sdf -k` validation. Link masses sum to 5,500 kg and
all inertia tensors satisfy positive principal moments and triangle inequalities.
A temporary sensor-free copy of the world was tested with the installed DART
backend, leaving the project world's cameras intact. All bales settled at center
height 0.45 m. After a 0.5 m/s forward command for 16 seconds, the center bale
moved from x = 7.0 m to about 10.604 m; the other bales stayed in place and all
three remained at ground level. This verifies basic contact and pushing.

Gazebo reported `malloc_consolidate(): unaligned fastbin chunk detected` during
shutdown and the temporary process required forced termination. This shutdown
issue also appeared in earlier contact checks and remains unresolved; the
runtime contact measurements above succeeded before shutdown.
