# The standard parts (`lib/`)

Five parts ship with the tool. Each is an ordinary `part` (see `PARTS.md`), imported by path:

```
import "../lib/plate_cam.mech"
```

The path is resolved next to the importing file, then in each directory of `MECH_PATH`
(colon-separated). None of them carries a `drive`: the mechanism decides what turns them,
which is what lets a gear, another part, or a motor do it.

Every one is tested against a closed form (`mech test`, the `[imports M10]` suite), not
just run: the formula is written out in the test and compared to the simulation.

## `plate_cam.mech`

A plate cam with a translating roller follower, cut from a displacement diagram.

| parameter | default | meaning |
|---|---|---|
| `lift` | 0.012 | follower stroke, m |
| `base_r` | 0.025 | base circle radius, m |
| `roller_r` | 0.006 | roller radius, m |
| `rise_deg` | 100 | cam angle of the rise (cycloidal) |
| `dwell_deg` | 60 | dwell at the top |
| `return_deg` | 140 | cam angle of the return (modified trapezoid); then it dwells the rest of the turn |
| `rail_h` | 0.09 | height of the rail anchor; only has to be above `base_r + roller_r + lift` |

Ports: `cam` (the shaft), `lobe` (the surface), `follower` (`follower.tip` is the roller centre).
Closed form checked: the roller centre rests at `base_r + roller_r` above the axle and peaks at
`base_r + roller_r + lift`. The compiler's cam checks (undercut, pressure angle) run as usual,
so `mech sweep ... --set c.rise_deg=...` finds the shortest rise that can be cut.

## `crank_slider.mech`

A slider-crank. Parameters: `crank_len` (0.3), `rod_len` (1.0), `offset` (0; the rail's height
above the crank axle). Ports: `crank` and `conrod` (rods; A, B ends), `piston` (`piston.guide` is the
wrist pin).

Closed form checked over a full turn, for `offset` 0 and non-zero:
`x(t) = r cos t + sqrt(l² − (r sin t − e)²)`, with the piston on the line `y = e` the whole time.
The dead centres (`l + r`, `l − r`) are checked too.

## `four_bar.mech`

A four-bar linkage with the ground link along +x. Parameters: `ground_len` (0.4), `crank_len` (0.1),
`coupler_len` (0.35), `rocker_len` (0.3). Ports: `crank`, `coupler`, `rocker`. The defaults make a
crank-rocker: the crank turns fully and the rocker rocks.

Closed form checked at every sample of a full crank turn, Freudenstein's equation
`cos(θ − φ) = K3 + (d/a) cos φ − (d/c) cos θ`, `K3 = (a² − b² + c² + d²)/(2ac)`.

## `gear_pair.mech`

Two spur gears on parallel shafts, centres `module × (teeth_a + teeth_b)/2` apart. Parameters:
`teeth_a` (20), `teeth_b` (40), `module` (0.005). Ports: shafts `sa`, `sb`; gears `ga`, `gb`.

Closed form checked for three tooth pairs: `Δθb = −(teeth_a/teeth_b) Δθa` (the gears start at a mesh
phase the compiler picks, so the comparison is of changes), and the centre distance.

## `ratchet.mech`

A ratchet wheel with `teeth` generated sawtooth teeth and a gravity pawl. Parameters: `teeth` (12), `r_root`
(0.030), `r_tip` (0.036), `fillet_r` (0.0005), `pawl_len` (0.04), `pawl_deg` (-40), `wheel_mass` (0.05), `mu` (0.3).
Ports: `wheel`, `pawl` (`pawl.B` the tip), `rim`. A dynamic part: the mechanism supplies the `world` and the
torque (`drive r.wheel torque -0.004`, clockwise, is the ratcheting direction).

The tooth outline is a `for` loop inside `polygon:` (vertex pairs root_k, tip_k at `pi/2 + k * 2 pi / teeth`), so a
root is always at the top where the pawl rests, whatever the count. The tooth count is a parameter and a sweep axis.
Checked against closed forms: the polygon's area `teeth * (1/2) r_root r_tip sin(2 pi / teeth)`; about one impact
per tooth of advance for 6 to 16 teeth; locked within a tenth of a tooth under reverse torque; the energy ledger
closing. The defaults suit about 6 to 16 teeth: at 24 the teeth are too fine for this pawl and it chatters (32
impacts, hardly any advance), which is a property of the geometry and not of the part.

## Placing them at an angle

Every library part is written upright (a vertical cam rail, a ground link along +x) and is placed
with a rotated mount: `use plate_cam as east on bench at (0.04, 0) rotate -90 deg`. The rotation
turns the part's ground and the angles inside it (see `PARTS.md`, section 3); a cam keeps its lift
against its own shaft angle whichever way its rail points. `examples/camstar.mech` mounts four cams
around a hub with their followers pushing outward.

## Using them together

`examples/camshaft.mech` imports `plate_cam` once and uses it twice, then puts a gear on each
instance's `cam` shaft from outside and meshes them. The library part has no idea it is geared; the
mechanism composes it.

## Writing your own

A library file is a `.mech` file with parts and nothing else (it may import other libraries). Put
the formula that tells you the part is right in the header comment, and a test for it next to
the others.

