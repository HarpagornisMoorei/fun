# Rack and pinion, belts and chains (M16)

Two more ways to move motion around: a pinion that rolls along a straight rack, and a belt (or chain) looped
over pulleys. Neither adds a new kind of joint to the solver. Each is a rolling-without-slipping relation, like a gear
mesh, so the rows stay ordinary constraint rows and the dynamics, sweeps, CSV output and drawings all work on them.

## `mesh PINION on BODY.ANCHOR`: a pinion on a rack

```
ground frame { anchor hub at (0, 0.03)  anchor rail at (0, 0) dir (1, 0) }
body wheel { anchor C at (0, 0) }   pin wheel.C -> frame.hub
gear pinion { teeth: 30  module: 0.002 } on wheel.C

body rack { anchor G at (0, 0) dir (1, 0) }   slide rack.G -> frame.rail
mesh pinion on rack.G                          // the pitch circle rolls on the line through G
```

The **rack** is a straight pitch line: the line through a rack anchor, along the anchor's `dir`. It rides the body
the anchor is on, so the rack can be on the ground (a wheel rolling along a fixed rack), on a slider (the usual
case), or on a body that turns. The pinion is any `gear`, and it can be on any moving body.

* The pinion's pitch circle must touch the pitch line when the model assembles: `mech check` reports
  `pinion 'p' is 0.040000 m from the rack line, but its pitch radius is 0.030000`. Teeth are not modelled (as for gear
  meshes); the tooth marks in the drawing are for the eye.
* Which side of the line the pinion is on is read from the assembled pose, so a pinion below the line runs the
  other way (the rack moves the opposite way for the same turn). The tests check both.
* The relation is *rolling*: with `s` the position of the pinion's centre along the line (measured from the
  anchor), `s + side · r · (θ_pinion − θ_rack) = constant`. A rack on a turning body is exact too: the rack's own
  turn cancels out of the rate of change, because the centre stays at distance `r` from the line.

### Two rows or one

A rack costs one row: the rolling relation. The pinion's centre is held at distance `r` from the line by the rest of the
mechanism (a pin to the ground, a carrier), which is what you want, and the mobility count (`mech check`) shows it
that way. If *nothing else* holds the centre there, say `tangent`:

```
mesh tyre on frame.slope tangent
```

and the mesh adds a second row that keeps the pitch circle on the line, exactly as `tangent` does for gears. A wheel
rolling down a slope (`examples/rollingwheel.mech`) needs it; a pinion on a fixed shaft must not (the row would be
redundant with the pin).

## `belt NAME g1 -> g2 -> g3`: a belt over pulleys

```
gear ga { radius: 0.02 } on pa.s
gear gb { radius: 0.05 } on pb.s
gear gc { radius: 0.03 } on pc.s
belt strap ga -> gb -> gc reverse gc      // gc is crossed: the belt makes a figure of eight there
```

The pulleys are `gear`s used at their pitch radii (`radius:` or `teeth:` and `module:`). A belt of fixed length makes every
pulley's rim move at the same speed, so

```
s_i · r_i · θ_i      is the same for every pulley i  (s_i = -1 for a pulley after `reverse`)
```

and the output turns at `r_in / r_out` of the input. A chain is the same thing: list the sprockets.

* List the pulleys in the order the belt visits them; the belt closes from the last back to the first.
* Every pulley's centre must be a fixed point: its body is pinned to the ground at the gear's anchor. A belt over
  pulleys that move would change length, which the model does not capture, so it is refused (`not on a fixed shaft`).
* Pulleys on one shaft cannot be joined by a belt, a pulley cannot appear twice, and two pulleys so close that one
  nests inside the other have no tangent between them (`mech check` says so).
* Slip, stretch and sag are not modelled, nor are tension limits (a belt can push in the model as well as pull).
  The reported force is the force transmitted across a link.

## What you read out

`mech run --headless --csv` adds, in a dynamic mechanism:

| column | meaning |
|---|---|
| `<pinion>.rack[N]` | the tangential force between the pinion and the rack (the multiplier of the rolling row) |
| `<belt>.link<i>[N]` | the force carried by the belt between the *i*th and *(i+1)*th pulley (the difference of the tensions) |

`mech check` prints a line per rack and per belt with the speed ratio.

## Drawings

`mech render` and the OpenGL viewer draw them from the same scene (`src/scene.*`): the rack as a toothed bar along its pitch
line (as long as the pinion's travel needs), the belt as straight runs tangent to the pulleys and arcs where it wraps
them, with small marks that travel with the belt so the motion is visible. A crossed pulley is wrapped the other way, and
the run between unlike wraps is the inner tangent. The tests read the SVG back: every run ends on its pulleys' pitch
circles, perpendicular to the radius, with length `sqrt(L² − (r₁ ∓ r₂)²)`.

## Examples, and the closed forms they are tested against

| file | shows | oracle |
|---|---|---|
| `examples/rackpinion.mech` | a 30-tooth pinion drives a rack along a rail | rack x = r θ (31.4 mm per second at 60°/s); −r θ with the pinion below the line |
| `examples/rollingwheel.mech` | a disc rolling down a toothed incline | ω = g sin a /(r(1 + I/(m r²))); contact force `m g sin a · k/(1+k)`, k = I/(m r²) — for 20°, 30° and 55° |
| `examples/beltdrive.mech` | a belt over three pulleys, one crossed | θ_b = (r_a/r_b) θ_a, θ_c = −(r_a/r_c) θ_a |

Also tested: a belt drive loaded by a flywheel has the reflected inertia `I₁ + I₂ (r₁/r₂)²` and carries `I₂ α₂ / r₂`; a rack on a
turning body keeps the pinion turning with it; the velocity-level bias of the rack rows agrees with finite differences
(`biasScalingRatio`: the drift falls by 8 per halving of the step).

## Limits

* The rack is straight and its teeth are not modelled: no backlash, no tooth phase, no contact loss. For a rack and
  pinion with the pinion held off its pitch line (a loose mesh) there is no unilateral contact.
* A belt does not slip, stretch or go slack, and its pulleys must be on fixed shafts.
* A rack's pitch line is infinite; the bar is drawn as long as the pinion's travel and a margin.
