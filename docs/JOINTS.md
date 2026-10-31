# Prismatic joints, slots and linear drives (M14)

Until M13 a `slide` could only run on a rail fixed to the ground, and every drive was a rotation. That left out
a whole family of mechanisms: anything whose sliding part runs along a *moving* body (a telescoping arm, a
hydraulic cylinder, a quick-return lever), anything where a pin runs in a slot cut in another moving part
(a Scotch yoke), and any linear actuator. M14 adds them.

## `slide`: a prismatic joint between any two bodies

```
slide A.guide -> B.rail
```

The guide anchor `A.guide` is kept on the line through the rail anchor `B.rail`, and the two directions are
kept parallel. Both anchors need a `dir`. Either body may be the ground or any moving body. Written
ground-first (`slide frame.rail -> piston.guide`) it keeps its old meaning: the moving body is the guide.
Two ground anchors, or one body on itself, are errors.

The slide removes two degrees of freedom: the guide's offset from the rail line and its rotation relative to
the rail. What is left is how far the guide has travelled along the rail, `s = u . (guide - rail anchor)`, with
`u` the rail's direction. `s` is the slide's coordinate.

A free slide starts with its guide at the rail anchor (s = 0); put the rail anchor where you want the part to
start, as the rotating tube in `examples/slidingbead.mech` does (the anchor `R` is 0.1 m out).

## `slot`: a point on a line fixed in another body

```
slot P.pin -> L.slot
```

Only the first constraint: the point `P.pin` is kept on the line through `L.slot` (which needs a `dir`). The point
needs no direction and the two bodies may turn freely against each other. This is a pin-in-slot, a **slot** removes
one degree of freedom where a slide removes two. It is what a Scotch yoke and a Whitworth quick-return are made of.

A slot is a *line*, not a half-line: the pin may be on either side of the slot's anchor, and if a mechanism
passes through the anchor the body can come out the other side. Keep the pin on one side by the geometry (a
quick-return crank is shorter than the distance between the pivots).

## Linear drives

```
drive piston.guide at 0.02 m/s from 0.50 m     // the rate form: s = 0.50 + 0.02 t
drive piston.guide force 500                   // a constant force, N, along the rail
```

A linear drive names the slide by its **guide anchor** (`body.anchor`, the first anchor of the `slide` or `slot`
statement). `m/s` is a unit like `deg/s`.

* **Rate drive.** Prescribes `s(t) = s0 + rate t` as one more constraint, like a rotating drive does for an
  angle. `from X m` is `s0`, measured from the rail anchor (default 0; the `m` is optional). It removes one
  degree of freedom, and its multiplier is the force the actuator must deliver.
* **Force drive.** A force along the rail pushes the guide forward and the rail's body back (equal and
  opposite), like a spring or a torque drive it is a force, not a constraint; it makes the mechanism dynamic.

A slide has at most one linear drive.

## Where a free body's mass sits

Sizing an actuator needs the boom's weight where it really acts. A `body` has its mass at its frame origin unless told:

```
body boom { anchor A at (0, 0)  anchor tip at (1, 0)  mass: 10  com: (0.5, 0)  inertia: 0.8 }
```

`com: (x, y)` is the centre of mass in the body's frame and `inertia: I` the moment of inertia about it (kg m²,
default 0.01 of the mass). Without them a free body is a point mass at its origin, as before. (A `rod` already
has its mass along its length.)

## What you read out

`mech run --headless --csv` gains two kinds of column:

* `<guide>.travel[m]` for every slide and slot: how far the guide has travelled along its rail;
* `<guide>.force[N]` for every linear drive (dynamic runs): for a rate drive the force the actuator delivers along
  the rail (positive extends), for a force drive the force it applies. The energy ledger counts the actuator's work.

This is the number for sizing a hydraulic cylinder: run the motion, read the force column.

## Examples, and the closed forms they are tested against

| example | shows | closed form |
|---|---|---|
| `scotchyoke` | pin in a slot on a sliding yoke | yoke x = r cos θ, exactly |
| `quickreturn` | crank pin in a slot on a pivoted lever | lever angle = atan2(r sin θ, d + r cos θ); forward 180 + 2 asin(r/d) degrees of crank, return 180 − 2 asin(r/d) (here 240 and 120: a ratio of 2) |
| `boom` | a cylinder and piston, both moving, driving a boom | the actuator length from the boom angle (law of cosines) is 0.50 + 0.02 t |
| `slidingbead` | a bead free on a rotating tube (dynamic) | r = r₀ cosh(ω t): the centrifugal term, with the ledger closing |

Also checked: a mass under a constant force (x = F t² / 2m, the ledger closing with the force's work); a bead on a
rotating tube held to a constant radial speed (the force needed is exactly −m ω² r); a boom held still (the
cylinder's force against the torque balance about the pivot); a physical pendulum from `com`/`inertia`
(period 2π√((I + m d²)/(m g d))); and, for every new row type, the second-order terms by the finite-difference
trajectory test (a correct bias scales like h³, a wrong one like h²).

## The dynamics

The solver is the existing one: every row has an analytic Jacobian and a velocity-dependent bias, and the slide rows
are derived the same way. For the guide's anchor point `pA`, the rail anchor `pB`, `v = pA − pB`, `u` the rail
direction (turning at the rail body's rate `wB`) and `vdot` the relative velocity of the two points:

```
offset row   f1 = u x v          bias = (−wB² u) x v + 2 (wB perp(u)) x vdot + u x (−wA² rA + wB² rB)
direction    f2 = a x u          bias = −(wA − wB)² f2
linear drive f3 = u . v − target bias = (−wB² u) . v + 2 wB perp(u) . vdot + u . (−wA² rA + wB² rB)
```

(`x` is the 2-D cross product, `a` the guide's direction, `rA`, `rB` the offsets of the two anchors from their
bodies' origins.) For a ground rail `wB = 0` and these reduce to the old rows. The Coriolis term
`2 wB perp(u) x vdot` is what a bead sliding in a rotating tube needs.

## A numerical fix that came with it

The multipliers come from solving `A λ = rhs` with a small Tikhonov damping. It used to be one absolute number
scaled by the *largest* diagonal entry of `A`, so with masses spanning many decades (a 10 kg boom and light
cylinder parts) it swamped the rows that come from the heavy bodies and skewed the actuator force by percent.
It is now relative to each row's own diagonal; a test holds a boom with its cylinder and piston from 10⁻² to
10⁻⁶ kg and the force stays on the torque balance.

## Limits

* The viewer (`mech run` without `--headless`) does not draw a rail that rides a moving body (the SVG does: `mech render`).
* A slot is a full line, not a half-line, and a slide has no end stops: the guide can run off either end of its
  rail. Limits and a hydraulic's stroke are not modelled.
* One linear drive per slide; a prismatic drive with a speed-dependent (hydraulic) force law is not modelled.
* Rack-and-pinion, belts and cables are still missing, and so are joint friction and backlash.
