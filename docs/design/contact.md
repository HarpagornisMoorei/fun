# Design note: contact and higher pairs (M6)

Status: **6a, 6b, 6c and 6d implemented, plus 6e (polygons and the ratchet)** (profiles `circle`/`polar`/`points`,
bilateral kinematic `touch`, `cam` displacement-diagram synthesis with
pressure-angle and undercut reports; the eccentric, lobe-cam and plate-cam
examples; unilateral contact in the dynamics with `jumpcam`; non-circular
gears with `ellipgears`). This note fixed the representation before code was written,
because contact is where the design is hardest to change afterwards.

## Why

M1–M5 can express lower pairs only: `pin`, `slide`, and the gear `mesh`
(which is already a rolling contact, but only between circles). Everything a
general mechanism tool is expected to do next is a higher pair:

| mechanism | needs |
|---|---|
| plate cam + follower | point/roller on an arbitrary closed curve |
| Geneva drive | pin in a slot, engaging and disengaging |
| ratchet and pawl | one-sided contact that separates and re-makes |
| non-circular (elliptical) gears | rolling contact between two arbitrary curves |
| escapement pallets | the same one-sided contact (this retires the `mode` event summary) |

All four are one idea: a **gap function** `g(q)` between two bodies, with
`g = 0` while touching.

## Principles carried over

* **Non-causal.** A contact is a relation, not an assignment. A cam drives a
  follower by constraint; run the same file with the follower driven and the
  cam is solved instead.
* **Small kernel, deep sugar.** The kernel gets one new feature (`profile`)
  and one new joint (`touch`). Cam displacement diagrams, Geneva wheels and
  so on are intent-layer sugar that *synthesizes* profiles, exactly as
  `reduce` synthesizes tooth counts.
* **Honest non-smoothness.** Separation and impact reuse the `mode`
  hybrid machinery and its energy ledger; they do not get a second event
  system.

## Kernel

### `profile` — a curve attached to a body

A profile is a closed (or open) C² planar curve `c(s)` in the host body's
frame, like a `gear` is a pitch circle in it. Kernel forms:

```
profile lobe on cam.A { circle: 0.03 }                  // degenerate: a disc
profile lobe on cam.A { polar: (0 deg, 0.030) (60 deg, 0.034) ... }   // periodic spline
profile slot on wheel.A { points: (0.0, 0.0) (0.01, 0.0) ... open }   // open spline
```

`polar` covers plate cams, ovals and eccentrics (star-shaped about the
anchor); `points` covers everything else. The follower tip is a point, and a
roller of radius `r` is handled by *offsetting* the profile by `r` (the
standard pitch-curve trick) so the solver only ever sees point-on-curve.
Flat-faced followers are a separate, later case (line-on-curve).

### `touch` — point on curve

```
touch follower.A on lobe [roller 0.008]
```

One equation: the signed gap from a body-fixed point to the curve is zero.
It removes one degree of freedom, so Grubler–Kutzbach gains a term: each
bilateral `touch` counts as `k = 1`, like a mesh does today.

## Solver formulation

Let the curve be on body A, the point `p` on body B, `d = p_w − c_w(s)`.
The foot parameter `s` is the closest point: `tau_w(s) · d = 0`, with
`tau_w = R(theta_A) c'(s)`. The row is the signed gap
`g = n_w · d`, `n_w = perp(tau_w) / |tau_w|`.

**`s` is not a generalized coordinate.** This keeps `q` at three entries per
body and the mass matrix untouched. *As implemented in 6a* the foot is not
tracked like the mesh's `psi`: it is searched afresh at every evaluation
(coarse scan over a precomputed outline, then bracketed Newton), so the row
is a pure function of `q` with no hidden state to go stale between LM trial
steps. The cost is a few hundred distance tests per evaluation. Tracking
remains an option if dynamics (6c) makes that cost matter, but a re-make
after separation would then need the global scan anyway.

**First-order Jacobian needs no `ds` term** (envelope theorem; at the foot
`d` is parallel to `n_w`, so `d · dn_w = 0`, and `n_w · tau_w = 0`):

```
dg/d(x_B, y_B) = +n_w            dg/dtheta_B = +n_w · perp(R(theta_B) p_local)
dg/d(x_A, y_A) = -n_w            dg/dtheta_A = -n_w · perp(R(theta_A) c(s))
```

This is the pin-row pattern already in `kinematics.cpp`, projected on the
contact normal. I checked it against central finite differences on an
ellipse (non-circular, rotated, both bodies displaced): agreement to
~1e-10.

**The second-order bias term (derived in 6c; see docs/LANG.md 6.5 for the
formula and its verification).** What follows is the original statement of the
problem, kept for the record: the second-order bias term `J̇ q̇` that dynamics needs.
It differs from the pin case because the contact point slides:
`g̈` picks up a curvature term (`kappa`, and `1 − kappa g` in the foot
rate). The existing finite-difference constraint-acceleration test is the
oracle; the derivation should be written next to the row, as the other
rows are, before the code.

## Kinematic vs dynamic files

* **Kinematic file (rate drives only).** Contact is bilateral — force
  closure is assumed, as in every textbook cam analysis. `mech check` should
  *warn* that closure is not verified, and `mech run` can report the
  contact normal's sign pattern once masses exist.
* **Dynamic file.** Contact is unilateral: `g ≥ 0`, force `λ ≥ 0`,
  `g λ = 0`. Treat each `touch` as an implicit two-state mode, reusing the
  existing integrator:

  | transition | guard | action |
  |---|---|---|
  | touching → separated | `λ` crosses 0 downward | disable the row |
  | separated → touching | `g` crosses 0 downward | impact: project velocities (the existing relock projection), restitution `e` (default 0), loss goes to the ledger's `impact` term |

  This needs two new guard fields beside `theta | omega | advance`:
  `gap` and `force`. Bisection localization already exists.

## Phased plan

| phase | deliverable | acceptance |
|---|---|---|
| **6a** | `profile` (`circle`, `polar`, `points`) + bilateral `touch`, kinematic | eccentric disc + knife-edge follower matches `y = e sinθ + sqrt(R² − e² cos²θ)` to solver tolerance; Jacobian finite-difference test |
| **6b** | `cam` displacement-diagram sugar (rise / dwell / return; cycloidal, harmonic, modified-trapezoid laws); `mech check` reports pressure angle and **undercut** (min radius of curvature vs roller radius) | dwell is exactly flat; pressure angle matches hand calculation |
| **6c** | unilateral `touch` in dynamics, `gap`/`force` guards, restitution, ledger | follower with spring preload jumps at high speed; ledger closes |
| **6d** | `roll` contact (arc-length matching of two profiles) generalizing `mesh` | elliptical gear pair; a circle–circle `roll` reproduces `mesh` exactly |

**6b as implemented, and where it departed from the table.**

* Segments are *sequential* (`rise 0.012 over 100 deg`, `dwell 60 deg`), not
  absolute angle ranges (`over 0..120 deg`): they cannot overlap or leave gaps,
  and a bare final `dwell` fills the rest of the turn.
* **Undercut is an error, not a report.** A cam whose inward offset folds over
  itself cannot be cut, so there is nothing sensible to simulate.
* The pressure angle is the diagram formula `tan(a) = s'/r`, exact for a rail
  through the axle, which is what `cam` synthesizes. A pressure-angle sweep
  for *arbitrary* profiles and followers (offset rails, swing arms) needs the
  follower's allowed direction of motion at each pose and is left for later.
* A `touch` on a cam defaults to the roller the cam was cut for; an explicit
  different roller draws an assembly warning.
* `modtrap` has the lowest peak *acceleration* of the laws that ease in and
  out (4.888 vs 6.283), but the same peak velocity as `cycloidal` (2.0);
  `harmonic` has the lowest peak velocity (1.571) but an acceleration jump
  at each end, which a C2 spline surface cannot represent (normal off by
  ~0.045 degrees at those two junctions).

**6c as implemented, and where it departed from the table.**

* The guards are *the contact force going negative* (lift-off) and *the gap
  closing* (touch-down), reused on the existing bisection loop, which now
  also runs for files with contacts but no `mode`. Events are processed
  earliest-first rather than in declaration order, so one cannot mask another.
* The energy ledger needed a new term, **drive work**. A cam on a rate drive
  does work on the system continuously and, because it is "infinitely heavy",
  during every impact on it; without that term the ledger cannot close for
  any driven mechanism. The impact's own dissipation is `impulse work - dT`.
* Dynamic runs now start from velocities consistent with the rate drives. The
  old zero-velocity start spun a driven shaft up impulsively in the first step
  — harmless, but it made the ledger open by the shaft's kinetic energy.
* **Plastic cutoff.** A first implementation bounced a resting follower
  forever. The cause was numerical, not physical: a bounce restarts inside the
  contact tolerance band, and the free acceleration turns that sliver into a
  landing speed floor of `sqrt(2 a delta)`, above the cutoff I had picked.
  Landings under 2 mm/s are now plastic, which clears that floor with margin.
* A guard that was "already true" at the start of a sub-step because the body
  sat 1e-13 *inside* the surface after a bounce silently lost the next
  landing. Touch-down is now armed within the contact band.
* Friction is still deferred; the sliding speed `u` it needs falls out of the
  bias term.

**6d as implemented, and where it departed from the table.**

The table proposed a `roll` contact (arc-length matching of two profiles). What
shipped is `mesh` over two profiles, and the reasons are worth recording.

* *No new keyword.* A gear mesh is already "roll without slip between two pitch
  curves"; a non-circular gear is the same statement with `r(phi)` in place of a
  constant radius, so `mesh` accepts two profiles.
* *No arc-length rows.* Arc-length matching between arbitrary curves needs the two
  contact points as unknowns (an auxiliary coordinate, or implicit derivatives of a
  closest-pair search), threaded through the solver and the mass matrix. For fixed
  shafts the contact point always lies on the line of centers, which makes the whole
  relation a function of the two shaft angles alone — one row, no new unknowns,
  and it carries over to the dynamics with only `r_A'`.
* *The row is the tangent branch, not `r_A + r_B = d`.* That algebraic form is
  singular at the symmetric poses (the Jacobian is exactly zero where `r'`
  vanishes) and has two mirrored solutions, one not tangent. Integrating the
  instantaneous-center ratio gives `thB - thB0 - F(psi - thA)` with Jacobian
  `(rho, 1)`: never singular, and on the right branch by construction.
* *Conjugacy is verified, not assumed.* The row only describes rolling for a
  conjugate pair, so the compiler sweeps the whole turn and refuses a pair that
  crosses instead of rolling. Arc-length equality is then a *test* (independent
  brute-force geometry on the solved motion), not a constraint.
* *Scope.* Fixed centers and external contact. Moving centers (a carrier) and
  internal contact need the contact point as an unknown after all; that, and
  rolling between curves that are not a conjugate pair, is what the arc-length
  formulation would buy, and is left for later.
* *A solver bug surfaced.* The multi-start restarts had always been broken (an
  unsigned `(i % 3) - 1` wrapped to 2^64 - 1 and sent every angle coordinate to
  ~1e18), and a failed solve left `q` at the last attempt rather than the closest.
  It never mattered because the first attempt nearly always converges; an
  elliptical pair started on its singular symmetric pose was the first mechanism
  to need a restart. Both are fixed.

**6e: sharp-cornered parts (not in the original plan).** The Geneva drive and the
ratchet both need corners, and the spline-based `points` profile cannot hold one (a
cubic spline through a corner rings by tens of percent, and the contact normal
with it). `polygon` is therefore an *exact* curve of edges and tangent circular
fillets, not a spline, with an analytic nearest-point search; `fillet` is required
because a perfectly sharp corner has no contact normal. The ratchet is the
sharpest test of 6c — contact made and lost once per tooth, an impact each time.

What it taught: (1) **the ledger needed torque work** (a torque drive does work too;
without it a torque-driven contact mechanism reported a misleading residual);
(2) **a frictionless locked pawl rings** — the system is conservative, so it
oscillates about the jam instead of settling, and the right assertion is the
wheel's own equation of motion (impulse-momentum, to 0.01%) rather than "at
rest"; (3) a pawl spring strong enough to matter *stalls* the forward direction
(its normal force on the ramp cancels the drive torque), which is correct physics.
The Geneva drive remains open: it needs two-sided slot contact and a locking arc
(curve-against-curve contact, not point-on-curve).

Acceptance examples to add under `examples/`: `eccentric`, `platecam`,
`geneva`, `ratchet`, `ellipgears`.

## Risks and open points

* **Closest-point ambiguity.** On non-convex outlines the foot can jump.
  Continuity tracking handles smooth rolling; a re-make after separation
  needs the global scan. Multiple simultaneous contact on one curve is out
  of scope for 6a.
* **Undercut.** If the offset curve's radius of curvature goes non-positive
  the pitch curve cusps. Detect and report at `mech check` time, do not
  silently produce garbage.
* **Open slot vs closed curve.** A Geneva slot is contact on *both* sides, so
  it is two `touch` rows on offset curves (or one two-sided row). Which one
  is a question for 6d, not for 6a.
* **Units.** SI throughout, as today; `mm` could become a unit word later,
  since cam dimensions are awkward in metres.
* **Friction.** Deliberately deferred. The tangential slip speed is
  available from the foot rate `ds/dt`, so Coulomb friction can be added
  without changing the row layout.
