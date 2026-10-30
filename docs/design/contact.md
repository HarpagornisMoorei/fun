# Design note: contact and higher pairs (M6)

Status: proposal, nothing implemented. This note fixes the representation
before any code is written, because contact is where the design is hardest
to change afterwards.

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

**`s` is not a generalized coordinate.** It is tracked like the mesh's
`psi`: warm-started from the previous evaluation, refined by a scalar Newton
iteration, and found by a coarse global scan only on first use or after a
re-make. This keeps `q` at three entries per body and the mass matrix
untouched.

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

**Not yet derived: the second-order bias term** `J̇ q̇` that dynamics needs.
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
