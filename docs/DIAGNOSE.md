# Is the pose regular? (M22)

`assembly converged [ok] (max residual 1e-16)` says every constraint is met. It does not say the mechanism is where its author meant it to be.
A linkage can satisfy all its constraints on a **folded branch** (two rods lying on top of each other), on a **dead point**, at the flat pose
where it **branches**, or, when driven too far, not at all. The solver cannot tell these apart, because to the solver they are all solutions. The
Jacobian can: at a regular pose its rows (one per active constraint) are independent, and the nearer a pose is to a singular one, the more nearly
some row is a combination of the others.

`mech check` and `mech run` now look at that, once at the assembled pose and at every step of a run. Nothing here changes what the solvers
compute. The test suite checks that a monitored run ends in bit-for-bit the same state as an unmonitored one, and a byte comparison of the CSV and the
printed table of every example against the build before M22 found no difference.

## The measure: a separation angle

Take the rows of the Jacobian in a fixed order. The angle between row *i* and the span of the rows before it says how far that constraint is from
being implied by the others: 90 degrees for rows that have nothing to do with each other, 0 for a dependent row. It comes from a Cholesky
factorization of the Gram matrix (the *i*-th pivot is the squared length of the part of row *i* the earlier rows do not span), done in the same
sparse skyline form the solvers use, so it costs about one more Jacobian assembly per step.

Two things make the number mean the same thing for a clock and for a crane:

* it is an angle, so it does not depend on how long a row is, and the units a constraint is written in do not matter (a pin row is in metres, a drive row in radians);
* the unknowns are measured as displacements: a rotation of a body that reaches *l* metres from its origin moves its far end by *l* times the angle,
  so the angle's column is weighted by 1/*l*. The reach is the farthest anchor, gear or profile the body carries (a cam shaft reaches as far as its cam does) and, at the pose, the farthest a row actually reaches
  (a bead that has slid 27 m along a tube makes that tube's angle worth 27 m of motion, whatever the tube was drawn with). A body that reaches nowhere is taken to reach as far as
  the largest, and a small part is not given more reach to make it count for more: a train of 1 mm gears inside a 1.5 m mechanism is as regular as the same train drawn big
  (clamping the reach from below would report such a train 5 degrees from dependent where it is 30).

The same linkage drawn ten times bigger, or with its equations in other units, has the same separation angles: to rounding (about 1e-15 in a scratch check up to a thousand times bigger; the test suite asserts 1e-9).
Typical values are exact fractions of familiar angles: 90 degrees for an unconnected constraint, 45 for two joined at a right angle, 30, 35.26.

| separation of the closest constraint | meaning |
|---|---|
| below 0.0018 degrees (`kDependentRatio = 1e-9` as sin²) | **dependent**: a rank deficiency; the constraint is implied by the others |
| below 0.57 degrees (`kNearRatio = 1e-4`) | **near-singular**: within that angle of a dependent pose |
| above | regular |

Every example in `examples/` starts at least 7.7 degrees from dependent (the hydraulic boom, near the end of its stroke). Over a run of 12 s (3 s for
dynamic files) none gets closer than 13 degrees (the Geneva drive), except the two that are about reaching a singular pose: the boom once it runs past its
stroke, and the Peaucellier cell at its bifurcation and again at the other end of its range (below).

## At assembly: `mech check`, `mech run`

```
assembly converged [ok] (max residual 4.44e-16, 9 iterations)
pose regular: the 9 constraints are independent (closest to dependent: 31.0 deg) [ok]
```

Three other outcomes. `mech run` prints these findings (not the `[ok]` line) to standard error before it starts, as `warning:` or `note:`; none stops the run.

**A folded branch.** Bodies that lie in exactly the same place while the Jacobian has lost rank:

```
[warn] singular pose: 1 of 21 constraints are dependent here (rank 20)
  oq and os, pq and ps, qr and sr lie, pair by pair, in exactly the same place: a folded branch, with two copies of one chain on top of each other
  the solver found it because the bodies started alike; start the bodies of one chain near where they belong with `from: <angle> deg` in their braces (a rough angle is enough)
```

Rods and bodies are started at a default tilt that depends only on their order, so a symmetric linkage (a rhombus, a parallelogram, a mirrored pair)
can fall into its symmetric solution, which satisfies every constraint and does nothing useful. `from:` seeds the solver; seeds up to some 30 degrees
off found the right assembly in the Peaucellier cell, and all the bodies of the chain need one (seeding only the mirrored half is not enough).

**Dependent constraints the count already admits.** More constraints than unknowns, as when two pins hold one plate. A note, not an alarm:

```
1 dependent constraint, as the count of rows against unknowns says (over-constrained)
```

**Dependent constraints the count does not explain.**

```
singular pose: 1 of 12 constraints are dependent here (rank 11)
  dependent: pin c3.B -> coupler.r (y)
  this pose is a dead point or a branch point, or the design is over-constrained on purpose (parallel cranks, say); if it is a dead point, start a body elsewhere with `from:`
```

The rank alone cannot say which: a mechanism with three equal parallel cranks has a dependent constraint at every pose, by design, while a slider-crank
started at full stretch has one by accident. So this is a note, and the constraint it names is one of the dependent set (which of them is first depends on the order
the rows are taken in).

**What it cannot see.** A different *regular* assembly mode. The same cell can also assemble with its far corner R lying on P (the rhombus folded along its other
diagonal): the Jacobian there is full rank, 19 degrees from dependent, and the mechanism is simply a different one from the one intended. The same is true of an
open and a crossed four-bar. Only the author knows which is meant.

## Along a run: `mech run --headless`, and the window

`mech run` watches the Jacobian at every step of a run, kinematic or dynamic. What it reports, once per stretch of the run, after the table (headless) or as the
stretch begins (window):

```
warning: singular configuration at t = 3.517 .. 3.567 s: the constraints came within 0.17 deg of dependent (closest at t = 3.55 s)
  involving, for instance: pin qr.B -> sr.B (y)
  the solver may continue on a different branch after this point; if the motion after it looks wrong, keep the drive away from this pose
```

and, when the mechanism has run out of range:

```
warning: singular configuration at t = 8.083 .. 12 s: 1 more constraint became dependent on the others (rank 8 of 9)
  involving, for instance: drive piston.guide
  the constraints could no longer be met there (largest residual 0.019): the mechanism has gone past the end of its range, and what the run shows after it is not a pose of the mechanism
```

The two are told apart by the constraint residual the run already computes: a branch point meets its constraints, an end of range does not. In the window the second
message follows the first as a separate line when the residual shows it (a few steps in). The monitor never evaluates residuals itself: every evaluation updates the
state the solvers track from step to step (a gear mesh's centre-line angle), and that changes the last digits of a run. Measured: one extra evaluation of the residuals per step
leaves the tracked angle different in six of the examples with gear meshes, and changes the final pose of `camstar` and `orbit` bit for bit.

*Normal* redundancy is not news. The first sample fixes how many constraints are dependent to begin with (a mechanism over-constrained on purpose), and only more of
them are reported; the count is taken again whenever the number of rows changes (a mode hold, a contact engaging or lifting off).

The exit status is not changed by any of this, and the CSV and the printed table are what they were.

### What it can promise

It samples. A singular configuration that the run steps over, with no sample landing within 0.57 degrees of it, goes unseen, and nothing says so. For the Peaucellier
cell, whose near-singular stretch is about 2 degrees of crank wide, frames of 0.5 to 2.25 degrees of crank travel (`--dt` 1/60 to 0.075 s at 30 deg/s) all saw it in these runs
(a frame of 2 degrees or less always does), and frames of 3 degrees and more did not (closest approach seen at 3 degrees: 0.85). A fast drive makes a coarse frame: 120 rpm is 12 degrees per frame at the default 1/60 s. If a run moves its drive
more than a degree or two per frame, lower `--dt` before trusting its silence.

`closest approach` is what the samples saw; the monitor keeps it (`RunMonitor::closestDegrees`) but a run prints only what it flagged.

## Two worked examples

**`examples/peaucellier.mech`.** A Peaucellier-Lipkin cell: the far corner R of the rhombus is the inverse of P in O, so R runs along x = (L² − l²)/2c = 0.36 m. Assembled
without `from:` seeds the solver folds the rhombus (above). Seeded, the cell is regular (closest 20 degrees), and `qr.B.x` is 0.36 to 3e-10 in every row of a 3.4 s run. The
crank cannot go all the way round: when |OP| falls to k = √(L² − l²) = 0.6 the rhombus goes flat and the cell branches, at crank angle 2 acos(k/2c) = 106.26 degrees, which is
t = 3.542 s at 30 deg/s. The run flags t = 3.517 .. 3.567 s, closest 0.17 degrees. What the cell does after that is the other branch: R leaves its line.

**`examples/boom.mech`.** The actuator can reach at most |base − pivot| + 0.25 m = 0.6623 m, from 0.50 m at 20 mm/s: t = 8.1155 s. The monitor flags it from 8.083 s (within 0.51 degrees of
singular, still meeting every constraint), and from 8.117 s on one more constraint is dependent and the residual is 0.019: the end of the stroke. The example's own header says the
same in prose ("after that the boom is at full stretch and the drive can no longer be followed").

## Cost

About one Jacobian assembly and one sparse factorization per step: a sample costs 1.8 microseconds on the four-bar, of which the Jacobian (which the solver pays for too) is 1.0.
Measured on `mech run --headless` against the build before M22, best of three: +1 to +6 percent on the Geneva drive, ratchet and jump cam; +13 percent on cam-star;
+19 to +25 percent on the clock and on pendulum chains of 160 and 640 links; and +37 and +41 percent on the four-bar and the planetary train, which is 3 and 4 microseconds
a frame on mechanisms that cost 8 and 10. The factorization's fill-reducing order is cached by sparsity pattern, as the solvers' are, and its buffers are reused.

## Not covered

* The window and headless runs are watched; `mech sweep` and `mech render` are not.
* A different regular assembly mode, and a design that is over-constrained on purpose, are not mistakes the Jacobian can name (above).
* Frames coarser than the singular neighbourhood (above). Refining a frame by solving the sub-steps in between would catch these; it is not done.
* Contact in a dynamic run: rows come and go, the monitor retakes its baseline when they do, and a singular pose reached at the very step a row appears is measured against that
  new baseline.
* The thresholds are two constants (`kDependentRatio`, `kNearRatio` in `src/diagnose.hpp`).

## Tested against

Closed forms (`tests/tests.cpp`, `[diagnose M22]` and `[diagnose run M22]`): the cell's bifurcation at 2 acos(k/2c), the boom's limit at (|base − pivot| + 0.25 − 0.5)/0.02; an
independent dense rank (Gaussian elimination with full pivoting) against the sparse factorization on random rows with planted dependencies, and on every example's Jacobian; invariance to
row units and to the size of the mechanism; a bead flung to 27 m along a tube drawn 0.1 m long (regular all the way); every example regular at assembly and quiet over a run; the monitored run
ending in bit-for-bit the same state as the unmonitored one, for kinematic and dynamic files with gear meshes and contacts; the command line end to end.
