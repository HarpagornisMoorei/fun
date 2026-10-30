# The mech language — M5 specification

This document pins down the M5 kernel: lexical structure, grammar, static
semantics, the kinematic solver's mathematical formulation, the
constrained-dynamics integrator, the intent layer's synthesis rules, and the
hybrid elements (`mode` blocks with weight drives — the escapement's kernel).
It is the contract the implementation is tested against (`mech test`).

---

## 1. Ontology

A mechanism is a graph:

| element | role | M3 members |
|---------|------|------------|
| **body** | noun — a rigid frame with named anchors (ports) | `ground`, `rod`, `shaft`, `body` |
| **joint** | verb — couples two anchors | `pin` (revolute), `slide` (prismatic) |
| **feature** | noun — geometry attached to a body | `gear` (pitch circle, on any anchor) |
| **constraint** | grammar — equations the configuration must satisfy | pins, slides, drives, meshes, tangencies |
| **force** | energy — generalized forces, never coordinates | `spring`, `drive ... torque`, `drive ... weight`, gravity |
| **hybrid** | event — a state machine over constraint sets | `mode { state, -> when }` (M5) |
| **world** | stage — global physical parameters | `world { gravity, step }` |

Four principles govern the design:

1. **Non-causality.** A rate drive never assigns a variable; it contributes
   the row `theta_body = theta0 + omega * t` and the solver decides
   everything else. A torque drive is even purer: it adds *energy* (`Q_theta
   += tau`) and constrains nothing. The same file can describe a windshield
   wiper (drive the crank) or a clock (drive the weight).
2. **Small kernel, deep sugar.** The kernel has ten statement kinds; M4's
   intent layer (`reduce`, `beat`, `hand`, `dial`) is *solved* into that
   kernel — tooth counts, positions, lengths — so the semantics stay small
   enough to trust while the source shrinks to intent.
3. **Ports are points, sometimes axes.** An anchor is a named point in a
   body frame; with `dir (...)` it also names an axis, which is what makes
   prismatic joints expressible.
4. **Forces and constraints share the stage.** Gravity, springs and torque
   drives act through the same constraint solver that pins and meshes
   define — one linear algebra core serves both worlds.
5. **One non-smooth element, done honestly.** A `mode` block is a hybrid
   automaton: continuous dynamics punctuated by discrete events whose
   energy is bookkept to the joule. It is the kernel's only escape from
   smoothness, and the escapement is why it exists.

## 2. Lexical structure

* Identifiers: `[A-Za-z_][A-Za-z0-9_]*`.
* Numbers: non-negative decimals (`3`, `1.5`, `0.8`). Unary minus is a parser
  concern: `-60` lexes as `-` then `60`.
* Units are reserved words: `deg/s`, `rpm`, `deg`.
* Keywords: `mechanism ground rod anchor at pin drive length from body shaft
  gear mesh slide on world spring torque reduce hand dial mode`.
* `dir`, `tangent`, `gravity`, `step`, `mass`, `k`, `rest`, `damp`, `ratio`,
  `via`, `same`, `opposite`, `spread`, `beat`, `sec`, `major`, `minor`,
  `radius`, and the whole M5 hybrid vocabulary (`state`, `when`, `hold`,
  `impulse`, `maintain`, `theta`, `omega`, `advance`, `weight`, `barrel`,
  `drop`, `kg`, `m`, `mJ`) are *contextual* identifiers (not keywords),
  usable as names.
* Punctuation: `{ } ( ) , : ; . - -> < >`
* Comments: `// line` and `# line`.
* Whitespace (including newlines) is insignificant; statements may be
  separated by `;` or `,` or nothing.

## 3. Grammar (M3)

```ebnf
program     = mechanism { mechanism } ;

mechanism   = "mechanism" IDENT "{" { stmt } "}" ;

stmt        = groundDecl | bodyDecl | rodDecl | shaftDecl | worldDecl
            | gearDecl | meshDecl | springDecl | pinDecl | slideDecl
            | driveDecl
            | reduceDecl | handDecl | dialDecl
            | modeDecl | weightDriveDecl ;               (* M5 *)

groundDecl  = "ground" IDENT "{" { anchorDecl } "}" ;
bodyDecl    = "body" IDENT "{" { anchorDecl | "mass" ":" num } "}" ;
anchorDecl  = "anchor" IDENT "at" "(" num "," num ")"
              [ "dir" "(" num "," num ")" ] ;

rodDecl     = "rod" IDENT [ attach ] "{" { rodProp } "}" [ attach ] ;
rodProp     = "length" ":" num | "from" ":" num "deg" | "mass" ":" num
            | "beat" ":" num [ ( "s" | "sec" ) ] ;          (* M4 *)
shaftDecl   = "shaft" IDENT [ attach ] ;        (* M4: attach optional *)
attach      = ( "on" | "at" ) ref ;                    (* pins anchor A *)

worldDecl   = "world" "{" { ( "gravity" | "step" ) ":" num } "}" ;

gearDecl    = "gear" IDENT "{" { gearProp } "}" "on" ref ;
gearProp    = ( "teeth" | "module" | "radius" | "mass" ) ":" num ;
meshDecl    = "mesh" IDENT IDENT [ "tangent" ] ;

springDecl  = "spring" IDENT ref "->" ref "{" { springProp } "}" ;
springProp  = ( "k" | "rest" | "damp" ) ":" num ;

pinDecl     = "pin" ref "->" ref ;
slideDecl   = "slide" ref "->" ref ;
driveDecl   = "drive" IDENT "at" num ( "deg/s" | "rpm" )
              [ "from" num "deg" ]
            | "drive" IDENT "torque" num ;

reduceDecl  = "reduce" IDENT "->" IDENT "ratio" num [ ":" num ]
              { "via" IDENT { "," IDENT | IDENT }
              | "same" | "opposite"
              | "module" ":" num
              | "spread" ":" num "deg" } ;           (* M4, any order *)

handDecl    = "hand" IDENT "on" ref [ "." IDENT ] [ "{" { "length" ":" num } "}" ] ;
dialDecl    = "dial" [ "at" ref ] [ "{" { dialProp } "}" ] ;
dialProp    = ( "radius" | "major" | "minor" ) ":" num ;

(* M5: the hybrid element *)
modeDecl    = "mode" IDENT "{" { stateDecl | transition } "}" ;
stateDecl   = "state" IDENT "{" { "hold" IDENT } "}" ;
transition  = IDENT "->" IDENT "when" guard [ ":" reset { "," reset } ] ;
guard       = IDENT "." field ( ">" | "<" ) num [ unit ] ;
field       = "theta" | "omega" | "advance" ;
reset       = "impulse" IDENT num [ "mJ" ]
            | "impulse" IDENT "maintain" num [ "deg" ] ;
unit        = "deg" | "deg/s" ;          (* field must match: theta/advance
                                             take deg, omega takes deg/s; a
                                             bare number is radians / rad/s *)

(* M5: the weight drive; 'cw'/'ccw' set which way the cord unwinds
   (default ccw = +theta). A clock's hands sweep clockwise, so its
   weight drive is 'cw' and its advance guards read negative. *)
weightDriveDecl = "drive" IDENT "weight" num [ "kg" ]
                  [ "barrel" num [ "m" ] ] [ "drop" num [ "m" ] ]
                  [ "cw" | "ccw" ] ;

ref         = IDENT "." IDENT ;
num         = [ "-" ] NUMBER ;
```

Units: `deg/s` -> `pi/180` rad/s; `rpm` -> `pi/30` rad/s; `deg` -> `pi/180`
rad. A rod's attach sugar may appear before the block, after it, or not at
all (an unattached rod is a free body, legal in dynamic mode). A `beat` time
is seconds whether written `1s`, `1 s`, `1 sec` or `1` (the unit is a
contextual identifier, so `s` remains usable as a name elsewhere). A bare
`shaft s` declares a body with no position — it must be pinned by `at`, by a
`reduce` via list, or synthesized by a reduce; otherwise the model build
fails with a floating error.

## 4. Static semantics

Checks run at model build time, each error pointing at a source line:

* Exactly one `ground` per mechanism; it is parked at body index 0 and its
  anchor coordinates and directions are world quantities.
* Body names unique; anchor names unique per body; gear and spring names
  unique.
* Directions are normalized; a zero-length direction is an error.
* Rod `length` must be present and positive; `mass`, when given, must be
  non-negative (and positive in dynamic mechanisms).
* Every `pin`/`slide`/`spring` reference must resolve (`body.anchor`); a
  body cannot pin to itself.
* A `slide` must join one moving body (the guide, whose anchor needs `dir`)
  and one ground anchor (the rail, which also needs `dir`). M3 restricts
  rails to ground.
* A `gear` host must exist; the host anchor must resolve (`gear ... on a`
  defaults to anchor `A` — ground bodies have no default and need an
  explicit anchor). Ground hosts are legal: fixed rings and stationary
  gears. Specify either `teeth` (with optional `module`, default 0.01,
  positive) or `radius`, not both.
* A `mesh` must reference two distinct known gears; the hosts must differ
  (coaxial gears cannot mesh) and cannot both be ground (nothing would
  roll). Meshing gears specified via teeth must share a module.
* A `spring` needs a positive `k`; `rest` (default: captured at assembly)
  and `damp` (default 0) must be non-negative.
* A `drive` must target a non-ground body (driving a gear name is
  redirected with a hint), and each body carries at most one drive of
  either kind.
* `world` `gravity` must be non-negative, `step` in (0, 0.1] seconds.
* A `drive ... weight` needs a positive mass; `barrel` and `drop` must be
  positive when present. Without `barrel` the host must carry at least one
  gear (the drum defaults to its largest pitch radius), else the build fails.
* `mode` blocks (M5): at least one `state`; state names unique within the
  mode; every transition's `from`/`to` must name declared states. Guards
  watch moving bodies (never ground); so do `hold` and `impulse` targets.
  A state may not hold the same body twice. A held body may not carry a
  rate drive — but it *may* carry a torque or weight drive, which is exactly
  the escapement's pair (a wheel driven by a weight and locked by a pallet).
  One hold row exists per held body across the whole mechanism: two modes
  cannot hold the same body.

**M4 intent checks** (all with line-accurate diagnostics):

* `beat` requires a dynamic mechanism (add a `world` block) and excludes
  `length` on the same rod. See 4.3 for the length it synthesizes.
* `reduce` endpoints must exist, differ, and be non-ground; the first
  endpoint must be statically positioned (pinned to ground directly or
  placed by an earlier reduce). `via` bodies must exist, be non-ground, not
  repeat, and not be endpoints. Ratio bounds: at least the smallest stage
  (24:16 = 1.5) and at most 12.5^5 (five stages of 100:8).
* `hand` hosts must exist and move (a hand on ground never moves); the mount
  anchor defaults to `A`. `length` defaults to 0.3 m.
* `dial` is anonymous (like `mesh`). With `at` it must reference a ground
  anchor; without `at` the mechanism must contain at least one hand
  (auto-centering needs one). `minor` must be a multiple of `major`.
* A bare `shaft` that no pin and no reduce ever places is a floating error.

**Dynamic mode** activates when a `world` block, any spring, any torque or
weight drive, or any `mode` block is present. Kinematic files compile and
run exactly as in M2.

### 4.1 Mobility (the DOF type system)

Planar Gruebler-Kutzbach, with drives as single-coordinate constraints:

```text
F_free     = 3(n - 1) - 2*j - k - kt     n  = bodies (ground included)
                                          j  = pins + slides (2 DOF each)
                                          k  = meshes (roll row, 1 DOF)
                                          kt = tangent meshes (contact row,
                                               1 further DOF)
F_residual = F_free - d                  d  = rate drives
```

`mech check` reports both. Interpretation:

| F_residual | verdict | behavior |
|-----------|---------|----------|
| 0 | fully determined | animation proceeds |
| > 0 (kinematic) | indeterminate motion | warning; solver holds free DOF at assembly values |
| > 0 (dynamic) | governed by dynamics | masses + forces determine it; no warning |
| < 0 | over-constrained | warning; damped least squares tolerates *consistent* redundancy (e.g. a carrierless planet between sun and ring) |

Springs and torque drives are forces: they remove no DOF.

### 4.2 Masses

Dynamic mechanisms need mass on every body; it is auto-derived from geometry
when not given, so plain geometric files get sensible physics for free:

| element | auto mass | center of mass | inertia |
|---------|-----------|----------------|---------|
| rod `length: L` | `rho * L` (rho = 1 kg/m), or `mass:` | `(L/2, 0)` | `m L^2 / 12` |
| shaft | 0.05 kg | origin | 1e-4 kg*m^2 |
| free body | 1 kg, or `mass:` | origin | `0.01 m` |
| gear on any anchor | `rhoA * pi r^2` (rhoA = 1 kg/m^2), or `mass:` | the host anchor | `m r^2 / 2` |

Multiple features on one body combine: total mass, mass-weighted center,
parallel-axis inertia sum. `rod ... { from: 60 deg }` seeds assembly at that
angle (release posture); undriven bodies otherwise get a deterministic tilt
to avoid dead-point singularities. Shafts synthesized by `reduce` are shafts
like any other (0.05 kg, origin).

### 4.3 The intent layer (M4)

The intent layer *solves* rather than desugars blindly: the author states a
ratio, a period, or a face, and the compiler picks numbers. Everything lands
in the M3 kernel, so the solver and the dynamics never learn that `reduce`
existed.

**`reduce A->B ratio R`** synthesizes a compound gear train from A (fast) to
B (slow). The ratio may be written `1:1800`, `1800:1` or `1800` — the larger
side is always the fast side; `a:b` must be integral. Synthesis searches
depth-first over stage ratios `wheel/pinion` with pinion teeth in 8..16 and
wheel teeth in 24..100 (so every stage ratio lies in [1.5, 12.5]), requiring
the product to equal R *exactly* as a rational. Preference order:

1. fewest stages (1..5);
2. among those, the smallest maximum stage ratio (balance);
3. then the smallest total tooth count.

Each stage places its pinion on the driving (fast) body and its wheel on the
driven (slow) one — so intermediates become compound gears automatically.
`via` fixes the intermediate bodies (and hence the stage count); without it
they are synthesized and named `<A>_1`, `<A>_2`, ... (a name collision is an
error suggesting `via`).

`same`/`opposite` constrain the rotation sense of B relative to A. Each
external mesh flips sense, so a train of n meshes turns B against A iff n is
odd. The synthesizer prefers a stage count with the requested parity; when
none exists it inserts one **idler shaft** — a single gear of 16..64 teeth
meshed between A's first pinion and the first wheel, contributing ratio 1
and one extra flip. `mech check` reports the sense and any idler.

**Layout** (who positions the unpinned shafts): the chain is walked from the
statically-known A. Free shafts are placed greedily — stage j leaves its
driving body at angle `spread + j * (-60 deg)`, at exactly the mesh distance
`module*(wheel+pinion)/2` — so every synthesized mesh is tangent by
construction. When the *next* shaft is pinned by the author, the current one
is instead placed at the intersection of the two circles it must satisfy
(choosing the solution nearer the greedy direction); if the circles do not
meet, the build fails quoting the distances. The author pins what they care
about (the dial center, say) and lets the solver fold the train around it.
Overlapping non-meshed wheels are *not* an error: the view is a 2-D top view
of wheels that sit at different depths on their arbors.

**`beat`** (on a rod) states the pendulum's beat — horology's half swing, the
tick or the tock. For the uniform compound rod the linearized period is
`T = 2*pi*sqrt(2L/(3g))`, hence

```text
L = 3 g beat^2 / (2 pi^2)
```

A `1s` beat under standard gravity gives L = 1.4909 m — the compound-rod
seconds pendulum. The formula is the small-amplitude limit; at a 3 degree
half-swing the exact period deviates by well under 0.1%, and the integrator
reproduces `T = 2*beat` to within 0.5% in the test suite.

**`hand`/`dial`** are pure render intent: no constraint rows, no mass. A hand
is a tapered needle mounted on a host anchor (default `A`) pointing along
the host's local +x; its reading is whatever drive or dynamics turns the
host. A dial is a static tick ring: with `at` it sits on that ground anchor,
otherwise it auto-centers on the average hand mount (captured at assembly,
like mesh phase); `radius` defaults to 1.2x the longest hand. `major` ticks
(default 12) are drawn longer and brighter than `minor` (default 60); the
clock face reads with 12 o'clock up.

## 5. Kinematic semantics

### 5.1 Configuration

Non-ground body `i` carries a pose `q_i = (theta_i, x_i, y_i)`; the unknown
vector `q` concatenates them. Ground is fixed at identity. The world position
of an anchor is

```text
world(b, a) = R(theta_b) * local_a + (x_b, y_b),      R = 2-D rotation
```

### 5.2 Constraint system

One algebraic system collects all equations, evaluated at time `t`:

```text
pin rows (2 per joint):     world(A, a) - world(B, b)              = 0
slide rows (2 per joint):   crossz(dRail, pGuide - pRail)          = 0
                            crossz(R(th) dGuide, dRail)            = 0
drive rows (1 per drive):   theta_body - (theta0 + omega*t)        = 0
mesh rows (1 per mesh):     rA*thA + s*rB*thB - (rA + s*rB)*psi - c0 = 0
tangent rows (1 per tangent mesh):
                            |cB - cA|^2 - (rA + s*rB)^2            = 0
```

`crossz(u, v)` is the 2-D cross product. The slide rows say: the guide point
stays on the rail line, and the guide axis stays parallel to the rail axis
(zero at both parallel and anti-parallel — the solver picks the near branch).

**The mesh row is the epicyclic rolling law.** `psi` is the angle of the
center line `cA -> cB`, and `s = +1` for an external mesh, `s = -1` for an
internal one (auto-detected at assembly from the center distance versus the
pitch radii). With fixed centers `psi` is constant and the row is the plain
M2 roll relation. With moving centers — a planet on a spinning carrier —
the `psi` term is what folds the carrier's rotation into the rolling law,
which is exactly the classic planet-gear kinematics (for a ring-fixed
planetary, `omega_sun = (1 + N_ring/N_sun) * omega_carrier`). `psi` is
tracked with branch continuity: at each evaluation it advances by the
smallest signed increment, so a carrier may wind through unlimited
revolutions without an atan2 branch cut ever injecting a `2*pi` jump.

The tangent row enforces pitch-circle contact *live*. A mesh flagged
`tangent` is a complete gear joint by itself: a planet given only an initial
position orbits its sun with no carrier and no pins. Two tangent meshes to
ground are rank-deficient together (consistent redundancy) — flagged by the
DOF report, tolerated by the damped solver.

The analytic Jacobian follows from `d/dtheta [R(theta) v] = perp(R(theta) v)`
and `d psi / d(delta) = crossz(delta, .) / |delta|^2`. External vs internal
and the phase constant `c0` are captured at assembly; tangency is validated
geometrically after assembly (`mech check` reports, `mech run` aborts).

### 5.3 Solving

Levenberg-Marquardt on the damped normal equations
`(J^T J + lambda^2 I) delta = -J^T c`, with step clamping, adaptive lambda,
and deterministic multi-start (the incoming guess, then two tilted
restarts). Convergence tolerance: `max|c| < 1e-10`; animation steps are
warm-started from the previous frame, so steady-state solves take 1-3
iterations.

Assembly (t = 0) is seeded greedily: a graph traversal over pins and slides
places each body so that the anchor it shares with an already-placed body
sits exactly on target. Driven bodies start at their prescribed angle;
`from:` bodies at their release angle; other bodies get a small
deterministic tilt (a fully collinear four-bar is a dead point). After
placement, mesh types and phases and spring rest lengths are captured.
Remaining residuals belong to Newton.

## 6. Dynamic semantics (M3)

### 6.1 Equations of motion

The mechanism obeys the Lagrangian equations with constraint forces via
Lagrange multipliers:

```text
M(q) qdd = Q(q, qd) + gyro(q, qd) + J(q)^T lambda        (1)
c(q, t) = 0                                              (2)
```

Each body's mass matrix block, for a center of mass at body-frame offset
`com` (`Rc = R(theta) com`, `p = perp(Rc)`):

```text
[[ I + m|com|^2 ,  m p_x ,  m p_y ],
 [  m p_x       ,  m     ,  0     ],
 [  m p_y       ,  0     ,  m     ]]
```

symmetric positive definite (`det = m^2 I_com`). Forces:
gravity `(-m g Rc_x, 0, -m g)` per body; springs apply the Hooke (and
optional viscous) tension at their anchors, with torque about each host's
origin; torque drives add straight to the theta column. The gyroscopic term
is `(0, m w^2 Rc_x, m w^2 Rc_y)`.

Differentiating (2) twice gives the acceleration-level system
`J qdd = -bias` with `bias = Jdot qdot` analytic per row type: pins
`(-w_A^2 Ra + w_B^2 Rb)`, slides `(-crossz(railDir, w^2 Rg), -w^2 c2)`,
mesh rolls `(rA + s rB)/d^2 * crossz(delta, w_B^2 Rb - w_A^2 Ra)
+ 2 (rA + s rB) psidot ddot / d`, tangencies `2 |delta_dot|^2 + 2 delta *
(w_A^2 Ra - w_B^2 Rb)`. For every row type in the kernel the
acceleration-level Jacobian equals the position-level Jacobian, so the KKT
reuses the kinematic `J` wholesale. The suite validates this exactly: along
`q + qd t + (1/2) qdd t^2`, constraints hold to `O(t^3)`.

Eliminating `qdd` through the blockwise inverse `W = M^-1` (Schur
complement):

```text
(J W J^T) lambda = -(J W (Q + gyro) + bias)
qdd = W ((Q + gyro) + J^T lambda)
```

Redundant rows are handled by Tikhonov damping on the same SPD system. Rate
drives stay exactly on schedule; their `lambda` entries are the reaction
torques they transmit (reported by `mech run --headless`), which is the
kernel's route to engineering answers like "how much torque does this
escapement wheel see?".

### 6.2 Integration

Classical **RK4** on `(q, qd)` with the constrained acceleration as `qd'`.
The acceleration is exact given a state (section 6.1), so the remaining
error is pure quadrature; RK4 keeps energy flat to print precision on
chaotic systems (the double pendulum test holds |E - E0| under 1e-3 J over
5 s of chaos) where semi-implicit Euler would drift at first order because
the coupled mass matrix depends on `q`. After each step: an LM position
solve re-anchors `q` on the manifold, then a **mass-metric velocity
projection** `qd -= W J^T mu` with `(J W J^T) mu = J qd - nu` enforces the
velocity-level constraints. The projection acts through the same map as
constraint forces and therefore does no work on the manifold — the
Euclidean least-norm variant would inject energy whenever joint rows couple
several bodies.

Everything is deterministic: fixed `world.step` (default 2 ms), no
randomness, identical outputs for identical inputs.

### 6.3 Energy

`DynamicsSystem::kinetic / potential` implement the bookkeeping
(`T = 1/2 m |v_com|^2 + 1/2 I w^2` per body, `V = m g y_com + 1/2 k
(L - rest)^2` per spring). The viewer shows the live total in the window
title; `mech run --headless` tabulates it.

### 6.4 Hybrid dynamics (M5)

A `mode` element makes the mechanism a *hybrid automaton*: smooth constrained
motion (6.1-6.2) punctuated by discrete events. Nothing in the smooth kernel
changes — events wrap around it.

**Holds are drive rows.** A state's `hold b` is the drive row
`theta_b = target` with `omega = 0`, exactly the prescribed-coordinate form
of section 5.2, so switching states switches the active constraint set by
enabling and disabling rows. The row set is recomputed on every evaluation;
a disabled hold contributes neither rows nor DOF. On every state entry the
target is *captured* — set to the body's current angle — which is why
consecutive locks step a wheel forward without any explicit position reset:
the state simply locks wherever the crossing left the wheel. The hold's
Lagrange multiplier is the torque the lock transmits (reported like any
drive reaction): for a weight-driven escapement it equals the weight's
torque while engaged.

**Guards are rising edges.** For a transition out of the *current* state,

```text
g(q, qd) = value - threshold          for '>'
g(q, qd) = threshold - value          for '<'
value   = theta_b | omega_b | (theta_b - theta_b(at state entry))
```

The transition fires when `g` crosses zero *upward* during a step: `g <= 0`
at the step's start and `g > 0` at its end. A guard that is already true
when its state is entered cannot fire until it goes false and rises again —
this is the edge-triggering that makes tick/tock chatter-free and
self-transitions (a one-state ratchet stepping on every rise) safe.

**Events are localized, not sampled.** When a step contains a rising
crossing, the integrator bisects the step: it re-integrates from the saved
start state at sub-lengths `h/2, h/4, ...` (10 halvings, resolution
`h/1024`), keeping the smallest interval whose end still shows `g > 0`, and
lands there. Simultaneous rises within one step fire in declaration order
(then the remainder is re-scanned); a mode that would fire more than 8 times
in a single step finishes that step unwatched — a guard against
pathological chatter.

**Resets are energy, stated as energy.** At the crossing instant, before
the state switch:

```text
impulse b dE:   omega_b' = sgn(omega_b) * sqrt(omega_b^2 + 2 dE / I0)
                I0 = I_com + m |com|^2      (inertia about the frame origin)
```

`dE` is the stated joules (sign allowed: a negative impulse brakes). A body
at rest receives nothing — energy has no direction of its own. The
`maintain A` form computes `dE` so the body's *next turning point* sits `A`
past its natural hang angle `theta_hang = -pi/2 - atan2(com.x, com.y)` (the
com-straight-down equilibrium):

```text
E(theta, w) = 1/2 I0 w^2 + m g (y + (R(theta) com).y)
dE = E(theta_hang + dir*A, 0) - E(theta, w)        dir = sgn(w)
```

Top up when the swing is short, brake when it is long: a pendulum pinned at
its origin converges onto exactly the stated half-swing, one beat at a time.
The velocity jump is exact in the energy bookkeeping (`delta T = dE` to
round-off), and the post-event mass-metric projection — with `nu = 0` it can
only remove energy — discards whatever the new rows forbid, which is the
relock impact a real pallet takes.

**Weight drives are torques with a budget.** `drive b weight m barrel r
drop d cw` applies `Q_theta += ± m g r` about the body's origin — the sign
is the cord's unwind sense, `cw` pulling along `-theta` (the direction clock
hands sweep) and the default `ccw` along `+theta`. The cord pays out
ratchet-style — only rotation in the unwind sense pays — and once `r *
|dtheta|` of cord reaches `d`, the torque ceases and the mechanism runs down
(a mid-unlock exhaustion leaves the wheel coasting into its relock; the
*next* unlock finds a wheel at rest with nothing to drive it, and the clock
stops in the free state).

**The ledger closes.** Every run bookkeeps, in joules:

```text
drawn   = sum over steps of torque * |dtheta| in the unwind sense   (weight work)
impulse = sum of applied reset energies                 (net, sign included)
impact  = kinetic energy removed by event projections   (relock losses)
dE      = E_mech(end) - E_mech(start)                   (kinetic + potential)
```

and `dE = drawn + impulse - impact` holds to integration accuracy (the test
suite holds it under 5e-3 J over ten seconds of escapement; short runs close
to 1e-9 J). The weight's own `m g h` never enters `E_mech` — it is the
budget, and `drawn` is what the mechanism actually collected.

**Determinism.** Fixed step, fixed bisection depth, fixed declaration
order: identical inputs give bit-identical trajectories (tested).

## 7. Desugaring register

| surface | kernel expansion | status |
|---------|------------------|--------|
| `rod r { length: L }` | rigid body + anchors `A=(0,0)`, `B=(L,0)` | M1 |
| `rod r on A.p { length: L }` | the above + `pin r.A -> A.p` | M2 |
| `rod r { ... } on A.p` | same (attach sugar, either side of the block) | M3 |
| `shaft s at A.p` | body with anchor `A=(0,0)` + `pin s.A -> A.p` | M2 |
| `gear g { teeth: N } on b` | pitch circle `r = m*N/2` at anchor `b.A` | M2 |
| `gear g { ... } on b.c` | pitch circle centered at any host anchor | M3 |
| `mesh g1 g2` | roll row (with the psi carrier term) | M3 |
| `mesh g1 g2 tangent` | roll row + live contact row | M3 |
| `drive b at w from a0 deg` | row `theta_b - (a0 + w*t)` | M1 |
| `drive b torque tau` | force `Q_theta_b += tau` | M3 |
| `spring sp a.p -> b.q { k }` | Hooke force pair at the anchors | M3 |
| `world { gravity: g }` | dynamics mode + gravity forces | M3 |
| `rod r { from: a deg }` | assembly seed angle for r | M3 |
| `reduce s1->s4 ratio 1:1800 [via s2 s3]` | solve tooth counts (exact rational search), emit shafts + gears + meshes | M4 |
| `reduce ... same/opposite` | parity-preferred stage count, or one idler shaft | M4 |
| unpinned reduce chain shafts | ground anchors + pins synthesized at the solved layout | M4 |
| `rod p { beat: 1s }` | solve `L = 3g*beat^2/(2 pi^2)`, emit that rod | M4 |
| `hand h on s4 { length: L }` | render-only needle on host anchor | M4 |
| `dial [at g.c] { ... }` | static tick ring; center/radius captured at assembly | M4 |
| `mode m { state s { hold b } ... }` | rate-0 drive rows (one per held body), enabled per active state, targets captured on state entry | M5 |
| `a -> b when c.theta > 5 deg` | rising-edge zero-crossing event on `theta(c) - 5 deg` | M5 |
| `... when c.advance > 6 deg` | zero-crossing on `(theta(c) - theta_at_state_entry) - 6 deg` — advance is *signed* travel, so a clockwise wheel guards on `< -6 deg` | M5 |
| `... : impulse c 2 mJ` | velocity jump `omega +- sqrt(omega^2 + 2 dE/I0)` along current motion | M5 |
| `... : impulse c maintain 3 deg` | dE computed so the next turning point is 3 deg past the hang angle | M5 |
| `drive b weight m [barrel r] [drop d] [cw]` | torque `±m*g*r` (sense: `cw` -> `-theta`), ratchet pays in that sense while `(paid + r*|dtheta|) < d` | M5 |

## 8. Test matrix

`mech test` (13409 checks) covers: lexer tokens/units/keywords/comments/errors;
parser AST shapes for all M3 and M4 forms and all error classes; model checks,
desugaring, gear radius arithmetic, module rules, mass derivation (rods,
discs, compound assemblies, overrides), DOF arithmetic including tangent
meshes; solver exactness (pendulum closed-form, gear ratio and reversal,
compound-train output, crank-slider rail lock and stroke, loop closure and
rod-length invariants, min-norm freezing, cold-start robustness, tangency
reporting); M3 mesh semantics (planetary ratio `1 + N_r/N_s` held exactly
over multiple carrier revolutions — the psi branch-cut immunity test —
carrierless orbits held by tangent rows alone, consistent redundancy,
radius-only gears with moving centers); dynamics (free-fall matched to
the exact semi-implicit bound, compound-pendulum period `T = 2 pi
sqrt(2L/3g)` within 0.6%, machine-flat energy on single/double pendulum,
undriven four-bar flop and torque-spun planetary — the work-energy identity
— plus the analytic spring oscillator `x(t) = -0.5 + 0.5 cos(3t)`, flywheel
spin-up `omega = tau/I t`, and drive reaction torque equal to the gravity
torque it transmits). The finite-difference constraint-acceleration test
validates every analytic bias term against numerical differentiation.

The M4 suite adds: beat length formula (standard and custom gravity, conflict
and kinematic-mode errors); reduce synthesis (exact stage products, tooth
bounds, pinion-on-fast anatomy, compound intermediates, auto-named and via
chains, name-collision diagnostics); direction control (parity-preferred
stage counts, idler insertion with a single shared gear and unchanged DOF);
ratio bounds and unreachable-ratio diagnostics; layout (greedy placement,
`spread`, circle-intersection closure onto pinned ends with exact tangency,
incompatible-ends errors); hands and dials (defaults, mount resolution,
auto-center/auto-radius capture, tick divisibility); solved trains obeying
their ratios and senses over time; the idler kinematics; and the beat
pendulum integrated to `T = 2*beat` within 0.5%. Both M4 examples build,
assemble, and validate.

The M5 suite adds: the hybrid grammar (guard fields/units, reset forms,
weight-drive clauses, every error class); the model layer (hold rows as
rate-0 drives, per-state enablement, one-row-per-body dedupe, assembly
capture, weight barrel defaulting, hold/torque coexistence); and the hybrid
dynamics — impulse resets delivering their energy to the joule (`dE = fires
x 1 mJ` over six seconds), `maintain` converging a released-fast pendulum
onto exactly its 3-degree limit cycle, the four-state escapement advancing
its wheel exactly half a tooth per beat (6.000-6.15 degrees) with the
pendulum held at 3 degrees and constraints intact throughout, bit-exact
determinism across runs, a weight-driven flywheel matching the torque-drive
spin-up analytically, the run-down (a 0.0052 m cord stops the clock
mid-state after paying out its whole budget of exactly `m g drop` joules),
and the energy ledger closing against the change in mechanical energy.
The M5 direction work is covered too: a `cw` weight spins its flywheel to
the analytic *negative* rate and pays cord for negative travel, the same
escapement wound clockwise steps *minus* half a tooth per beat with the
ledger still closing, the flagship's equal-tooth-sum motion work holds
`hour = minute/12` exactly (stud at `-minute/3`) over multiple revolutions,
and the flagship file itself is regression-tested end to end: escape arbor
minus 6 degrees per beat, minute `= escape/60`, hour `= minute/12`, all
clockwise, constraints and ledger intact.

## 9. Milestone history & reserved ground

M1 walked the skeleton (lexer/parser/rods/pins/drives); M2 added shafts,
gears and meshes; M3 added slides, the epicyclic mesh row with live
tangency, and the full constrained dynamics; M4 solved the intent layer
(`reduce`, `beat`, `hand`, `dial`). M5 closed the loop the language was
always aiming at: the beat pendulum and the going train finally shake hands
through a `mode` element — an anchor escapement, weight-driven, stepping
half a tooth per beat and paying for its impulse from a falling weight with
a closing ledger. The flagship is a whole clock: small seconds on the
escape arbor, a synthesized 1:60 going train down to the minute arbor, and
the horologist's motion work (an equal-tooth-sum 10:30 x 8:32 stack, two
coaxial shafts on one anchor) carrying the minute and hour hands — every
hand sweeping clockwise because the weight is `cw`.

Motion-work *synthesis* (the compiler choosing the equal-tooth-sum stack
itself, rather than the flagship writing it out), contact-rich pallet
geometry (impulse faces as constraint rows rather than event summaries),
and multiple mechanisms in one file sharing a world are reserved for later
milestones.
