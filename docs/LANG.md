# The mech language — M13 specification

This document pins down the M6f kernel: lexical structure, grammar, static
semantics, the kinematic solver's mathematical formulation, the
constrained-dynamics integrator, the intent layer's synthesis rules, the
hybrid elements (`mode` blocks with weight drives — the escapement's kernel),
contact (`profile` curves and point-on-curve `touch` joints — the first
higher pair — bilateral in kinematic files, unilateral with lift-off and
impact in dynamic ones), plate cams stated as displacement diagrams, and
non-circular gear pairs (`mesh` between two profiles), and polygon profiles
with filleted corners (ratchets and anything with teeth), and Coulomb friction at
contacts, plus (M8) named parameters, numeric expressions and sweeps, and (M9) reusable
parts, and (M10) `import` with a library of standard parts, (M11) rotated mounts, (M12) loops and indexed names, and (M13) SVG drawings. It is the contract
the implementation is tested against (`mech test`).

---

## 1. Ontology

A mechanism is a graph:

| element | role | M3 members |
|---------|------|------------|
| **body** | noun — a rigid frame with named anchors (ports) | `ground`, `rod`, `shaft`, `body` |
| **joint** | verb — couples two anchors | `pin` (revolute), `slide` (prismatic) |
| **feature** | noun — geometry attached to a body | `gear` (pitch circle, on any anchor), `profile` (closed contact curve, M6) |
| **constraint** | grammar — equations the configuration must satisfy | pins, slides, drives, meshes, tangencies, touches (M6) |
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
* Strings: `"text"` on one line, no escapes; used only for the path of an `import` (M10).
* Units are reserved words: `deg/s`, `rpm`, `deg`.
* Keywords: `mechanism ground rod anchor at pin drive length from body shaft
  gear mesh slide on world spring torque reduce hand dial mode profile touch`.
* `dir`, `tangent`, `gravity`, `step`, `mass`, `k`, `rest`, `damp`, `ratio`,
  `via`, `same`, `opposite`, `spread`, `beat`, `sec`, `major`, `minor`,
  `radius`, and the whole M5 hybrid vocabulary (`state`, `when`, `hold`,
  `impulse`, `maintain`, `theta`, `omega`, `advance`, `weight`, `barrel`,
  `drop`, `kg`, `m`, `mJ`) and the M6 contact vocabulary (`circle`, `polar`,
  `points`, `roller`, `restitution`, `ellipse`, `focus`, `polygon`, `fillet`, `friction`) and the M6b cam vocabulary (`cam` at statement start,
  `base`, `start`, `turn`, `rise`, `fall`, `return`, `over`, `dwell`,
  `cycloidal`, `harmonic`, `modtrap`) are *contextual* identifiers (not
  keywords), usable as names — a body called `cam` is fine. So is `param`, which
  only means something at statement start (M8, see below).
* Punctuation: `{ } ( ) [ ] , : ; . - -> < > + * / =`
* Comments: `// line` and `# line`.
* Whitespace (including newlines) is insignificant; statements may be
  separated by `;` or `,` or nothing.

## 3. Grammar (M3)

```ebnf
program     = { importDecl | partDecl | mechanism } ;   (* parts before their first use *)
importDecl  = "import" STRING ;                          (* M10: top level only *)

mechanism   = "mechanism" IDENT "{" { stmt } "}" ;

stmt        = groundDecl | bodyDecl | rodDecl | shaftDecl | worldDecl
            | gearDecl | meshDecl | springDecl | pinDecl | slideDecl
            | driveDecl
            | reduceDecl | handDecl | dialDecl
            | modeDecl | weightDriveDecl                 (* M5 *)
            | profileDecl | touchDecl                      (* M6 *)
            | camDecl                                     (* M6b *)
            | paramDecl                                   (* M8 *)
            | useDecl                                     (* M9 *)
            | forStmt ;                                   (* M12 *)

groundDecl  = "ground" IDENT "{" { anchorDecl } "}" ;
bodyDecl    = "body" IDENT "{" { anchorDecl | "mass" ":" num
                                 | "from" ":" num "deg" } "}" ;      (* from: M6e, as for a rod *)
anchorDecl  = "anchor" IDENT "at" "(" num "," num ")"
              [ "dir" "(" num "," num ")" ] ;

rodDecl     = "rod" IDENT [ attach ] "{" { rodProp } "}" [ attach ] ;
rodProp     = "length" ":" num | "from" ":" num "deg" | "mass" ":" num
            | "beat" ":" num [ ( "s" | "sec" ) ] ;          (* M4 *)
shaftDecl   = "shaft" IDENT [ attach ] ;        (* M4: attach optional *)
attach      = ( "on" | "at" ) ref ;                    (* pins anchor A *)

worldDecl   = "world" "{" { ( "gravity" | "step" ) ":" num } "}" ;

paramDecl   = "param" IDENT "=" expr ;                       (* M8 *)
forStmt     = "for" IDENT "=" expr "to" expr "{" { stmt } "}" ;      (* M12 *)
(* `for` also stands where a list item does: in the anchors of a ground or body, and among the
   tuples of `polygon:`, `points:` and `polar:` ( `for k = 0 to n - 1 { (x, y) (x, y) }` ). *)
name        = IDENT [ "[" expr "]" ] ;                       (* M12: IDENT_k, k a whole number >= 0 *)
partDecl    = "part" IDENT [ "(" paramInit { "," paramInit } ")" ] "{" { stmt } "}" ;   (* M9 *)
paramInit   = IDENT "=" expr ;
useDecl     = "use" IDENT [ "(" paramInit { "," paramInit } ")" ] "as" IDENT
              [ "on" IDENT "at" "(" expr "," expr ")" [ "rotate" expr "deg" ] ] ;     (* rotate: M11 *)
(* A reference to a name is `qname`: IDENT { "." IDENT }, followed through while the prefix
   names an instance (`left.follower.tip`); `ref = qname "." IDENT`. *)
(* Everywhere this grammar writes `num`, M8 accepts an expression that evaluates to one. *)
expr        = term { ( "+" | "-" ) term } ;
term        = unary { ( "*" | "/" ) unary } ;
unary       = [ "-" | "+" ] unary | primary ;
primary     = NUMBER | "pi" | IDENT                          (* a declared parameter *)
            | FUNC1 "(" expr ")"                             (* sqrt sin cos tan atan abs *)
            | FUNC2 "(" expr "," expr ")"                    (* min max pow *)
            | "(" expr ")" ;

gearDecl    = "gear" IDENT "{" { gearProp } "}" "on" ref ;
gearProp    = ( "teeth" | "module" | "radius" | "mass" ) ":" num ;
meshDecl    = "mesh" IDENT IDENT [ "tangent" ] ;     (* two gears, or (M6d) two profiles *)

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

(* M6: contact. The host may precede or follow the block, like a rod's
   attach; an omitted anchor means "A". *)
profileDecl = "profile" IDENT [ "on" IDENT [ "." IDENT ] ] "{" shape "}"
              [ "on" IDENT [ "." IDENT ] ] ;
shape       = "circle" ":" num
            | "ellipse" ":" num num [ "focus" ]                  (* semi-axes a, b; M6d *)
            | "polygon" ":" { "(" num "," num ")" } "fillet" num   (* M6e *)
            | "polar" ":" { "(" num [ "deg" ] "," num ")" }      (* angle, radius *)
            | "points" ":" { "(" num "," num ")" } ;
touchDecl   = "touch" ref "on" IDENT { "roller" num | "restitution" num | "friction" num } ;

(* M6b: a plate cam stated as motion. Segments run in the order the follower
   meets them as the cam turns, and must fill exactly one turn. *)
camDecl     = "cam" IDENT [ "on" IDENT [ "." IDENT ] ] "{" { camItem } "}"
              [ "on" IDENT [ "." IDENT ] ] ;
camItem     = "base" num | "roller" num | "start" num "deg"
            | "turn" ( "ccw" | "cw" )
            | ( "rise" | "fall" ) num "over" num "deg" [ law ]
            | "return" "over" num "deg" [ law ]
            | "dwell" [ num "deg" ] ;                (* bare dwell: rest of the turn *)
law         = "cycloidal" | "harmonic" | "modtrap" ;

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

**M13 drawing:** `mech render` is tooling, not language (see [`RENDER.md`](RENDER.md)): it poses the model
the way `run` does (kinematic files re-solved in steps of at most 1/60 s, dynamic ones integrated) and writes the
geometry in world coordinates. A mechanism that does not assemble is an error. `Model::assemble` returns a model
that has been run before to its build-time state, including a mode's hold drives (they used to keep the angle
pinned at the end of the last run, so a second assembly of the clock began from it).

**M12 loop checks:**

* `for i = a to b { ... }` runs its block for i = a, a+1, ..., b (inclusive; no iterations if b < a).
  Both bounds are expressions that must be whole numbers; one loop has at most 1000 iterations and a
  file at most 20000 in all. The loop variable is a number inside the block (usable in any expression,
  and in names) and is gone after it; it may not be an existing parameter or a reserved word.
* A name may be written `ident[expr]`, which is the identifier `ident_k`, `k` the value of `expr`
  (a whole number >= 0). It is lexical sugar that works anywhere a name does, loop or not: `cam[2]` and
  `cam_2` are the same name, so a looped file is addressed from outside by the plain identifiers.
* A `param` cannot be declared inside a loop (it would be declared once per iteration). A part used
  inside a loop is its own scope, so its own `param`s are fine there.
* A loop is expanded while parsing, like `use`. Nothing past the parser knows about it, and a looped
  file equals the file with the iterations written out.

**M11 rotation checks:**

* `rotate A deg` after the mount turns the part rigidly about the mount point: its ground anchors
  (position and `dir`), the `from:` of its rods and bodies, the start angle of a rate drive on a rod or
  body, a cam's `start`, and the solver's default starting guess for its rods and bodies. A drive on a
  shaft, gravity, and anything written outside the part do not turn. Rotations nest and add; `rotate 0 deg`
  is bit-for-bit no rotation. A part containing a `mode` cannot be rotated. See [`PARTS.md`](PARTS.md).

**M10 import checks:**

* `import "path.mech"` is a top-level statement, resolved before parsing on the token stream: the
  named file's parts replace it. The path must end in `.mech`; it is looked for next to the importing
  file and then in each directory of `MECH_PATH`.
* A library file holds parts and imports only. A file is spliced once however it is spelled; a cycle
  is an error naming the chain.
* Every token carries its file (`line = file * 2^20 + line-in-file`), so parse and model errors from
  inside a library are reported with the library's path and line. Specified in [`PARTS.md`](PARTS.md)
  and the standard parts in [`LIB.md`](LIB.md).

**M9 part checks:**

* A part is expanded at parse time: its body is read again for each `use` with every
  name it declares or mentions qualified by the instance (`left.follower`), so the
  model sees one flat mechanism. Full specification in [`PARTS.md`](PARTS.md).
* A part must be declared before the part or mechanism that uses it, which also rules
  out recursion. Its body is checked at its definition with the default arguments.
* A part has no `world` block. A ground inside a part merges into the ground named by
  `on ... at (x, y)`, translated by `(x, y)` (no rotation); a part with a ground needs
  the `on` clause and a part without one must not have it.
* Instance names are unique per mechanism and cannot be body names. Arguments are
  evaluated in the caller's scope; each instance's parameters are `instance.name` to
  `--set` and to sweeps.

**M8 parameter checks:**

* `param name = expr` is a parse-time constant: it may use only earlier parameters,
  the numbers it is written with, `pi` and the functions above. A parameter is
  scoped to its mechanism, and declaring one twice is an error.
* A parameter name may not be a unit token (`deg`, `rpm`, ...), `pi` or a function
  name. Division by zero, a domain error (`sqrt(-1)`), a non-finite result, an unknown
  function and a wrong argument count are errors at the offending token.
* `--set name=value` replaces the declared right-hand side before anything that uses it
  is read, so every dependent parameter follows. An override naming no parameter is an
  error that lists the declared ones. Parameters leave no trace in the model: a file
  built with `--set` is bit-for-bit the model of the file with the number edited.
* `mech sweep` is specified in [`PARAMS.md`](PARAMS.md).

**M6e polygon checks:**

* `polygon: (x, y) ... fillet r` is a closed polygon in the anchor's frame whose every
  corner is rounded by a circular arc of radius `r`. At least 3 vertices (at most
  1024), no coincident neighbours, a non-degenerate area; clockwise lists are
  reversed so the curve is counter-clockwise. `fillet` is **required and positive**:
  a perfectly sharp corner has no contact normal, and a tiny fillet (`0.0002`) costs
  nothing. At a corner whose adjacent edges are too short for the fillet the arc is
  clamped so neighbouring fillets never overlap (each uses at most half an edge); if
  that shrinks any fillet below 5% of the request it is an error ("does not fit")
  rather than a silent change of shape. The curve is not checked for
  self-intersection.
* A free `body` may carry `from: <deg>`, a release angle like a rod's, so a wheel can
  start at a chosen angle instead of the solver's deterministic tilt.

**M6d non-circular mesh checks:**

* `ellipse: a b` needs two positive semi-axes; with `focus` the anchor is the
  right-hand focus (the ellipse extends along +x from it: `r(phi) = p/(1 - e cos
  phi)`, `p = b^2/a`) and `a >= b` is required. Without `focus` the anchor is the
  center (`r(phi) = ab / sqrt(b^2 cos^2 phi + a^2 sin^2 phi)`) and either axis may be
  longer.
* `mesh p q` joins two **profiles** when neither name is a gear. Both must be
  *star-shaped* about their anchor (`circle`, `polar`, `ellipse` — a point list
  is not), on distinct rotating bodies, each pinned to ground at the profile's
  own anchor (fixed centers; moving centers and internal contact are not yet
  supported). `tangent` is a gear-mesh option and is an error here. A gear and a
  profile cannot mesh.
* **The pair must be conjugate**, and the compiler checks it rather than
  trusting the author: at every polar angle of `p` it finds where `q` meets it on
  the line of centers (`r_q = d - r_p`) and requires the log-slopes to agree,
  `r_p'/r_p = r_q'/r_q`, to 1e-3 over the whole turn. A pair that cannot reach the
  line of centers at the needed radius ("cannot roll on each other") and a pair
  that reaches it but crosses instead of rolling ("not a conjugate pair") are
  separate errors with the radius, angle and center distance in the message.

**M6a contact checks:**

* Profile names are unique; the host body and anchor must exist (a ground
  host needs an explicit anchor, as for gears). The profile's origin is the
  host anchor and its axes are the host body's.
* `circle` needs a positive radius. `polar` needs at least 3 `(angle,
  radius)` pairs, angles strictly increasing within `[0, 360)` degrees (a
  bare number is radians), radii positive. `points` needs at least 3 points,
  no coincident neighbours, and a non-degenerate enclosed area; clockwise
  lists are reversed so the curve is always counter-clockwise. At most 1024
  pairs or points. The curve must be simple (self-intersection is not checked).
* A `touch` names a point (`body.anchor`) and a profile; they may not share a
  body (so ground cannot touch its own profile). `roller` must be positive;
  without it the point is a knife-edge tip.
* A `touch` is **bilateral** in a kinematic mechanism and **unilateral** in a
  dynamic one (6.5). `restitution e` (Newton's coefficient, `0 <= e <= 1`, default
  0) and `friction mu` (Coulomb, `mu >= 0`, default 0) belong to the dynamic case; in
  a kinematic mechanism nothing lands or slides under a force, so each is an error
  rather than silently ignored.

**M6b cam checks** (each pointing at the offending segment's own line):

* `base` is required and positive; `roller` is non-negative. The cam shares
  the profile namespace and host rules (`touch f.tip on lobe` works for both).
* The diagram must be a closed, full-turn, non-negative motion: segment
  angles positive and summing to exactly 360 degrees (a bare final `dwell`
  fills the remainder, and must come last), no `fall` below the base circle,
  no `return` from the base circle, and the follower back on the base circle at
  the end ("does not close" names the leftover height).
* **Undercut is an error.** If the pitch curve's radius of curvature, over its
  convex parts, falls to the roller radius or below, the inward offset folds
  over itself and no such cam can be cut. The message gives the radius, the
  cam angle, and the levers (`base`, lift, segment length, `roller`).
* A `touch` on a cam takes the roller the cam was cut for unless it gives its
  own; an explicit different roller is accepted but `assemble` warns that the
  follower will not follow the diagram.

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
F_free     = 3(n - 1) - 2*j - k - kt - c n  = bodies (ground included)
                                          j  = pins + slides (2 DOF each)
                                          k  = meshes, gear or non-circular (roll row, 1 DOF)
                                          kt = tangent meshes (contact row,
                                               1 further DOF)
                                          c  = touches (gap row, 1 DOF)
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

### 4.4 Cam synthesis (M6b)

`cam` is intent-layer sugar over the M6a kernel, in the spirit of `reduce`:
the author states the *motion*, the compiler solves the *geometry*, and what
lands in the model is an ordinary closed `points` profile. The solver never
learns cams exist.

**Setting.** A translating roller follower on a rail through the axle, or a
knife-edge tip (`roller` omitted). With base radius `Rb` and roller radius
`rho` the prime circle has radius `r0 = Rb + rho`, and the roller centre — the
*pitch point* — sits at `r(theta) = r0 + s(theta)`, `s` being the displacement
diagram. The cam turns by `theta`; in its own frame the rail lies along polar
angle `phi = start - sigma*theta` (`start` defaults to 90 degrees, i.e. the
rail along the cam frame's +y at `theta = 0`; `sigma = +1` for `turn ccw`, the
default, `-1` for `turn cw`). The pitch curve is `P(theta) = r(theta)(cos phi,
sin phi)` and the **cam surface is that curve offset inward by `rho`** along its
outward normal, sampled at 720 points and splined. With the follower meeting
the cam on a rail through the axle, the follower height is then `r0 + s(theta)`
exactly (verified over two turns to 1e-8, for every law, both turning senses
and a rotated `start`). Offset rails and swinging followers are not
synthesized; a cam still works under them as a plain profile.

**The diagram.** Segments are `rise h`, `fall h`, `return` (back to the base
circle) and `dwell`, each over an angle, in the order the follower meets them.
Within a segment of lift `L` over angle `beta`, `s = s0 + L f(x)`,
`x = (theta - theta0)/beta`, where `f` is a normalized rest-to-rest law:

| law | `f(x)` | peak `f'` | peak `f''` | at the ends |
|-----|--------|-----------|------------|-------------|
| `cycloidal` (default) | `x - sin(2 pi x)/(2 pi)` | 2.000 | 6.283 | `f'' = 0`: eases in and out |
| `harmonic` | `(1 - cos(pi x))/2` | 1.571 | 4.935 | `f'' = 4.935`: **acceleration jumps** |
| `modtrap` | modified trapezoid (sine ramps around a constant-acceleration plateau; closed form, antisymmetric about `x = 1/2`) | 2.000 | 4.888 | `f'' = 0`: eases in and out |

`harmonic` has the lowest peak velocity — hence the gentlest pressure angle —
but the jump in acceleration at its ends puts a curvature discontinuity in the
surface; the sampled C2 spline cannot hold it, so the contact normal is off by
about 0.045 degrees at those junctions (the follower *position* still matches
to 1e-8). `modtrap` has the lowest peak acceleration of the two laws that
ease in and out. A dwell has `s' = s'' = 0` exactly.

**Diagnostics**, computed on a 0.05-degree grid and printed by `mech check`:

```text
pressure angle   tan(alpha) = s'(theta) / r(theta)        (rail through the axle)
undercut         kappa = -sigma * crossz(P', P'') / |P'|^3   (> 0: convex)
                 smallest 1/kappa over convex parts must exceed rho
```

The pressure angle is the angle between the contact normal and the rail; the
report warns above 30 degrees, the usual limit for a translating follower.
It was checked two independent ways: against a brute-force maximization of the
formula over the diagram, and against the *actual contact normal* of the
solved mechanism at every step (agreement to 1e-6 rad). The curvature radius
was checked against the polar-curve formula `(r^2 + 2r'^2 - r r'')/(r^2 +
r'^2)^(3/2)`, and a zero-lift cam reports exactly `r0`.

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
polar mesh rows (1 each):   thB - thB0 - F(psi - thA)                = 0      (M6d)
tangent rows (1 per tangent mesh):
                            |cB - cA|^2 - (rA + s*rB)^2            = 0
touch rows (1 per touch):   gap(p, profile) - roller               = 0
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

**The touch row is a signed distance.** For a point `p` on body B and a
profile `c(s)` on body A (origin at the host anchor `oA`), let `s*` be the
foot of the perpendicular — the closest point of the curve to `p` — and `n`
the outward unit normal there (the right-hand normal of the counter-clockwise
tangent). The row is `gap = n . (p - c(s*))`, positive outside the curve, less
the roller radius when there is one. A roller of radius `rho` centered on the
anchor is therefore in contact exactly when the center sits `rho` outside the
curve: the *true* offset curve, so a concave region tighter than the roller
simply cannot be reached (no cusped pitch curve is ever constructed).

Because `gap` is the distance to the *closest* point it is stationary in `s`
at the foot (`p - c` is parallel to `n`, and `n . c'(s) = 0`), so the Jacobian
needs no `ds` term — the pin-row pattern projected on the world normal:

```text
d gap/d(xB, yB) = +n           d gap/d thB = +n . perp(R(thB) pLocal)
d gap/d(xA, yA) = -n           d gap/d thA = -n . perp(R(thA) (oA + c(s*)))
```

(verified against central finite differences on a moving oval profile, to
better than 1e-6 over random off-manifold states). The foot is searched afresh
at every evaluation — a coarse scan over a precomputed outline (>= 256
samples), then a bracketed Newton iteration to machine precision — so the row
is a pure function of `q` with no tracked state to go stale between solver
trial steps (unlike `psi`, which must be tracked). Curves are C2: a circle and an
ellipse are exact; `polar` is the periodic cubic spline `r(phi)`; `points` is a pair
of periodic cubic splines `x(s)`, `y(s)` on chord length. A **polygon** is exact
and only C1: straight edges joined by tangent circular fillets, parameterized by
arc length, so the curvature is 0 on an edge and `+-1/r` on a fillet (negative
at a reflex corner, where the fillet curves away from the material). Its foot
search is analytic — the closest point of every edge and arc, best wins — with
no sample table, and the normal is continuous across every junction. This is
why polygons exist: a cubic spline through a corner rings by tens of percent, and
the contact normal with it.

In a *kinematic* mechanism a contact is **bilateral**: the row holds in both
directions, i.e. force closure is assumed (a spring or gravity keeps a real
follower on its cam). `mech check` labels each touch accordingly; the dynamic
case is 6.5. The mechanism stays non-causal:
the cam in `examples/eccentric.mech` is driven, but the contact relates the
two bodies symmetrically.

**The non-circular mesh (M6d).** Two star-shaped profiles `r_A(phi)`, `r_B(phi)` on
shafts pinned a distance `d` apart roll on each other with their contact point on
the line of centers, at `r_A(phi_A)` from one shaft and `r_B(phi_B)` from the
other, where `phi` is the polar angle of the contact in each body's own frame:
`phi_A = psi - thA`, `phi_B = psi + pi - thB` (`psi` is the world angle of the
line of centers A -> B). Touching alone is the algebraic relation `r_A + r_B = d`,
but that is a poor *row*: it is singular wherever `r'` vanishes (the symmetric
poses of an ellipse pair, where the Jacobian is exactly zero and a solver started
there has nothing to push against) and it has two mirrored solutions, only one of
them tangent. The rolling relation is its tangent branch. The instantaneous
center of relative rotation lies on the line of centers, dividing it inversely
to the angular speeds, so `d thB = -rho d thA` with `rho = r_A/(d - r_A)`, which
integrates to

```text
thB - thB0 - F(psi - thA) = 0,       F(phi) = integral from 0 to phi of rho,
J = (rho, 1)        never zero; and it cannot land on the mirrored branch
```

`thB0` is the phase constant captured at assembly, exactly as a gear mesh's roll
constant is. For circles `rho` is the constant `rA/rB` and the row is the plain
mesh row. `F` is tabulated at 2048 nodes (3-point Gauss quadrature, so the table
is not the weak link) and interpolated by a quintic Hermite using the exact `F`,
`F' = rho` and `F'' = rho'` at the nodes: `F` is C2 and as accurate as the
analytic terms the Jacobian and the dynamics use. (A cubic Hermite was tried
first; its second derivative is off by ~1e-3, which shows up as a bias error.)

The row *describes* rolling; it is only rolling for a **conjugate** pair, which
is why the compiler verifies conjugacy over the whole turn (the M6d checks in
section 4) instead of trusting the author. For identical ellipses pivoted at their foci and `2a` apart
the speed ratio is `(1 - e)/(1 + e)` to `(1 + e)/(1 - e)` — 1/3 to 3 at
`e = 0.5` — and the average over a turn is exactly 1:1 (`F(2 pi) = 2 pi`).
Assembly seeds the second shaft on the tangent branch (the root of
`r_B = d - r_A` whose log-slope agrees) whatever angle the first starts at, and
`validateAssembly` rejects a pose that is not touching (for instance two drives
that disagree about the starting angles).

### 5.3 Solving

Levenberg-Marquardt on the damped normal equations
`(J^T J + lambda^2 I) delta = -J^T c`, with step clamping, adaptive lambda,
and deterministic multi-start (the incoming guess, then two tilted
restarts). Convergence tolerance: `max|c| < 1e-10`; animation steps are
warm-started from the previous frame, so steady-state solves take 1-3
iterations. If every attempt fails, `q` is left at the *closest* one (lowest
residual) and `solve` returns false. (Until M6d the restart offsets were
computed with an unsigned `(i % 3) - 1`, which wrapped to 2^64 - 1 for every
angle coordinate and sent the restarts to ~1e18; it never showed because the
first attempt almost always converges, and failed solves left `q` at the last
attempt rather than the closest. The first mechanism to start on a singular pose
— the symmetric start of an elliptical pair under a naive row — exposed both.)

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

### 6.4b Non-circular mesh rows (M6d)

The non-circular mesh row `R = thB - thB0 - F(psi - thA)` has `psi` and `d`
constant (fixed centers), `F' = rho` and `dphi_A/dt = -wA`, so

```text
R'  = wB + rho wA
R'' = aB + rho aA - rho'(phi_A) wA^2          rho' = d r_A' / (d - r_A)^2
J qdd + bias = 0,    J = (rho, 1),    bias = -rho' wA^2
```

— only `r_A` and `r_A'` are needed, no second derivative of the profile.
Verified by the trajectory test of 6.5: advance `q + h qd + h^2/2 qdd` and the
residual (with its exact first-order part removed) scales as `h^3` (halving `h`
cuts it by ~8), where a wrong bias would leave an `h^2` term (~4). Here the
signal is far above the solver's noise floor — the ratio swings 9:1 and the
third derivative is large — so the scaling, not an absolute bound, is the
right instrument.

### 6.5 Unilateral contact (M6c)

In a dynamic mechanism a `touch` is a two-state element: **engaged** (its row
is an ordinary equality constraint, `gap = roller`, and its multiplier is the
normal force) or **in flight** (no row). The force may not pull, so the
element switches on two zero-crossings, found by the same bisection loop as
the mode guards:

| event | guard fires when | action |
|-------|------------------|--------|
| lift-off | the normal force `lambda` falls below zero | row off; velocity unchanged |
| touch-down | `gap - roller` closes to zero | Newton impact, then row on (or stay in flight if it bounces) |

**The second-order term.** Dynamics needs `g'' = J q'' + bias`. For a point
`B` against a curve on body `A` (`g` the signed gap, `n` and `t` the unit
normal and tangent at the foot, `kappa` the curve's curvature there, `rA` and
`rB` the arms to the foot and to the anchor, `w` the point's velocity relative
to the curve's material point at the foot):

```text
w     = vB - vA + wB perp(rB) - wA perp(rA)
u     = (t . w - g wA) / (1 + g kappa)           foot's sliding speed along the curve
Om    = wA + kappa u                             angular rate of the normal
bias  = n . (-wB^2 rB + wA^2 rA) + wA u + Om (t . w)
```

Derived by differentiating the foot condition `tau . d = 0` (which fixes `u`),
and checked two ways: against finite differences of a moving ellipse to better
than 1e-6 relative, and by a trajectory test — advance `q + h qd + h^2/2 qdd`
and the residual stays at the solver's noise floor, whereas breaking any one of
the five terms leaves an `O(h^2)` drift that fails the test. For a roller `g` is
its radius, so `1 + g kappa` is exactly the stretch of the offset curve over
the surface; it is singular only if the point reaches the curve's centre of
curvature.

**Impact.** Landing with approach speed `v = -gdot-` applies Newton's law in
the mass metric. With `qs` the pre-impact velocity projected so the contact
row is at rest (every other active constraint still holds — the existing
relock projection),

```text
qd+ = (1 + e) qs - e qd-          contact speed -e gdot-, impulse along the row
```

A prescribed (rate-driven) body is "infinitely heavy", so the impulse it feels
does work: the energy ledger is

```text
dE = weight + impulse + drive - impact - friction
drive  = integral of (reaction torque x prescribed rate) dt   (Simpson over the
         four RK4 stages) + the work of each impact's impulse on driven rows
impact = (impulse work) - dT   >= 0, the energy the landing discarded
```

and closes to 1e-6 J against 0.023 J of cam work over two turns of the
jump-cam example. Landings slower than 2 mm/s are
plastic (`e = 0`): a bounce restarts inside the 1 nm contact band, so the guard
can only see the next landing after `sqrt(2 a delta)` of free speed (about
0.3 mm/s at 56 m/s^2), and without a cutoff above that floor a resting contact
would bounce forever. A genuine bounce sequence is finite: `e = 0.5` dropped
0.1 m settles after 11 impacts, at 0.428 s — the analytic accumulation time
`t1 (1 + 2e/(1 - e))`, with `t1 = sqrt(2 h0 / g)`, is 0.4284 s.

**State.** A contact starts engaged only if the point is within 1 um of the
curve where it was *placed* (so `rail` anchors set the starting posture); a
point placed in the air starts in flight and its first touch-down is a real
impact. A `settle` pass at each step start resolves what the event loop could
not (a point already inside its curve lands; an engaged contact that would pull
lets go), and events are processed earliest-first, up to 32 per step.
Dynamic runs also now start from velocities consistent with the rate drives
(previously a driven shaft spun up in the first step, an impulsive start that
no ledger term covered).

The ledger counts a **torque drive's** work too (`torque x the body's angular
velocity`, Simpson over the RK4 stages), alongside a rate drive's, so a
torque-driven ratchet closes: `drive + ... - impact = dE`.

A frictionless contact is conservative, and the ratchet shows what that means. Under
reverse torque the pawl locks the wheel — it advances a degree or two to bring a
radial face onto the tip, and is held — but a jammed frictionless pair keeps ringing
by about a degree, the force swinging around the value that balances the torque
(from ~0.2 N to over 16 N at the extreme, where the pawl nears dead center). The wheel's own
equation of motion still holds exactly: `integral of N cross(footArm, n) dt = tau T -
I (w_end - w_start)`, which the tests check to 0.01%. Friction is what brings it to
rest (6.6).

### 6.6 Friction (M6f)

`touch ... friction mu` adds Coulomb friction at an engaged contact: a tangential
force of magnitude at most `mu N` opposing the slip. The slip speed is the
tangential part of the point's velocity relative to the curve's material point at
the foot, `s = t . w` (the same `w` as in 6.5), and it acts through the slip
Jacobian `Jt` (`Jt_B = (t . perp(rB), t)`, `Jt_A = -(t . perp(rA), t)`) as the
generalized force `Q = -F_t Jt^T`. Like `restitution`, it belongs to dynamic
mechanisms; in a kinematic one nothing slides under a force, so it is an error.

**Regularized, not stuck.** The force is smooth, `F_t = mu N tanh(s / eps)`, so an
explicit integrator can take it. Below `eps` (2 mm/s, and widened as below) a contact
*creeps* rather than sticking; there is no static-friction branch. For a tip or a
brake this is invisible; for a block that should hold on a slope it means a
very slow slide at about `eps`. A locked ratchet is held by geometry, not by
static friction, and settles to a still wheel (angle constant to 1e-3 degree).

**Three problems, three answers.**

* *`N` is the multiplier of the solve it loads.* Solve, add friction from that `N`,
  solve again; three passes. This converges unless the geometry is jammed
  (a Painleve configuration, which a rigid-body model with Coulomb friction can
  genuinely fail to resolve).
* *Stiffness.* The friction "spring" `mu N / eps` acting on a light body is stiff,
  and explicit RK4 blows up past `h (mu N / eps) mobility ~ 2.7`. The width is
  therefore `eps_eff = max(eps, mu N h mobility)`, where `mobility` is the
  *constrained* inverse inertia along the slip direction,
  `Jt W Jt^T - (Jt W J^T)(J W J^T)^-1 (J W Jt^T)`. A 0.0001 kg m^2 wheel under
  `mu = 0.5` and a 2 ms step stops in the predicted 0.04 s and does not overshoot.
* *Energy.* Friction dissipates `F_t s >= 0`; it is integrated over the RK4 stages
  (Simpson, like the drives' power) into the ledger as `friction`, so the books still
  close: `dE = weight + impulse + drive - impact - friction`.

Friction is applied at the anchor point, which is right for a knife-edge tip. With a
`roller` it is an approximation (a freely rolling roller would feel almost none).
Impacts are frictionless: no tangential impulse at a landing.

Not modelled: static friction proper (stick), tangential impact impulses, rolling
resistance, more than one contact point per `touch`, and compliant
(spring-damper) contact.

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
| `mesh p q` (two profiles, fixed centers) | row `thB - thB0 - F(psi - thA)`, `F = integral of r_A/(d - r_A)`; conjugacy of the pair checked over a full turn at build time | M6d |
| `profile p on b.A { ellipse: a b [focus] }` | exact ellipse, anchor at the center or the right focus; exact polar form `r, r', r''` | M6d |
| `profile p on b.A { polygon: ... fillet r }` | exact edges and tangent fillet arcs (clamped per corner), arc-length parameterized, analytic foot search | M6e |
| `profile p on b.c { circle: R }` | closed curve `c(s) = R(cos s, sin s)` at anchor `b.c`, riding `b` | M6a |
| `profile p on b.c { polar: ... }` | periodic cubic spline `r(phi)` through the `(angle, radius)` knots | M6a |
| `profile p on b.c { points: ... }` | periodic cubic splines through the points, normalised counter-clockwise | M6a |
| `touch f.tip on p [roller r]` | row `signedGap(f.tip, p) - r`; on a cam, `r` defaults to the cam's roller | M6a |
| `touch ... friction mu` (dynamic) | tangential force `mu N tanh(s/eps)` opposing the slip, widened for stability, with its dissipation in the ledger | M6f |
| `touch ... restitution e` (dynamic) | two-state element: row while engaged, lift-off at `lambda < 0`, Newton impact `qd+ = (1+e) qs - e qd-` at touch-down | M6c |
| `cam c on b.a { base Rb roller rho rise h over a deg ... }` | solve the pitch curve `r0 + s(theta)`, offset inward by `rho`, emit a 720-point closed `points` profile; reject undercut | M6b |
| `rise/fall/return/dwell` segments | piecewise displacement diagram `s(theta)` with rest-to-rest laws; must close on the base circle in exactly one turn | M6b |
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

`mech test` (15338 checks) covers: lexer tokens/units/keywords/comments/errors;
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

The M6a suite adds: the profile curves themselves (a constant-radius polar
table equal to a circle to 1e-12 in value and both derivatives; spline error
and periodicity; C2 across a knot; the foot search against a brute-force
reference to 1e-9 on 300 random points, with correct signs and
orientation-normalised clockwise input); the grammar (shapes, host either
side of the block, degree units, every error class with its line number); the
model (every profile and touch diagnostic, the dynamic-mode rejection, DOF
arithmetic); and the kinematics. The acceptance test is the eccentric disc:
over two turns at 5-degree steps the follower height matches the closed form
`y = e sin(theta) + sqrt((R + rho)^2 - e^2 cos^2(theta))` to 1e-9, with and
without a roller, through the exact circle and a constant-radius polar spline
(and to 2e-6 through a 64-point spline circle, listed both ways round). The analytic
Jacobian matches central differences on a moving oval profile with a swing-arm
roller follower; a ground-hosted profile pins a rod tip exactly on a fixed
circle; an unreachable contact is reported rather than hidden; and both
shipped cams (`eccentric`, `lobecam`) build, assemble and solve over two
turns, with the plate cam's follower dwelling at 36 mm and 46 mm to 20 um.

The M6b suite adds: the three motion laws (rest-to-rest, `v` and `a` really
the derivatives, monotone, `s(x) + s(1-x) = 1`, and the textbook peak
accelerations 6.283, 4.935 and 4.888); the diagram (exact values at known
angles, dwells flat to the bit, periodicity, and every closure and range
error with its segment index); the design numbers (pressure angle against a
brute-force maximization of the hand formula — including the fact that a
symmetric rise and return share their maximum — curvature radius against the
independent polar formula, a zero-lift cam reporting exactly `r0`, and
undercut detection); the grammar and every model diagnostic with its source
line, the shared namespace, and the roller default and mismatch warning; and
the kinematics: through the real solver, for each law, the follower height
equals `r0 + s(theta)` to 1e-8 over two turns (knife-edge, `turn cw` with a
negative rate, and a rotated `start` included), the contact normal agrees with
the pressure-angle formula, and a 60-degree dwell leaves the follower still to
1e-9. Reversing the sign of the roller offset breaks twelve of these checks.

The M6c suite adds: the second-order contact term (the trajectory-drift test on
two geometries where both bodies move and the foot slides, with every term of
the formula mutation-checked); a ball dropped on a fixed circle in closed form
(first-impact time, rebound peaks `e^(2k) h0` to 2e-5 m, dissipation `(1-e^2) m
g h0`, a finite Zeno sequence ending at rest under exactly its own weight, the
elastic and plastic limits, ledger closure); the spring-loaded cam follower
(contact force equal to Newton's closed form `N cos(alpha) = m(omega^2 s'' + g)
+ k(y - rest)` to 2e-3 N while engaged, no lift-off at 300 rpm, lift-off at 600
rpm within one step of the angle where that closed form reaches zero, flights
about 2 mm clear that never dig into the cam, turn two repeating turn one
exactly, the ledger closing, bit-for-bit determinism); and the initial-state
rules.

The M6d suite adds: the ellipse's polar form (the point `(r cos phi, r sin phi)`
lies on the ellipse to 1e-12, both anchor placements; `r'` and `r''` against
finite differences); the grammar and every mesh diagnostic (point lists, a gear
against a profile, unpinned or ground-hosted or coaxial profiles, `tangent`, a
mismatched pair that crosses instead of rolling, a pair too far apart to touch);
circle profiles reproducing a gear `mesh` to 1e-9, with the exact ratio `-rA/rB`;
and an elliptical pair judged by **independent geometry** rather than by its own
row — over two turns the contact point found by brute force on each curve
coincides on both to 1e-8, the tangents there are parallel to 1e-6, and the arc
lengths rolled on the two curves (numerically integrated) are equal to 1e-6, i.e.
no slip — plus the speed surge read off the solved motion (`-3` at the far vertex
and `-1/3` half a turn later, to 1e-5), one turn of A equal to exactly one
reversed turn of B (1e-9), the same pair driven from the *other* shaft, assembly
finding the tangent branch from any starting angle, and two drives that disagree
about the starting pose being rejected. In the dynamics: the trajectory-scaling
test of 6.4b, and work-energy — a constant torque on A delivers `tau dtheta_A` as
kinetic energy to 1e-6 J however the ratio surges, with `wB/wA = -rho` throughout.
Breaking the bias, the sign of the Jacobian, or the ratio `rho` breaks 2, 6 and 12
of these checks.

The M6e suite adds: the polygon curve in closed form on a rounded square given
*clockwise* (perimeter, signed distances inside, outside and off a corner,
curvature `1/r` on a fillet and 0 on an edge, the nearest point against brute
force on 400 random points with no sign errors, a continuous tangent across every
junction), a reflex corner (curvature `-1/r`, a notch point correctly *outside*),
and the fillet clamp; the grammar and every polygon diagnostic; and the ratchet
as a mechanism: forward, the wheel turns through 10 teeth with one lift-off per
tooth tip passed (each ending in an impact), never digging into a tooth, and the
ledger — the torque's work `tau x angle` in, impacts out — closing to 2e-6 J;
reversed, the wheel is held, and the impulse-momentum identity above holds to
0.01% through the force spikes; and bit-for-bit determinism. Dropping the torque
from the ledger, flipping the polygon's normal or the sign of its curvature
breaks checks.

The M6f suite adds: the friction clause (any order with `roller` and `restitution`,
negative and kinematic uses rejected, zero legal); a **brake** in closed form — a
wheel spinning against a spring-loaded shoe on a rail, so `N = k (R - rest) = 4 N`
and the friction torque is exactly `mu N R`: the wheel decelerates at 5.00000 rad/s^2
to five digits, stops at 1.0 s and stays stopped without overshoot, the spring's
force and the shoe's position are undisturbed, doubling `mu` doubles the
deceleration, zero friction does not decelerate at all, the same holds spinning
the other way, and the energy dissipated is `(1/2) I w^2` to 1e-4 with the ledger
closing to 1e-6 J; the stiffness stress (a 0.0001 kg m^2 wheel, `mu = 0.5`, a 2 ms
step) stopping in the predicted 0.04 s without blowing up or overshooting; the
locked ratchet coming to rest under friction (angle constant to 1e-3 degree) with the
static balance `N cross(footArm, n) = tau` to 1e-5; the forward ratchet with friction
(slower, still one click per tooth, ledger closing to 2e-6 J with a friction term);
and bit-for-bit determinism. Flipping the friction's sign, dropping its dissipation
from the ledger, removing the stability widening, or mis-scaling `mu` breaks checks.

The M7 suite adds: the Recorder's columns (unique names, units in the name, one header
cell per column, no energies or velocities in a kinematic run); the exported eccentric
disc equal to its closed form to 1e-9 through the CSV text (12 significant digits
round-trip), the probe equal to the body's pose, the foot on the disc and the normal
through its centre; the cam's design pressure angle recovered from the exported
contact normal (20.59 deg both ways, at the same cam angle to the 1-degree grid); a
brake's ledger residual under 1e-6 J *at every sample* with the spring's 4 N, the
deceleration and monotone kinetic energy as columns; and the command line end to end
(row count, last time, closed form from the file itself; misuse — unknown or
ground-hosted probe, `--csv` without `--headless`, `--every 0`, `--dt` on a dynamic
file — refused with a nonzero status). Dropping friction from the exported ledger,
flipping the normal, mis-indexing the probe, or ignoring `--every` breaks checks.

The M8 suite adds: expression precedence, associativity and unary signs against
hand-computed values; every function and its domain errors; parameters in every
numeric position of every statement (the parameterized model equals the literal one
bit for bit, assembled state included); `--set` overrides, dependent parameters
following them, and the unused-override error; sweep axes (ranges including
inclusive endpoints with floating-point steps, lists, refusals); kinematic sweeps
against closed forms for all cells of a grid, first axis slowest; the jumpcam
lift-off threshold within one step of the closed-form 443.9 rpm; a design that stops
building partway through a sweep reporting the compiler's message in its own row; each
metric reduction against its definition; and the command line end to end, including
exit status 2 for a sweep with invalid designs. Reversing subtraction or division,
ignoring an override, accepting an unused one, swapping grid order, or dropping the
absolute value in the ledger metric breaks checks.

The M9 suite adds: the oracle that an instanced file *is* the flat file — two cams of
different lifts and speeds, nested parts (mount offsets added the way the flat file adds
them), a ground-less `link` part used twice to rebuild the crank-slider (CSV rows identical
to `crankslider.mech`), and the whole clock wrapped in one part, 30 s of hybrid dynamics,
identical in every row and column; the jumping cam as a part, with `--set u.speed` equal
to `--set speed` on the flat file and a sweep over `u.speed` giving the same lift-off
counts and ledger residuals; defaults, arguments and overrides in their precedence order;
the diagnostics (unknown part and argument, duplicates, mounting rules, recursion, a
mistake inside a never-used part reported at its own line, a semantic error naming the
instance); probes, CSV columns and sweep reports through dotted names. Mutation checks
cover each rule of the expansion — the prefix, the ground merge, the mount offset in x
and y, nested offsets, scope restoration, argument binding, override keys, the recursion
guard, the definition-time check, and every name site (`mesh`, `reduce`, `via`, `hold`,
mode guards and resets, `drive`, hand and gear hosts, touch profiles, mode and hand names).

The M10 suite adds: the string token and its error; the standard parts against closed forms (gear
ratio and centre distance for three tooth pairs, the slider-crank formula over a full turn with and
without offset and its dead centres, Freudenstein's equation at every sample of a full crank turn,
the cam's rest and peak heights); an imported part equal to the same part pasted in place; the
loader's rules (not found, extension, the shape of the statement, parts only, direct and indirect
cycles, once-only under three spellings, relative to the importing file, `MECH_PATH` after the file's
own directory, no import inside a mechanism); errors inside a library carrying the library's file and
line (lexical, syntax at the definition, semantic from the model with the instance's name); and the
command line end to end (`check` listing imports and overrides, an error inside a library printed with
its path and line, a missing import, `sweep` resolving an import relative to the file from another
directory). Mutation checks cover the nesting depth, the parts-only rule, the extension, cycles, dedupe,
canonical paths, line tagging, the search order, relative resolution, the lexer's string check, and the
error plumbing in `check` and `sweep`.

The M11 suite adds: covariance of rotated library parts at four angles (positions turn about the mount
point, rod angles shift by the rotation, a slider's angle is compared modulo pi, shaft angles do not shift);
the plate cam's lift along the turned rail equal to the unrotated lift and its shaft angle unchanged; a drive
written inside a rotated part equal to the same drive written outside with the angle added, and a shaft's
drive left alone; a rotated dynamic part (a pendulum under gravity) equal to the flat file with the turned
numbers written out, bit for bit; `rotate 0 deg` identical to no rotation and `rotate 360 deg` equal to it to 1e-9;
nested rotations adding (the inner mount carried into the world, the rail turned 180 degrees); an angle that is
an expression and sweeps; `examples/camstar.mech` (four followers on four radial lines); and the refusals
(missing `deg`, `rotate` without a mount, a division by zero in the angle, a `mode` at one and at two levels
down). Mutation checks cover the rotation's sign, the ground positions and directions, `from:` on rods and
bodies, the drive rule for rods and shafts, a cam's `start`, the default starting guess, nested angle and
offset, the stored rotation of a ground, the `mode` refusal and the syntax checks.

The M12 suite adds: a statement loop equal to its hand-unrolled file, bit for bit (the declaration and then
the simulated rows); the count as a parameter (`--set n=6` six bodies, `n=0` none); nested loops and an
index that is an expression; loops in anchor lists and in `polygon:`, `points:` and `polar:` lists; regular
polygons of 3, 5, 8 and 17 sides against the closed-form area `(n/2) R^2 sin(2 pi / n)` with every vertex on
the circle; the library ratchet (area `teeth * (1/2) r_root r_tip sin(pitch)`, a root at the top, about one
impact per tooth of advance over several teeth for 6, 8, 12 and 16 teeth, locked within a tenth of a tooth
in reverse, the energy ledger closing); `examples/camstar.mech` for 3, 4 and 6 cams (each follower on its own
radial line, 71 to 83 mm) and equal to the file written out by hand; a loop inside a part, parts inside a loop
with per-instance parameters addressed by indexed names; the loop variable ending with its loop; and the
refusals (fractional or oversized bounds, runaway nesting, a `param` or an existing name as the variable, a
missing `to`, an unterminated block, a negative or fractional index, an error in the body reported at the
body's line). Mutation checks cover the index suffix and its checks, the range's ends, the variable's scope
and collision rules, the loop limit, the integer checks and the three places a loop may appear.

The M13 suite adds: every number in a rendered SVG read back and compared with the solver (rod endpoints, pins,
pitch circles and the first tooth, 240 points of every profile, rollers and contact points, spring ends, hand tips,
trace points, rail spans), for a kinematic four-bar, a cam on a gear, a dynamic jumping cam against an
independent integration, the clock, a plate cam's rail, and an animation (frame *k* is the pose at its time, the
one `animate` cycling `#f0..#fN` over span / speed); the root transform mapping the drawing into the page, the
themes, the width, the ring-gear rule, the ground bar rule; the refusals (a trace that names nothing, a mechanism that
does not assemble) and the command line end to end; and, for every example file, that posing a model twice gives
identical poses. Mutation checks cover the transform, each kind of element, the animation's times, values and duration,
the trace, the rail span, the assembly and pose checks, the dynamic stepping, the ring rule, the model reset and the
command-line refusals.

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

M6a opened contact (see `docs/design/contact.md`): `profile` curves and
bilateral, kinematic point-on-curve `touch` joints — the first higher pair,
enough for plate cams with translating or swinging followers. One deviation
from the design note: the foot of the perpendicular is searched afresh at
each evaluation rather than tracked like `psi`, which keeps the row a pure
function of `q`.

M6b added the first intent-layer sugar over contact: `cam` states a
displacement diagram and the compiler cuts the surface — `rise`, `fall`,
`return` and `dwell` with cycloidal, harmonic or modified-trapezoid motion —
and `mech check` reports the pressure angle and refuses an undercut cam. Not
yet covered: offset and swinging followers (the cam is synthesized for a rail
through the axle), flat-faced followers, and a pressure-angle sweep for
arbitrary profiles.

M6c made contact real in the dynamics: a `touch` in a dynamic file is
unilateral — the follower lifts off when the cam could only pull it, flies, and
lands with a Newton impact — with the second-order term derived and verified,
the energy ledger extended to the work of prescribed drives, and
`examples/jumpcam.mech` showing a spring-loaded follower that holds at 300 rpm
and jumps at 600. This is also what contact-rich pallet geometry needs.

M6d added non-circular gears: `mesh` between two star-shaped profiles on fixed
shafts, with an exact `ellipse` profile, a one-row non-singular rolling relation
whose conjugacy is verified at build time, and the matching dynamics bias —
`examples/ellipgears.mech` is a pair of elliptical gears whose output surges
between 3x and 1/3x. Not covered: moving centers (a carrier), internal contact,
and rolling between arbitrary non-conjugate curves, which needs the contact
point as an unknown (the arc-length formulation in the design note).

M6e added sharp-cornered parts: `polygon` profiles (exact edges and fillets, so a
corner is a real corner rather than a ringing spline), torque work in the energy
ledger, and `from:` on free bodies — `examples/ratchet.mech` is a ratchet wheel
and gravity pawl that clicks once per tooth forward and locks in reverse.

M6f added friction: `touch ... friction mu`, regularized Coulomb with a stability
width taken from the constrained inertia and its dissipation in the ledger. The
ratchet's reverse lock now comes to rest (it rang forever without).

M7 is tooling rather than language: `mech run --headless --csv` exports a run as a
time series (see `docs/EXPORT.md`) — poses, contact gap and force, velocities,
energies, and every term of the energy ledger *with its residual as a column*, so
conservation is something to plot rather than a number at the end. It is built on a
`Recorder` that the tests drive directly; the exported contact normal reproduces the
plate cam's design pressure angle (20.590 deg against the formula's 20.59 deg).

M8 added `param` and numeric expressions, `--set`, and `mech sweep` (see
`docs/PARAMS.md`): the language stays a description of one mechanism, and designs are
compared by running many of it. Reusable parts and composition remain open.

M9 added reusable parts: `part` with parameters and `use ... as name on ground at (x, y)`
(see `docs/PARTS.md`), expanded at parse time so the solver, dynamics and tooling are
unchanged, and an instanced file runs bit for bit like the flat one.

M13 added `mech render` (SVG drawings, `docs/RENDER.md`) and fixed a bug it exposed: assembling a model that had been
run before began from the angles its mode holds had pinned at the end of that run (the viewer's reset key was affected).

M12 added `for` loops, indexed names (`cam[i]`), and loops inside anchor and shape lists (see
`docs/PARTS.md`), so structure can depend on a number: the count of a part's contents is a parameter and
a sweep axis, and `lib/ratchet.mech` generates its teeth. Loops are expanded by the parser, like `use`.

M11 added `rotate A deg` to a mount (see `docs/PARTS.md`): a part can be placed at any angle, and for
kinematic parts the result is the unrotated one turned, which the tests check as covariance.

M10 added `import` and `lib/` (plate cam, slider-crank, four-bar, gear pair; see `docs/LIB.md`). Imports are
resolved on the token stream, so nothing downstream changed: a part from a library runs like the part
pasted in place.

The rest is reserved: static friction proper (the regularization creeps at ~2 mm/s
instead of sticking), tangential impact impulses, curve-against-curve contact (the
Geneva drive's locking arc), and turning the escapement's event summary into contact
rows. Also reserved: motion-work *synthesis* (the
compiler choosing the equal-tooth-sum stack itself, rather than the flagship
writing it out), contact-rich pallet geometry (impulse faces as constraint
rows rather than event summaries — what 6c unlocks), and multiple mechanisms
in one file sharing a world.
