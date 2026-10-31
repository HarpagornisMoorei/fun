# Parts and instances (M9)

A mechanism is rarely made of unrelated pieces. A cam and its follower, a gear
pair, a ratchet and its pawl: the same assembly appears twice, with different
numbers. M9 lets a `.mech` file write such an assembly once, as a **part**, and
make **instances** of it.

Nothing about what a mechanism *means* changes. An instance is expanded while the
file is parsed: the part's body is read again for each `use`, with every name
qualified by the instance. By the time the model is built it is one flat
mechanism, so the solver, the dynamics, the energy ledger, the CSV export and
sweeps all work on it unchanged. The tests rest on that: an instanced file builds
the same model, and runs the same numbers bit for bit, as the flat file written
out by hand (see section 7).

## 1. Declaring a part

```
part plate_cam(lift = 0.012, base_r = 0.025, speed = 30) {
  ground frame {
    anchor axle at (0, 0)
    anchor rail at (0, 0.09) dir (0, 1)
  }
  shaft cam at frame.axle
  cam lobe on cam.A { base base_r  roller 0.006  rise lift over 100 deg cycloidal  dwell 60 deg
                      return over 140 deg modtrap  dwell }
  body follower { anchor tip at (0, 0) dir (1, 0) }
  slide follower.tip -> frame.rail
  touch follower.tip on lobe
  drive cam at speed rpm
}
```

* A part is declared at the top of the file, outside any `mechanism`, **before**
  the mechanisms (or other parts) that use it, or brought in with `import` (section 6). It is
  visible to every mechanism in the file.
* The parenthesised list gives its **parameters** with defaults. Defaults are
  expressions (see `PARAMS.md`) and may use the parameters before them
  (`ext = len * 2`). `part p { ... }` with no list is a part with no parameters.
* The body takes the same statements as a mechanism: bodies, rods, shafts, gears,
  `reduce`, springs, `pin`, `slide`, `drive`, `mode`, profiles, `touch`, `cam`,
  `hand`, `dial`, `param`, and `use` (parts can contain parts). Two exceptions:
  a part has no `world` block (the world belongs to the mechanism), and it cannot
  use itself.
* The body is checked where it is written, with the default values, even if the
  part is never used. A mistake is reported at its own line.

Inside the body, every name belongs to the instance. There is no way to reach
out to a name of the enclosing mechanism; a part is closed. It connects to the
rest through its **ground** and through pins written by the mechanism (below).

## 2. Using a part

```
mechanism twincam {
  ground bench { anchor origin at (0, 0) }
  use plate_cam(lift = 0.020) as left  on bench at (0, 0)
  use plate_cam               as right on bench at (0.2, 0)
}
```

`use PART [ (arg = expr, ...) ] as NAME [ on GROUND at (x, y) [ rotate A deg ] ]`

* Arguments are expressions evaluated in the **caller's** scope (they may use the
  mechanism's own `param`s). Arguments not given take the part's defaults.
* **Names.** Everything the part declares gets the instance's name as a prefix:
  `left.cam`, `left.lobe`, `left.follower`. You refer to them that way from
  outside: `drive left.cam at 30 rpm`, `pin left.follower.tip -> ...`. Anchors are
  per body, so `left.follower.tip` is the anchor `tip` of the body `left.follower`.
  The same names appear in `mech check`, in error messages, in CSV columns
  (`left.follower.y[m]`), in `--probe left.follower.tip` and in sweep reports
  (`max:left.follower.y`).
* **The ground.** If the part has a `ground`, its anchors are added to the ground
  the instance is mounted `on`, shifted by `at (x, y)`, and named
  `left.frame.axle`, `left.frame.rail`. Inside the part you keep writing
  `frame.axle`; the prefix and the shift are applied for you. The shift is a
  translation, unless `rotate` turns the part (section 3). A ground-less part needs no
  `on ... at`, and giving one is an error; a part with a ground that is used
  without it is an error too.
* An instance name cannot be reused, and cannot be the name of a body.

A part is joined to the rest of the mechanism with ordinary statements that
mention qualified names. `examples/linkage_parts.mech` builds a crank-slider from
one ground-less `link` part used twice:

```
use link(len = 0.3) as crank
use link(len = 1.0) as conrod
pin conrod.r.A -> crank.r.B
```

## 3. Rotating a mount

```
use plate_cam(lift = 0.02) as east on bench at (0.04, 0) rotate -90 deg
```

`rotate A deg` turns the part rigidly by `A` (counter-clockwise, an expression, so it can be a
parameter and be swept) about the mount point `(x, y)`. Everything the part contributes to the ground
turns with it: the positions of its ground anchors and their `dir`. That is what lets a
vertical-rail cam point its follower any way you like.

What else turns, and what does not:

| in the part | with `rotate` |
|---|---|
| ground anchors: position and `dir` | turn about the mount point |
| a rod's or body's `from:` angle | gains `A` |
| a rate drive on a **rod or body**, `drive r at ... from 20 deg` (the start angle) | gains `A` |
| a rate drive on a **shaft** | unchanged: a shaft's angle is measured against what it carries |
| a cam's `start` (where its rail lies in the cam frame) | gains `A`: the cam is cut for the rail that turned with the part, so its lift against the shaft angle is the same whichever way the rail points |
| the default starting guess of the solver for rods and bodies | turns with the part, so a rotated copy assembles on the same branch |
| gravity | does **not** turn |
| a drive written *outside* the part (`drive c.cam ...`) | is a world-frame statement and is not adjusted |

A part with a `mode` cannot be rotated (an error at the `mode`, inside the part): its guards are angles
in the gravity field, and a rotated clock is a different clock. Rotations nest: an inner `rotate` is in
the outer part's frame, and the angles add (`rotate 90` inside `rotate 90` is 180).

What this gives you, and what is tested: for kinematic parts, a rotated instance is the unrotated one turned
about the mount point. Every body's position turns with it and the angles of rods shift by `A`, at any
angle, for the slider-crank, four-bar, gear pair and plate cam; a dynamic part (a pendulum, for
instance) equals the flat file with the turned numbers written out, bit for bit; `rotate 0 deg` is no
rotation, bit for bit.

## 4. Parameters of an instance

Each instance has its own copy of the part's parameters, named `instance.param`
(`left.lift`). These are the parameters of `PARAMS.md`:

```bash
mech check examples/twincam.mech                     # params: left.lift = 0.02 ...
mech run   examples/twincam.mech --headless 2 --set left.lift=0.03
mech sweep examples/twincam.mech --set left.lift=0.012:0.036:0.008 --time 2 \
     --report max:left.follower.y --report max:right.follower.y
```

* `--set` beats an argument, which beats the default. Whatever depends on the
  parameter follows (`ext = len * 2` inside the part).
* `param` statements inside a part are the instance's own derived constants and
  are addressed the same way (`left.total`).
* The bare name `lift` is **not** a parameter of the mechanism. A `--set lift=...`
  is the usual "no such parameter" error, which lists what the file declares
  (`left.lift, left.base_r, ...`).
* An instance inside a part has the full dotted name, `outer.inner.param`; an
  outer argument flows into the inner one (`lift = lift * 2`), and an override of
  the outer parameter reaches both.

## 5. Parts in parts

```
part pair(lift = 0.012) {
  ground g { anchor m at (0, 0) }
  use plate_cam(lift = lift)     as a on g at (0, 0)
  use plate_cam(lift = lift * 2) as b on g at (0.2, 0)
}

mechanism m {
  ground base { anchor origin at (0, 0) }
  use pair(lift = 0.01) as p on base at (0.1, 0)
  drive p.a.cam at 30 rpm
}
```

The inner part is mounted on the outer part's ground, whose own position is the
sum of the offsets. A part can only use parts declared before it, which also rules
out recursion (the error says so).

## 6. Importing parts

```
import "../lib/plate_cam.mech"
```

A statement at the top of a file, before the parts and mechanisms that use what it brings in. The
named file's parts take the statement's place, as if they had been written there; nothing else about
the language changes. The standard parts are in `lib/` (see `LIB.md`).

* **Where it looks.** The path is resolved relative to the *importing* file (so a library can import
  its neighbour by a plain name, whatever the working directory), then in each directory of the
  `MECH_PATH` environment variable (colon-separated). The path must end in `.mech`. A file that cannot
  be found is an error that lists every place tried.
* **What a library holds.** Parts, and imports of other libraries, nothing else; a `mechanism` in a
  library is an error.
* **Once.** A file is spliced in once however many files import it, and however its path is spelled
  (`c.mech`, `./c.mech`, `sub/../c.mech`). Two *different* files defining the same part name is the usual
  "already declared" error, pointing at the second.
* **Cycles.** A file that imports itself, directly or through others, is an error that names the chain.
* **Errors point at the right file.** A mistake inside a library, a bad parameter that the library's
  `cam` rejects, a typo in a part, is reported with the library's path and line, in `check`, `run` and
  `sweep` alike. `mech check` lists what was imported.
* **Not a module system.** There are no namespaces: imported parts share the file's one namespace. An
  `import` inside a `mechanism` or `part` is not a statement.

## 7. What it does not do

* **No reflection** when mounting (rotation is supported, mirroring is not); no mounting on a moving body. To
  attach an instance to a moving body, give the part a ground-less body and pin it
  from the mechanism, as in the linkage example.
* **No conditionals or loops**: the structure of an instance does not depend on
  its parameters (the numbers do). Five identical instances are five `use` lines.
* **Rotation is about the part's mount point and only for what the part owns.** It does not rotate gravity,
  and refuses a part with a `mode`. A drive you write outside the part is in the world frame: when you
  drive a rod of a rotated part from outside, add the rotation yourself (`from 20 + A deg`).
* A part cannot declare a `world`, and a file's mechanisms do not share instances.

## 8. Errors

Errors inside a part carry the line of the part's own text, and the model builder
names the instance:

```
error: cam 'left.lobe': 'base' must be positive      (line 8, inside the part)
```

Misuse is caught at the `use` (a `rotate` without a mount, without `deg`, or on a part containing a `mode` included): unknown part (listing the declared ones), an
argument the part does not have (listing the ones it does), an argument given
twice, a ground-less part given `on ... at`, a part with a ground and no `on`, a
mount that is not a ground declared before, a name already taken.

## 9. How it is checked

* Two instances of the plate-cam part, with different lifts and speeds, run
  **bit for bit** like the flat file with the same bodies written out.
* The whole `examples/clock.mech` (escapement with its hybrid dynamics, weight
  drive, a synthesized reduction, hands, dials) wrapped in one part and used once
  gives **identical** numbers over 30 s of simulated time: every row, every
  column. Moved elsewhere on the ground, it is the same clock.
* `examples/jumpcam.mech` as a part: `--set u.speed=300` equals `--set speed=300`
  on the flat file, and a sweep over `u.speed` finds the same lift-off counts and
  energy-ledger residuals.
* Nested parts equal the flat file, with the mount offsets added as the flat file
  adds them.
* Rotated mounts: covariance (positions turn about the mount point, rod angles shift by the
  rotation) at four angles for the slider-crank, four-bar and gear pair, and the plate cam's lift
  along the turned rail equal to the unrotated one; a drive inside a rotated part equals the same
  drive written outside with the angle added; a rotated pendulum equals the flat file with the
  turned numbers written out, bit for bit; nested rotations add; `rotate 0 deg` changes nothing.
* Mutation checks: breaking prefixing, the ground merge, the mount offset, the
  scope of parameters, argument binding, override keys, the recursion guard, the
  definition-time check, or any one of the name sites (`mesh`, `reduce`, `via`,
  `hold`, mode guards and resets, `drive`, hands, gear hosts, touch profiles)
  fails tests.
