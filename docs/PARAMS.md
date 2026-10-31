# Parameters, expressions and sweeps (M8)

A design is rarely one set of numbers. M8 lets a `.mech` file name its numbers,
override them from the command line, and ask the tool to run many variants and
report one number per variant.

Nothing here changes what a mechanism *means*: parameters are resolved while
parsing, so by the time the model is built every number is a plain `double`
and the solver, dynamics and tests see exactly what they saw before.

## 1. Declaring parameters

```
mechanism jumpcam {
  param speed    = 600            // cam speed, rpm
  param preload_k = 200           // spring stiffness, N/m
  param lift     = 0.012
  param half     = lift / 2

  drive cam at speed rpm
  spring s a -> b { k: preload_k  rest: 0.020 }
}
```

* `param name = expression` appears inside a `mechanism`, anywhere a statement may.
* A parameter can only be used **after** its declaration (it is a parse-time
  constant, not a lazily solved equation). Later parameters may use earlier ones.
* The name is an ordinary identifier, with two restrictions: it must not collide
  with a unit token (`rpm`, `deg`, `mm`, ...: `param rpm = 5` is rejected, use
  `speed`), and it may not be declared twice.
* Parameters are scoped to their mechanism.

## 2. Expressions

Wherever the grammar wanted a number it now accepts an expression:

| form | meaning |
| --- | --- |
| `a + b`, `a - b`, `a * b`, `a / b` | usual precedence; `*` `/` bind tighter than `+` `-` |
| `-a`, `+a` | unary sign |
| `( ... )` | grouping |
| `pi` | 3.14159... |
| `sqrt sin cos tan atan abs` `(x)` | one argument; trig in **radians** |
| `min max pow` `(x, y)` | two arguments |
| a parameter name | its current value (declared or overridden) |

Units still attach to the finished number: `rise lift * 2 over 100 deg` is
`rise (lift*2) over 100 deg`. Expressions are checked as they are evaluated:
division by zero, a domain error (`sqrt(-1)`), a non-finite result, an unknown
function or the wrong number of arguments are parse errors pointing at the
offending token.

Because `-` is now an operator, `a-b` is a subtraction. A negative literal such
as `at (-0.01, 0)` works as before.

## 2b. Conditions and `if` (M19)

Comparisons and logic are numbers, 1 for true and 0 for false (anything but 0 counts as true):

| form | meaning |
| --- | --- |
| `a < b`, `a > b`, `a <= b`, `a >= b`, `a == b`, `a != b` | comparison (arithmetic binds tighter) |
| `!c`, `c && d`, `c \|\| d` | not, and, or (`&&` binds tighter than `\|\|`) |
| `if(c, a, b)` | `a` if `c` is true, else `b`; only the chosen one is evaluated, so `if(n > 1, 1 / (n - 1), 0)` is safe |

They are read on the right of a `param`, inside parentheses, in a part's default or a `use` argument, and in `if` conditions. A bare
expression elsewhere is still arithmetic only (the `<` and `>` of a `mode` guard keep their meaning).

```
if n > 2 {
  rod big on g.a { length: 0.3 }
} else if n > 1 {
  rod mid on g.a { length: 0.2 }
} else {
  rod small on g.a { length: 0.1 }
}
```

The condition is decided while parsing, like a `for` loop's count, so the structure can depend on a parameter (and on `--set`, and on a
sweep axis). Exactly one branch is read; the others are skipped without being parsed (they may name things that do not exist
when they are not chosen). `if` works wherever `for` does: among a mechanism's statements, a body's anchors, and a polygon's vertices
(`polygon: (0, 0) (1, 0) if tip { (0.5, 1) }`), and the two nest in either order. A `param` cannot be declared inside an `if` or a `for`.

`require COND "message"` stops with that message if the condition is false. A part uses it to refuse parameters it cannot build:

```
part spoked(n = 4) {
  require n >= 2 "a wheel needs at least two spokes"
  ...
```

The error points at the `require` line (inside the library file, for an imported part). `lib/geneva.mech` states its limits this way: at
least three slots, a lock gap of at least 0.1 mm (a tighter one jams), a pin that fits the slot.

## 3. Overriding from the command line

```bash
mech check examples/jumpcam.mech --set speed=300
mech run   examples/jumpcam.mech --headless 1 --set speed=300 --set preload_k=400
```

`--set name=value` replaces the number on the right of the `param`. Every other
parameter that uses it follows, because the override is applied when the
declaration is read.

* The value must be a number (no expressions on the command line).
* An override naming a parameter the file does not declare is an **error**, not
  a silent no-op, and the message lists what the file does declare.
* `mech check` prints the effective values and what the file said:
  `params:  speed = 300 (set; the file says 600)`.

`--set` is accepted by `check` and `run` (headless and the viewer alike), and
`sweep` takes it in the extended axis form below.

## 4. Sweeps

```bash
mech sweep FILE --set AXIS [--set AXIS ...] --time T --report METRIC [...] [--jobs N]
```

An **axis** is a parameter with several values:

| form | values |
| --- | --- |
| `name=a:b:s` | a, a+s, ... up to b (inclusive; no accumulated drift) |
| `name=v1,v2,v3` | exactly these |
| `name=v` | a single value (fixes the parameter) |

With several axes the tool runs the full grid, **first axis slowest**. Grids over
5000 points are refused.

A **metric** reduces one run to one number:

| form | meaning |
| --- | --- |
| `max:COL`, `min:COL`, `mean:COL`, `final:COL` | over the sampled rows of a CSV column (see `docs/EXPORT.md`); the unit may be omitted, `follower.y` resolves to `follower.y[m]` |
| `liftoffs`, `impacts` | counts of contact events |
| `maxres` | worst constraint residual over the run |
| `ledger` | worst absolute energy-ledger residual |

Rows run on all cores (`--jobs N` picks the number of threads, 1 for none); the table is the same either way. Output is an aligned table; `--csv FILE` writes it as `axes...,metrics...,error`.

```bash
# where does the follower start to leave the cam?
mech sweep examples/jumpcam.mech --set speed=300:800:50 --time 0.15 --report liftoffs

# how much does eccentricity change the follower stroke?
mech sweep examples/eccentric.mech --set e=0.005:0.02:0.005 \
     --time 1 --report max:follower.y --report min:follower.y

# a two-axis grid
mech sweep examples/jumpcam.mech --set speed=300:700:200 --set preload_k=100,200,400 \
     --time 0.15 --report liftoffs
```

### 4.1 Designs that do not build

A sweep is exactly where a variant crosses a limit the compiler enforces (an
undercut cam, a gear pair that cannot mesh, a mechanism that cannot assemble).
Such a row is **not fatal**: its metrics are blank (`nan`), its `error` column
carries the compiler's own message, and the command exits with status 2 so a
script can tell that some designs were invalid. The first design in the grid
must build, otherwise the file itself is broken and the sweep stops with that
error. For `platecam.mech`:

```bash
mech sweep examples/platecam.mech --set rise_deg=100,60,30,20,10 --time 2 --report max:follower.y
```

builds the 100, 60 and 30 degree rises (stroke 43 mm each) and reports the 20 and
10 degree ones as undercut, with the compiler's own message, then exits with
status 2.

### 4.2 Choosing `--time`

Metrics are over the simulated window only. A shaft driven at 30 deg/s needs
12 s to complete a turn, so `max:follower.y` over 1 s sees only the first 30
degrees of the lift. Pick a window that covers the event you care about.

## 5. Checked against closed forms

* `jumpcam` follower lift-off: the sweep brackets the threshold at 444 rpm; the
  closed form (spring + gravity can accelerate the follower down no faster than
  the cam surface's peak deceleration) gives 443.9 rpm.
* `eccentric`: the maximum follower height is `R + rho + e`, the minimum
  `R + rho - e`, for each `e` in the sweep.
* Overrides are bit-equal to editing the number in the file (the test builds
  both and compares every field of the model).

## 6. Limits

* Parts (`docs/PARTS.md`, M9) give each instance its own parameters, addressed as `instance.name`
  in `--set` and in sweep axes (`--set left.lift=0.012:0.036:0.008`).
* Parameters are numbers. There are no parameter-valued strings, lists, or
  conditionals; structure cannot depend on a parameter.
* Command-line overrides are literals.
* Sweeps re-parse and rebuild the model per row; a 1000-row sweep of a
  kinematic design takes seconds, of a hybrid-dynamic one proportionally more.
