# The Geneva drive and intermittent motion (M17)

A Geneva drive turns a steady rotation into stop-and-go: a crank carries a pin, and once per turn the pin enters a
slot of the wheel, carries it through 360/N degrees, leaves, and a locking disc holds the wheel still until the
next turn. It was the open item of the contact design (`docs/design/contact.md`: "curve-against-curve contact,
not point-on-curve"). The answer turned out to need no new joint: a Geneva drive is a handful of ordinary
unilateral contacts (M6c), and what M17 adds is the part that sets them up correctly, the tests that hold it to a
closed form, and two things the build exposed.

```
import "../lib/geneva.mech"
world { gravity: 0  step: 0.0002 }
ground bench { anchor o at (0, 0) }
use geneva(N = 4) as gv on bench at (0, 0)
drive gv.crank at 60 deg/s
```

`mech run examples/geneva.mech --headless 10` and `mech render examples/geneva.mech --frames 48 --span 6 -o geneva.svg`.

## How it is made of contacts

* **The pin in the slot.** The crank's pin anchor is a `touch` with a roller on the wheel's `slots` profile, a polygon
  profile whose slots are notches. The pin is *inside* the slot, i.e. outside the profile's material, so the contact is the
  ordinary signed-gap row of M6 with the nearest wall.
* **The locking.** Each slot-mouth corner of the wheel (two per slot, as anchors) is a `touch` against the crank's `lock`
  profile, a polygon: a disc with a notch cut out around the pin. Disc on concave arc is replaced by corners on a disc,
  which constrains the wheel's turn in the same way and needs only point-on-curve contact.
* All of it is unilateral and dynamic: lift-off, impact with a plastic landing, the energy ledger.

The design numbers come from the geometry, all computed in the part: with centre distance `a`, the pin radius on the crank is
`a sin(π/N)` and the wheel's radius at the slot mouths is `a cos(π/N)`, which is where the pin is tangent to the slot
as it enters (the right angle at the pin of the triangle wheel–crank–pin). The locking disc has the radius of the circle
about the crank through the two mouth corners that face it when the wheel is locked, less a small gap.

## What it is checked against

While the pin is in a slot, the wheel's angle is the pin's direction seen from the wheel's shaft. With `c` the crank angle,
the pin is at `(a + r cos c, r sin c)` from the shaft and

```
φ(c) = atan2(r sin c, a + r cos c) − π/N  ± asin(clearance / |pin|)
```

(`+` on the leading wall, `−` on the trailing wall). The pin has a little side clearance (`clearance`, default 0.1 mm per
side; it is zero in the ideal Geneva, where the pin would chatter between the walls and the contact solver with it), so the
pin presses one wall until the crank is at 180° and the wheel stops speeding up, then the *other* wall: the wheel coasts
across the 2·clearance of backlash and the contact switches with an impact. The tests hold the wheel to the
leading-wall form to 10⁻⁷ rad before that and to the trailing-wall form after, for N = 3, 4 and 6, check that the wheel does not
move at all before the pin comes in, that it ends within 0.8° of a full step `−360/N`, and that the energy ledger closes
(impacts accounted) to 10⁻⁵ J.

## What went wrong on the way, and the rules that came out of it

* **A locking disc that is too snug jams the mechanism.** The disc is a convex circle against the wheel's mouth corners; with a
  gap of 20–50 µm the rim's edge catches a corner on its way in or out and the drive is forced through it (the energy
  ledger blows up to hundreds of joules). With 0.3 mm (`lock_gap`) the four sizes N = 3 … 6 are clean (ledger ≤ 10⁻⁷ J); the
  price is 0.3 mm of slack in the lock, about 0.35° of wheel.
* **The notch must be wide enough for the mouth corners to pass.** The corners come closer to the crank than the notch's floor
  unless the floor is well inside their closest approach (`Rn = ½(a − Rtip)`) and the notch is as wide as the engagement
  (`90 − 180/N` degrees half-width: 45° for N = 4). Narrower, and a corner hits the disc in mid-index.
* **`mech check` now warns when a contact starts inside its curve** ("starts 0.297 mm inside the curve"). A dynamic contact
  starting in flight used to be the only possibility the check recognised, and a disc a little too big looked like one.
* **The mobility count ignores unilateral contacts in a dynamic mechanism.** Nine contacts are not nine constraints; the report said
  "over-constrained" for a mechanism that moves.

## Limits

* It is an external Geneva drive with radial slots and a locking disc that is a disc with a notch; no internal Geneva,
  no multi-pin crank, no Maltese-cross variations other than `N`.
* The lock is a point-contact stand-in for a conformal arc, and the wheel has the lock's slack (a third of a degree at the
  default `lock_gap`) rather than none.
* Contact is frictionless unless you add `friction` (the part does not). Zero `clearance` chatters; keep it positive.
* Runs are not fast: nine contacts with a 0.2 ms step is about 1.7 s of computing per simulated second.
