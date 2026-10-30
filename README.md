# mech

A small declarative language for planar mechanisms. You describe bodies,
joints and forces; the solver decides everything else — nothing is assigned
causally, so the same file can describe a wiper (drive the crank) or a clock
(drive the weight).

```
mechanism fourbar {
  ground base {
    anchor p1 at (0, 0)
    anchor p2 at (3, 0)
  }
  rod crank   { length: 0.8 }
  rod coupler { length: 2.6 }
  rod rocker  { length: 2.2 }

  pin crank.A   -> base.p1
  pin coupler.A -> crank.B
  pin rocker.A  -> base.p2
  pin coupler.B -> rocker.B

  drive crank at 36 deg/s from 60 deg
}
```

A closed kinematic loop like this is something tree-based formats
(URDF/SDF) cannot express.

## Build and run

Plain C++17, no dependencies. GLFW is optional (for the window).

```bash
make            # builds ./mech (headless-only if GLFW is missing)
make test       # unit tests
```

```bash
./mech check examples/fourbar.mech                 # parse, DOF analysis, initial assembly
./mech run   examples/fourbar.mech --headless 5    # simulate without a window
./mech run   examples/fourbar.mech --trace coupler.B   # needs GLFW: draw the coupler curve
```

## What it can express

| | |
|---|---|
| bodies | `ground`, `rod`, `shaft`, `body` |
| joints | `pin` (revolute), `slide` (prismatic) |
| gears | `gear`, `mesh` (external, internal, epicyclic) |
| contact | `profile` (circle, polar table, point list), `touch` — cams with translating or swinging followers, optional roller |
| teeth and corners | `polygon` profiles — straight edges with filleted corners, exact (no spline ringing) — a ratchet wheel and pawl that clicks forward and locks in reverse |
| non-circular gears | `mesh` between two profiles (circle, polar table, `ellipse`) on fixed shafts — elliptical gear pairs whose output surges; the pair's conjugacy is checked at build time |
| cams | `cam` — state the motion (`rise`, `dwell`, `return`; cycloidal, harmonic, modified-trapezoid) and the compiler cuts the surface; `check` reports pressure angle and rejects undercut |
| forces | gravity, `spring`, torque and weight `drive`s |
| hybrid | `mode` state machines (the escapement) |
| intent | `reduce`, `beat`, `hand`, `dial` — solved into the kernel |

Contact is bilateral in a kinematic file (force closure assumed) and unilateral
in a dynamic one: a spring-loaded cam follower holds the cam at low speed, and
at high speed lifts off, flies and lands with a Newton impact, with every joule
accounted for (`examples/jumpcam.mech`).

Kinematic files are solved with an analytic-Jacobian Levenberg–Marquardt
assembly; files with masses and forces are integrated as constrained
dynamics, with an energy ledger that has to close.

## Examples

| file | shows |
|---|---|
| `fourbar`, `crankslider` | closed loops, prismatic joints |
| `geartrain`, `train`, `planetary` | meshes, synthesized reductions, epicyclics |
| `eccentric`, `lobecam`, `platecam` | cams: a closed-form eccentric disc, a plate cam from a polar table, and one synthesized from a displacement diagram |
| `ratchet` | a ratchet wheel and gravity pawl: one click per tooth forward, locked in reverse; energy ledger closes |
| `ellipgears` | a pair of elliptical gears: input steady, output surging between 3x and 1/3x |
| `jumpcam` | the plate cam with mass, a spring and a contact that can let go — cam jump, impacts, energy ledger |
| `pendulum`, `doublepend`, `flywheel`, `oscillator`, `orbit` | dynamics |
| `clock`, `clockface` | the flagship: a complete weight-driven clock |

## Documentation

[`docs/LANG.md`](docs/LANG.md) is the language specification — grammar,
semantics, solver formulation — and the contract `mech test` checks the
implementation against. [`docs/design/contact.md`](docs/design/contact.md)
is the design for contact (cams, Geneva drives, ratchets, non-circular
gears); M6a through M6e (profiles, cam synthesis, unilateral contact,
non-circular gears, polygons and a ratchet) are implemented. `scripts/` holds offscreen-rendering harnesses for
screenshots without a display.
