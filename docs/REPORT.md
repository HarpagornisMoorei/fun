# `mech report`: what a mechanism has to carry (M20)

```bash
mech report examples/slidingbead.mech --time 1
mech report examples/geneva.mech --time 6 --top 4 --csv loads.csv
```

The loads on a design are what you size parts with, and the solver already computes them: every constraint row has a multiplier,
which is the force (or moment) the joint must supply. `mech report` runs a dynamic mechanism and reduces each load to a **peak**
(with the time it happens), an **RMS** and a **mean**.

| group | what | what to size with it |
|---|---|---|
| drives | torque, speed and power of each rate drive; force, speed and power of a linear drive | the motor: peak torque, RMS torque against its continuous rating, peak speed, peak and mean power |
| pins | the force a pin carries, as a size (N) | the bearings: peak and RMS radial load |
| slides and slots | the side force on the guide (N) and, for a slide, the moment (N·m) | the rail and the carriage's bearings |
| springs | tension | the spring's rating and fatigue range |
| contacts | normal force of each touch | the contact stress, the cam and follower |
| racks, belts | the tooth force of a rack; the force carried by each belt link | the teeth, the belt's tension rating |

Mean power is signed: a negative mean means the mechanism gives energy back to the drive over the window. A drive's power is its
torque times the body's speed, the same quantity the energy ledger integrates.

## Options

* `--time s` simulated seconds (default 2). **Choose a whole number of cycles** (or the part of the cycle you care about): RMS and mean
  are averages over the window.
* `--set name=value` overrides a parameter, as everywhere, so a sizing study is a loop in the shell (or use `mech sweep` for a number
  per run).
* `--top n` shows the n heaviest joints in a group (default 8; 0 shows all).
* `--csv file` writes every row: `group,name,quantity,unit,peak,peak_at_s,rms,mean`.

## What the numbers are, and are not

They are the loads of an **ideal** model: rigid bodies, frictionless joints (except where a `touch` has `friction`), no joint clearance, no
elasticity. They are what the constraints must supply for the motion the dynamics produces, which is what you need to size the parts;
they are not a stress analysis (no bending of a rod, no bearing life), and a real joint's friction adds drive torque this model does not
have. A kinematic mechanism has no masses and nothing to carry, so a report needs a `world` block.

For every force there is a CSV column too: `mech run --headless ... --csv` does not include them by default (the columns are many), and
`simulate(..., forces = true)` / the report add `<A>.<a>-<B>.<b>.pin[N]`, `<guide>.normal[N]`, `<guide>.moment[N*m]` and
`<spring>.spring[N]`.

## Checked against closed forms

* A bead in a tube turned at constant ω: the side force is the Coriolis force `2 m r₀ ω² sinh ωt` and the drive's torque `2 m r₀² ω² cosh ωt sinh ωt`
  (peak, time of peak and RMS).
* A mass m at distance d from its pin, turned at constant speed in gravity, over a whole turn: drive torque peak `m g d`, RMS `m g d/√2`, mean 0; power
  peak `ω m g d`, mean 0; pin force peak `m(ω²d + g)` and RMS `m √(ω⁴d² + g²)`.
* At rest: a hanging mass loads its pin with `m g`; a block on a rail loads it with `m g` and no moment; a stretched spring has tension `k x`, and an oscillating
  one has RMS `k x₀/√2` over whole periods.
