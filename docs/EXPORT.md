# Exporting a run

`mech run --headless` prints a table every half second. To *measure* a mechanism —
plot a follower's height, check a contact force, watch the energy ledger — export the
whole run as CSV:

```bash
mech run examples/platecam.mech --headless 2 --csv plate.csv
mech run examples/jumpcam.mech  --headless 1 --csv jump.csv --every 10
mech run examples/eccentric.mech --headless 1 --csv ecc.csv --probe follower.tip
```

| option | meaning |
|---|---|
| `--csv <file>` | write the run (needs `--headless <sec>`) |
| `--probe <body.anchor>` | also record that anchor's world `x, y` (repeatable) |
| `--every <n>` | record every n-th step (the first row, t = 0, is always written) |
| `--dt <sec>` | kinematic files only: the frame step (default 1/60). A dynamic file integrates at its `world` step; asking for `--dt` is an error rather than silently ignored |

Misuse is refused with a nonzero exit status: an unknown or ground-hosted probe, `--csv`
without `--headless`, `--every 0`, `--dt` on a dynamic file.

## Columns

Everything is SI, and the unit is part of the name (`cam.theta[rad]`). Names are unique;
the header is one line, comma-separated; values are written to 12 significant digits.

| column | when | meaning |
|---|---|---|
| `t[s]` | always | time |
| `<body>.theta[rad]`, `.x[m]`, `.y[m]` | always | each moving body's pose (its origin, unwrapped angle) |
| `<body>.<anchor>.x[m]`, `.y[m]` | `--probe` | that anchor in the world |
| `maxRes` | always | the largest constraint residual — how well the mechanism is assembled |
| `<body>.<anchor>@<profile>.gap[m]` | per `touch` | signed gap to the curve, less the roller radius: `0` while in contact, `> 0` in flight |
| `.foot.x[m]`, `.foot.y[m]`, `.normal[rad]` | per `touch` | the contact point and the angle of the outward normal there |
| `<body>.omega[rad/s]`, `.vx[m/s]`, `.vy[m/s]` | dynamic | velocities |
| `<body>.<anchor>@<profile>.N[N]`, `.engaged` | dynamic, per `touch` | the normal force; `1` while on the curve, `0` in flight |
| `drive.<body>[N*m]` | dynamic, per rate drive | the torque the drive transmits |
| `KE[J]`, `PE[J]`, `E[J]` | dynamic | mechanical energy |
| `ledger.weight[J]`, `.impulse[J]`, `.drive[J]`, `.impact[J]`, `.friction[J]` | dynamic | the cumulative terms of the energy ledger |
| `ledger.residual[J]` | dynamic | `(E - E0) - (weight + impulse + drive - impact - friction)` |

The last column is the one to watch. It is the error in the books at every sample, not a
single number at the end of the run: if it drifts, something dissipates or injects energy
that the model does not account for.

## Recipes

A dependency-free summary (min, max, mean, final of any column, how often a contact let
go, and the ledger's worst residual):

```bash
python3 scripts/summarize_run.py jump.csv 'follower.y[m]' 'follower.tip@lobe.N[N]'
```

The pressure angle of a cam, from the *solved contact* rather than the design formula.
The rail is vertical, so it is the normal's angle from vertical:

```python
import numpy as np, pandas as pd
df = pd.read_csv("plate.csv")                       # mech run examples/platecam.mech --headless 2 --dt 0.001 --csv plate.csv
alpha = np.degrees(np.abs(df["follower.tip@lobe.normal[rad]"] - np.pi / 2))
print(alpha.max())                                   # 20.590
```

`mech check examples/platecam.mech` prints 20.59 degrees from the displacement diagram's
formula. The two routes — a formula, and the actual solved geometry — agree.

Any tool reads the file (a spreadsheet, gnuplot, matplotlib, R). The tests cover the
Recorder, the CSV text and the command line; the pandas recipe above was run, but there is
no plotting code in this repository.

## Notes

* A contact that lifts off and lands between two recorded rows is invisible in the file if
  you thin it with `--every`; the summarizer says so. Record every step (the default) to
  see all of them.
* `.normal[rad]` is the *world* angle of the outward normal; for a follower on a vertical
  rail the pressure angle is its distance from `pi/2`.
* The kinematic file records the same pose columns as the dynamic one but no velocities,
  energies or ledger: there are no masses to give them meaning.
