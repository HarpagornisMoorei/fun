#!/usr/bin/env python3
"""Summarize a CSV written by `mech run --headless --csv`.

Standard library only. For each column: min, max, mean, final value. Columns named
`*.engaged` also report how often the contact changed state (lift-offs and landings),
and the energy ledger's worst residual is called out, since "does the energy close?"
is the first thing to ask of a dynamic run.

    python3 scripts/summarize_run.py run.csv                   # every column
    python3 scripts/summarize_run.py run.csv 'KE[J]' 'E[J]'    # just these
"""
import csv
import sys


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    with open(argv[1], newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [[float(c) for c in r] for r in reader if r]
    if not rows:
        print("no data rows")
        return 1
    want = argv[2:] or header
    missing = [w for w in want if w not in header]
    if missing:
        print("no such column:", ", ".join(missing))
        print("columns:", ", ".join(header))
        return 1
    t = [r[0] for r in rows]
    print(f"{len(rows)} rows, t = {t[0]:g} .. {t[-1]:g} s, {len(header)} columns\n")
    w = max(len(n) for n in want)
    print(f"{'column':<{w}}  {'min':>13} {'max':>13} {'mean':>13} {'final':>13}")
    for name in want:
        col = [r[header.index(name)] for r in rows]
        print(f"{name:<{w}}  {min(col):>13.6g} {max(col):>13.6g} "
              f"{sum(col) / len(col):>13.6g} {col[-1]:>13.6g}")
    for name in header:
        if name.endswith(".engaged"):
            col = [r[header.index(name)] for r in rows]
            off = sum(1 for a, b in zip(col, col[1:]) if a == 1 and b == 0)
            on = sum(1 for a, b in zip(col, col[1:]) if a == 0 and b == 1)
            print(f"\n{name}: {off} lift-off(s), {on} landing(s) between recorded rows "
                  f"(thin the output with --every and you may miss some)")
    if "ledger.residual[J]" in header:
        res = [abs(r[header.index("ledger.residual[J]")]) for r in rows]
        print(f"\nenergy ledger: worst |residual| = {max(res):.3g} J")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
