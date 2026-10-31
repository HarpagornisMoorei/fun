# Speed (M18)

The simulation core was profiled, and the three things that dominated were made faster without changing a single
output number (the CSV of every example is byte-identical to what it was; `scripts/bench.sh save`/`check` does that comparison).

| what | where | before → after |
|---|---|---|
| polygon nearest-point search | `ProfileCurve::nearest` | a Geneva drive: 1.7 s of computing per simulated second → 0.4 s |
| the constraint matrix `J W J^T` and the multiplier solve | `DynamicsSystem` | an 80-link pendulum chain: 13.6 s → 2.1 s per simulated second |
| sweeps | `mech sweep` | rows run on every core: 3.2× on 4 cores, same output |

## What was done

* **Polygon contact.** A polygon profile used to measure the distance to every straight piece and every fillet (with
  trigonometry for the fillets) on every query; a notched 90-facet disc has 180 pieces. Each piece now has a cheap
  lower bound on its distance (to its bounding circle), the piece with the smallest bound is measured first, and any
  piece whose bound is above the best so far is skipped. The answer, including which piece wins a tie, is
  unchanged; the test compares it to the brute-force distance to every segment of the outline.
* **Sparse rows.** A joint's constraint row touches two bodies, so a row of `J` has a handful of nonzeros however big the
  mechanism. The products that form `J W J^T`, `J qd` and `W J^T lambda` visit only those; the terms left out are
  exact zeros, so every sum is bit for bit what it was.
* **Envelope Cholesky.** The factor of `J W J^T` has the same envelope as the matrix (row `i` of `L` is zero left of the
  first nonzero of row `i`), and a chain of joints makes that envelope narrow. The factorization and both
  substitutions visit only the terms inside it. A dense matrix has a full envelope and gets the plain algorithm. The test
  compares against the dense algorithm on chain, arrow and dense matrices: identical on the machine it was developed on (the
  bench script's byte comparison), and to rounding error in the test, because another compiler may fuse multiply-adds differently.
* **Scratch buffers.** Those matrices were allocated and zeroed afresh on every step (a large share of the time of a big
  mechanism was the operating system handing out and reclaiming memory); they are now reused.
* **Parallel sweeps.** Every row of a sweep parses and builds its own model from the shared tokens, so the rows are
  independent. `mech sweep ... --jobs N` runs them on N threads (default one per core); each row writes its own slot, so
  the output is the same whatever N, and a design that does not build is an error row in its place.

## Measuring

```bash
sh scripts/bench.sh                 # seconds of computing for the Geneva, a ratchet, a cam, a clock, and chains of 20/40/80 links
sh scripts/bench.sh save base       # keep the results ...
sh scripts/bench.sh check base      # ... and compare after a change: every file must be identical
```

Typical numbers on this machine (seconds of computing, for the simulated time shown): Geneva 1.1 s for 3 s; ratchet 0.3 s for
3 s; jumpcam 0.6 s for 1 s; clock 0.5 s for 20 s; chains of 20, 40 and 80 links 0.2, 0.6 and 2.1 s for 1 s.

## What is still slow

* The cost of a large mechanism still grows faster than linearly (80 links is 3.5× the cost of 40): the multiplier matrix is stored
  densely and the Cholesky factor is only skipped where it is zero, not stored compactly, and the Jacobian is built dense.
  A fully sparse formulation would be the next step for hundreds of bodies.
* Contact search is per touch per evaluation; many touches against one big profile still pay the lower-bound pass
  over all its pieces (a spatial grid would remove that).
* Kinematic runs (a solve per step) were not the problem and were not touched.
