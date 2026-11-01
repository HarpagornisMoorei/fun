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

## The sparse solver (M21)

The first round made the dense solver skip its zeros; M21 removed the dense matrices. A constraint row touches the bodies it joins and
nothing else, so everything the solvers build is sparse, and now it is *stored* sparse too:

* **Rows.** `ConstraintSystem::jacobianSparse` gives the Jacobian as sparse rows (columns ascending, exact zeros left out); the dense
  `jacobian` is the same assembly into a matrix.
* **The multiplier system** `A = J W J^T` (dynamics, impacts, velocity projection) is formed only between rows that share a body, stored as a
  skyline (each row keeps the columns from its first nonzero to the diagonal), and factored by a skyline Cholesky with the same relative
  damping as before. The factor is computed once per solve and reused by the friction passes.
* **The kinematic solve** (Levenberg-Marquardt, every step of a kinematic run and the stabilization of a dynamic one) forms `J^T J` the same way.
* **Ordering.** The unknowns are numbered in the order the bodies were declared and the rows in the order the joints were written. For a chain
  declared in order that is the best order there is; declared at random it would make the envelope as big as the matrix. So the
  pattern is looked at once (and again only when it changes) and, if a reverse Cuthill-McKee numbering shrinks the envelope below 70%
  of the natural one, the solve works in that numbering.

Measured (seconds of computing per second simulated at 1 ms steps, an N-link pendulum chain):

| links | before M18 | after M18 | now (M21) |
|---|---|---|---|
| 40 | 1.7 | 0.6 | 0.36 |
| 80 | 13.6 | 2.1 | 0.7 |
| 160 | | 6.8 | 0.9 |
| 320 | | 33 | 2.9 |
| 1280 | | | 10.6 |

(blank: not measured at that stage)

The cost is linear in the size of a chain, declared in order or at random (`scripts/bench.sh`: `chain320` and `shuf320` take the same time).

**What changed in the numbers.** Declared in order, nothing: the sparse sums are the dense sums with the zeros left out, in the same order, and
`scripts/bench.sh check` compares the CSV of every benchmark with a saved baseline (up to the sign of an exact zero, which a different
arithmetic order can flip). Where the reordering applies (a mechanism written in an unlucky order, such as `examples/clock.mech`) the
arithmetic is done in a different order and results agree to rounding: the clock's CSV differs from before by 7e-12 relative after 20 s.

## What is still slow

* A hub body that many rows touch (a planet carrier with hundreds of planets) makes those rows a clique in `A`; that is dense in the number of planets.
* Contact search is per touch per evaluation; many touches against one big profile still pay the lower-bound pass over all its pieces
  (a spatial grid would remove that).
* Per step the code still allocates (sparse rows, hashing the pattern, scratch vectors); a profile of a 1280-link chain is spread over
  assembling, factoring and solving rather than dominated by one thing.
* Kinematic runs (a solve per step) benefit from the sparse LM solve, but the multi-start search for a hard configuration is unchanged.
