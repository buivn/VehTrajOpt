# 41 — Reeds–Shepp curves: giving the Ackermann lattice a reverse gear

Step 1 of adding reverse motion to the Ackermann planner (docs/39-40's
heading-9 trap). Pure math module + tests only — no wiring into
`lattice_ackermann.py` or the controller yet (those are steps 2-3).

## 1. General idea / theory

`dubins.py` (docs/24) finds the shortest bounded-curvature path between
two oriented poses using **forward motion only** — three segments from
{L, R, S}. **Reeds–Shepp curves** solve the same problem but let the car
also drive **backward**, with **cusps** (direction-reversal points)
between segments. Reeds–Shepp is provably never longer than Dubins for
the same two poses (forward-only is a strict subset of what's allowed),
and for configurations like heading 9's trap — proven in docs/40 by
exhausting the reachable state space, 877 states, none of them useful —
it's often the *only* way to connect two poses within a bounded local
region at all, not just the shorter way.

Pipeline placement is unchanged from Dubins: this is still purely the
**edge primitive** feeding the same state-lattice search (`lattice.py`).
The search doesn't need to know curves can reverse — a primitive is still
an opaque `(dh, cost, samples)`; only what step 2/3 do with a segment's
*direction* is new.

## 2. Method selection

- **Reeds–Shepp (chosen):** the standard closed-form answer to exactly
  this problem, same mathematical family as Dubins (same local-frame
  reduction), and what docs/24-25 already committed to ("Reeds–Shepp
  deferred behind the same `DubinsPath` interface").
- **Scope within Reeds–Shepp — deliberately not the full 48-word catalog.**
  Implemented: the CSC family (8 words: same-turn/opposite-turn × reflect ×
  timeflip) and the CCC family *including* the reversed-middle-segment word
  ("C\|C\|C", e.g. L+R−L+ — literally a K-turn). Not implemented: CCCC,
  CCSC, CCSCC. Those remaining families shave a few percent off path
  length in narrow angular ranges but add no new *reachability* — CSC+CCC
  already provably connects any two poses (the same completeness argument
  as parallel parking: small enough CCC maneuvers achieve arbitrary
  relative reorientation). Given the actual goal is eliminating "no path"
  cases (docs/40), not shaving a few percent off already-solvable ones,
  this is the right amount of catalog to implement — same kind of
  deliberate-subset call as docs/25's pruning-depth choice.
- **Closed-form CSC, numeric CCC — a deliberate, honest split, not
  laziness.** The CSC formulas are the same well-established pair
  (same-turn/opposite-turn tangent-circle constructions) that appear
  near-identically across the literature; re-deriving them carries modest
  risk. The reversed-middle CCC word's closed form is considerably more
  intricate, and getting a sign or angle-offset subtly wrong there would
  be easy to miss and hard to catch by inspection. Rather than trust a
  hand-derived trig identity for exactly the family this whole effort
  exists to add, it's solved with a bounded Newton iteration (≤30 steps,
  numeric 3×3 Jacobian, several seeds) directly against the forward
  kinematics — still O(1) per query (a small constant, not a growing
  search), just not closed-form. Rejected: memorizing/re-deriving the
  literature closed form for this one family purely to preserve "no
  iteration anywhere" — that consistency isn't worth the correctness risk
  for the one family that's the actual point of this doc.
- **Correctness safety net, not just a testing convention:** every
  candidate word — closed-form or numeric — is forward-simulated and
  checked against the real goal *inside* `reeds_shepp_shortest_path`
  itself before being offered as a candidate (`_verify`, tolerance 1e-6).
  A formula bug degrades to "this word quietly doesn't fire" rather than
  a wrong, possibly-colliding path silently entering the lattice. This
  is a real design decision, not boilerplate: it means the module's
  correctness doesn't depend on trusting the derivation, only on the
  (much simpler, much more obviously correct) forward-kinematics check.

## 3. Coding — design

**New file `reeds_shepp.py`** (`vto_planning/vto_planning/`), same shape as
`dubins.py`: a `ReedsSheppPath` class, `reeds_shepp_shortest_path()`,
`reeds_shepp_path_length()`, `sample_reeds_shepp()`. Key structural
differences from `DubinsPath`:
- `segments: list[(letter, signed_value)]` instead of `word` (3-char
  string) + `seg_norm` (3 unsigned floats) — a word's *letters* no longer
  determine cost/direction on their own once segments can independently
  be forward or backward, so direction has to travel with each segment,
  not be inferred from a fixed-length word string.
- `sample_reeds_shepp` needed **no new integration math** — `_arc_end`
  (the per-segment constant-curvature update) is exactly `_sample_arc`'s
  formula generalized to a nonzero starting heading, and it already
  produces the correct backward-motion result for a *negative* signed arc
  length, since the underlying ODE (`dx/ds=cos θ, dy/ds=sin θ, dθ/ds=κ`) is
  valid for negative `s` without modification. Backward motion is a sign
  choice on an already-correct formula, not a separate code path.

**Symmetries, not separate formulas, for most of the word count.** Rather
than deriving 8 independent CSC formulas, two base ones (`_LpSpLp`,
`_LpSpRp`) generate the rest via **reflect** (mirror across the local
x-axis: negate y and heading, swap L↔R in the result — proven correct by
symmetry, not assumed) and **timeflip** (derived carefully in the module
docstring/commit reasoning: solving the *swapped* start↔goal query, then
reversing segment order and negating every magnitude, produces the path
that drives the original query's shape entirely backward). Same idea for
the two CCC-reversed words (base + reflect + their timeflips = 4
candidates).

## Verification

- 8 targeted tests (`test_reeds_shepp.py`), including: endpoint-matching
  across 9 configurations × 3 radii (closed-form and numeric words alike);
  `reeds_shepp_length <= dubins_length` on every case tried (the
  reverse-is-optional invariant); a pure U-turn in place costing **π** vs.
  Dubins' 7.33 (more than half the length, not just "technically less");
  curvature-bound compliance; and a direct reproduction of docs/40's
  heading-9 scenario (3/3 synthetic targets solved from the exact trapped
  heading).
- **500-trial random fuzz** (fully random start/goal/radius, not curated
  cases): 0 failures, max position error 0.0, max heading error 0.0 (both
  below the tolerance's display precision) — the strongest evidence
  available that the closed-form + numeric mix is correct across the
  space, not just on hand-picked examples.
- **Cost, measured honestly:** ~4.1 ms/call (200-call average), against
  Dubins' near-instant closed form — the 4 numeric CCC solves (each up to
  5 seeded Newton attempts × 30 iterations) dominate. Fine for the
  occasional query this doc tests with; **step 2 needs to reckon with
  this directly** — enumerating an entire primitive library the way
  `make_ackermann_primitive_library` does (~3600 candidate calls at
  reach=7) would cost on the order of 15+ seconds if every candidate goes
  through Reeds–Shepp, a one-time startup cost, not a per-tick one, but
  real enough to design around rather than ignore.

## Interview angle

**Q: Why bother implementing Dubins at all if Reeds–Shepp is strictly more capable?**
A: Reeds–Shepp answers "what's the shortest path if reverse is allowed,"
which is a *different modeling assumption* than what many real systems
want. A car that should never reverse near pedestrians, or where reverse
maneuvers are expensive/risky for other reasons, correctly wants the
forward-only Dubins answer even though it's sometimes longer. They're not
strictly ordered by usefulness — Dubins encodes a real constraint
Reeds–Shepp doesn't.

**Q: What's the actual geometric reason a "K-turn" primitive can solve cases Dubins can't?**
A: Forward-only motion means every arc segment's *heading change per unit
distance* is bounded by curvature in one consistent rotational sense per
segment — composing several still only ever traces out headings reachable
by a chain of same-direction-of-travel arcs. A cusp breaks that: reversing
direction lets the SAME steering geometry produce a heading change with
the *opposite* net rotational effect for a given position displacement,
which is exactly what's needed to reorient in a space too small for a
forward-only loop of the required radius (a real car parallel-parks this
way — it can't turn its wheel radius tight enough to enter a tight spot
forward, but back-and-forth with reversals gets there in the same footprint).

**Q: Why verify every candidate by resimulating it instead of just trusting the derivation?**
A: Because trusting the derivation is exactly the risk being managed —
this is intricate closed-form trigonometry with several sign conventions
in play (curvature sign, direction-of-travel sign, which frame an angle is
measured in), and a subtle bug there wouldn't look like a crash, it would
look like a slightly-wrong path that might clip an obstacle. Forward
kinematics (`_arc_end`) is comparatively trivial to get right and easy to
eyeball-verify, so making *that* the final gate — not the formula — moves
the correctness burden onto the simpler, more obviously-correct code.

**Q: The docs/40 fix needed reverse capability, but why is that a planning-only story so far?**
A: Because it isn't finished — a path with a cusp in it is meaningless to
a controller that has no notion of "this stretch is driven in reverse."
`pure_pursuit_ackermann_node.py` today is a purely geometric tracker with
no way to know when to flip `v`'s sign. That's explicitly deferred to step
3 (docs/39's scope note), not an oversight: solving the planning side
first, and proving it's correct in isolation via the tests here, is the
right order — there's no point designing the controller-side interface
before knowing what shape of information (cusp locations? per-point
direction flags?) it actually needs to carry.
