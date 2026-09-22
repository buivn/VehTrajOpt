# 40 — Benchmark tour validation, and proving a real heading-trap in the forward-only lattice

Triggered by watching the Ackermann pipeline drive the maze in Gazebo
(docs/39's smoke test, visualized): repeated wall collisions during turns.
This doc covers the investigation in order: what turned out to be wrong
with the benchmark's own tour-building, the fix, and — more importantly —
a real planning-completeness bug in `lattice_ackermann.py` this
investigation surfaced along the way, which is bigger than a benchmark
issue.

## 1. General idea / theory

`vto_bench`'s `benchmark_node.py` drives a controller through a
deterministic, seeded **goal tour**: chain `num_legs` random goals,
accepting each only if it's reachable from wherever the previous leg ends
and its path length falls in `[leg_min_dist, leg_max_dist]`. The
reachability check has always been **A\*** — a plain grid search that
only knows about occupied/free cells, nothing about heading or turning
radius. That's a fine proxy for the diff-drive robot: it can pivot in
place for free, so if a cell is grid-reachable, the robot can always
orient itself to actually drive there. It is **not** a fine proxy for a
forward-only Ackermann car, which has no such escape hatch — A\* will
happily accept a goal that's only reachable by reversing out of a
dead-end, because A\* has no concept of "reversing" or "dead-end" at all.

## 2. Method selection

Three ways to make the tour builder respect this robot's real kinematics,
weighed in `docs/39`'s follow-on conversation:

- **Heuristic post-filter on A\*'s path** (e.g. reject if A\* path length
  ≫ straight-line distance, as a proxy for "had to double back"). Rejected:
  a proxy for a proxy — can both miss real infeasible legs (a short
  detour into a dead-end could still look length-reasonable) and reject
  legs that are fine but just winding.
- **Manually curate the goal list** after eyeballing one run. Rejected:
  doesn't generalize — a new seed or a regenerated maze needs the same
  manual pass again.
- **Re-validate with the real Ackermann lattice search (chosen).** Run
  the actual forward-only Reeds-Shepp... — at this point in the build,
  Dubins-only — lattice search (`lattice()` + `ackermann_primitive_table()`)
  as a *second* gate, after the cheap A\* check. A candidate is only
  accepted if the vehicle's own planner agrees it's reachable, chaining
  the assumed start heading leg-to-leg using the search's own final
  heading (the same information the real robot would approximately carry
  into the next leg). This fixes the problem at the source — such a goal
  is never chosen — rather than filtering symptoms after the fact, and
  it's exactly what `docs/39 §2a` had already flagged as a known gap
  ("A\* reachable ≠ Ackermann-lattice reachable").

## 3. Coding — design

**`benchmark_node.py`** (shared with the diffbot benchmark — touched
carefully): new `goal_validator` parameter (`"astar"` default, unchanged
behavior for diffbot; `"lattice_ackermann"` for this robot), plus
`turning_radius_m` and `spawn_yaw` parameters so the validator can build
the same physically-derived primitive table the live planner node uses.
`_build_tour()` now always runs the cheap A\* check first (unconditionally,
even in `lattice_ackermann` mode) — a candidate that fails A\* fails
*strictly looser* reachability too, so it can never pass the harder check,
and gating the expensive search behind the cheap one avoids paying for it
on the large majority of attempts that fail for an obvious reason
(distance band, real disconnection).

**Performance had to be engineered, not assumed.** A naive
"lattice-validate every random attempt" design was tried first and hung —
a genuinely infeasible candidate makes `lattice()` exhaust its *entire*
reachable state space before returning `None`, measured at multiple
minutes per call on this map. Fixed two ways: (a) the A\*-first gating
above, and (b) a new `max_expansions` parameter added to `lattice()`
itself (`lattice.py`) — capping how many states get popped before giving
up early, same return (`None`) as genuine infeasibility. Calibrated
empirically, not guessed: a known-*solvable* ~20 m leg needed between
12,000 and 20,000 expansions (the plain octile-in-cell-space heuristic,
which ignores heading, is fairly weak for a 16-heading state space, so
even successful searches explore a lot) — the cap is set to 25,000 for
headroom. `max_expansions=None` (unlimited) is the default, so this adds
zero behavior change for the live planner node or any other existing
caller; it's opt-in.

**`bench_ackermann.launch.py`**: sets `goal_validator:="lattice_ackermann"`
and `spawn_yaw` to match this robot's real spawn heading. Also changed
`repeats`' default to 1 (was 2, matching diffbot): a repeat round's first
leg goes from wherever the tour ended back to its own first goal — an
edge `_build_tour()` never validates against *any* check, goal_validator
included, since it isn't part of the goal chain the builder constructs at
all. That's a second, separate gap from the one this doc fixes; not
addressed here.

## 4. The stall that turned into a real finding

Even with both performance fixes, a 30-leg tour build stalled completely
at **leg 8**: over 8 minutes and ~1,300 different candidate goals, *every
single one* rejected as lattice-infeasible from wherever leg 7's search
said the robot would end up (heading index 9, a room with 8 cells of
clearance — not a tiny pocket).

Two hypotheses, tested in order:

1. **"The 25,000-expansion cap is too tight for a large reorientation."**
   Disproven directly: an *unlimited*-expansion query from heading 9 to
   the same candidates returned `None` in **0.025 seconds** — far too fast
   to be exhausting a large state space, which ruled out a budget
   artifact immediately.
2. **"Zero primitives are viable at all from heading 9."** Also disproven
   — directly inspecting all 12 primitives at that heading showed every
   one individually clear (`end_free=True, path_clear=True`). The car
   *can* move from there.

The actual mechanism, found by instrumenting the search loop directly
(logging each expansion's state and how many new neighbors it pushed):
the search expands completely normally — 7-12 new states per expansion
early on — but its entire reachable set from heading 9 saturates at
**exactly 877 states** before the open heap empties. That's a real,
cleanly *proven* local infeasibility: from that heading, every
composition of ≤90°-per-primitive forward-only turns eventually revisits
already-explored states without ever making net progress toward the
room's exits, however far the search is allowed to run. Testing all 16
headings from the identical position confirmed the shape of the trap: a
contiguous 7-heading band (indices 5-13, a 180° arc centered on 9) found
**zero** feasible routes to any of 15 sampled candidates each, while the
other 9 headings found several each.

## Why this matters beyond the benchmark

This isn't a tour-building artifact — it's a property of
`lattice_ackermann.py`'s *forward-only* primitive library (Dubins-based at
the time of this investigation) interacting with real corridor geometry.
The live planner (`lattice_ackermann_planner_node.py`) has the exact same
blind spot: if `pure_pursuit_ackermann` ever leaves the real robot at a
heading like this — and heading 9 was reached *organically*, as leg 7's
own planned end-heading, nothing exotic — the live planner would return
"no path found" for *any* requested goal from there, and the robot would
simply be stuck. This is the finding that motivated docs/41: adding
Reeds-Shepp reverse motion to the control set, since a short reversal
(the maneuver a forward-only car structurally cannot perform) is exactly
what escapes a trap like this.

## Interview angle

**Q: How do you tell a "slow but correct" search from a "wrong/broken" one when both return the same answer?**
A: Instrument the loop, don't just trust the return value. Timing alone
was actively misleading here — 0.025 seconds for a claimed "no path"
result looked suspicious *because* a genuinely large exhaustive search had
already been observed taking minutes elsewhere in the same investigation.
Logging expansions-vs-heap-size at fixed checkpoints turned "why is this
returning None" into a directly observable fact (877 states, then
nothing) rather than a guess.

**Q: Why fix the tour-builder's reachability check at the source (re-validate with the real planner) instead of filtering bad results after generation?**
A: Because a proxy reachability check (A\*) and the real constraint
(forward-only, bounded curvature) aren't just "sometimes different by a
little" — they can disagree completely, as heading 9 shows: A\* says a
candidate 15+ meters away is trivially reachable, while the real
kinematics say *nothing* is reachable from that heading no matter how far
the search runs. A filter applied after generation is only as good as the
signal you filter on; validating with the actual constraint at
generation time can't miss cases the filter's proxy metric wasn't
designed to catch.

**Q: The tour builder now runs two searches (A\* then lattice) per candidate — doesn't that make it strictly slower than either alone?**
A: Slower per rejected-by-A\* candidate would be true if the lattice
search ran unconditionally, but it's gated behind A\* passing first —
since lattice-feasible implies A\*-feasible (forward-only-with-heading
reachability is a strict subset of grid reachability), any candidate the
expensive check *would* reject either also fails the cheap check (skip
early, no cost added) or passes both (one lattice call, unavoidable if
you want the stronger guarantee at all). The two-gate order only adds
cost on candidates that were going to need the expensive check regardless.
