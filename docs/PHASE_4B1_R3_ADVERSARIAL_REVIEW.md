# Phase 4B-1 R3 — Independent Adversarial Falsification Report

- **Author role:** adversarial validation agent (Agent B), independent of the R3 implementer.
- **Target revision:** `de6a3b0c63293442aca85ed3170f54e86f0e5104` (branch `audit/r3-falsification`, production sources untouched — test-only diff).
- **Contract sources:** `docs/FUSION_TIMELINE_DESIGN.md` (Phase 4A decisions §4–§7) + the frozen header comments of `shadow_timeline.hpp`. The R3 implementation prose was deliberately not used as evidence.
- **Deliverable:** `super_odometry_vio/test/test_fusion_adversarial.cpp` (24 cases, ~1500 lines, in an isolated detached-worktree copy of the repo; main checkout left untouched).
- **Final run:** build clean; **70 / 75 tests pass; 5 adversarial cases FAIL against `de6a3b0`** (all 51 pre-existing tests remain green; the 6th "failure" in the ctest summary is the aggregate row for the failing executable).

## Executive summary

Five real violations of the signed contract were falsified with deterministic,
self-contained reproductions. Two of them (F5, F9) are one-line root causes in
`shadow_timeline.cpp`; two (F7) are robustness/hygiene violations around IMU
gaps; one (F1) is a measurable arrival-order dependence of the posterior. No
production code was modified: all failures below are backed by the failing test
on the audit branch.

| ID | Area | Verdict | Root cause |
|----|------|---------|------------|
| F1 | Arrival-order independence | **VIOLATION (posterior)** | `tryInsertLioFactors` scan pointer skips never-attempted intervals → later-fed brackets dropped, posterior order-dependent |
| F5 | 150 ms interpolation boundary | **VIOLATION (boundary)** | `shadow_timeline.cpp:343` `gap_sec > 0.15` becomes exclusive through ns→double rounding |
| F7a | IMU gap then recovery | **VIOLATION (crash)** | bias variable `b1` missing from `Values` → `std::out_of_range` on later interval |
| F7b | IMU gap then recovery | **VIOLATION (silent invalid state)** | placeholder identity state exposed by `anchorState()` / propagated forward as `valid=1` |
| F9 | One-shot identity `(source, epoch, k)` | **VIOLATION (identity collapse)** | dedup keyed by interval only → new-epoch key rejected as `REJECT_DUPLICATE` |

F2, F3, F4, F6, F10, F11, F12: no violation found (see §Passing evidence).
F8/F12-locality were characterised, not falsified (see §Contract gaps).

## F1 — Arrival-order independence of the posterior (VIOLATION)

**Claim tested:** identical LIO sample *set* must yield identical lookups, gate
decisions and posteriors regardless of arrival order (design §6: insertion is
key-addressed, "与到达顺序无关").

**Result:** lookup results, gate decisions (`acc`, `rej_t`, `rej_r`) and
`totalGraphFactors` are order-independent **only when the LIO samples arrive
before the closing IMU sample of their interval**. The *posterior* diverges
otherwise, and the divergence depends on the order:

| arrival variant | posterior Δ vs reference (anchor k=5) |
|---|---|
| orders 0,1 + LIO-first (all orders) | 3.6e-5 · k m (single-attempt path) |
| orders 2–4 + IMU-first | 4.3e-2 m at k=5 — **order-dependent** |
| order 4 (fixed permutation) + IMU-first | 1.9e-2 m at k=5 — differs from orders 2–3 |

**Root cause:** `tryInsertLioFactors()` only scans intervals
`[next_lio_scan_k_, last_closed_k_]` and advances `next_lio_scan_k_` past
intervals that were skipped for lack of a bracket (`lio_no_bracket`). A sample
set arriving after the closing IMU sample therefore never gets a second
attempt: the interval stays permanently unconstrained *or* is constrained from
a later, differently-shaped sample subset, which changes the inserted factor
and hence the posterior.

**Severity:** medium. One-shot semantics partially mask it (one factor per
interval), but the *content* of the factor — and therefore the optimized
posterior — depends on arrival timing.

**Repro:** `F1_ArrivalOrder.InterpolationGateAndPosteriorIdenticalInAllOrders`
(10 permutations × {LIO-first, IMU-first}; 30+ DIVERGENT lines in the log).

## F5 — 150 ms interpolation boundary is exclusive (VIOLATION)

**Claim tested:** `max_interpolation_gap_sec = 0.15` is inclusive ("双侧包围
样本…间隔 ≤ 0.15 s"); a bracket pair exactly 150,000,000 ns apart must be usable.

**Result:** `[F5] exact-boundary 0.15 s span: … span>0.15 -> 1` — the exact
boundary is rejected. Root cause is `shadow_timeline.cpp:343`:

```cpp
const double gap_sec = static_cast<double>(s_after->stamp_ns - s_before->stamp_ns) * 1e-9;
if (gap_sec <= 0.0 || gap_sec > config_.max_interpolation_gap_sec) return false;
```

`150000000 * 1e-9` rounds to `0.15000000000000002` in double, so `>` is true.
**Fix hint:** compare in integer nanoseconds:
`span_ns > (int64_t)std::llround(max_interpolation_gap_sec * 1e9)`.

**Repro:** `F5_EdgeCases.BracketGapAndRotationBoundaries`.

## F7a — IMU gap poisons all later intervals (CRASH, VIOLATION)

**Claim tested:** after a gap leaves interval 0 unclosed, later *valid* IMU
regions must still close intervals normally (design: malformed interval is
dropped, stream continues).

**Result:** `std::out_of_range` from GTSAM:

```
[F7] after gap: intervals_closed=1 anchors=3 factors=5 threw=1
what = Attempting to at the key "b1", which does not exist in the Values.
```

Interval 1 closes and adds its ImuFactor, but the *bias variable* `b1` was
never inserted into `values_` on the gapped path, so the finalisation path
throws when interval 1's factor is inserted. One 50 ms gap permanently poisons
every subsequent interval: **the timeline is unrecoverable after a single
dropped sample window**.

**Repro:** `F7_ImuGaps.MultipleBadGapsThenValidRegion`.

## F7b — Placeholder anchor state silently reported as valid (VIOLATION)

**Claim tested:** a query selecting an anchor that was never finalised (its
interval failed) must not pretend to have a valid state.

**Result:**

```
[F7] reference (no gap) x(0.15 s) = 0.1125 (analytic 0.1125)   -- sanity
[F7] gapped: anchorState(1) ok=1 x=0 ; propagateTo(0.15) valid=1 x=0.0125
```

Anchor 1 keeps the identity placeholder pushed at creation time;
`anchorState(1)` returns `true` with pose = identity (true value: 0.1125 m),
and `propagateTo()` integrates **from the placeholder** and reports `valid=1`
with a wrong position (0.0125 vs 0.1125 m). Downstream consumers cannot
distinguish "finalised state" from "never-optimised placeholder".

**Repro:** `F7_ImuGaps.PlaceholderAnchorStateIsSilentlyReportedAsValid`.

## F9 — One-shot identity ignores the epoch dimension (VIOLATION)

**Claim tested:** dedup identity is `(source, epoch, k)` (design §19
`constraint_id = hash(source, source_epoch, key_i, key_j)`; frozen header:
*"Source constraint identifier: (source, epoch, k)"*). A key that was never
inserted must not be reported as `REJECT_DUPLICATE` merely because the same
*interval* is already constrained by a different epoch.

**Result:** `insertRelativeConstraint(ConstraintKey{0, 1, 0}, …)` after
`(0, 0, 0)` was accepted returns **REJECT_DUPLICATE**. The dedup/pinning is
keyed by interval only, so the epoch dimension of the identity is collapsed: a
source epoch change (e.g. VIO clearState → epoch++) can never re-constrain an
interval that epoch 0 already covered.

**Severity:** medium–high: it directly blocks the epoch-rollover recovery path
the design §11–§13 prescribes.

**Repro:** `F9_OneShot.OutOfOrderArrivalFillsHolesAndIsIdempotent` (also
verifies the positive half: out-of-order arrival *does* fill earlier holes, and
identical replays are idempotent-by-identity for same-epoch keys).

## Passing evidence (no violation found)

- **F2 (measurement-time grid):** anchors land on the exact `t0 + k·dt_a` grid
  (int64 ns), never at sample times; verified over multiple anchor rates × offsets.
- **F3 (monotonic epoch / stale isolation / no central reset):** stale epoch-0
  samples after the epoch bump are counted in `lio_stale_skipped`, produce no
  factors and no central reset; later valid epoch-1 intervals close and accept
  normally (acc=6, no_bracket=16, too_late=0, dup=0).
- **F4 (SE(3) interpolation order):** gate innovation matches the analytic
  interpolate-→-lever-arm-→-differencing subgroup pose to 1.2e-4 m trans /
  1e-6 rad — exactly the contract-mandated LERP+slerp chord-vs-arc floor, and
  35× away from raw-differencing (1.1e-2) and ~0.9 away from either
  lever-arm-miscomposition. Constant-velocity translation is exact to the
  bisection precision (0.11 vs 0.11).
- **F6 (mutable-position-then-static pattern):** mixed 10 Hz / 200 Hz stream
  closes every interval with exact factor accounting.
- **F10 (immutable pre-fusion reference):** a 0.9 m source factor accepted on
  (X_0, X_1) moves the posterior anchor-1 by >0.3 m while every `imuRef(k)`
  stays bit-identical (`equals 1e-15`) to the IMU-only run — the frozen
  reference is never recomputed from optimized states.
- **F11 (noise/tangent order):** `makePoseNoise` places rot sigmas at indices
  0–2 and trans at 3–5; a 1.0 m translation mismatch whitens by 2.0 (error
  0.125, not 5000); a 0.5 rad rotation mismatch whitens by 0.01; gate blocks
  route to `REJECT_INNOVATION_ROT` / `REJECT_INNOVATION_TRANS` correctly.
- **F12 (graph lifecycle / factor accounting):** 50 intervals → 51 anchors
  retained (no eviction), factor count = 3 + 3n + n exactly, one-time gauge
  priors never recur, and a source-epoch change does not reset or shrink the
  central graph.

## Contract gaps / characterisations (deterministic-input-policy gaps, not violations)

1. **Posterior locality (F12):** the signed contract is silent on whether a
   source factor on (X_5, X_6) may move non-adjacent intervals' relative
   measurements. Observed: it does, decaying away from k=5 (≤0.06 m / ≤0.07 rad
   for a 0.08 m factor over 10 intervals). Pinned with a generous 0.25 m sanity
   bound; recommended for the design's next revision.
2. **One-shot terminal rejection:** a gate-rejected interval is pinned;
   corrected samples for the same interval in the same epoch are never retried.
   The design does not define this case; pinned as observed.
3. **`max_imu_dt_sec` recovery semantics:** an internal gap > 0.05 s drops the
   malformed interval, but the design does not state whether the timeline may
   later recover (F7 shows the current answer is "it cannot").

## Reproduction

```bash
cd /home/peter/d_livo/super_odom_ws/audit_r3
./run_wt.sh wt6        # build + ctest in Docker; artifacts: wt6_*.log
# failing cases:
#   F1_ArrivalOrder.InterpolationGateAndPosteriorIdenticalInAllOrders
#   F5_EdgeCases.BracketGapAndRotationBoundaries
#   F7_ImuGaps.MultipleBadGapsThenValidRegion
#   F7_ImuGaps.PlaceholderAnchorStateIsSilentlyReportedAsValid
#   F9_OneShot.OutOfOrderArrivalFillsHolesAndIsIdempotent
```

Test-only diff on branch `audit/r3-falsification` (worktree `audit_r3/wt`):
`super_odometry_vio/test/test_fusion_adversarial.cpp` (new) +
`super_odometry_vio/CMakeLists.txt` (one `ament_add_gtest` line).


