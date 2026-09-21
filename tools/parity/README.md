# Phase 4B-2 Parity Sidecar Toolchain (`tools/parity/`)

Read-only observability tooling for comparing the shadow central backend against
the legacy SuperOdom LIO/IMU path.

```text
parity trace = observation artifact
             != production state
             != optimizer input
             -> cannot and must not change callback / factor / graph behaviour
```

Python 3 stdlib only (`json`, `argparse`, `dataclasses`, `pathlib`, `unittest`).
No pandas, no scipy, no rosbag reader, no plotting. KB/MB-sized synthetic
fixtures, CPU only, well below 1 GB RAM.

---

## 1. Files

| File | Role |
|---|---|
| `schema.json` | Single source of truth: required fields, type vocabulary, layer order, payload field aliases, reject reasons, tolerance defaults |
| `parity_schema.py` | Shared library used by all three tools (schema loading, payload canonicalisation, tolerance classification, leaf comparison) |
| `validate_trace.py` | JSONL / field / vocabulary / lifecycle validator |
| `diff_trace.py` | Parity differ: first divergent interval, layer, field + summary counts |
| `summarize_trace.py` | Compact per-trace summary (counts, ranges, reasons, gaps) |
| `fixtures/` | Synthetic equal/divergent trace pairs + `make_fixtures.py` generator |
| `tests/` | `unittest` suite for all three tools |

Run the tests from the repository root:

```bash
python3 -m unittest discover tools/parity/tests -v
```

---

## 2. Record model

One JSONL line = one immutable observation. Minimum fields
(`schema.json: record_required_fields`):

```json
{
  "schema_version": 1,
  "producer": "shadow|legacy",
  "type": "timeline|input|factor|gate|state|trajectory",
  "timestamp_ns": 0,
  "k": 0,
  "source": "imu|lio|central",
  "epoch": 0,
  "payload": {}
}
```

`type` accepts **both** spellings:

* the layer-level names from the taskbook (`timeline`, `input`, `factor`, `gate`,
  `state`, `trajectory`), and
* the event names the current tracer emits (`TIMELINE_ANCHOR_OPEN`,
  `TIMELINE_ANCHOR_CLOSE`, `INPUT_CONSUME`, `FACTOR_INSERT`, `GATE_EVALUATION`,
  `STATE_SNAPSHOT`, `TRAJECTORY_POSE`, `FUSION_SEGMENT_RESET`).

`source` is matched case-insensitively, so both `LIO`/`IMU`/`SYSTEM` (current
tracer) and `lio`/`imu`/`central` (taskbook) are accepted.

Forbidden in a trace: full ROS message dumps, point cloud payloads, unselected
large covariance matrices.

### SE(3) representation (locked)

```text
quaternion order      = x, y, z, w
translation unit      = meter
rotation unit         = radian (gate norms, innovation, noise sigmas)
7-vector pose         = [tx, ty, tz, qx, qy, qz, qw]
GTSAM tangent 6-vector= [rotation(3), translation(3)]   (R4.2 signed order)
```

`makePoseNoise()` in `shadow_timeline.cpp` writes rotation first; the differ uses
that same order when it picks the rotation/translation tolerance of each
`innovation6` / `noise_sigmas` entry. Any producer using another internal
representation must convert at serialization time.

### Noise representation

```json
{"sigma_rot_xyz": [0, 0, 0], "sigma_trans_xyz": [0, 0, 0]}
```

or the flat 6-vector form `noise_sigmas` / `noise_diag`, which the toolchain
interprets as GTSAM order `[rotation(3), translation(3)]`. A bare `sigma6`
without a stated order is only accepted as an alias of the same locked order.

### Rejected intervals stay in the trace

The tools never hide rejection events. `NO_BRACKET`, `CROSS_EPOCH`, `TOO_LATE`,
`GATE_REJECT`, `SLOT_OCCUPIED`, `DUPLICATE_IDENTITY`, `IMU_GAP` (plus the
`REJECT_*` / `INNOVATION_TOO_LARGE_*` / `late_after_watermark` / `stale_skipped`
strings the current tracer uses) are part of the controlled vocabulary in
`schema.json: reject_reasons`, validated by `validate_trace.py` and reported by
`summarize_trace.py`. Only-accepted-factor traces are considered incomplete.

---

## 3. `validate_trace.py`

```bash
python3 tools/parity/validate_trace.py shadow_parity_trace.jsonl
python3 tools/parity/validate_trace.py --producer shadow --format json trace.jsonl
python3 tools/parity/validate_trace.py --strict trace.jsonl
```

Checks: valid JSONL, required fields, `schema_version`, `timestamp_ns` integer,
`k` integer/null and non-negative, producer vocabulary, `type` vocabulary,
known reject reasons, `decision ∈ {ACCEPTED, REJECTED}`, factor
`committed`/`finalized` consistency and vector shapes (keys 2, measurement 7,
sigmas 6), monotonic anchor-status transitions, sticky terminal status
(`SOLVED`/`INVALID_GAP`), one-shot gate slots, factor finalized-before-committed.

Deliberately **not** an error: source event timestamps arriving out of order.
Callback arrival order is not event order; out-of-order events are counted as
information only (`info.out_of_order_timestamps`).

Leniency policy: `producer` and `schema_version` are *warnings* when missing
(the current tracer does not emit them yet) and become hard errors with
`--strict`. `--producer` supplies the missing field. `--allow-unknown-reasons`
downgrades unknown reject reasons to warnings. `--allow-schema-version N`
extends the accepted versions (default `1`).

Exit codes: `0` valid, `1` invalid, `2` usage / unreadable input.

---

## 4. `diff_trace.py`

```bash
python3 tools/parity/diff_trace.py --shadow shadow.jsonl --legacy legacy.jsonl
python3 tools/parity/diff_trace.py --shadow s.jsonl --legacy l.jsonl \
    --abs-tol 1e-5 --type gate --k-min 10 --ignore-fields watermark_ns --format json
```

Reports the **first divergent interval `k`**, the **first divergent layer**, the
**first divergent field**, the shadow and legacy records, the field-level diff and
summary counts per layer.

Comparison order (fixed): `timeline` → `input` → `factor` → `gate` → `state` →
`trajectory`. Layer priority decides "first": a timeline divergence at `k=2` is
reported before a gate divergence at `k=1`. A layer without a legacy counterpart
record is itself a divergence (`kind=record_missing`, `reason=missing-record`).

Options: `--ignore-fields` (names/globs, repeatable, comma separated), `--abs-tol`,
`--rel-tol`, `--trans-tol`, `--rot-tol`, `--vel-tol`, `--bias-tol`, `--type`
(layer or event type, repeatable), `--k-min`, `--k-max`,
`--allow-extra-fields` (compare only fields present on both sides), `--collapse
{none,first,last}`, `--all-divergences`, `--max-divergences`, `--format
{text,json}`, `--output FILE`.

Matching: records are paired by `(layer, k, source, epoch, occurrence)` after
canonicalising `source` case and payload field names, so a trace that says
`measurement_T_Bi_Bj` compares directly against one that says `measurement_se3`.
Records with `k = null` (input/trajectory observations) are paired by file order
within their group. Repeated observations of the same `(layer,k,source,epoch)`
(for example the state snapshot re-emitted on every `flushOptimizer()`) are
compared occurrence by occurrence; `--collapse last` keeps only the last one per
group when a producer emits a different number of repeats.

Exit codes: `0` no divergence, `1` divergence found, `2` usage / unreadable input.

### No automatic prettification

The differ performs no time-shift optimisation, no trajectory SE(3) alignment, no
scale correction and no nearest-neighbour rematching. A pure timestamp shift is
reported as a divergence. Those operations belong to the trajectory evaluation
layer, executed explicitly, and never inside the first five parity layers.

---

## 5. Tolerances

Defaults (`schema.json: tolerances_default`, not scattered in code):

| Category | Default | Applies to |
|---|---|---|
| `trans` | `1e-6 m` | translation parts of poses, `trans_norm`, translation part of `innovation6` |
| `rot` | `1e-6 rad` | quaternion parts, `rot_norm`, rotation part of `innovation6` / `noise_sigmas` |
| `vel` | `1e-6 m/s` | `state.velocity` / `v_W` |
| `bias` | `1e-8` | `state.bias_acc` / `bias_gyro` |
| `exact` | no tolerance | `*_ns` timestamps, `t_k`, statuses, reasons, decisions, keys |

Every category is overridable from the CLI (`--abs-tol` for trans/rot/vel,
`--rel-tol` as a relative floor, plus the per-category flags).

---

## 6. `summarize_trace.py`

```bash
python3 tools/parity/summarize_trace.py shadow_parity_trace.jsonl
python3 tools/parity/summarize_trace.py --format json --gap-threshold-ns 100000000 t.jsonl
```

Prints record counts by type/layer, anchor range (`k` and stamps), epochs, drop
reasons, input actions, gate accept/reject counts and reason histogram, factor
counts by type with committed/finalized counts, first/last timestamp and span,
out-of-order count, and gap counts (`invalid_gap_anchors`, `coverage_gap_events`,
`imu_gap_drops`, plus observation-timestamp gaps above `--gap-threshold-ns`).
No plots, no pandas.

Exit codes: `0` summarized, `2` usage / unreadable input.

---

## 7. Fixtures

| Pair | Expected result |
|---|---|
| `equal_shadow.jsonl` / `equal_legacy.jsonl` | no divergence |
| `factor_divergence_*` | first divergence in layer `factor`, field `payload.measurement_se3[0]` |
| `gate_divergence_*` | first divergence in layer `gate`, field `payload.trans_norm` |
| `state_divergence_*` | first divergence in layer `state`, field `payload.velocity[0]` |
| `timeline_divergence_*` | gate diverges at `k=1`, timeline at `k=2` → chronological interval order reports gate at `k=1` (timeline divergence at `k=2` retained in per-layer) |
| `layer_names_*` | taskbook layer names + canonical spellings vs emitted names → parity |

Regenerate deterministically with `python3 tools/parity/fixtures/make_fixtures.py`.

---

## 8. Integration notes for the main agent

1. Emit `producer` (`"shadow"` / `"legacy"`) and `schema_version: 1` in every
   line — the current `ParityTracer::write()` omits both, so `--strict` fails and
   the producer has to be supplied via `--producer`.
2. Keep `timestamp_ns` an integer in ns (`int64_t`), never a float.
3. Keep the immutable interval reference `dT_imu_ref(k,k+1)` available to the
   tracer: `GATE_EVALUATION` currently writes `dT_imu_ref: 0` and
   `innovation_6d: [0,...]`, so a gate-level root cause cannot be diffed yet.
4. `interpolation_brackets` is emitted as `[0,0]` in the current tracer; the real
   bracket pair is needed to diff `NO_BRACKET` / `CROSS_EPOCH` causes.
5. Emit `STATE_SNAPSHOT` / `TRAJECTORY_POSE` once per solve step (or use
   `--collapse last`); the current tracer re-emits every solved anchor on every
   flush, which is valid but verbose.
6. A legacy producer only needs the same JSONL contract — these tools never read
   ROS messages, never require a bag, and never build the workspace.

---

## 9. Verified against a real shadow trace

```bash
python3 tools/parity/validate_trace.py --producer shadow test_shadow_trace.jsonl
# -> RESULT: VALID (43 records, 0 errors, 43 schema_version warnings)
python3 tools/parity/summarize_trace.py test_shadow_trace.jsonl
# -> timeline=12 input=1 factor=1 gate=2 state=27, anchors k=[0,6], epochs [0,1],
#    gate accepted=1 rejected=1, factors committed=1, 1 timestamp gap of 0.4 s
python3 tools/parity/diff_trace.py --shadow test_shadow_trace.jsonl \
                                   --legacy <same content, producer=legacy>
# -> RESULT: PARITY  (no false divergence on the real emitted format)
```
