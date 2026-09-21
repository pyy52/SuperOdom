#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compact summarizer for Phase 4B-2 parity traces.

Prints record counts by type/layer, anchor range, epochs, drop reasons, gate
accept/reject counts, factor counts, first/last timestamp, gap counts and
committed/finalized counts.

Deliberately no plotting and no pandas: stdlib only, so the tool stays usable in
the same low-memory, CPU-only, no-ROS-build setting as the rest of the sidecar.

Exit codes: 0 = summarized, 2 = usage / unreadable input.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))

from parity_schema import (  # noqa: E402  (path bootstrap must precede import)
    TraceRecord,
    canonicalize_payload,
    layer_order,
    read_jsonl,
)

GAP_DROP_REASONS = {"imu_gap", "coverage_gap", "gap_detected", "GAP_DETECTED"}
DEFAULT_GAP_THRESHOLD_NS = 200_000_000  # 0.2 s, heuristic observation-gap bound


def summarize(path: str, gap_threshold_ns: int = DEFAULT_GAP_THRESHOLD_NS) -> Dict[str, Any]:
    """Summarize one trace file into a plain dict."""
    records, parse_errors = read_jsonl(path)

    counts_by_type: Counter = Counter()
    counts_by_layer: Counter = Counter()
    anchor_status: Counter = Counter()
    decisions: Counter = Counter()
    gate_reasons: Counter = Counter()
    drop_reasons: Counter = Counter()
    input_actions: Counter = Counter()
    factor_types: Counter = Counter()
    epochs: set = set()
    anchor_ks: set = set()

    ts_min: Optional[int] = None
    ts_max: Optional[int] = None
    previous_ts: Optional[int] = None
    out_of_order = 0
    gap_count = 0
    largest_gap: Optional[int] = None
    anchor_stamp_min: Optional[int] = None
    anchor_stamp_max: Optional[int] = None

    committed = 0
    finalized = 0
    status_counts: Counter = Counter()
    coverage_gap_events = 0
    imu_gap_drops = 0
    invalid_gap_anchors: set = set()

    for rec in records:
        counts_by_type[rec.type or "<missing type>"] += 1
        if rec.layer is not None:
            counts_by_layer[rec.layer] += 1
        if isinstance(rec.epoch, int) and not isinstance(rec.epoch, bool):
            epochs.add(rec.epoch)

        ts = rec.raw.get("timestamp_ns")
        if isinstance(ts, int) and not isinstance(ts, bool):
            ts_min = ts if ts_min is None else min(ts_min, ts)
            ts_max = ts if ts_max is None else max(ts_max, ts)
            if previous_ts is not None:
                if ts < previous_ts:
                    out_of_order += 1
                delta = ts - previous_ts
                if 0 <= delta and delta > gap_threshold_ns:
                    gap_count += 1
                    if largest_gap is None or delta > largest_gap:
                        largest_gap = delta
            previous_ts = ts

        canonical, _, _ = canonicalize_payload(rec.layer, rec.payload)

        if isinstance(rec.k, int) and not isinstance(rec.k, bool):
            if rec.layer in ("timeline", "state", "gate", "factor"):
                anchor_ks.add(rec.k)

        status = canonical.get("anchor_status")
        if isinstance(status, str):
            anchor_status[status] += 1
            if status == "INVALID_GAP" and isinstance(rec.k, int):
                invalid_gap_anchors.add(rec.k)

        stamp = canonical.get("t_k")
        if isinstance(stamp, int) and not isinstance(stamp, bool):
            anchor_stamp_min = stamp if anchor_stamp_min is None else min(anchor_stamp_min, stamp)
            anchor_stamp_max = stamp if anchor_stamp_max is None else max(anchor_stamp_max, stamp)

        if rec.layer == "gate":
            decision = canonical.get("decision")
            if isinstance(decision, str):
                decisions[decision] += 1
            reason = canonical.get("reason")
            if isinstance(reason, str):
                gate_reasons[reason] += 1

        if rec.layer == "input":
            reason = canonical.get("drop_reason")
            if isinstance(reason, str):
                drop_reasons[reason] += 1
                if reason.lower() in {item.lower() for item in GAP_DROP_REASONS}:
                    imu_gap_drops += 1
            action = canonical.get("action")
            if isinstance(action, str):
                input_actions[action] += 1
            if canonical.get("coverage_gap") is True:
                coverage_gap_events += 1

        if rec.layer == "factor":
            factor_type = canonical.get("factor_type")
            factor_types[factor_type if isinstance(factor_type, str) else "<unknown>"] += 1
            if canonical.get("committed") is True:
                committed += 1
            if canonical.get("finalized") is True:
                finalized += 1
            status_value = canonical.get("status")
            if isinstance(status_value, str):
                status_counts[status_value] += 1

    gaps = {
        "invalid_gap_anchors": len(invalid_gap_anchors),
        "coverage_gap_events": coverage_gap_events,
        "imu_gap_drops": imu_gap_drops,
        "timestamp_gap_count": gap_count,
        "timestamp_gap_threshold_ns": gap_threshold_ns,
        "largest_timestamp_gap_ns": largest_gap,
    }
    gaps["gap_count"] = (len(invalid_gap_anchors) + coverage_gap_events
                         + imu_gap_drops or gap_count)

    return {
        "file": path,
        "malformed_lines": len(parse_errors),
        "records": len(records),
        "producer": next((rec.producer for rec in records if rec.producer), None),
        "schema_version": next((rec.raw.get("schema_version") for rec in records
                                if isinstance(rec.raw.get("schema_version"), int)), None),
        "counts_by_type": dict(sorted(counts_by_type.items())),
        "counts_by_layer": {layer: counts_by_layer.get(layer, 0) for layer in layer_order()},
        "first_timestamp_ns": ts_min,
        "last_timestamp_ns": ts_max,
        "span_ns": (ts_max - ts_min) if (ts_min is not None and ts_max is not None)
                   else None,
        "out_of_order_timestamps": out_of_order,
        "anchor_range": {
            "k_min": min(anchor_ks) if anchor_ks else None,
            "k_max": max(anchor_ks) if anchor_ks else None,
            "k_count": len(anchor_ks),
            "stamp_min_ns": anchor_stamp_min,
            "stamp_max_ns": anchor_stamp_max,
        },
        "anchor_status_counts": dict(sorted(anchor_status.items())),
        "epochs": sorted(epochs),
        "drop_reason_counts": dict(sorted(drop_reasons.items())),
        "input_action_counts": dict(sorted(input_actions.items())),
        "gate": {
            "evaluations": sum(decisions.values()),
            "accepted": decisions.get("ACCEPTED", 0),
            "rejected": decisions.get("REJECTED", 0),
            "decision_counts": dict(sorted(decisions.items())),
            "reason_counts": dict(sorted(gate_reasons.items())),
        },
        "factors": {
            "total": counts_by_layer.get("factor", 0),
            "by_type": dict(sorted(factor_types.items())),
            "committed": committed,
            "finalized": finalized,
            "status_counts": dict(sorted(status_counts.items())),
        },
        "gaps": gaps,
        "state_snapshots": counts_by_layer.get("state", 0),
        "trajectory_poses": counts_by_layer.get("trajectory", 0),
    }


def _counter_line(name: str, counts: Dict[str, Any]) -> str:
    if not counts:
        return f"{name}: (none)"
    return name + ": " + ", ".join(f"{key}={value}" for key, value in counts.items())


def print_text_summary(summary: Dict[str, Any]) -> None:
    print(f"== summarize_trace: {summary['file']}")
    print(f"records={summary['records']} malformed_lines={summary['malformed_lines']} "
          f"producer={summary['producer']} schema_version={summary['schema_version']}")
    print(_counter_line("counts by layer", summary["counts_by_layer"]))
    print(_counter_line("counts by type", summary["counts_by_type"]))
    print(f"timestamps: first={summary['first_timestamp_ns']} "
          f"last={summary['last_timestamp_ns']} span_ns={summary['span_ns']} "
          f"out_of_order={summary['out_of_order_timestamps']}")
    anchor = summary["anchor_range"]
    print(f"anchor range: k=[{anchor['k_min']}, {anchor['k_max']}] "
          f"distinct_k={anchor['k_count']} "
          f"stamps=[{anchor['stamp_min_ns']}, {anchor['stamp_max_ns']}]")
    print(_counter_line("anchor status", summary["anchor_status_counts"]))
    print(f"epochs: {summary['epochs'] if summary['epochs'] else '(none)'}")
    print(_counter_line("drop reasons", summary["drop_reason_counts"]))
    print(_counter_line("input actions", summary["input_action_counts"]))
    gate = summary["gate"]
    print(f"gate: evaluations={gate['evaluations']} accepted={gate['accepted']} "
          f"rejected={gate['rejected']}")
    print(_counter_line("gate reasons", gate["reason_counts"]))
    factors = summary["factors"]
    print(f"factors: total={factors['total']} committed={factors['committed']} "
          f"finalized={factors['finalized']}")
    print(_counter_line("factor types", factors["by_type"]))
    print(_counter_line("factor status", factors["status_counts"]))
    gaps = summary["gaps"]
    print(f"gaps: count={gaps['gap_count']} "
          f"(invalid_gap_anchors={gaps['invalid_gap_anchors']} "
          f"coverage_gap_events={gaps['coverage_gap_events']} "
          f"imu_gap_drops={gaps['imu_gap_drops']}) "
          f"timestamp_gaps>{gaps['timestamp_gap_threshold_ns']}ns="
          f"{gaps['timestamp_gap_count']} largest={gaps['largest_timestamp_gap_ns']}")
    print(f"state snapshots={summary['state_snapshots']} "
          f"trajectory poses={summary['trajectory_poses']}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="summarize_trace.py",
        description="Summarize a Phase 4B-2 parity trace JSONL (stdlib only, no "
                    "plotting).")
    parser.add_argument("files", nargs="+", metavar="TRACE.jsonl",
                        help="trace file(s) to summarize")
    parser.add_argument("--gap-threshold-ns", type=int, default=DEFAULT_GAP_THRESHOLD_NS,
                        help="observation-timestamp gap threshold in ns used for the "
                             "gap count (default: 200000000 = 0.2 s)")
    parser.add_argument("--format", choices=("text", "json"), default="text",
                        help="report format (default: text)")
    parser.add_argument("--output", metavar="FILE",
                        help="also write the JSON summary to FILE")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    for path in args.files:
        if not Path(path).is_file():
            print(f"summarize_trace.py: cannot read '{path}'", file=sys.stderr)
            return 2

    summaries = [summarize(path, args.gap_threshold_ns) for path in args.files]
    if args.output:
        Path(args.output).write_text(json.dumps(summaries, indent=2) + "\n",
                                     encoding="utf-8")
    if args.format == "json":
        print(json.dumps(summaries, indent=2))
    else:
        for index, summary in enumerate(summaries):
            if index:
                print()
            print_text_summary(summary)
    return 0


if __name__ == "__main__":
    sys.exit(main())
