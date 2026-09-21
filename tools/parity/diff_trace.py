#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline parity differ for Phase 4B-2 traces.

Compares ``shadow_parity_trace.jsonl`` against ``legacy_parity_trace.jsonl`` and
reports the **first divergent interval / layer / field**, per layer, in the
prescribed order:

    1. timeline
    2. input consumption
    3. factor
    4. gate
    5. state
    6. trajectory

Behaviour that is deliberately *not* implemented (spec section 6): no time
shift optimisation, no SE(3) trajectory alignment, no scale correction, no
nearest-neighbour rematching. The first five layers keep their raw semantics;
the tools never "prettify" a trace into agreement.

Exit codes: 0 = no divergence, 1 = divergence found, 2 = usage / unreadable input.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field as dataclass_field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))

from parity_schema import (  # noqa: E402  (path bootstrap must precede import)
    TraceRecord,
    canonical_leaves,
    compare_leaves,
    default_tolerances,
    group_occurrences,
    is_number,
    key_sort_tuple,
    layer_of_type,
    layer_order,
    leaf_category,
    path_ignored,
    read_jsonl,
    record_match_key,
    tolerance_expectation,
)

KIND_VALUE = "value_mismatch"
KIND_FIELD = "field_missing"
KIND_RECORD = "record_missing"
KIND_TYPE = "type_mismatch"


@dataclass
class DiffOptions:
    """Everything that changes comparison semantics for one run."""

    ignore_fields: List[str] = dataclass_field(default_factory=list)
    allow_extra_fields: bool = False
    types: Optional[Set[str]] = None
    k_min: Optional[int] = None
    k_max: Optional[int] = None
    collapse: str = "none"
    tolerances: Dict[str, float] = dataclass_field(default_factory=default_tolerances)


def build_tolerances(args: argparse.Namespace) -> Dict[str, float]:
    """Resolve CLI tolerances on top of the schema defaults.

    Nothing is hard-coded in scattered places: the defaults come from
    ``schema.json`` (``tolerances_default``) and every CLI override lands in one
    dict that the comparison functions receive.
    """
    tolerances = default_tolerances()
    if args.abs_tol is not None:
        for name in ("trans", "rot", "vel"):
            tolerances[name] = args.abs_tol
    if args.rel_tol is not None:
        tolerances["rel"] = args.rel_tol
    for name, value in (("trans", args.trans_tol), ("rot", args.rot_tol),
                        ("vel", args.vel_tol), ("bias", args.bias_tol)):
        if value is not None:
            tolerances[name] = value
    return tolerances


def _type_filter(options: DiffOptions, rec: TraceRecord) -> bool:
    if not options.types:
        return True
    for token in options.types:
        if token in layer_order() and rec.layer == token:
            return True
        if token == rec.type:
            return True
    return False


def _k_filter(options: DiffOptions, rec: TraceRecord) -> bool:
    if not isinstance(rec.k, int) or isinstance(rec.k, bool):
        return True  # records without an interval index are always kept
    if options.k_min is not None and rec.k < options.k_min:
        return False
    if options.k_max is not None and rec.k > options.k_max:
        return False
    return True


def load_index(path: str, options: DiffOptions) -> Tuple[Dict[Tuple[Any, ...], TraceRecord],
                                                         Dict[str, int]]:
    """Load one trace into a ``key -> record`` index plus per-layer counts."""
    records, parse_errors = read_jsonl(path)
    if parse_errors:
        first = parse_errors[0]
        raise ValueError(f"{path}:{first['line']}: {first['error']} "
                         f"({len(parse_errors)} malformed line(s) total)")

    records = [rec for rec in records
               if _type_filter(options, rec) and _k_filter(options, rec)]

    if options.collapse in ("first", "last"):
        chosen: Dict[Tuple[Any, ...], TraceRecord] = {}
        for rec in records:
            group = (rec.layer, rec.k if isinstance(rec.k, int)
                     and not isinstance(rec.k, bool) else None, rec.source, rec.epoch)
            if options.collapse == "first" and group in chosen:
                continue
            chosen[group] = rec
        records = list(chosen.values())

    index = group_occurrences(records)
    counts: Dict[str, int] = {layer: 0 for layer in layer_order()}
    for key in index:
        counts[key[0]] = counts.get(key[0], 0) + 1
    return index, counts


def format_key(key: Tuple[Any, ...]) -> str:
    layer, k, source, epoch, occurrence = key
    return f"layer={layer} k={k} source={source} epoch={epoch} #{occurrence}"


def _base_divergence(kind: str, key: Tuple[Any, ...], shadow_rec: Optional[TraceRecord],
                     legacy_rec: Optional[TraceRecord]) -> Dict[str, Any]:
    layer, k, source, epoch, occurrence = key
    return {
        "kind": kind,
        "layer": layer,
        "k": k,
        "source": source,
        "epoch": epoch,
        "occurrence": occurrence,
        "key": format_key(key),
        "type_shadow": shadow_rec.type if shadow_rec else None,
        "type_legacy": legacy_rec.type if legacy_rec else None,
        "shadow_line": shadow_rec.line_no if shadow_rec else None,
        "legacy_line": legacy_rec.line_no if legacy_rec else None,
        "shadow_record": shadow_rec.raw if shadow_rec else None,
        "legacy_record": legacy_rec.raw if legacy_rec else None,
        "field": None,
        "shadow_field": None,
        "legacy_field": None,
        "category": None,
        "shadow": None,
        "legacy": None,
        "abs_diff": None,
        "rel_diff": None,
        "tolerance": None,
        "missing_side": None,
    }


def compare_pair(key: Tuple[Any, ...], shadow_rec: TraceRecord, legacy_rec: TraceRecord,
                 options: DiffOptions) -> Tuple[List[Dict[str, Any]], int]:
    """Field-level comparison of one matched observation pair.

    Returns ``(divergences, ignored_leaf_count)``. The pair is compared in raw
    semantics: nothing is aligned, shifted or rematched.
    """
    layer = key[0]
    divergences: List[Dict[str, Any]] = []
    ignored = 0
    tolerances = options.tolerances

    if not path_ignored("timestamp_ns", None, None, options.ignore_fields):
        left = shadow_rec.raw.get("timestamp_ns")
        right = legacy_rec.raw.get("timestamp_ns")
        result = compare_leaves("exact", left, right, tolerances)
        if not result["ok"]:
            item = _base_divergence(KIND_VALUE, key, shadow_rec, legacy_rec)
            item.update({
                "field": "timestamp_ns", "shadow_field": "timestamp_ns",
                "legacy_field": "timestamp_ns", "category": "exact",
                "shadow": left, "legacy": right,
                "abs_diff": result["abs_diff"], "rel_diff": result["rel_diff"],
                "tolerance": tolerance_expectation("exact", tolerances),
            })
            divergences.append(item)
    else:
        ignored += 1

    shadow_leaves, shadow_canon, shadow_raw = canonical_leaves(layer, shadow_rec.payload)
    legacy_leaves, legacy_canon, legacy_raw = canonical_leaves(layer, legacy_rec.payload)
    ordered_paths = list(shadow_leaves.keys()) + [
        path for path in sorted(legacy_leaves.keys()) if path not in shadow_leaves]

    for path in ordered_paths:
        canon = dict(shadow_canon)
        canon.update(legacy_canon)
        raw_names = dict(shadow_raw)
        raw_names.update(legacy_raw)
        if path_ignored(path, canon, raw_names, options.ignore_fields):
            ignored += 1
            continue
        in_shadow = path in shadow_leaves
        in_legacy = path in legacy_leaves
        if in_shadow and in_legacy:
            category = leaf_category(layer, path, canon)
            result = compare_leaves(category, shadow_leaves[path], legacy_leaves[path],
                                    tolerances)
            if result["ok"]:
                continue
            item = _base_divergence(
                KIND_TYPE if result["kind"] == "type_mismatch" else KIND_VALUE,
                key, shadow_rec, legacy_rec)
            item.update({
                "field": path,
                "shadow_field": shadow_raw.get(path),
                "legacy_field": legacy_raw.get(path),
                "category": category,
                "shadow": shadow_leaves[path],
                "legacy": legacy_leaves[path],
                "abs_diff": result["abs_diff"],
                "rel_diff": result["rel_diff"],
                "tolerance": tolerance_expectation(category, tolerances),
            })
            divergences.append(item)
            continue
        if options.allow_extra_fields:
            continue
        item = _base_divergence(KIND_FIELD, key, shadow_rec, legacy_rec)
        item.update({
            "field": path,
            "shadow_field": shadow_raw.get(path),
            "legacy_field": legacy_raw.get(path),
            "shadow": shadow_leaves.get(path),
            "legacy": legacy_leaves.get(path),
            "missing_side": "legacy" if in_shadow else "shadow",
        })
        divergences.append(item)

    return divergences, ignored


def diff_traces(shadow_path: str, legacy_path: str, options: DiffOptions,
                all_divergences: bool = False,
                max_divergences: int = 50) -> Dict[str, Any]:
    """Compare two traces and return the full diff report as a dict."""
    shadow_index, shadow_counts = load_index(shadow_path, options)
    legacy_index, legacy_counts = load_index(legacy_path, options)

    all_keys = set(shadow_index) | set(legacy_index)
    per_layer: List[Dict[str, Any]] = []
    first: Optional[Dict[str, Any]] = None
    collected: List[Dict[str, Any]] = []
    ignored_total = 0
    matched_total = 0
    divergent_total = 0
    missing_total = 0
    all_divergences_list: List[Dict[str, Any]] = []

    for layer in layer_order():
        entry: Dict[str, Any] = {
            "layer": layer,
            "matched": 0,
            "divergent": 0,
            "missing_in_shadow": 0,
            "missing_in_legacy": 0,
            "first_divergence": None,
        }
        keys = sorted([key for key in all_keys if key[0] == layer], key=key_sort_tuple)
        for key in keys:
            shadow_rec = shadow_index.get(key)
            legacy_rec = legacy_index.get(key)
            if shadow_rec is None or legacy_rec is None:
                if shadow_rec is None:
                    key = record_match_key(layer, legacy_rec.k, legacy_rec.source,
                                           legacy_rec.epoch, key[4])
                divergences = [build_missing_record(layer, key, shadow_rec, legacy_rec)]
                ignored = 0
                missing_total += 1
            else:
                entry["matched"] += 1
                matched_total += 1
                divergences, ignored = compare_pair(key, shadow_rec, legacy_rec, options)
            ignored_total += ignored
            if not divergences:
                continue
            entry["divergent"] += 1
            divergent_total += 1
            if shadow_rec is None:
                entry["missing_in_shadow"] += 1
            if legacy_rec is None:
                entry["missing_in_legacy"] += 1
            if entry["first_divergence"] is None:
                entry["first_divergence"] = divergences[0]
            all_divergences_list.extend(divergences)
            if all_divergences and len(collected) < max_divergences:
                collected.extend(divergences[:max_divergences - len(collected)])
        per_layer.append(entry)

    # Determine global first divergence:
    # 1. Chronological interval order first: earliest k (or timestamp if k is None).
    # 2. If multiple layers diverge at that interval, earliest causal layer in layer_order().
    first: Optional[Dict[str, Any]] = None
    first_chronological: Optional[Dict[str, Any]] = None
    first_causal_layer_at_interval: Optional[str] = None
    first_interval_divergences: List[Dict[str, Any]] = []

    if all_divergences_list:
        order = layer_order()

        def div_timestamp(d: Dict[str, Any]) -> int:
            t_s = d.get("shadow_record", {}).get("timestamp_ns") if d.get("shadow_record") else None
            t_l = d.get("legacy_record", {}).get("timestamp_ns") if d.get("legacy_record") else None
            valid_ts = [t for t in (t_s, t_l) if isinstance(t, int)]
            return min(valid_ts) if valid_ts else 0

        indexed_divs = list(enumerate(all_divergences_list))

        # Group by interval k (or timestamp if k is None)
        def interval_sort_key(d: Dict[str, Any]) -> Tuple[int, int, int]:
            k_val = d.get("k")
            ts = div_timestamp(d)
            if isinstance(k_val, int):
                return (0, k_val, ts)
            return (1, 0, ts)

        # Chronological order: by interval and timestamp
        sorted_by_chrono = [
            item[1] for item in sorted(
                indexed_divs,
                key=lambda item: (
                    interval_sort_key(item[1]),
                    item[0],
                )
            )
        ]
        first_chronological = sorted_by_chrono[0]

        # Preserve discovery order of fields for same key/layer
        sorted_by_interval = [
            item[1] for item in sorted(
                indexed_divs,
                key=lambda item: (
                    interval_sort_key(item[1]),
                    order.index(item[1].get("layer", "")) if item[1].get("layer") in order else 99,
                    item[0],
                )
            )
        ]
        first = sorted_by_interval[0]
        earliest_k = first.get("k")
        first_causal_layer_at_interval = first.get("layer")
        first_interval_divergences = [
            d for d in all_divergences_list
            if d.get("k") == earliest_k
        ]

    report: Dict[str, Any] = {
        "tool": "diff_trace.py",
        "shadow": shadow_path,
        "legacy": legacy_path,
        "tolerances": dict(options.tolerances),
        "ignore_fields": list(options.ignore_fields),
        "allow_extra_fields": options.allow_extra_fields,
        "collapse": options.collapse,
        "type_filter": sorted(options.types) if options.types else None,
        "k_range": [options.k_min, options.k_max],
        "layers_compared": layer_order(),
        "record_counts": {"shadow": shadow_counts, "legacy": legacy_counts},
        "matched_keys": matched_total,
        "divergent_keys": divergent_total,
        "missing_keys": missing_total,
        "ignored_leaf_count": ignored_total,
        "divergence": first is not None,
        "first_divergent_interval_k": None if first is None else first["k"],
        "first_divergent_layer": None if first is None else first["layer"],
        "first_divergent_field": None if first is None else first["field"],
        "first_divergence": first,
        "first_chronological_divergence": first_chronological,
        "first_causal_layer_at_interval": first_causal_layer_at_interval,
        "first_interval_divergences": first_interval_divergences,
        "per_layer": per_layer,
    }
    if all_divergences:
        report["divergences"] = collected
    return report


def build_missing_record(layer: str, key: Tuple[Any, ...],
                         shadow_rec: Optional[TraceRecord],
                         legacy_rec: Optional[TraceRecord]) -> Dict[str, Any]:
    """A layer that has no counterpart record is itself a divergence."""
    item = _base_divergence(KIND_RECORD, key, shadow_rec, legacy_rec)
    item["layer"] = layer
    item["reason"] = "missing-record"
    item["missing_side"] = "shadow" if shadow_rec is None else "legacy"
    return item


def _format_number(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.12g}"
    return repr(value)


def print_text_report(report: Dict[str, Any]) -> None:
    tolerances = report["tolerances"]
    print(f"== parity diff: shadow={report['shadow']} legacy={report['legacy']}")
    print("tolerances: " + " ".join(
        f"{name}={tolerances[name]:g}" for name in ("trans", "rot", "vel", "bias", "rel")))
    print("layers compared: " + ",".join(report["layers_compared"])
          + (f"  types={','.join(report['type_filter'])}" if report["type_filter"] else ""))
    print("ignore-fields: " + (",".join(report["ignore_fields"]) or "(none)")
          + ("  allow-extra-fields=on" if report["allow_extra_fields"] else ""))
    shadow_counts = report["record_counts"]["shadow"]
    legacy_counts = report["record_counts"]["legacy"]
    print("records: shadow=" + ",".join(
        f"{layer}:{shadow_counts.get(layer, 0)}" for layer in report["layers_compared"]))
    print("         legacy=" + ",".join(
        f"{layer}:{legacy_counts.get(layer, 0)}" for layer in report["layers_compared"]))
    print(f"keys: matched={report['matched_keys']} divergent={report['divergent_keys']} "
          f"missing={report['missing_keys']} ignored_leaves={report['ignored_leaf_count']}")
    print()

    first = report["first_divergence"]
    if first is None:
        print("FIRST DIVERGENCE: none (all compared layers agree within tolerance)")
    else:
        print("FIRST DIVERGENCE")
        print(f"  layer      : {first['layer']}")
        print(f"  interval k : {first['k']}")
        print(f"  key        : {first['key']}")
        print(f"  kind       : {first['kind']}"
              + (f" (reason: {first['reason']})" if first.get("reason") else ""))
        print(f"  record     : shadow={first['type_shadow']} (line {first['shadow_line']}) "
              f"vs legacy={first['type_legacy']} (line {first['legacy_line']})")
        if first["field"] is not None:
            print(f"  field      : {first['field']}")
            if first["shadow_field"] and first["legacy_field"] and \
                    first["shadow_field"] != first["legacy_field"]:
                print(f"  field names: shadow='{first['shadow_field']}' "
                      f"legacy='{first['legacy_field']}'")
            print(f"  shadow     : {_format_number(first['shadow'])}")
            print(f"  legacy     : {_format_number(first['legacy'])}")
            if first["abs_diff"] is not None:
                print(f"  abs diff   : {first['abs_diff']:.6g} ({first['tolerance']})")
            if first["rel_diff"] is not None:
                print(f"  rel diff   : {first['rel_diff']:.6g}")
        if first.get("missing_side"):
            print(f"  missing on : {first['missing_side']}")
        print("  shadow rec : " + json.dumps(first["shadow_record"]))
        print("  legacy rec : " + json.dumps(first["legacy_record"]))
        if report.get("first_chronological_divergence") and report["first_chronological_divergence"] != first:
            fc = report["first_chronological_divergence"]
            print(f"  first chronological: layer={fc['layer']} k={fc['k']} field={fc['field']}")
        other_divs = [d for d in report.get("first_interval_divergences", []) if d != first]
        if other_divs:
            other_layers = sorted(set(d["layer"] for d in other_divs))
            print(f"  other divergences at k={first['k']}: layers={','.join(other_layers)} (count={len(other_divs)})")
    print()
    print("per-layer summary (comparison order = "
          "timeline,input,factor,gate,state,trajectory)")
    for entry in report["per_layer"]:
        marker = "  <-- first divergent layer" \
            if report["first_divergent_layer"] == entry["layer"] else ""
        first_field = entry["first_divergence"]["field"] if entry["first_divergence"] else "-"
        print(f"  {entry['layer']:<11} matched={entry['matched']:<4} "
              f"divergent={entry['divergent']:<4} "
              f"missing_in_shadow={entry['missing_in_shadow']:<3} "
              f"missing_in_legacy={entry['missing_in_legacy']:<3} "
              f"first_field={first_field}{marker}")
    print()
    print(f"RESULT: {'DIVERGENT' if report['divergence'] else 'PARITY'}")


def _split_csv(values: Optional[Sequence[str]]) -> List[str]:
    out: List[str] = []
    for value in values or []:
        out.extend(token.strip() for token in value.split(",") if token.strip())
    return out


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="diff_trace.py",
        description="Offline parity diff of shadow vs legacy Phase 4B-2 traces. "
                    "No time shift, no SE(3) alignment, no scale correction, no "
                    "nearest-neighbour rematching.")
    parser.add_argument("--shadow", required=True, metavar="FILE",
                        help="shadow trace JSONL")
    parser.add_argument("--legacy", required=True, metavar="FILE",
                        help="legacy trace JSONL")
    parser.add_argument("--ignore-fields", action="append", metavar="LIST",
                        help="comma separated field names/globs to skip "
                             "(e.g. 'watermark_ns,insertion_time_ns'); repeatable")
    parser.add_argument("--abs-tol", type=float, default=None,
                        help="absolute tolerance override for translation/rotation/"
                             "velocity (default from schema.json: 1e-6)")
    parser.add_argument("--rel-tol", type=float, default=None,
                        help="relative tolerance floor (default 0)")
    parser.add_argument("--trans-tol", type=float, default=None,
                        help="translation tolerance in meters (default 1e-6)")
    parser.add_argument("--rot-tol", type=float, default=None,
                        help="rotation tolerance in radians (default 1e-6)")
    parser.add_argument("--vel-tol", type=float, default=None,
                        help="velocity tolerance in m/s (default 1e-6)")
    parser.add_argument("--bias-tol", type=float, default=None,
                        help="bias tolerance (default 1e-8)")
    parser.add_argument("--type", action="append", metavar="LAYER_OR_TYPE",
                        help="restrict comparison to a layer (timeline|input|factor|"
                             "gate|state|trajectory) or an event type; repeatable")
    parser.add_argument("--k-min", type=int, default=None,
                        help="lowest interval index k to compare (records with k=null "
                             "are always kept)")
    parser.add_argument("--k-max", type=int, default=None,
                        help="highest interval index k to compare")
    parser.add_argument("--allow-extra-fields", action="store_true",
                        help="compare only fields present on both sides")
    parser.add_argument("--collapse", choices=("none", "first", "last"), default="none",
                        help="collapse repeated observations of the same "
                             "(layer,k,source,epoch) to the first/last one "
                             "(default: none = strict, every occurrence must match)")
    parser.add_argument("--all-divergences", action="store_true",
                        help="include every divergence (not only the first per layer)")
    parser.add_argument("--max-divergences", type=int, default=50,
                        help="cap for --all-divergences (default: 50)")
    parser.add_argument("--format", choices=("text", "json"), default="text",
                        help="report format (default: text)")
    parser.add_argument("--output", metavar="FILE",
                        help="also write the JSON report to FILE")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    tokens = _split_csv(args.type)
    types: Optional[Set[str]] = set(tokens) if tokens else None
    if types:
        allowed = set(layer_order())
        for token in types:
            if token not in allowed and layer_of_type(token) is None:
                print(f"diff_trace.py: unknown --type '{token}' "
                      f"(expected a layer or a known event type)", file=sys.stderr)
                return 2

    options = DiffOptions(
        ignore_fields=_split_csv(args.ignore_fields),
        allow_extra_fields=args.allow_extra_fields,
        types=types,
        k_min=args.k_min,
        k_max=args.k_max,
        collapse=args.collapse,
        tolerances=build_tolerances(args),
    )

    for path in (args.shadow, args.legacy):
        if not Path(path).is_file():
            print(f"diff_trace.py: cannot read '{path}'", file=sys.stderr)
            return 2

    try:
        report = diff_traces(args.shadow, args.legacy, options,
                             all_divergences=args.all_divergences,
                             max_divergences=args.max_divergences)
    except ValueError as exc:
        print(f"diff_trace.py: {exc}", file=sys.stderr)
        return 2

    if args.output:
        Path(args.output).write_text(json.dumps(report, indent=2) + "\n",
                                     encoding="utf-8")
    if args.format == "json":
        print(json.dumps(report, indent=2))
    else:
        print_text_report(report)

    return 1 if report["divergence"] else 0


if __name__ == "__main__":
    sys.exit(main())
