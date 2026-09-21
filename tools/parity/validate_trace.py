#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline validator for Phase 4B-2 parity traces (shadow / legacy JSONL).

Checks (taskbook section 8):

* valid JSONL (one JSON object per non-empty line)
* required fields / ``schema_version`` / producer / type vocabulary
* ``timestamp_ns`` integer, ``k`` integer-or-null and non-negative
* known reject reasons and gate decisions
* factor ``committed`` / ``finalized`` consistency and vector shapes
* monotonic lifecycle transitions (anchor status, one-shot gate slots)

Explicitly *not* an error: source event timestamps arriving out of order.
A callback may legitimately receive a source event whose measurement stamp is
older than the previous one; that is reported as information only.

Exit codes: 0 = valid, 1 = invalid, 2 = usage / unreadable input.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))

from parity_schema import (  # noqa: E402  (path bootstrap must precede import)
    TraceRecord,
    anchor_status_order,
    anchor_status_terminal,
    canonicalize_payload,
    known_reject_reasons,
    known_sources,
    known_types,
    layer_of_type,
    lenient_metadata_fields,
    producers,
    read_jsonl,
    required_fields,
)

GATE_DECISIONS = ("ACCEPTED", "REJECTED")
TERMINAL_GATE_REASONS = {
    "TOO_LATE", "REJECT_TOO_LATE", "SLOT_OCCUPIED", "REJECT_SLOT_OCCUPIED",
    "CROSS_EPOCH", "REJECT_CROSS_EPOCH", "NO_BRACKET", "REJECT_NO_BRACKET",
    "INNOVATION_TOO_LARGE_TRANS", "INNOVATION_TOO_LARGE_ROT",
    "REJECT_INNOVATION_TRANS", "REJECT_INNOVATION_ROT", "GATE_REJECT",
}
STATUS_RANK = {name: index for index, name in enumerate(anchor_status_order())}


class Report:
    """Collects findings for one trace file."""

    def __init__(self, path: str) -> None:
        self.path = path
        self.records = 0
        self.malformed_lines = 0
        self.errors: List[Dict[str, Any]] = []
        self.warnings: List[Dict[str, Any]] = []
        self.info: Dict[str, Any] = {
            "out_of_order_timestamps": 0,
            "out_of_order_note": ("source event timestamps are not required to be "
                                  "callback-arrival monotonic; this is not an error"),
        }
        self.counts_by_type: Dict[str, int] = {}
        self.counts_by_layer: Dict[str, int] = {}
        self.observed_producer: Optional[str] = None
        self.observed_schema_version: Optional[int] = None

    def error(self, line: Optional[int], code: str, message: str) -> None:
        self.errors.append({"line": line, "code": code, "severity": "error",
                            "message": message})

    def warn(self, line: Optional[int], code: str, message: str) -> None:
        self.warnings.append({"line": line, "code": code, "severity": "warning",
                              "message": message})

    @property
    def ok(self) -> bool:
        return not self.errors

    def to_dict(self) -> Dict[str, Any]:
        return {
            "file": self.path,
            "ok": self.ok,
            "records": self.records,
            "malformed_lines": self.malformed_lines,
            "producer": self.observed_producer,
            "schema_version": self.observed_schema_version,
            "errors": self.errors,
            "warnings": self.warnings,
            "info": self.info,
            "counts_by_type": dict(sorted(self.counts_by_type.items())),
            "counts_by_layer": dict(sorted(self.counts_by_layer.items())),
        }


def _check_payload(rec: TraceRecord, report: Report, args: argparse.Namespace) -> None:
    """Layer specific payload checks (reasons, factor shape, decisions)."""
    if rec.layer is None or not isinstance(rec.payload, dict):
        return
    line = rec.line_no
    canonical, _, _ = canonicalize_payload(rec.layer, rec.payload)
    vocabulary = set(known_reject_reasons())

    def check_reason(value: Any, code: str) -> None:
        if not isinstance(value, str) or value in vocabulary:
            return
        message = (f"unknown {code} '{value}' outside the controlled vocabulary "
                   f"(see tools/parity/schema.json reject_reasons)")
        (report.warn if args.allow_unknown_reasons else report.error)(
            line, f"unknown_{code}", message)

    if rec.layer == "gate":
        check_reason(canonical.get("reason"), "reason")
        decision = canonical.get("decision")
        if decision is not None and decision not in GATE_DECISIONS:
            report.error(line, "gate_decision",
                         f"'decision' must be one of {GATE_DECISIONS}, got {decision!r}")

    if rec.layer == "input":
        for field in ("drop_reason", "action"):
            check_reason(canonical.get(field), field)

    if rec.layer == "factor":
        keys = canonical.get("keys")
        if keys is not None and (not isinstance(keys, list) or len(keys) != 2):
            report.error(line, "factor_keys",
                         f"'keys' must be a 2-element list, got {keys!r}")
        measurement = canonical.get("measurement_se3")
        if measurement is not None and (not isinstance(measurement, list)
                                        or len(measurement) != 7):
            report.error(line, "factor_measurement",
                         "'measurement_se3' must be [tx,ty,tz,qx,qy,qz,qw] (7 values), "
                         f"got {measurement!r}")
        sigmas = canonical.get("noise_sigmas")
        if sigmas is not None and (not isinstance(sigmas, list) or len(sigmas) != 6):
            report.error(line, "factor_noise",
                         "'noise_sigmas' must be 6 values in GTSAM order "
                         f"[rotation(3), translation(3)], got {sigmas!r}")
        committed = canonical.get("committed")
        finalized = canonical.get("finalized")
        for name, value in (("committed", committed), ("finalized", finalized)):
            if value is not None and not isinstance(value, bool):
                report.error(line, "factor_flag",
                             f"'{name}' must be a boolean, got {value!r}")
        if finalized is True and committed is False:
            report.error(line, "factor_consistency",
                         "'finalized' is true while 'committed' is false; a factor cannot be "
                         "finalized without being committed")
        status = canonical.get("status")
        if status is not None and status not in ("COMMITTED", "FINALIZED"):
            report.error(line, "factor_status",
                         f"'status' must be COMMITTED or FINALIZED, got {status!r}")
        if status == "FINALIZED" and committed is False:
            report.error(line, "factor_consistency",
                         "status FINALIZED while 'committed' is false")


def _check_required_and_common(rec: TraceRecord, report: Report,
                               args: argparse.Namespace) -> None:
    """Field presence, types and controlled vocabularies for one record."""
    line = rec.line_no
    for field in required_fields():
        if field in rec.raw:
            continue
        if field in lenient_metadata_fields() and not args.strict:
            continue
        report.error(line, "missing_field", f"missing required field '{field}'")

    producer = rec.raw.get("producer")
    if producer is None:
        if args.producer is None:
            message = ("'producer' missing and no --producer given; cannot attribute "
                       "this trace to shadow or legacy")
            (report.error if args.strict else report.warn)(line, "producer", message)
        elif report.observed_producer is None:
            report.observed_producer = args.producer
    elif not isinstance(producer, str) or producer.lower() not in producers():
        report.error(line, "producer",
                     f"'producer' must be one of {producers()}, got {producer!r}")
    else:
        if report.observed_producer not in (None, producer.lower()):
            report.error(line, "producer",
                         f"file mixes producers ('{report.observed_producer}' and "
                         f"'{producer.lower()}')")
        report.observed_producer = producer.lower()

    version = rec.raw.get("schema_version")
    allowed = set(args.allow_schema_version)
    if version is None:
        message = ("'schema_version' missing; assuming 1 (use --strict to require the "
                   "field explicitly)")
        (report.error if args.strict else report.warn)(line, "schema_version", message)
    elif not _is_int(version):
        report.error(line, "schema_version",
                     f"'schema_version' must be an integer, got {version!r}")
    elif version not in allowed:
        report.error(line, "schema_version",
                     f"unsupported schema_version {version} (allowed: {sorted(allowed)})")
    else:
        report.observed_schema_version = version

    if rec.type is None:
        report.error(line, "type", "missing 'type'")
    elif not isinstance(rec.type, str):
        report.error(line, "type",
                     f"'type' must be a string, got {type(rec.type).__name__}")
    else:
        layer = layer_of_type(rec.type)
        if layer is None:
            report.error(line, "type",
                         f"unknown type '{rec.type}' (known: {', '.join(known_types())})")
        else:
            report.counts_by_type[rec.type] = report.counts_by_type.get(rec.type, 0) + 1
            report.counts_by_layer[layer] = report.counts_by_layer.get(layer, 0) + 1

    if "timestamp_ns" in rec.raw and not _is_int(rec.raw.get("timestamp_ns")):
        report.error(line, "timestamp_ns",
                     f"'timestamp_ns' must be an integer (ns), got {rec.raw.get('timestamp_ns')!r}")

    if "k" in rec.raw:
        k = rec.raw["k"]
        if k is not None:
            if not _is_int(k):
                report.error(line, "k", f"'k' must be an integer or null, got {k!r}")
            elif k < 0:
                report.error(line, "k", f"'k' must be non-negative, got {k}")

    if "source" in rec.raw:
        source = rec.raw["source"]
        lower_sources = {item.lower() for item in known_sources()}
        if not isinstance(source, str):
            report.error(line, "source", f"'source' must be a string, got {source!r}")
        elif source.lower() not in lower_sources:
            report.error(line, "source",
                         f"unknown source '{source}' (expected one of "
                         f"{', '.join(known_sources())})")

    if "epoch" in rec.raw:
        epoch = rec.raw["epoch"]
        if not _is_int(epoch):
            report.error(line, "epoch", f"'epoch' must be an integer, got {epoch!r}")
        elif epoch < 0:
            report.error(line, "epoch", f"'epoch' must be non-negative, got {epoch}")

    if "payload" in rec.raw and not isinstance(rec.raw["payload"], dict):
        report.error(line, "payload", "'payload' must be a JSON object")


def _is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _check_lifecycle(records: List[TraceRecord], report: Report) -> None:
    """Monotonic lifecycle transitions and one-shot semantics across the file."""
    anchor_state: Dict[Any, Any] = {}
    gate_state: Dict[Any, Dict[str, Any]] = {}
    factor_state: Dict[Any, Dict[str, Any]] = {}
    previous_ts: Optional[int] = None

    for rec in records:
        ts = rec.raw.get("timestamp_ns")
        if _is_int(ts):
            # Information only: source event stamps are NOT required to be
            # callback-arrival monotonic (taskbook section 8).
            if previous_ts is not None and ts < previous_ts:
                report.info["out_of_order_timestamps"] += 1
            previous_ts = ts

        if not isinstance(rec.payload, dict):
            continue
        canonical, _, _ = canonicalize_payload(rec.layer, rec.payload)

        status = canonical.get("anchor_status")
        if isinstance(status, str) and isinstance(rec.k, int):
            if status in anchor_status_terminal():
                rank = len(anchor_status_order())
            else:
                rank = STATUS_RANK.get(status)
            if rank is not None:
                previous = anchor_state.get(rec.k)
                if previous is not None:
                    previous_rank, previous_status, previous_line = previous
                    if previous_rank >= len(anchor_status_order()):
                        if status != previous_status:
                            report.error(rec.line_no, "lifecycle",
                                         f"anchor k={rec.k} left terminal status "
                                         f"'{previous_status}' (line {previous_line}) for "
                                         f"'{status}'")
                    elif rank < previous_rank:
                        report.error(rec.line_no, "lifecycle",
                                     f"anchor k={rec.k} status went backwards: "
                                     f"'{previous_status}' (line {previous_line}) -> "
                                     f"'{status}'")
                anchor_state[rec.k] = (rank, status, rec.line_no)

        decision = canonical.get("decision")
        if rec.layer == "gate" and isinstance(decision, str):
            key = (rec.source, rec.epoch, rec.k)
            state = gate_state.setdefault(key, {"terminal": False, "line": None})
            if decision == "ACCEPTED" and state["terminal"]:
                report.error(rec.line_no, "gate_one_shot",
                             f"gate slot {key} accepted after a terminal rejection at "
                             f"line {state['line']}")
            state["line"] = rec.line_no
            reason = canonical.get("reason")
            if decision == "REJECTED" and isinstance(reason, str) and \
                    reason in TERMINAL_GATE_REASONS:
                state["terminal"] = True

        committed = canonical.get("committed")
        finalized = canonical.get("finalized")
        if rec.layer == "factor" and (committed is not None or finalized is not None):
            key = (rec.source, rec.epoch, rec.k)
            state = factor_state.setdefault(key, {"committed": False, "committed_line": None,
                                                  "finalized": False})
            # Within one record, 'committed' and 'finalized' may both be true:
            # the record states the end-of-step status. Finalized without any
            # committed record (here or earlier) is inconsistent.
            if committed is True:
                state["committed"] = True
                state["committed_line"] = state["committed_line"] or rec.line_no
            if finalized is True and not state["committed"]:
                report.error(rec.line_no, "factor_finalize_before_commit",
                             f"factor slot {key} finalized before any committed record")
            if finalized is True:
                state["finalized"] = True


def validate_file(path: str, args: argparse.Namespace) -> Report:
    """Validate one trace file and return its report."""
    report = Report(path)
    records, parse_errors = read_jsonl(path)
    report.malformed_lines = len(parse_errors)
    for err in parse_errors:
        report.error(err["line"], "jsonl", err["error"])
    report.records = len(records)
    for rec in records:
        _check_required_and_common(rec, report, args)
        _check_payload(rec, report, args)
    _check_lifecycle(records, report)
    if len(report.errors) > args.max_errors:
        report.warn(None, "truncated",
                    f"only the first {args.max_errors} of {len(report.errors)} errors "
                    "are reported")
        report.errors = report.errors[:args.max_errors]
    return report


def _print_text(report: Report, quiet: bool) -> None:
    print(f"== validate_trace: {report.path}")
    print(f"records={report.records} malformed_lines={report.malformed_lines} "
          f"producer={report.observed_producer} "
          f"schema_version={report.observed_schema_version}")
    print(f"errors={len(report.errors)} warnings={len(report.warnings)}")
    if report.counts_by_layer:
        print("layers: " + ", ".join(f"{name}={count}"
                                     for name, count in sorted(report.counts_by_layer.items())))
    if report.counts_by_type:
        print("types: " + ", ".join(f"{name}={count}"
                                    for name, count in sorted(report.counts_by_type.items())))
    if not quiet:
        print(f"info: out_of_order_timestamps={report.info['out_of_order_timestamps']} "
              f"({report.info['out_of_order_note']})")
    for item in report.errors + report.warnings:
        line = "-" if item["line"] is None else item["line"]
        print(f"  [{item['severity']}] line {line} code={item['code']} {item['message']}")
    print(f"RESULT: {'VALID' if report.ok else 'INVALID'}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="validate_trace.py",
        description="Validate Phase 4B-2 parity trace JSONL (read-only observation "
                    "artifact).")
    parser.add_argument("files", nargs="+", metavar="TRACE.jsonl",
                        help="trace file(s) to validate")
    parser.add_argument("--producer", choices=producers(),
                        help="producer to assume when a line omits the 'producer' field")
    parser.add_argument("--strict", action="store_true",
                        help="require 'producer' and 'schema_version' on every line")
    parser.add_argument("--allow-unknown-reasons", action="store_true",
                        help="downgrade unknown reject reasons to warnings")
    parser.add_argument("--allow-schema-version", type=int, action="append",
                        metavar="N", default=None,
                        help="schema_version accepted by this validator (default: 1, "
                             "repeatable)")
    parser.add_argument("--max-errors", type=int, default=50,
                        help="maximum number of errors reported per file (default: 50)")
    parser.add_argument("--format", choices=("text", "json"), default="text",
                        help="report format (default: text)")
    parser.add_argument("-q", "--quiet", action="store_true",
                        help="omit informational lines")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.allow_schema_version is None:
        args.allow_schema_version = [1]

    reports: List[Report] = []
    for path in args.files:
        if not Path(path).is_file():
            print(f"validate_trace.py: cannot read '{path}'", file=sys.stderr)
            return 2
        reports.append(validate_file(path, args))

    if args.format == "json":
        payload = {"tool": "validate_trace.py",
                   "ok": all(report.ok for report in reports),
                   "reports": [report.to_dict() for report in reports]}
        print(json.dumps(payload, indent=2, sort_keys=False))
    else:
        for index, report in enumerate(reports):
            if index:
                print()
            _print_text(report, args.quiet)
        print()
        print(f"SUMMARY: {len(reports)} file(s), "
              f"{sum(1 for r in reports if r.ok)} valid, "
              f"{sum(1 for r in reports if not r.ok)} invalid, "
              f"{sum(len(r.errors) for r in reports)} error(s)")

    return 0 if all(report.ok for report in reports) else 1


if __name__ == "__main__":
    sys.exit(main())
