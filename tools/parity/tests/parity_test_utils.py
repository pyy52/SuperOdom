# -*- coding: utf-8 -*-
"""Shared helpers for the parity toolchain unit tests (stdlib unittest only)."""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

PARITY_DIR = Path(__file__).resolve().parents[1]
FIXTURES_DIR = PARITY_DIR / "fixtures"
if str(PARITY_DIR) not in sys.path:
    sys.path.insert(0, str(PARITY_DIR))

import diff_trace  # noqa: E402
import parity_schema  # noqa: E402
import summarize_trace  # noqa: E402
import validate_trace  # noqa: E402


def fixture(name: str) -> str:
    return str(FIXTURES_DIR / name)


def write_jsonl(records: Iterable[Dict[str, Any]], directory: Optional[str] = None) -> str:
    """Write records to a temporary JSONL file and return its path."""
    handle = tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False,
                                         dir=directory, encoding="utf-8")
    with handle:
        for record in records:
            handle.write(json.dumps(record) + "\n")
    return handle.name


def write_text(text: str, suffix: str = ".jsonl") -> str:
    handle = tempfile.NamedTemporaryFile("w", suffix=suffix, delete=False,
                                         encoding="utf-8")
    with handle:
        handle.write(text)
    return handle.name


def record(producer: str, type_name: str, timestamp_ns: int, k: Optional[int],
           source: str, epoch: int, payload: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "schema_version": 1,
        "producer": producer,
        "type": type_name,
        "timestamp_ns": timestamp_ns,
        "k": k,
        "source": source,
        "epoch": epoch,
        "payload": payload,
    }


def validate_args(files: Optional[List[str]] = None, **overrides: Any):
    """Argument namespace with the same defaults as validate_trace.main()."""
    # argparse requires at least one positional file; a placeholder keeps the
    # default namespace identical to the CLI without touching the filesystem.
    argv = list(files) if files else ["<placeholder>.jsonl"]
    args = validate_trace.build_parser().parse_args(argv)
    args.allow_schema_version = args.allow_schema_version or [1]
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


def validate_records(records: List[Dict[str, Any]], **overrides: Any) -> Any:
    path = write_jsonl(records)
    args = validate_args(**overrides)
    return validate_trace.validate_file(path, args)


def diff_options(**overrides: Any) -> Any:
    argv: List[str] = ["--shadow", "s.jsonl", "--legacy", "l.jsonl"]
    args = diff_trace.build_parser().parse_args(argv)
    options = diff_trace.DiffOptions(
        ignore_fields=diff_trace._split_csv(args.ignore_fields),
        allow_extra_fields=args.allow_extra_fields,
        types=set(diff_trace._split_csv(args.type)) or None,
        k_min=args.k_min,
        k_max=args.k_max,
        collapse=args.collapse,
        tolerances=diff_trace.build_tolerances(args),
    )
    for key, value in overrides.items():
        setattr(options, key, value)
    return options


def summarize_records(records: List[Dict[str, Any]], **kwargs: Any) -> Dict[str, Any]:
    path = write_jsonl(records)
    return summarize_trace.summarize(path, **kwargs)
