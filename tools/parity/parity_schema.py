#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Shared schema helpers for the Phase 4B-2 parity trace toolchain.

Pure Python stdlib only (json, dataclasses, statistics, argparse callers).
The vocabulary lives in ``schema.json`` next to this file so that the
validator, the differ and the summarizer cannot drift apart.

Design rules taken from 25_PARITY_TRACE_TOOL_SPEC.md:

* a trace line is an *observation artifact*: never production state, never
  optimizer input, never a reason to change callback / factor / graph behaviour;
* SE(3) serialization is always ``t_xyz`` first for 7-vectors
  (``[tx,ty,tz,qx,qy,qz,qw]``, quaternion order x,y,z,w) and always
  ``[rotation(3), translation(3)]`` for GTSAM tangent 6-vectors (R4.2);
* tolerances are configuration, never hard-coded in scattered places;
* the tools never "align" or "prettify" data (no time shift, no SE(3)
  alignment, no scale correction, no nearest-neighbour rematching).
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Optional, Sequence, Tuple

SCHEMA_PATH = Path(__file__).resolve().parent / "schema.json"

# ---------------------------------------------------------------------------
# Embedded defaults (used only when schema.json is missing/unreadable).
# ---------------------------------------------------------------------------

_LAYER_ORDER = ["timeline", "input", "factor", "gate", "state", "trajectory"]

_TYPE_TO_LAYER = {
    # timeline
    "TIMELINE_ANCHOR_OPEN": "timeline",
    "TIMELINE_ANCHOR_CLOSE": "timeline",
    "TIMELINE_INTERVAL_CLOSE": "timeline",
    "TIMELINE_INTERVAL_STATUS": "timeline",
    "FUSION_SEGMENT_RESET": "timeline",
    "timeline": "timeline",
    # input
    "INPUT_CONSUME": "input",
    "INPUT_SAMPLE": "input",
    "INPUT_DROP": "input",
    "input": "input",
    # factor
    "FACTOR_INSERT": "factor",
    "FACTOR_FINALIZE": "factor",
    "factor": "factor",
    # gate
    "GATE_EVALUATION": "gate",
    "GATE_DECISION": "gate",
    "gate": "gate",
    # state
    "STATE_SNAPSHOT": "state",
    "STATE_UPDATE": "state",
    "state": "state",
    # trajectory
    "TRAJECTORY_POSE": "trajectory",
    "TRAJECTORY_STATE": "trajectory",
    "trajectory": "trajectory",
}

_REQUIRED_FIELDS = [
    "schema_version",
    "producer",
    "type",
    "timestamp_ns",
    "k",
    "source",
    "epoch",
    "payload",
]
_LENIENT_METADATA_FIELDS = ["producer", "schema_version"]

_PRODUCERS = ["shadow", "legacy"]
_SOURCES_CANONICAL = ["imu", "lio", "central"]
_SOURCES_ACCEPTED = ["IMU", "LIO", "SYSTEM", "CENTRAL", "VIO"]

_FIELD_ALIASES: Dict[str, Dict[str, List[str]]] = {
    "timeline": {
        "t_k": ["t_k", "anchor_stamp_ns", "interval_stamp_ns"],
        "anchor_status": ["anchor_status"],
        "interval_status": ["interval_status"],
        "fusion_segment": ["fusion_segment", "fusion_segment_status"],
    },
    "input": {
        "sensor_type": ["sensor_type"],
        "sample_time_ns": ["sample_time_ns", "imu_stamp_ns", "lio_stamp_ns"],
        "action": ["action"],
        "drop_reason": ["drop_reason", "reason"],
        "coverage_gap": ["coverage_gap"],
        "epoch_changed": ["epoch_changed"],
        "lateness_ns": ["lateness_ns"],
    },
    "factor": {
        "factor_type": ["factor_type"],
        "key_i": ["key_i"],
        "key_j": ["key_j"],
        "keys": ["keys"],
        "measurement_se3": ["measurement_se3", "measurement_T_Bi_Bj", "measurement"],
        "noise_sigmas": ["noise_sigmas", "noise_diag", "sigmas", "sigma6"],
        "committed": ["committed"],
        "finalized": ["finalized"],
        "status": ["status"],
        "insertion_time_ns": ["insertion_time_ns"],
    },
    "gate": {
        "dT_imu_ref": ["dT_imu_ref"],
        "dT_source": ["dT_source"],
        "innovation6": ["innovation6", "innovation_6d", "innovation_6"],
        "trans_norm": ["trans_norm", "translation_norm"],
        "rot_norm": ["rot_norm", "rotation_norm"],
        "thresholds": ["thresholds"],
        "decision": ["decision"],
        "reason": ["reason"],
        "lateness_ns": ["lateness_ns"],
        "watermark_ns": ["watermark_ns"],
        "left_bracket_ns": ["left_bracket_ns"],
        "right_bracket_ns": ["right_bracket_ns"],
        "interpolation_brackets": ["interpolation_brackets"],
    },
    "state": {
        "pose": ["pose", "T_W_B"],
        "velocity": ["velocity", "v_W"],
        "bias_acc": ["bias_acc"],
        "bias_gyro": ["bias_gyro"],
        "anchor_status": ["anchor_status"],
    },
    "trajectory": {
        "pose": ["pose", "T_W_B"],
        "output_stamp_ns": ["output_stamp_ns", "stamp_ns"],
    },
}

_REJECT_REASONS = {
    "spec_seven": [
        "NO_BRACKET",
        "CROSS_EPOCH",
        "TOO_LATE",
        "GATE_REJECT",
        "SLOT_OCCUPIED",
        "DUPLICATE_IDENTITY",
        "IMU_GAP",
    ],
    "shadow_accept_decision": [
        "ACCEPTED",
        "REJECT_TOO_LATE",
        "REJECT_DUPLICATE",
        "REJECT_NO_REF",
        "REJECT_NO_BRACKET",
        "REJECT_INNOVATION_TRANS",
        "REJECT_INNOVATION_ROT",
        "REJECT_SLOT_OCCUPIED",
        "REJECT_CROSS_EPOCH",
    ],
    "shadow_gate_reason": [
        "SUCCESS",
        "TOO_LATE",
        "INNOVATION_TOO_LARGE_TRANS",
        "INNOVATION_TOO_LARGE_ROT",
    ],
    "input_drop_reason": [
        "late_after_watermark",
        "stale_skipped",
        "coverage_gap",
        "imu_gap",
        "duplicate_identity",
        "slot_occupied",
        "cross_epoch",
        "no_bracket",
        "too_late",
    ],
}

_ANCHOR_STATUS_ORDER = ["SCHEDULED", "GRAPH_INSERTED", "SOLVED"]
_ANCHOR_STATUS_TERMINAL = ["SOLVED", "INVALID_GAP", "INVALID"]

_TOLERANCES_DEFAULT = {
    "trans": 1e-6,
    "rot": 1e-6,
    "vel": 1e-6,
    "bias": 1e-8,
    "rel": 0.0,
}


# ---------------------------------------------------------------------------
# Schema loading
# ---------------------------------------------------------------------------

def load_schema(path: Optional[os.PathLike] = None) -> Dict[str, Any]:
    """Load schema.json, falling back to the embedded defaults."""
    p = Path(path) if path is not None else SCHEMA_PATH
    schema: Dict[str, Any] = {
        "schema_version": 1,
        "record_required_fields": list(_REQUIRED_FIELDS),
        "lenient_metadata_fields": _LENIENT_METADATA_FIELDS,
        "producers": list(_PRODUCERS),
        "sources": {"canonical": _SOURCES_CANONICAL,
                    "accepted_case_insensitive": _SOURCES_ACCEPTED},
        "layer_order": list(_LAYER_ORDER),
        "types": {},
        "payload_fields": _FIELD_ALIASES,
        "reject_reasons": _REJECT_REASONS,
        "anchor_status_order": list(_ANCHOR_STATUS_ORDER),
        "anchor_status_terminal": list(_ANCHOR_STATUS_TERMINAL),
        "tolerances_default": dict(_TOLERANCES_DEFAULT),
    }
    for layer, types in _TYPE_TO_LAYER.items():
        if types == layer:
            continue
        schema["types"].setdefault(types, []).append(layer)
    try:
        raw = json.loads(Path(p).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return schema

    for key in ("schema_version", "record_required_fields",
                "lenient_metadata_fields", "producers", "sources",
                "layer_order", "payload_fields", "reject_reasons",
                "anchor_status_order", "anchor_status_terminal",
                "tolerances_default"):
        if key in raw:
            schema[key] = raw[key]
    if "types" in raw:
        merged: Dict[str, List[str]] = {layer: list(items)
                                        for layer, items in schema["types"].items()}
        for layer, items in raw["types"].items():
            merged.setdefault(layer, [])
            for item in items:
                if item not in merged[layer]:
                    merged[layer].append(item)
        schema["types"] = merged
    for layer, canon in _FIELD_ALIASES.items():
        for field_name, aliases in canon.items():
            have = schema["payload_fields"].setdefault(layer, {}).setdefault(field_name, [])
            for alias in aliases:
                if alias not in have:
                    have.append(alias)
    return schema


SCHEMA: Dict[str, Any] = load_schema()


def layer_order() -> List[str]:
    return list(SCHEMA["layer_order"])


def required_fields() -> List[str]:
    return list(SCHEMA["record_required_fields"])


def lenient_metadata_fields() -> List[str]:
    return list(SCHEMA["lenient_metadata_fields"])


def producers() -> List[str]:
    return list(SCHEMA["producers"])


def known_sources() -> List[str]:
    src = SCHEMA.get("sources", {})
    return list(src.get("canonical", [])) + list(src.get("accepted_case_insensitive", []))


def known_types() -> List[str]:
    out: List[str] = []
    for items in SCHEMA.get("types", {}).values():
        out.extend(items)
    return out


def known_reject_reasons() -> List[str]:
    out: List[str] = []
    for items in SCHEMA.get("reject_reasons", {}).values():
        out.extend(items)
    return sorted(set(out))


def default_tolerances() -> Dict[str, float]:
    tol = SCHEMA.get("tolerances_default", {})
    return {name: float(tol.get(name, _TOLERANCES_DEFAULT[name]))
            for name in ("trans", "rot", "vel", "bias", "rel")}


def anchor_status_order() -> List[str]:
    return list(SCHEMA.get("anchor_status_order", _ANCHOR_STATUS_ORDER))


def anchor_status_terminal() -> List[str]:
    return list(SCHEMA.get("anchor_status_terminal", _ANCHOR_STATUS_TERMINAL))


def layer_of_type(type_name: Any) -> Optional[str]:
    """Map a record ``type`` to one of the six parity layers.

    Accepts both the emitted event names (``TIMELINE_ANCHOR_OPEN``) and the
    layer-level names from the taskbook schema section (``timeline``, ``gate``).
    """
    if not isinstance(type_name, str):
        return None
    if type_name in _TYPE_TO_LAYER:
        return _TYPE_TO_LAYER[type_name]
    upper = type_name.upper()
    if upper in _TYPE_TO_LAYER:
        return _TYPE_TO_LAYER[upper]
    for layer, items in SCHEMA.get("types", {}).items():
        if type_name in items or upper in {str(i).upper() for i in items}:
            return layer
    # Last resort: prefix match, e.g. GATE_FOO -> gate
    low = type_name.lower()
    for layer in _LAYER_ORDER:
        if low.startswith(layer):
            return layer
    return None


def normalize_source(value: Any) -> Optional[str]:
    """Case-insensitive source normalization (imu|lio|central)."""
    if value is None:
        return None
    if not isinstance(value, str):
        return None
    low = value.strip().lower()
    if low in ("imu",):
        return "imu"
    if low in ("lio", "lidar", "laser"):
        return "lio"
    if low in ("central", "system", "backend", "optimizer"):
        return "central"
    if low in ("vio", "visual"):
        return "vio"
    return low


def normalize_producer(value: Any) -> Optional[str]:
    if not isinstance(value, str):
        return None
    low = value.strip().lower()
    return low if low in producers() else None


def field_alias_table(layer: str) -> Dict[str, List[str]]:
    return dict(SCHEMA.get("payload_fields", {}).get(layer, {}))


# ---------------------------------------------------------------------------
# Records
# ---------------------------------------------------------------------------

@dataclass
class TraceRecord:
    """One immutable observation (one JSONL line)."""

    line_no: int
    raw: Dict[str, Any]
    producer: Optional[str] = None
    type: Optional[str] = None
    layer: Optional[str] = None
    timestamp_ns: Any = None
    k: Any = None
    source: Optional[str] = None
    epoch: Any = None
    payload: Any = None

    @property
    def k_sort(self) -> int:
        """Sort surrogate: records without an interval index sort first."""
        if isinstance(self.k, int) and not isinstance(self.k, bool):
            return self.k
        return -1

    def status_label(self) -> str:
        return f"k={self.k} source={self.source} epoch={self.epoch} type={self.type}"


def make_record(raw: Any, line_no: int) -> TraceRecord:
    if not isinstance(raw, dict):
        return TraceRecord(line_no=line_no, raw={"__non_object__": raw})
    rec = TraceRecord(line_no=line_no, raw=raw)
    rec.producer = raw.get("producer")
    rec.type = raw.get("type")
    rec.layer = layer_of_type(raw.get("type"))
    rec.timestamp_ns = raw.get("timestamp_ns")
    rec.k = raw.get("k")
    rec.source = normalize_source(raw.get("source"))
    rec.epoch = raw.get("epoch")
    rec.payload = raw.get("payload")
    return rec


def read_jsonl(path: os.PathLike) -> Tuple[List[TraceRecord], List[Dict[str, Any]]]:
    """Read a JSONL trace.

    Returns ``(records, parse_errors)`` where ``parse_errors`` is a list of
    ``{"line": int, "error": str}``. Blank lines are skipped; a malformed line
    never aborts the parse so the validator can report every problem at once.
    """
    records: List[TraceRecord] = []
    errors: List[Dict[str, Any]] = []
    try:
        with open(path, "r", encoding="utf-8") as handle:
            for line_no, line in enumerate(handle, start=1):
                stripped = line.strip()
                if not stripped:
                    continue
                try:
                    raw = json.loads(stripped)
                except ValueError as exc:
                    errors.append({"line": line_no, "error": f"invalid JSON: {exc}"})
                    continue
                if not isinstance(raw, dict):
                    errors.append({"line": line_no,
                                   "error": "record is not a JSON object"})
                    continue
                records.append(make_record(raw, line_no))
    except OSError as exc:
        raise FileNotFoundError(str(exc)) from exc
    return records, errors


# ---------------------------------------------------------------------------
# Payload canonicalization (event-name and taskbook-name agnostic)
# ---------------------------------------------------------------------------

def canonicalize_payload(layer: Optional[str], payload: Any) -> Tuple[Dict[str, Any],
                                                                     Dict[str, str],
                                                                     Dict[str, Any]]:
    """Map a payload onto canonical field names.

    Returns ``(canonical, raw_names, extras)``:

    * ``canonical``  : canonical field name -> value
    * ``raw_names``  : canonical field name -> key as it appeared in the file
    * ``extras``     : payload keys this layer's alias table does not know
    """
    canonical: Dict[str, Any] = {}
    raw_names: Dict[str, str] = {}
    if not isinstance(payload, dict):
        return canonical, raw_names, {"payload": payload}
    table = field_alias_table(layer or "")
    claimed = set()
    for canon_name, aliases in table.items():
        for alias in aliases:
            if alias in payload:
                canonical[canon_name] = payload[alias]
                raw_names[canon_name] = alias
                claimed.add(alias)
                break
    extras = {key: value for key, value in payload.items() if key not in claimed}
    return canonical, raw_names, extras


def flatten(value: Any, prefix: str, out: Dict[str, Any]) -> Dict[str, Any]:
    """Flatten nested dicts/lists into ``path -> leaf`` entries.

    Insertion order is preserved so that the reported "first divergent field"
    follows the schema field order on both sides.
    """
    if isinstance(value, dict):
        for key, item in value.items():
            flatten(item, f"{prefix}.{key}", out)
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            flatten(item, f"{prefix}[{index}]", out)
    else:
        out[prefix] = value
    return out


def canonical_leaves(layer: Optional[str], payload: Any) -> Tuple[Dict[str, Any],
                                                                  Dict[str, str],
                                                                  Dict[str, Any]]:
    """Flatten a payload into canonical leaf paths.

    Returns ``(leaves, canon_by_path, raw_by_path)``.
    """
    canonical, raw_names, extras = canonicalize_payload(layer, payload)
    leaves: Dict[str, Any] = {}
    canon_by_path: Dict[str, str] = {}
    raw_by_path: Dict[str, str] = {}
    for name, value in canonical.items():
        before = set(leaves.keys())
        flatten(value, f"payload.{name}", leaves)
        for path in leaves.keys():
            if path not in before:
                canon_by_path[path] = name
                raw_by_path[path] = raw_names.get(name, name)
    for name, value in extras.items():
        before = set(leaves.keys())
        flatten(value, f"payload.{name}", leaves)
        for path in leaves.keys():
            if path not in before:
                canon_by_path[path] = name
                raw_by_path[path] = name
    return leaves, canon_by_path, raw_by_path


# ---------------------------------------------------------------------------
# Tolerance model (never hard-coded in scattered places)
# ---------------------------------------------------------------------------

# category of indices [0, split) is `first`; indices [split, ...) get the other
# of (trans, rot). Vectors follow the R4.2 / spec-2 conventions:
#   * 7-vector pose: [tx,ty,tz, qx,qy,qz,qw] -> translation first
#   * GTSAM Pose3 tangent 6-vector: [rotation(3), translation(3)] -> rotation first
_VECTOR_RULES: Dict[Tuple[str, str], Tuple[str, int]] = {
    ("factor", "measurement_se3"): ("trans", 3),
    ("factor", "noise_sigmas"): ("rot", 3),
    ("gate", "innovation6"): ("rot", 3),
    ("gate", "dT_imu_ref"): ("trans", 3),
    ("gate", "dT_source"): ("trans", 3),
    ("state", "pose"): ("trans", 3),
    ("trajectory", "pose"): ("trans", 3),
}

_EXACT_FIELD_SUFFIX = "_ns"
_EXACT_FIELDS = {"t_k", "timestamp_ns"}


def path_index(path: str) -> Optional[int]:
    """Return the list index encoded in a leaf path, e.g. ``...[4]`` -> 4."""
    if not path.endswith("]"):
        return None
    start = path.rfind("[")
    if start < 0:
        return None
    try:
        return int(path[start + 1:-1])
    except ValueError:
        return None


def path_canonical_name(path: str, canon_by_path: Optional[Dict[str, str]] = None) -> str:
    """Extract the canonical top-level payload field the leaf belongs to."""
    if canon_by_path and path in canon_by_path:
        return canon_by_path[path]
    base = path[len("payload."):] if path.startswith("payload.") else path
    stem = base.split(".")[0]
    return stem.split("[")[0]


def leaf_category(layer: Optional[str], path: str,
                  canon_by_path: Optional[Dict[str, str]] = None) -> str:
    """Classify a leaf as trans | rot | vel | bias | exact."""
    name = path_canonical_name(path, canon_by_path)
    if name in _EXACT_FIELDS or path.endswith(_EXACT_FIELD_SUFFIX):
        return "exact"
    if layer == "state" and name == "velocity":
        return "vel"
    if layer == "state" and name in ("bias_acc", "bias_gyro"):
        return "bias"
    if layer == "gate" and name == "trans_norm":
        return "trans"
    if layer == "gate" and name == "rot_norm":
        return "rot"
    rule = _VECTOR_RULES.get((layer or "", name))
    if rule is not None:
        first, split = rule
        index = path_index(path)
        if index is None:
            return first
        return first if index < split else ("rot" if first == "trans" else "trans")
    if name in ("decision", "reason", "status", "factor_type", "sensor_type",
                "action", "anchor_status", "interval_status", "fusion_segment",
                "keys", "coverage_gap", "epoch_changed", "committed", "finalized"):
        return "exact"
    return "trans"


def is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def compare_leaves(category: str, shadow_value: Any, legacy_value: Any,
                   tolerances: Dict[str, float]) -> Dict[str, Any]:
    """Compare two leaves.

    Returns ``{"ok": bool, "kind": str, "abs_diff": float|None,
    "rel_diff": float|None, "allowed": float|None}``.
    """
    result: Dict[str, Any] = {"ok": True, "kind": "equal", "abs_diff": None,
                              "rel_diff": None, "allowed": None}
    if is_number(shadow_value) and is_number(legacy_value):
        abs_diff = abs(float(shadow_value) - float(legacy_value))
        scale = max(abs(float(shadow_value)), abs(float(legacy_value)))
        rel_diff = abs_diff / scale if scale > 0 else abs_diff
        if category == "exact":
            allowed = 0.0
        else:
            allowed = max(float(tolerances.get(category, 0.0)),
                          float(tolerances.get("rel", 0.0)) * scale)
        return {"ok": abs_diff <= allowed,
                "kind": "exact" if abs_diff == 0 else "value",
                "abs_diff": abs_diff, "rel_diff": rel_diff, "allowed": allowed}
    if isinstance(shadow_value, bool) or isinstance(legacy_value, bool):
        ok = shadow_value == legacy_value
        return {**result, "ok": ok, "kind": "equal" if ok else "value"}
    if isinstance(shadow_value, str) and isinstance(legacy_value, str):
        ok = shadow_value == legacy_value
        return {**result, "ok": ok, "kind": "equal" if ok else "value"}
    if shadow_value is None and legacy_value is None:
        return result
    if shadow_value is None or legacy_value is None:
        return {**result, "ok": False, "kind": "value"}
    if type(shadow_value) is not type(legacy_value):
        return {**result, "ok": False, "kind": "type_mismatch"}
    ok = shadow_value == legacy_value
    return {**result, "ok": ok, "kind": "equal" if ok else "value"}


def path_ignored(path: str, canon_by_path: Optional[Dict[str, str]],
                 raw_by_path: Optional[Dict[str, str]],
                 patterns: Iterable[str]) -> bool:
    """Check ``--ignore-fields`` patterns against a leaf path.

    A pattern matches the full path (``payload.watermark_ns``), the path without
    the ``payload.`` prefix, the canonical field name, the name as emitted, the
    indexed leaf (``measurement_se3[0]``) or the bare field name. A trailing
    ``*`` turns the pattern into a prefix match.
    """
    pats = [p.strip() for p in patterns if p and p.strip()]
    if not pats:
        return False
    base = path[len("payload."):] if path.startswith("payload.") else path
    leaf = base.split(".")[-1]
    candidates = {path, base, leaf, leaf.split("[")[0]}
    if canon_by_path and path in canon_by_path:
        candidates.add(canon_by_path[path])
    if raw_by_path and path in raw_by_path:
        candidates.add(raw_by_path[path])
    for pat in pats:
        if pat.endswith("*"):
            prefix = pat[:-1]
            if any(candidate.startswith(prefix) for candidate in candidates):
                return True
        elif pat in candidates:
            return True
    return False


def tolerance_expectation(category: str, tolerances: Dict[str, float]) -> str:
    """Human-readable statement of the tolerance used for a leaf."""
    if category == "exact":
        return "exact (no tolerance)"
    unit = {"trans": "m", "rot": "rad", "vel": "m/s",
            "bias": "bias units"}.get(category, "")
    return f"abs_tol<= {tolerances.get(category, 0.0):g} {unit}".strip()


def record_match_key(layer: Optional[str], k: Any, source: Optional[str],
                     epoch: Any, occurrence: int) -> Tuple[Any, ...]:
    """Stable matching key used to pair shadow and legacy observations."""
    k_norm = k if isinstance(k, int) and not isinstance(k, bool) else None
    epoch_norm = epoch if isinstance(epoch, int) and not isinstance(epoch, bool) else epoch
    return (layer, k_norm, source, epoch_norm, occurrence)


def group_occurrences(records: Sequence[TraceRecord]) -> Dict[Tuple[Any, ...], TraceRecord]:
    """Map every record onto its matching key, keeping file order in ``occurrence``."""
    counters: Dict[Tuple[Any, ...], int] = {}
    keyed: Dict[Tuple[Any, ...], TraceRecord] = {}
    for rec in records:
        group = (rec.layer, rec.k if isinstance(rec.k, int) and not isinstance(rec.k, bool) else None,
                 rec.source, rec.epoch)
        occurrence = counters.get(group, 0)
        counters[group] = occurrence + 1
        keyed[record_match_key(rec.layer, rec.k, rec.source, rec.epoch, occurrence)] = rec
    return keyed


def key_sort_tuple(key: Tuple[Any, ...]) -> Tuple[int, int, str, str, int]:
    """Deterministic ordering inside one layer: k first, then source/epoch/occurrence."""
    layer, k, source, epoch, occurrence = key
    k_sort = k if isinstance(k, int) else -1
    return (k_sort, 0, str(source), str(epoch), occurrence)
