#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Deterministic generator for the parity trace fixtures.

Run from anywhere::

    python3 tools/parity/fixtures/make_fixtures.py

Every fixture pair is written with the *same* byte layout on both sides except
for the single divergence it is meant to demonstrate, so the expected
"first divergence" of each pair is unambiguous:

* ``equal_*``               -> no divergence at all
* ``factor_divergence_*``   -> first divergence in layer ``factor``
* ``gate_divergence_*``     -> first divergence in layer ``gate``
* ``state_divergence_*``    -> first divergence in layer ``state``
* ``timeline_divergence_*`` -> gate diverges at k=1, timeline at k=2; the
  prescribed layer order (timeline before gate) must still report timeline
* ``layer_names_*``         -> taskbook layer-level types with canonical field
  spellings on the legacy side and emitted spellings on the shadow side

The fixtures are synthetic, KB-sized and contain no ROS message dumps.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Tuple

HERE = Path(__file__).resolve().parent


def _rec(schema_version: int, producer: str, type_name: str, stamp_ns: int,
         k: Any, source: str, epoch: int, payload: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "schema_version": schema_version,
        "producer": producer,
        "type": type_name,
        "timestamp_ns": stamp_ns,
        "k": k,
        "source": source,
        "epoch": epoch,
        "payload": payload,
    }


def baseline(producer: str) -> List[Dict[str, Any]]:
    """Emitted-vocabulary baseline trace (mirrors the real tracer field names)."""
    return [
        _rec(1, producer, "TIMELINE_ANCHOR_OPEN", 1000000000, 0, "IMU", 0,
             {"t_k": 1000000000, "anchor_status": "GRAPH_INSERTED",
              "fusion_segment_status": "ACTIVE"}),
        _rec(1, producer, "TIMELINE_ANCHOR_CLOSE", 1000000000, 0, "IMU", 0,
             {"t_k": 1000000000, "anchor_status": "GRAPH_INSERTED",
              "fusion_segment_status": "ACTIVE"}),
        _rec(1, producer, "STATE_SNAPSHOT", 1000000000, 0, "SYSTEM", 0,
             {"T_W_B": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], "v_W": [0.0, 0.0, 0.0],
              "bias_acc": [0.0, 0.0, 0.0], "bias_gyro": [0.0, 0.0, 0.0],
              "anchor_status": "SOLVED"}),
        _rec(1, producer, "TIMELINE_ANCHOR_OPEN", 1100000000, 1, "IMU", 0,
             {"t_k": 1100000000, "anchor_status": "GRAPH_INSERTED",
              "fusion_segment_status": "ACTIVE"}),
        _rec(1, producer, "TIMELINE_ANCHOR_CLOSE", 1100000000, 1, "IMU", 0,
             {"t_k": 1100000000, "anchor_status": "GRAPH_INSERTED",
              "fusion_segment_status": "ACTIVE"}),
        _rec(1, producer, "INPUT_CONSUME", 1100000000, None, "LIO", 0,
             {"sensor_type": "LIO", "sample_time_ns": 1100000000, "action": "ACCEPTED",
              "drop_reason": None, "coverage_gap": False, "epoch_changed": False}),
        _rec(1, producer, "GATE_EVALUATION", 1200000000, 1, "LIO", 0,
             {"dT_imu_ref": 0.1, "dT_source": 0.1,
              "innovation_6d": [0.0, 0.0, 0.0, 0.004, 0.005, 0.002],
              "translation_norm": 0.0067082, "rotation_norm": 0.0,
              "thresholds": {"trans_m": 1.0, "rot_rad": 0.35},
              "decision": "ACCEPTED", "reason": "SUCCESS", "lateness_ns": 0,
              "watermark_ns": 1400000000,
              "interpolation_brackets": [1100000000, 1200000000]}),
        _rec(1, producer, "FACTOR_INSERT", 1200000000, 1, "LIO", 0,
             {"factor_type": "BetweenFactorPose3", "keys": ["X_1", "X_2"],
              "measurement_T_Bi_Bj": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
              "noise_diag": [0.02, 0.02, 0.02, 0.05, 0.05, 0.05],
              "insertion_time_ns": 1200000000, "committed": True, "finalized": True,
              "status": "FINALIZED"}),
        _rec(1, producer, "STATE_SNAPSHOT", 1100000000, 1, "SYSTEM", 0,
             {"T_W_B": [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], "v_W": [0.1, 0.0, 0.0],
              "bias_acc": [0.001, 0.0, 0.0], "bias_gyro": [0.0, 0.0, 0.0],
              "anchor_status": "SOLVED"}),
        _rec(1, producer, "TIMELINE_ANCHOR_OPEN", 1200000000, 2, "IMU", 0,
             {"t_k": 1200000000, "anchor_status": "GRAPH_INSERTED",
              "fusion_segment_status": "ACTIVE"}),
        _rec(1, producer, "STATE_SNAPSHOT", 1200000000, 2, "SYSTEM", 0,
             {"T_W_B": [0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], "v_W": [0.1, 0.0, 0.0],
              "bias_acc": [0.001, 0.0, 0.0], "bias_gyro": [0.0, 0.0, 0.0],
              "anchor_status": "SOLVED"}),
        _rec(1, producer, "TRAJECTORY_POSE", 1150000000, None, "SYSTEM", 0,
             {"T_W_B": [0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
              "output_stamp_ns": 1150000000}),
        _rec(1, producer, "INPUT_CONSUME", 900000000, None, "LIO", 0,
             {"sensor_type": "LIO", "sample_time_ns": 900000000,
              "action": "DROPPED_WATERMARK", "drop_reason": "late_after_watermark",
              "coverage_gap": False, "epoch_changed": False}),
    ]


def layer_named(producer: str, canonical_fields: bool) -> List[Dict[str, Any]]:
    """Layer-level event names; the two sides spell the same fields differently."""
    pose = "pose" if canonical_fields else "T_W_B"
    innovation = "innovation6" if canonical_fields else "innovation_6d"
    measurement = "measurement_se3" if canonical_fields else "measurement_T_Bi_Bj"
    sigmas = "noise_sigmas" if canonical_fields else "noise_diag"
    return [
        _rec(1, producer, "timeline", 1000000000, 0, "imu", 0,
             {"t_k": 1000000000, "anchor_status": "GRAPH_INSERTED",
              "interval_status": "OPEN", "fusion_segment": "ACTIVE"}),
        _rec(1, producer, "input", 1000000000, None, "lio", 0,
             {"sensor_type": "LIO", "sample_time_ns": 1000000000,
              "drop_reason": None, "coverage_gap": False}),
        _rec(1, producer, "gate", 1100000000, 0, "lio", 0,
             {"dT_imu_ref": 0.1, "dT_source": 0.1,
              innovation: [0.0, 0.0, 0.0, 0.004, 0.005, 0.002],
              "trans_norm": 0.0067082, "rot_norm": 0.0,
              "decision": "ACCEPTED", "reason": "SUCCESS",
              "left_bracket_ns": 1000000000, "right_bracket_ns": 1100000000,
              "watermark_ns": 1300000000}),
        _rec(1, producer, "factor", 1100000000, 0, "lio", 0,
             {"factor_type": "BetweenFactorPose3", "key_i": 0, "key_j": 1,
              measurement: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
              sigmas: [0.02, 0.02, 0.02, 0.05, 0.05, 0.05],
              "committed": True, "finalized": True}),
        _rec(1, producer, "state", 1100000000, 1, "central", 0,
             {pose: [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
              "velocity": [0.1, 0.0, 0.0], "bias_acc": [0.001, 0.0, 0.0],
              "bias_gyro": [0.0, 0.0, 0.0], "anchor_status": "SOLVED"}),
        _rec(1, producer, "trajectory", 1050000000, None, "central", 0,
             {pose: [0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
              "output_stamp_ns": 1050000000}),
    ]


def write(name: str, records: List[Dict[str, Any]]) -> None:
    path = HERE / name
    with path.open("w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record, separators=(",", ":")) + "\n")


def main() -> None:
    pairs: List[Tuple[str, List[Dict[str, Any]]]] = []

    pairs.append(("equal_shadow.jsonl", baseline("shadow")))
    pairs.append(("equal_legacy.jsonl", baseline("legacy")))

    # factor layer divergence: measurement and noise both move
    factor_legacy = baseline("legacy")
    factor_legacy[7]["payload"]["measurement_T_Bi_Bj"][0] = 0.101
    factor_legacy[7]["payload"]["noise_diag"][3] = 0.0501
    pairs.append(("factor_divergence_shadow.jsonl", baseline("shadow")))
    pairs.append(("factor_divergence_legacy.jsonl", factor_legacy))

    # gate layer divergence only (k=1)
    gate_legacy = baseline("legacy")
    gate_legacy[6]["payload"]["translation_norm"] = 0.5
    gate_legacy[6]["payload"]["decision"] = "REJECTED"
    gate_legacy[6]["payload"]["reason"] = "INNOVATION_TOO_LARGE_TRANS"
    pairs.append(("gate_divergence_shadow.jsonl", baseline("shadow")))
    pairs.append(("gate_divergence_legacy.jsonl", gate_legacy))

    # state layer divergence only (k=0 velocity)
    state_legacy = baseline("legacy")
    state_legacy[2]["payload"]["v_W"] = [0.5, 0.0, 0.0]
    pairs.append(("state_divergence_shadow.jsonl", baseline("shadow")))
    pairs.append(("state_divergence_legacy.jsonl", state_legacy))

    # layer priority: gate diverges at k=1, timeline at k=2 -> timeline wins
    timeline_legacy = baseline("legacy")
    timeline_legacy[6]["payload"]["translation_norm"] = 0.5
    timeline_legacy[9]["payload"]["fusion_segment_status"] = "BROKEN"
    pairs.append(("timeline_divergence_shadow.jsonl", baseline("shadow")))
    pairs.append(("timeline_divergence_legacy.jsonl", timeline_legacy))

    pairs.append(("layer_names_shadow.jsonl", layer_named("shadow", False)))
    pairs.append(("layer_names_legacy.jsonl", layer_named("legacy", True)))

    for name, records in pairs:
        write(name, records)
        print(f"wrote fixtures/{name} ({len(records)} records)")


if __name__ == "__main__":
    main()
