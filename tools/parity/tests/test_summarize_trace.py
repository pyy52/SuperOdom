# -*- coding: utf-8 -*-
"""Unit tests for tools/parity/summarize_trace.py (taskbook section 10)."""

from __future__ import annotations

import io
import json
import sys
import unittest
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import parity_test_utils as utils  # noqa: E402

summarize_trace = utils.summarize_trace


class FixtureSummaryTest(unittest.TestCase):
    def setUp(self) -> None:
        self.summary = summarize_trace.summarize(utils.fixture("equal_shadow.jsonl"))

    def test_record_counts(self) -> None:
        self.assertEqual(self.summary["records"], 13)
        self.assertEqual(self.summary["malformed_lines"], 0)
        self.assertEqual(self.summary["counts_by_layer"],
                         {"timeline": 5, "input": 2, "factor": 1, "gate": 1,
                          "state": 3, "trajectory": 1})
        self.assertEqual(self.summary["counts_by_type"]["TIMELINE_ANCHOR_OPEN"], 3)

    def test_timestamps_and_anchor_range(self) -> None:
        self.assertEqual(self.summary["first_timestamp_ns"], 900000000)
        self.assertEqual(self.summary["last_timestamp_ns"], 1200000000)
        self.assertEqual(self.summary["span_ns"], 300000000)
        self.assertEqual(self.summary["out_of_order_timestamps"], 3)
        self.assertEqual(self.summary["anchor_range"]["k_min"], 0)
        self.assertEqual(self.summary["anchor_range"]["k_max"], 2)
        self.assertEqual(self.summary["anchor_range"]["k_count"], 3)

    def test_epochs_and_status(self) -> None:
        self.assertEqual(self.summary["epochs"], [0])
        self.assertEqual(self.summary["anchor_status_counts"],
                         {"GRAPH_INSERTED": 5, "SOLVED": 3})

    def test_drop_reasons_and_actions(self) -> None:
        self.assertEqual(self.summary["drop_reason_counts"], {"late_after_watermark": 1})
        self.assertEqual(self.summary["input_action_counts"],
                         {"ACCEPTED": 1, "DROPPED_WATERMARK": 1})

    def test_gate_counts(self) -> None:
        self.assertEqual(self.summary["gate"]["accepted"], 1)
        self.assertEqual(self.summary["gate"]["rejected"], 0)
        self.assertEqual(self.summary["gate"]["reason_counts"], {"SUCCESS": 1})

    def test_factor_counts(self) -> None:
        self.assertEqual(self.summary["factors"]["total"], 1)
        self.assertEqual(self.summary["factors"]["by_type"], {"BetweenFactorPose3": 1})
        self.assertEqual(self.summary["factors"]["committed"], 1)
        self.assertEqual(self.summary["factors"]["finalized"], 1)

    def test_gap_counts_default(self) -> None:
        self.assertEqual(self.summary["gaps"]["gap_count"], 0)
        self.assertEqual(self.summary["gaps"]["invalid_gap_anchors"], 0)


class GapSummaryTest(unittest.TestCase):
    def test_gaps_are_counted_from_trace_evidence(self) -> None:
        records = [
            utils.record("shadow", "STATE_SNAPSHOT", 1000000000, 0, "SYSTEM", 0,
                         {"T_W_B": [0, 0, 0, 0, 0, 0, 1], "v_W": [0, 0, 0],
                          "anchor_status": "INVALID_GAP"}),
            utils.record("shadow", "INPUT_CONSUME", 900000000, None, "LIO", 0,
                         {"sensor_type": "LIO", "sample_time_ns": 900000000,
                          "action": "GAP_DETECTED", "drop_reason": "imu_gap",
                          "coverage_gap": True}),
        ]
        summary = utils.summarize_records(records)
        self.assertEqual(summary["gaps"]["invalid_gap_anchors"], 1)
        self.assertEqual(summary["gaps"]["imu_gap_drops"], 1)
        self.assertEqual(summary["gaps"]["coverage_gap_events"], 1)
        self.assertEqual(summary["gaps"]["gap_count"], 3)

    def test_timestamp_gap_count_uses_threshold(self) -> None:
        records = [
            utils.record("shadow", "TRAJECTORY_POSE", 1000000000, None, "SYSTEM", 0,
                         {"T_W_B": [0, 0, 0, 0, 0, 0, 1], "output_stamp_ns": 1000000000}),
            utils.record("shadow", "TRAJECTORY_POSE", 2000000000, None, "SYSTEM", 0,
                         {"T_W_B": [1, 0, 0, 0, 0, 0, 1], "output_stamp_ns": 2000000000}),
        ]
        path = utils.write_jsonl(records)
        default = summarize_trace.summarize(path)
        self.assertEqual(default["gaps"]["timestamp_gap_count"], 1)
        self.assertEqual(default["gaps"]["largest_timestamp_gap_ns"], 1000000000)
        relaxed = summarize_trace.summarize(path, gap_threshold_ns=5000000000)
        self.assertEqual(relaxed["gaps"]["timestamp_gap_count"], 0)

    def test_malformed_lines_are_counted_not_raised(self) -> None:
        path = utils.write_text(json.dumps(utils.record(
            "shadow", "STATE_SNAPSHOT", 1000000000, 0, "SYSTEM", 0,
            {"T_W_B": [0, 0, 0, 0, 0, 0, 1], "anchor_status": "SOLVED"})) + "\n{bad\n")
        summary = summarize_trace.summarize(path)
        self.assertEqual(summary["malformed_lines"], 1)
        self.assertEqual(summary["records"], 1)


class CliTest(unittest.TestCase):
    def test_exit_codes(self) -> None:
        with redirect_stdout(io.StringIO()):
            self.assertEqual(
                summarize_trace.main([utils.fixture("equal_shadow.jsonl")]), 0)
            self.assertEqual(summarize_trace.main(["/missing.jsonl"]), 2)

    def test_text_output_sections(self) -> None:
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            summarize_trace.main([utils.fixture("equal_shadow.jsonl")])
        output = buffer.getvalue()
        for token in ("counts by layer", "anchor range", "drop reasons",
                      "gate:", "factors:", "gaps:"):
            self.assertIn(token, output)

    def test_json_output(self) -> None:
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            code = summarize_trace.main(["--format", "json",
                                         utils.fixture("equal_shadow.jsonl")])
        payload = json.loads(buffer.getvalue())
        self.assertEqual(code, 0)
        self.assertEqual(payload[0]["records"], 13)


if __name__ == "__main__":
    unittest.main()
