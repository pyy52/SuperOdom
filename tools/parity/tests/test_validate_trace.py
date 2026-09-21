# -*- coding: utf-8 -*-
"""Unit tests for tools/parity/validate_trace.py (taskbook section 8)."""

from __future__ import annotations

import io
import json
import sys
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import parity_test_utils as utils  # noqa: E402

validate_trace = utils.validate_trace


def codes(report) -> set:
    return {item["code"] for item in report.errors}


def warn_codes(report) -> set:
    return {item["code"] for item in report.warnings}


BASELINE = [
    utils.record("shadow", "TIMELINE_ANCHOR_OPEN", 1000000000, 0, "IMU", 0,
                 {"t_k": 1000000000, "anchor_status": "GRAPH_INSERTED",
                  "fusion_segment_status": "ACTIVE"}),
    utils.record("shadow", "GATE_EVALUATION", 1100000000, 0, "LIO", 0,
                 {"dT_imu_ref": 0.1, "dT_source": 0.1, "innovation_6d": [0, 0, 0, 0, 0, 0],
                  "translation_norm": 0.0, "rotation_norm": 0.0, "decision": "ACCEPTED",
                  "reason": "SUCCESS", "lateness_ns": 0, "watermark_ns": 1200000000,
                  "interpolation_brackets": [1000000000, 1100000000]}),
    utils.record("shadow", "FACTOR_INSERT", 1100000000, 0, "LIO", 0,
                 {"factor_type": "BetweenFactorPose3", "keys": ["X_0", "X_1"],
                  "measurement_T_Bi_Bj": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                  "noise_diag": [0.02, 0.02, 0.02, 0.05, 0.05, 0.05],
                  "committed": True, "finalized": True, "status": "FINALIZED"}),
    utils.record("shadow", "STATE_SNAPSHOT", 1000000000, 0, "SYSTEM", 0,
                 {"T_W_B": [0, 0, 0, 0, 0, 0, 1], "v_W": [0, 0, 0],
                  "bias_acc": [0, 0, 0], "bias_gyro": [0, 0, 0],
                  "anchor_status": "SOLVED"}),
]


class FixtureValidationTest(unittest.TestCase):
    """Every committed fixture must be a valid trace."""

    def test_all_fixtures_validate(self) -> None:
        fixtures = sorted(utils.FIXTURES_DIR.glob("*.jsonl"))
        self.assertGreaterEqual(len(fixtures), 12)
        for path in fixtures:
            with self.subTest(fixture=path.name):
                report = validate_trace.validate_file(str(path), utils.validate_args())
                self.assertTrue(report.ok,
                                f"{path.name}: {[e['message'] for e in report.errors]}")

    def test_fixture_counts(self) -> None:
        report = validate_trace.validate_file(utils.fixture("equal_shadow.jsonl"),
                                              utils.validate_args())
        self.assertEqual(report.records, 13)
        self.assertEqual(report.observed_producer, "shadow")
        self.assertEqual(report.observed_schema_version, 1)
        self.assertEqual(report.counts_by_layer["timeline"], 5)
        self.assertEqual(report.counts_by_layer["gate"], 1)


class RecordLevelTest(unittest.TestCase):
    def test_baseline_is_valid(self) -> None:
        report = utils.validate_records(BASELINE)
        self.assertTrue(report.ok, [e["message"] for e in report.errors])

    def test_malformed_json_line_is_reported(self) -> None:
        path = utils.write_text(json.dumps(BASELINE[0]) + "\nnot json\n")
        report = validate_trace.validate_file(path, utils.validate_args())
        self.assertIn("jsonl", codes(report))
        self.assertEqual(report.malformed_lines, 1)

    def test_missing_required_field_lenient_vs_strict(self) -> None:
        records = [dict(BASELINE[0])]
        records[0].pop("producer")
        records[0].pop("schema_version")
        lenient = utils.validate_records(records)
        self.assertTrue(lenient.ok)
        self.assertIn("producer", warn_codes(lenient))
        strict = utils.validate_records(records, strict=True)
        self.assertFalse(strict.ok)
        self.assertIn("missing_field", codes(strict))

    def test_producer_can_be_supplied_by_flag(self) -> None:
        records = [dict(BASELINE[0])]
        records[0].pop("producer")
        report = utils.validate_records(records, producer="legacy")
        self.assertTrue(report.ok, [e["message"] for e in report.errors])
        self.assertEqual(report.observed_producer, "legacy")

    def test_invalid_producer_is_error(self) -> None:
        records = [dict(BASELINE[0])]
        records[0]["producer"] = "vins"
        self.assertIn("producer", codes(utils.validate_records(records)))

    def test_unknown_type_is_error(self) -> None:
        records = [dict(BASELINE[0])]
        records[0]["type"] = "SOMETHING_ELSE"
        self.assertIn("type", codes(utils.validate_records(records)))

    def test_timestamp_ns_must_be_integer(self) -> None:
        records = [dict(BASELINE[0])]
        records[0]["timestamp_ns"] = 1000000000.5
        self.assertIn("timestamp_ns", codes(utils.validate_records(records)))

    def test_negative_k_is_error_and_null_k_is_accepted(self) -> None:
        negative = [dict(BASELINE[0])]
        negative[0]["k"] = -1
        self.assertIn("k", codes(utils.validate_records(negative)))
        null_k = [dict(BASELINE[0])]
        null_k[0]["k"] = None
        self.assertTrue(utils.validate_records(null_k).ok)

    def test_unknown_source_is_error(self) -> None:
        records = [dict(BASELINE[0])]
        records[0]["source"] = "GNSS"
        self.assertIn("source", codes(utils.validate_records(records)))

    def test_unknown_reject_reason_is_error_unless_allowed(self) -> None:
        records = [json.loads(json.dumps(BASELINE[1]))]
        records[0]["payload"]["reason"] = "MYSTERY"
        self.assertIn("unknown_reason", codes(utils.validate_records(records)))
        relaxed = utils.validate_records(records, allow_unknown_reasons=True)
        self.assertTrue(relaxed.ok)
        self.assertIn("unknown_reason", warn_codes(relaxed))

    def test_unsupported_schema_version_is_error(self) -> None:
        records = [dict(BASELINE[0])]
        records[0]["schema_version"] = 2
        self.assertIn("schema_version", codes(utils.validate_records(records)))
        allowed = utils.validate_records(records, allow_schema_version=[1, 2])
        self.assertTrue(allowed.ok, [e["message"] for e in allowed.errors])


class PayloadConsistencyTest(unittest.TestCase):
    def test_factor_finalized_without_committed_is_error(self) -> None:
        records = [json.loads(json.dumps(BASELINE[2]))]
        records[0]["payload"]["committed"] = False
        self.assertIn("factor_consistency", codes(utils.validate_records(records)))

    def test_committed_and_finalized_in_one_record_is_accepted(self) -> None:
        record = json.loads(json.dumps(BASELINE[2]))
        self.assertTrue(utils.validate_records([record]).ok)

    def test_factor_noise_must_have_six_entries(self) -> None:
        records = [json.loads(json.dumps(BASELINE[2]))]
        records[0]["payload"]["noise_diag"] = [0.02, 0.02, 0.02, 0.05, 0.05]
        self.assertIn("factor_noise", codes(utils.validate_records(records)))

    def test_factor_measurement_must_have_seven_entries(self) -> None:
        records = [json.loads(json.dumps(BASELINE[2]))]
        records[0]["payload"]["measurement_T_Bi_Bj"] = [0.1, 0.0, 0.0]
        self.assertIn("factor_measurement", codes(utils.validate_records(records)))

    def test_factor_keys_must_be_pair(self) -> None:
        records = [json.loads(json.dumps(BASELINE[2]))]
        records[0]["payload"]["keys"] = ["X_0"]
        self.assertIn("factor_keys", codes(utils.validate_records(records)))

    def test_gate_decision_vocabulary(self) -> None:
        records = [json.loads(json.dumps(BASELINE[1]))]
        records[0]["payload"]["decision"] = "MAYBE"
        self.assertIn("gate_decision", codes(utils.validate_records(records)))


class LifecycleTest(unittest.TestCase):
    def test_anchor_status_must_not_go_backwards(self) -> None:
        solved = json.loads(json.dumps(BASELINE[3]))
        backwards = json.loads(json.dumps(BASELINE[3]))
        backwards["payload"]["anchor_status"] = "SCHEDULED"
        backwards["timestamp_ns"] = 1000000001
        self.assertIn("lifecycle", codes(utils.validate_records([solved, backwards])))

    def test_terminal_anchor_status_is_sticky(self) -> None:
        gap = utils.record("shadow", "STATE_SNAPSHOT", 1000000000, 0, "SYSTEM", 0,
                           {"T_W_B": [0, 0, 0, 0, 0, 0, 1], "v_W": [0, 0, 0],
                            "bias_acc": [0, 0, 0], "bias_gyro": [0, 0, 0],
                            "anchor_status": "INVALID_GAP"})
        solved = json.loads(json.dumps(BASELINE[3]))
        solved["timestamp_ns"] = 1000000001
        report = utils.validate_records([gap, solved])
        self.assertIn("lifecycle", codes(report))

    def test_gate_slot_is_one_shot(self) -> None:
        rejected = utils.record("shadow", "GATE_EVALUATION", 1100000000, 0, "LIO", 0,
                                {"dT_imu_ref": 0.1, "dT_source": 0.1,
                                 "innovation_6d": [0, 0, 0, 0, 0, 0],
                                 "translation_norm": 0.0, "rotation_norm": 0.0,
                                 "decision": "REJECTED", "reason": "TOO_LATE",
                                 "lateness_ns": 900000000, "watermark_ns": 1,
                                 "interpolation_brackets": [0, 0]})
        accepted = json.loads(json.dumps(BASELINE[1]))
        accepted["timestamp_ns"] = 1100000001
        self.assertIn("gate_one_shot", codes(utils.validate_records([rejected, accepted])))

    def test_duplicate_reject_after_accept_is_allowed(self) -> None:
        accepted = json.loads(json.dumps(BASELINE[1]))
        duplicate = json.loads(json.dumps(BASELINE[1]))
        duplicate["timestamp_ns"] = 1100000001
        duplicate["payload"]["decision"] = "REJECTED"
        duplicate["payload"]["reason"] = "REJECT_DUPLICATE"
        report = utils.validate_records([accepted, duplicate])
        self.assertTrue(report.ok, [e["message"] for e in report.errors])

    def test_out_of_order_timestamps_are_information_not_error(self) -> None:
        first = json.loads(json.dumps(BASELINE[0]))
        second = json.loads(json.dumps(BASELINE[1]))
        second["timestamp_ns"] = first["timestamp_ns"] - 5000000
        report = utils.validate_records([first, second])
        self.assertTrue(report.ok, [e["message"] for e in report.errors])
        self.assertEqual(report.info["out_of_order_timestamps"], 1)


class CliTest(unittest.TestCase):
    def test_exit_codes(self) -> None:
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertEqual(validate_trace.main([utils.fixture("equal_shadow.jsonl")]), 0)
            bad = utils.write_text("{}\n")
            self.assertEqual(validate_trace.main([bad]), 1)
            self.assertEqual(validate_trace.main(["/nonexistent/trace.jsonl"]), 2)

    def test_json_report_format(self) -> None:
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            code = validate_trace.main(["--format", "json",
                                        utils.fixture("equal_shadow.jsonl")])
        payload = json.loads(buffer.getvalue())
        self.assertEqual(code, 0)
        self.assertTrue(payload["ok"])
        self.assertEqual(payload["reports"][0]["records"], 13)


if __name__ == "__main__":
    unittest.main()
