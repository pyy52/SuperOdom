# -*- coding: utf-8 -*-
"""Unit tests for tools/parity/diff_trace.py (taskbook sections 5, 9)."""

from __future__ import annotations

import io
import json
import sys
import unittest
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import parity_test_utils as utils  # noqa: E402

diff_trace = utils.diff_trace


def run_diff(shadow: str, legacy: str, extra=()) -> dict:
    """Run the differ through the real CLI parser (mirrors main())."""
    argv = ["--shadow", shadow, "--legacy", legacy, *extra]
    args = diff_trace.build_parser().parse_args(argv)
    types = set(diff_trace._split_csv(args.type)) or None
    options = diff_trace.DiffOptions(
        ignore_fields=diff_trace._split_csv(args.ignore_fields),
        allow_extra_fields=args.allow_extra_fields,
        types=types,
        k_min=args.k_min,
        k_max=args.k_max,
        collapse=args.collapse,
        tolerances=diff_trace.build_tolerances(args),
    )
    return diff_trace.diff_traces(shadow, legacy, options,
                                  all_divergences=args.all_divergences,
                                  max_divergences=args.max_divergences)


class FixtureDiffTest(unittest.TestCase):
    def pair(self, name: str):
        return utils.fixture(f"{name}_shadow.jsonl"), utils.fixture(f"{name}_legacy.jsonl")

    def test_equal_traces_have_no_divergence(self) -> None:
        report = run_diff(*self.pair("equal"))
        self.assertFalse(report["divergence"])
        self.assertIsNone(report["first_divergence"])
        self.assertEqual(report["matched_keys"], 13)
        self.assertEqual(report["divergent_keys"], 0)

    def test_factor_difference_first_divergence_at_factor_layer(self) -> None:
        report = run_diff(*self.pair("factor_divergence"))
        self.assertEqual(report["first_divergent_layer"], "factor")
        self.assertEqual(report["first_divergent_interval_k"], 1)
        self.assertEqual(report["first_divergent_field"], "payload.measurement_se3[0]")
        self.assertEqual(report["first_divergence"]["kind"], "value_mismatch")

    def test_gate_difference_first_divergence_at_gate_layer(self) -> None:
        report = run_diff(*self.pair("gate_divergence"))
        self.assertEqual(report["first_divergent_layer"], "gate")
        self.assertEqual(report["first_divergent_field"], "payload.trans_norm")
        self.assertEqual(report["first_divergence"]["legacy"], 0.5)

    def test_state_difference_first_divergence_at_state_layer(self) -> None:
        report = run_diff(*self.pair("state_divergence"))
        self.assertEqual(report["first_divergent_layer"], "state")
        self.assertEqual(report["first_divergent_interval_k"], 0)
        self.assertEqual(report["first_divergent_field"], "payload.velocity[0]")

    def test_layer_priority_beats_interval_order(self) -> None:
        # legacy gate diverges at k=1, legacy timeline diverges at k=2
        report = run_diff(*self.pair("timeline_divergence"))
        self.assertEqual(report["first_divergent_layer"], "timeline")
        self.assertEqual(report["first_divergent_interval_k"], 2)
        gate_entry = next(entry for entry in report["per_layer"]
                          if entry["layer"] == "gate")
        self.assertEqual(gate_entry["divergent"], 1)

    def test_layer_names_and_field_aliases_are_interoperable(self) -> None:
        report = run_diff(*self.pair("layer_names"))
        self.assertFalse(report["divergence"])
        self.assertEqual(report["matched_keys"], 6)

    def test_no_alignment_is_performed(self) -> None:
        # A pure time shift must be reported, never silently compensated.
        shadow = [utils.record("shadow", "STATE_SNAPSHOT", 1000000000, 0, "SYSTEM", 0,
                               {"T_W_B": [0, 0, 0, 0, 0, 0, 1], "v_W": [0, 0, 0],
                                "anchor_status": "SOLVED"})]
        legacy = [utils.record("legacy", "STATE_SNAPSHOT", 1000000100, 0, "SYSTEM", 0,
                               {"T_W_B": [0, 0, 0, 0, 0, 0, 1], "v_W": [0, 0, 0],
                                "anchor_status": "SOLVED"})]
        report = run_diff(utils.write_jsonl(shadow), utils.write_jsonl(legacy))
        self.assertEqual(report["first_divergent_field"], "timestamp_ns")
        ignored = run_diff(utils.write_jsonl(shadow), utils.write_jsonl(legacy),
                           ["--ignore-fields", "timestamp_ns"])
        self.assertFalse(ignored["divergence"])


class OptionTest(unittest.TestCase):
    def small_divergence(self):
        """One gate field differs by 1e-5 m."""
        shadow = [utils.record("shadow", "GATE_EVALUATION", 1000000000, 0, "LIO", 0,
                               {"dT_imu_ref": 0.1, "dT_source": 0.1,
                                "innovation_6d": [0.0] * 6, "trans_norm": 0.1,
                                "rot_norm": 0.0, "decision": "ACCEPTED",
                                "reason": "SUCCESS", "lateness_ns": 0,
                                "watermark_ns": 1000000000,
                                "interpolation_brackets": [0, 0]})]
        legacy = json.loads(json.dumps(shadow))
        legacy[0]["producer"] = "legacy"
        legacy[0]["payload"]["trans_norm"] = 0.10001
        return utils.write_jsonl(shadow), utils.write_jsonl(legacy)

    def test_default_tolerance_catches_small_divergence(self) -> None:
        report = run_diff(*self.small_divergence())
        self.assertEqual(report["first_divergent_field"], "payload.trans_norm")
        self.assertEqual(report["tolerances"]["trans"], 1e-6)

    def test_abs_tol_override_relaxes_comparison(self) -> None:
        shadow, legacy = self.small_divergence()
        report = run_diff(shadow, legacy, ["--abs-tol", "1e-4"])
        self.assertFalse(report["divergence"])
        self.assertEqual(report["tolerances"]["trans"], 1e-4)

    def test_category_tolerance_override(self) -> None:
        shadow, legacy = self.small_divergence()
        self.assertTrue(run_diff(shadow, legacy,
                                 ["--trans-tol", "1e-6"])["divergence"])
        self.assertFalse(run_diff(shadow, legacy,
                                  ["--trans-tol", "1e-3"])["divergence"])

    def test_rel_tol_override(self) -> None:
        shadow, legacy = self.small_divergence()
        report = run_diff(shadow, legacy, ["--rel-tol", "1e-2"])
        self.assertFalse(report["divergence"])

    def test_ignore_fields_by_name(self) -> None:
        shadow, legacy = self.small_divergence()
        report = run_diff(shadow, legacy, ["--ignore-fields", "trans_norm"])
        self.assertFalse(report["divergence"])
        self.assertEqual(report["ignored_leaf_count"], 1)

    def test_type_filter_restricts_layers(self) -> None:
        shadow, legacy = utils.fixture("gate_divergence_shadow.jsonl"), \
            utils.fixture("gate_divergence_legacy.jsonl")
        self.assertTrue(run_diff(shadow, legacy, ["--type", "gate"])["divergence"])
        self.assertFalse(run_diff(shadow, legacy, ["--type", "state"])["divergence"])

    def test_k_range_filter(self) -> None:
        shadow, legacy = utils.fixture("gate_divergence_shadow.jsonl"), \
            utils.fixture("gate_divergence_legacy.jsonl")
        self.assertFalse(run_diff(shadow, legacy, ["--k-max", "0"])["divergence"])
        self.assertTrue(run_diff(shadow, legacy, ["--k-min", "1"])["divergence"])

    def test_allow_extra_fields(self) -> None:
        shadow = [utils.record("shadow", "STATE_SNAPSHOT", 1000000000, 0, "SYSTEM", 0,
                               {"T_W_B": [0, 0, 0, 0, 0, 0, 1], "v_W": [0, 0, 0],
                                "anchor_status": "SOLVED", "shadow_only": 1.0})]
        legacy = json.loads(json.dumps(shadow))
        legacy[0]["producer"] = "legacy"
        legacy[0]["payload"].pop("shadow_only")
        strict = run_diff(utils.write_jsonl(shadow), utils.write_jsonl(legacy))
        self.assertEqual(strict["first_divergence"]["kind"], "field_missing")
        relaxed = run_diff(utils.write_jsonl(shadow), utils.write_jsonl(legacy),
                           ["--allow-extra-fields"])
        self.assertFalse(relaxed["divergence"])

    def test_collapse_repeated_observations(self) -> None:
        shadow = [utils.record("shadow", "STATE_SNAPSHOT", 1000000000, 0, "SYSTEM", 0,
                               {"T_W_B": [0, 0, 0, 0, 0, 0, 1], "v_W": [0, 0, 0],
                                "anchor_status": "SOLVED"}),
                  utils.record("shadow", "STATE_SNAPSHOT", 1000000001, 0, "SYSTEM", 0,
                               {"T_W_B": [0, 0, 0, 0, 0, 0, 1], "v_W": [0, 0, 0],
                                "anchor_status": "SOLVED"})]
        legacy = [json.loads(json.dumps(shadow[0]))]
        legacy[0]["producer"] = "legacy"
        strict = run_diff(utils.write_jsonl(shadow), utils.write_jsonl(legacy))
        self.assertEqual(strict["first_divergence"]["kind"], "record_missing")
        collapsed = run_diff(utils.write_jsonl(shadow), utils.write_jsonl(legacy),
                             ["--collapse", "last", "--ignore-fields", "timestamp_ns"])
        self.assertFalse(collapsed["divergence"])

    def test_all_divergences_lists_every_field(self) -> None:
        report = run_diff(utils.fixture("factor_divergence_shadow.jsonl"),
                          utils.fixture("factor_divergence_legacy.jsonl"),
                          ["--all-divergences"])
        fields = {item["field"] for item in report["divergences"]}
        self.assertEqual(fields, {"payload.measurement_se3[0]", "payload.noise_sigmas[3]"})


class MissingRecordTest(unittest.TestCase):
    def test_missing_legacy_record_is_a_divergence(self) -> None:
        shadow = [utils.record("shadow", "TIMELINE_ANCHOR_CLOSE", 1100000000, 1, "IMU", 0,
                               {"t_k": 1100000000, "anchor_status": "GRAPH_INSERTED",
                                "fusion_segment_status": "ACTIVE"}),
                  utils.record("shadow", "TIMELINE_ANCHOR_CLOSE", 1200000000, 2, "IMU", 0,
                               {"t_k": 1200000000, "anchor_status": "GRAPH_INSERTED",
                                "fusion_segment_status": "ACTIVE"})]
        legacy = [json.loads(json.dumps(shadow[0]))]
        legacy[0]["producer"] = "legacy"
        report = run_diff(utils.write_jsonl(shadow), utils.write_jsonl(legacy))
        first = report["first_divergence"]
        self.assertEqual(first["kind"], "record_missing")
        self.assertEqual(first["reason"], "missing-record")
        self.assertEqual(first["missing_side"], "legacy")
        self.assertEqual(first["k"], 2)
        self.assertEqual(report["first_divergent_layer"], "timeline")

    def test_missing_shadow_record_is_reported(self) -> None:
        shadow = [utils.record("shadow", "STATE_SNAPSHOT", 1000000000, 0, "SYSTEM", 0,
                               {"T_W_B": [0, 0, 0, 0, 0, 0, 1], "v_W": [0, 0, 0],
                                "anchor_status": "SOLVED"})]
        legacy = [json.loads(json.dumps(shadow[0])),
                  utils.record("legacy", "STATE_SNAPSHOT", 1100000000, 1, "SYSTEM", 0,
                               {"T_W_B": [0, 0, 0, 0, 0, 0, 1], "v_W": [0, 0, 0],
                                "anchor_status": "SOLVED"})]
        legacy[0]["producer"] = "legacy"
        report = run_diff(utils.write_jsonl(shadow), utils.write_jsonl(legacy))
        self.assertEqual(report["first_divergence"]["missing_side"], "shadow")
        self.assertEqual(report["missing_keys"], 1)

    def test_type_mismatch_is_reported(self) -> None:
        shadow = [utils.record("shadow", "GATE_EVALUATION", 1000000000, 0, "LIO", 0,
                               {"trans_norm": 0.1, "rot_norm": 0.0,
                                "decision": "ACCEPTED", "reason": "SUCCESS"})]
        legacy = [utils.record("legacy", "GATE_EVALUATION", 1000000000, 0, "LIO", 0,
                               {"trans_norm": "0.1", "rot_norm": 0.0,
                                "decision": "ACCEPTED", "reason": "SUCCESS"})]
        report = run_diff(utils.write_jsonl(shadow), utils.write_jsonl(legacy))
        self.assertEqual(report["first_divergence"]["kind"], "type_mismatch")
        self.assertEqual(report["first_divergence"]["field"], "payload.trans_norm")

    def test_vector_vs_scalar_shape_change_is_reported(self) -> None:
        shadow = [utils.record("shadow", "STATE_SNAPSHOT", 1000000000, 0, "SYSTEM", 0,
                               {"T_W_B": [0, 0, 0, 0, 0, 0, 1], "anchor_status": "SOLVED"})]
        legacy = [utils.record("legacy", "STATE_SNAPSHOT", 1000000000, 0, "SYSTEM", 0,
                               {"T_W_B": "not-a-pose", "anchor_status": "SOLVED"})]
        report = run_diff(utils.write_jsonl(shadow), utils.write_jsonl(legacy))
        self.assertEqual(report["first_divergence"]["kind"], "field_missing")


class CliTest(unittest.TestCase):
    def _run(self, argv):
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            code = diff_trace.main(argv)
        return code, buffer.getvalue()

    def test_exit_codes_and_json_output(self) -> None:
        code, _ = self._run(["--shadow", utils.fixture("equal_shadow.jsonl"),
                             "--legacy", utils.fixture("equal_legacy.jsonl")])
        self.assertEqual(code, 0)

        code, _ = self._run(["--shadow", utils.fixture("factor_divergence_shadow.jsonl"),
                             "--legacy", utils.fixture("factor_divergence_legacy.jsonl")])
        self.assertEqual(code, 1)

        code, _ = self._run(["--shadow", "/missing.jsonl",
                             "--legacy", utils.fixture("equal_legacy.jsonl")])
        self.assertEqual(code, 2)

        code, output = self._run(["--format", "json",
                                  "--shadow", utils.fixture("gate_divergence_shadow.jsonl"),
                                  "--legacy", utils.fixture("gate_divergence_legacy.jsonl")])
        self.assertEqual(code, 1)
        payload = json.loads(output)
        self.assertEqual(payload["first_divergent_layer"], "gate")

    def test_malformed_input_returns_usage_error(self) -> None:
        broken = utils.write_text("{not json}\n")
        code, _ = self._run(["--shadow", broken,
                             "--legacy", utils.fixture("equal_legacy.jsonl")])
        self.assertEqual(code, 2)

    def test_text_report_mentions_first_divergence_and_results(self) -> None:
        _, output = self._run(["--shadow", utils.fixture("state_divergence_shadow.jsonl"),
                               "--legacy", utils.fixture("state_divergence_legacy.jsonl")])
        self.assertIn("FIRST DIVERGENCE", output)
        self.assertIn("first divergent layer", output)
        self.assertIn("RESULT: DIVERGENT", output)

    def test_output_file_is_written(self) -> None:
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "report.json"
            code, _ = self._run(["--format", "json", "--output", str(target),
                                 "--shadow", utils.fixture("equal_shadow.jsonl"),
                                 "--legacy", utils.fixture("equal_legacy.jsonl")])
            self.assertEqual(code, 0)
            self.assertTrue(target.is_file())
            self.assertFalse(json.loads(target.read_text())["divergence"])


if __name__ == "__main__":
    unittest.main()
