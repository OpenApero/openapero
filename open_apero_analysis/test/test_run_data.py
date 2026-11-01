"""Loader behavior over the current header and its manifest, plus batch accounting."""

import math
import shutil
import tempfile
from pathlib import Path

from _fixtures import current_header, manifest_for, write_run

from open_apero_analysis import (
    RunStatus,
    aggregate_report,
    load_run,
    load_runs,
)
from open_apero_analysis.run_data import REFERENCE_SCHEMA_COLUMNS


def test_header_only_run_is_skipped_not_crash():
    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        run_dir = write_run(root, header, num_rows=0)
        run = load_run(run_dir)
        assert run.status is RunStatus.SKIPPED
        assert "no data rows" in run.reason


def test_missing_csv_is_skipped():
    with tempfile.TemporaryDirectory() as root:
        empty = Path(root) / "HSSD" / "s" / "nbs" / "20260101_000000"
        empty.mkdir(parents=True)
        run = load_run(empty)
        assert run.status is RunStatus.SKIPPED


def test_undecodable_or_nul_bytes_skip_the_run():
    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        run_dir = write_run(root, header, num_rows=2)
        log = run_dir / "performance_log.csv"
        intact = log.read_bytes()
        for garbage in (b"\xff\xfe,1\n", b"1,\x00\n"):
            log.write_bytes(intact + garbage)
            run = load_run(run_dir)
            assert run.status is RunStatus.SKIPPED
            assert not run.loaded


def test_malformed_numeric_cell_skips_run_with_context():
    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        run_dir = write_run(root, header, num_rows=2)
        # Corrupt one numeric cell in the second data row.
        log = run_dir / "performance_log.csv"
        lines = log.read_text().splitlines()
        row = lines[2].split(",")
        row[0] = "not_a_number"
        lines[2] = ",".join(row)
        log.write_text("\n".join(lines) + "\n")

        run = load_run(run_dir)
        assert run.status is RunStatus.SKIPPED
        assert "malformed numeric cell" in run.reason
        assert "not_a_number" in run.reason
        assert not run.loaded


def test_empty_and_nan_cells_are_not_malformed():
    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        run_dir = write_run(root, header, num_rows=2)
        log = run_dir / "performance_log.csv"
        lines = log.read_text().splitlines()
        row = lines[1].split(",")
        row[0] = ""  # empty
        row[1] = "nan"  # explicit nan
        lines[1] = ",".join(row)
        log.write_text("\n".join(lines) + "\n")

        run = load_run(run_dir)
        assert run.loaded  # empty / 'nan' become NaN, not a skip


def test_malformed_manifest_yaml_is_distinguished_from_absent():
    from open_apero_analysis import TimerMetaState, load_timer_metadata

    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        # Malformed manifest: exists but unparseable.
        run_dir = write_run(root, header, manifest="format_version: [:\n")
        run = load_run(run_dir)
        assert run.manifest is not None and run.manifest.get("__malformed__")
        assert load_timer_metadata(run.manifest).state is TimerMetaState.INVALID
    with tempfile.TemporaryDirectory() as root:
        # Absent manifest -> None -> ABSENT.
        run = load_run(write_run(root, header))
        assert run.manifest is None
        assert load_timer_metadata(run.manifest).state is TimerMetaState.ABSENT


def test_valid_manifest_keeps_the_run_parsed_and_exposes_columns():
    from open_apero_analysis import TimerMetaState

    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        run = load_run(write_run(root, header, manifest=manifest_for(header)))
        assert run.status is RunStatus.PARSED, run.reason
        assert run.timer_metadata.state is TimerMetaState.VALID
        assert [c.name for c in run.timer_metadata.columns] == header.split(",")


def test_fingerprint_mismatch_degrades_the_run():
    from open_apero_analysis import TimerMetaState

    header = current_header()
    # A changed field with the recorded fingerprint left alone: exactly what an
    # edited or mis-versioned manifest looks like.
    columns = [(name, "double", "s", "") for name in header.split(",")]
    stale = manifest_for(header)  # fingerprint over the unmodified columns
    columns[0] = (columns[0][0], "int", "count", "")
    tampered = manifest_for(
        header, columns=columns, fingerprint=stale.split('schema_fingerprint: "')[1].split('"')[0]
    )

    with tempfile.TemporaryDirectory() as root:
        run = load_run(write_run(root, header, manifest=tampered))
        assert run.status is RunStatus.DEGRADED
        assert "no valid manifest" in run.reason
        assert "fingerprint" in run.reason
        assert run.timer_metadata.state is TimerMetaState.INVALID
        # The numbers themselves still loaded: only their description is suspect.
        assert run.num_rows == 2 and run.columns


def test_reordered_manifest_columns_degrade_the_run():
    header = current_header()
    names = header.split(",")
    reordered = [(name, "double", "s", "") for name in [names[1], names[0]] + names[2:]]
    # Self-consistent manifest (fingerprint recomputed) that disagrees with the
    # CSV it describes, so only the header cross-check can catch it.
    with tempfile.TemporaryDirectory() as root:
        run = load_run(write_run(root, header, manifest=manifest_for(header, columns=reordered)))
        assert run.status is RunStatus.DEGRADED
        assert "disagree with CSV header" in run.reason


def test_malformed_manifest_degrades_the_run():
    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        run = load_run(write_run(root, header, manifest="format_version: [:\n"))
        assert run.status is RunStatus.DEGRADED
        assert "failed to parse" in run.reason


def test_unsupported_future_format_version_degrades_the_run():
    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        run = load_run(write_run(root, header, manifest=manifest_for(header, format_version=99)))
        assert run.status is RunStatus.DEGRADED
        assert "format_version" in run.reason


def test_non_integer_manifest_version_degrades_without_raising():
    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        run = load_run(write_run(root, header, manifest=manifest_for(header, format_version="[1]")))
        assert run.status is RunStatus.DEGRADED
        assert "no valid manifest" in run.reason
        assert "format_version is not an integer" in run.reason


def test_bad_manifest_version_does_not_abort_batch_load():
    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        write_run(
            root,
            header,
            rel=("HSSD", "bad", "nbs", "20260101_000000"),
            manifest=manifest_for(header, format_version="[1]"),
        )
        write_run(
            root,
            header,
            rel=("HSSD", "good", "nbs", "20260101_000000"),
            manifest=manifest_for(header),
        )

        runs = load_runs(root)
        assert len(runs) == 2
        by_scene = {run.scene: run for run in runs}
        assert by_scene["bad"].status is RunStatus.DEGRADED
        assert by_scene["good"].status is RunStatus.PARSED


def test_a_run_without_a_valid_manifest_is_degraded_but_loaded():
    from open_apero_analysis import TimerMetaState

    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        run = load_run(write_run(root, header, num_rows=3))
        assert run.status is RunStatus.DEGRADED
        assert "no valid manifest: manifest is absent" in run.reason
        assert run.timer_metadata.state is TimerMetaState.ABSENT
        assert run.num_rows == 3 and all(len(values) == 3 for values in run.columns.values())
    # The writer always records the column schema.
    with tempfile.TemporaryDirectory() as root:
        run = load_run(write_run(root, header, manifest="format_version: 1\n"))
        assert run.status is RunStatus.DEGRADED
        assert "no column schema" in run.reason
        assert run.timer_metadata.state is TimerMetaState.INVALID


def test_current_header_with_its_manifest_is_parsed():
    from open_apero_analysis import TimerMetaState

    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        run = load_run(write_run(root, header, num_rows=3, manifest=manifest_for(header)))
        assert run.status is RunStatus.PARSED, run.reason
        assert run.timer_metadata.state is TimerMetaState.VALID
        assert not run.missing_columns(REFERENCE_SCHEMA_COLUMNS)
        assert run.num_rows == 3 and all(len(values) == 3 for values in run.columns.values())
        assert run.has_columns(["TotalCost", "CommandsDispatched", "FramesMapped", "RosTimeSec"])
        assert run.has_columns(["planning.StartNodeUpdate", "backtracking.TimesSelected"])


def test_layer_instances_the_manifest_lists_set_the_reference_columns():
    layers = {"explore": "planning", "recovery": "backtracking"}
    header = current_header(layers)
    with tempfile.TemporaryDirectory() as root:
        run = load_run(write_run(root, header, manifest=manifest_for(header, layers=layers)))
        assert run.status is RunStatus.PARSED, run.reason
    # Without a manifest the shipped instance names are the reference.
    with tempfile.TemporaryDirectory() as root:
        run = load_run(write_run(root, header))
        assert run.status is RunStatus.DEGRADED
        assert "planning.NumExpansions" in run.reason


def test_path_derived_identity():
    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        run = load_run(write_run(root, header, rel=("HSSD", "102344280", "spt", "20260101_120000")))
        assert run.scene == "102344280"
        assert run.algorithm == "spt"
        assert run.timestamp == "20260101_120000"
        assert run.rel_id == "HSSD/102344280/spt/20260101_120000"


def test_metadata_from_rosparams():
    header = current_header()
    rosparams = (
        "open_apero_node:\n" + "  cost_computer: {type: TrajectoryTime}\n" + "  cost_budget: 600\n"
    )
    with tempfile.TemporaryDirectory() as root:
        run = load_run(write_run(root, header, rosparams=rosparams))
        assert run.cost_computer_type == "TrajectoryTime"
        assert run.cost_budget == 600


EPISODE_SUMMARY_FIXTURE = Path(__file__).parent / "fixtures" / "episode_summary.yaml"


def test_episode_summary_is_parsed_with_an_unbounded_budget():
    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        run_dir = write_run(root, header)
        shutil.copyfile(str(EPISODE_SUMMARY_FIXTURE), str(run_dir / "episode_summary.yaml"))
        summary = load_run(run_dir).episode_summary
        assert summary["format_version"] == 1
        assert summary["end_reason"] == "budget_exhausted"
        assert summary["cycles"] == 42
        assert summary["total_cost"] == 601.25
        assert math.isinf(summary["cost_budget"]) and summary["cost_budget"] > 0
        assert summary["real_accumulated_gain"] == {"planning": 1234.5}


def test_episode_summary_is_none_without_the_file():
    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        assert load_run(write_run(root, header)).episode_summary is None


def test_aggregate_accounting_counts_every_status():
    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        write_run(root, header, rel=("HSSD", "s", "alg", "run_00"), manifest=manifest_for(header))
        write_run(root, header, rel=("HSSD", "s", "alg", "run_01"))
        write_run(root, header, rel=("HSSD", "s", "alg", "run_02"), num_rows=0)
        runs = load_runs(root)

    assert [run.status for run in runs] == [
        RunStatus.PARSED,
        RunStatus.DEGRADED,
        RunStatus.SKIPPED,
    ]
    report = aggregate_report(runs)
    assert "Discovered 3 run(s)" in report
    for status in ("parsed:", "degraded:", "skipped:"):
        assert "  %-8s 1" % status in report


def test_dedup_across_roots_earlier_root_wins():
    header = current_header()
    with tempfile.TemporaryDirectory() as root_a, tempfile.TemporaryDirectory() as root_b:
        rel = ("HSSD", "s", "nbs", "20260101_000000")
        write_run(root_a, header, num_rows=5, rel=rel)
        write_run(root_b, header, num_rows=9, rel=rel)
        runs = load_runs([root_a, root_b])
        # Same rel_id in both roots collapses to one; the first root wins.
        assert len(runs) == 1
        assert runs[0].num_rows == 5
        assert runs[0].shadowed == [Path(root_b).joinpath(*rel)]
        assert any("duplicate rel_id" in note for note in runs[0].notes)
        # The skipped duplicate is still counted, so the report adds up.
        report = aggregate_report(runs)
        assert "Discovered 2 run(s)" in report
        assert "shadowed: 1" in report


def test_runs_under_one_root_are_never_deduplicated():
    # Two trees above the task folders, whose runs can share a timestamp.
    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        for variant in ("a", "b"):
            write_run(
                root,
                header,
                rel=(variant, "explore", "HSSD", "s", "nbs", "20260101_000000"),
                manifest=manifest_for(header),
            )
        runs = load_runs(root)
        assert len({run.rel_id for run in runs}) == 1
        assert sorted(run.path.parts[-6] for run in runs) == ["a", "b"]
        assert all(not run.shadowed and not run.notes for run in runs)
        assert "shadowed" not in aggregate_report(runs)


def test_sweep_layout_identity_includes_the_task():
    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        rel = ("recon", "HSSD", "102344280", "spt", "20260101_120000")
        run = load_run(write_run(root, header, rel=rel))
        assert run.task == "recon"
        assert run.scene == "102344280"
        assert run.algorithm == "spt"
        assert run.rel_id == "recon/HSSD/102344280/spt/20260101_120000"


def test_layout_without_a_task_folder_has_no_task():
    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        # The folder above the experiment is an arbitrary data root, not a task.
        rel = ("open_apero_data", "HSSD", "102344280", "spt", "20260101_120000")
        run = load_run(write_run(root, header, rel=rel))
        assert run.task is None
        assert run.rel_id == "HSSD/102344280/spt/20260101_120000"


def test_dedup_keeps_same_named_runs_of_different_tasks():
    header = current_header()
    with tempfile.TemporaryDirectory() as root:
        for task in ("explore", "recon", "points"):
            write_run(
                root,
                header,
                rel=(task, "HSSD", "s", "nbs", "20260101_000000"),
                manifest=manifest_for(header),
            )
        runs = load_runs(root)
        assert sorted(run.task for run in runs) == ["explore", "points", "recon"]
        assert all(not run.notes for run in runs)


def test_dedup_across_sweep_roots_matches_on_the_task():
    header = current_header()
    with tempfile.TemporaryDirectory() as root_a, tempfile.TemporaryDirectory() as root_b:
        rel = ("points", "HSSD", "s", "nbs", "20260101_000000")
        write_run(root_a, header, num_rows=5, rel=rel)
        write_run(root_b, header, num_rows=9, rel=rel)
        write_run(root_b, header, rel=("explore",) + rel[1:])
        runs = load_runs([root_a, root_b])
        assert [(run.task, run.num_rows) for run in runs] == [("points", 5), ("explore", 2)]
        assert any("duplicate rel_id" in note for note in runs[0].notes)
