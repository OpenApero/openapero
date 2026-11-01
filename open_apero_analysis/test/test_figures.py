"""The figure tools' shared output arguments, provenance record and closing line."""

import argparse
import subprocess
from pathlib import Path
from unittest import mock

import yaml

from open_apero_analysis import figures, paths


def _parse(argv, single_file=True):
    parser = argparse.ArgumentParser()
    figures.add_figure_arguments(parser, single_file=single_file)
    return parser.parse_args(argv)


def test_the_figure_goes_below_the_figure_tree_by_default(monkeypatch):
    monkeypatch.setenv(paths.DATA_ROOT_VARIABLE, "/mnt/results")
    args = _parse([])
    figure = figures.figure_path(args, "/mnt/results/sweep/explore/HSSD/1", "curves.pdf")
    assert figure == Path("/mnt/results/figures/sweep/explore/HSSD/1/curves.pdf")


def test_out_dir_moves_the_tree_and_output_names_one_file():
    args = _parse(["--out-dir", "/tmp/figs"])
    assert figures.figure_path(args, "/elsewhere/scene", "curves.pdf") == Path(
        "/tmp/figs/elsewhere/scene/curves.pdf"
    )
    args = _parse(["--output", "/tmp/one.pdf", "--out-dir", "/tmp/figs"])
    assert figures.figure_path(args, "/elsewhere/scene", "curves.pdf") == Path("/tmp/one.pdf")


def test_a_tool_drawing_many_figures_has_no_output_argument():
    args = _parse(["--out-dir", "/tmp/figs"], single_file=False)
    assert not hasattr(args, "output")
    assert figures.figure_path(args, "/elsewhere/run", "overview.png") == Path(
        "/tmp/figs/elsewhere/run/overview.png"
    )


def test_a_figure_drawn_from_no_input_sits_at_the_top_of_the_tree(monkeypatch):
    monkeypatch.setenv(paths.DATA_ROOT_VARIABLE, "/mnt/results")
    assert figures.figure_path(_parse([]), None, "bars.pdf") == Path(
        "/mnt/results/figures/bars.pdf"
    )
    assert figures.figure_path(_parse(["--out-dir", "/tmp/figs"]), None, "bars.pdf") == Path(
        "/tmp/figs/bars.pdf"
    )


def test_the_arguments_are_absent_unless_given():
    # So a default-showing help formatter prints the defaults the help texts state.
    assert not figures.opens_viewer(_parse([]))
    assert figures.opens_viewer(_parse(["--open"]))
    assert not hasattr(_parse([]), "out_dir")


def test_command_line_quotes_the_arguments():
    line = figures.command_line("plot.py", ["explore/HSSD/1", "--y-axes", "Accumulated Gain"])
    assert line == "plot.py explore/HSSD/1 --y-axes 'Accumulated Gain'"


def test_the_record_sits_beside_the_figure_and_names_its_inputs(tmp_path):
    figure = tmp_path / "nested" / "curves.pdf"
    record = figures.write_record(
        figure, [tmp_path / "run_a", tmp_path / "run_b"], "plot.py x", extra={"task": "explore"}
    )
    assert record == tmp_path / "nested" / "curves.yaml"
    content = yaml.safe_load(record.read_text())
    assert content["figure"] == "curves.pdf"
    assert content["command"] == "plot.py x"
    assert content["inputs"] == [str(tmp_path / "run_a"), str(tmp_path / "run_b")]
    assert content["task"] == "explore"
    assert content["written_utc"].endswith("Z")


def test_finish_figure_prints_the_path_and_opens_on_request(tmp_path, capsys):
    figure = tmp_path / "overview.png"
    figure.write_bytes(b"\x89PNG")
    # The code version comes from git, which runs through Popen too; pin it to keep the viewer
    # call the only one.
    with mock.patch.object(figures, "_code_version", return_value="abc1234"), mock.patch.object(
        subprocess, "Popen"
    ) as popen:
        figures.finish_figure(figure, [tmp_path], "tool.py", open_file=False)
        assert popen.call_count == 0
        figures.finish_figure(figure, [tmp_path], "tool.py", open_file=True)
        assert popen.call_args[0][0][-1] == str(figure)
    out = capsys.readouterr().out.splitlines()
    assert out[0].startswith("Wrote %s" % figure.resolve())
    assert "overview.yaml" in out[0]
    assert yaml.safe_load((tmp_path / "overview.yaml").read_text())["code"] == "abc1234"


def test_a_missing_viewer_is_reported_not_raised(tmp_path, capsys):
    with mock.patch.object(subprocess, "Popen", side_effect=OSError("no xdg-open")):
        figures.open_in_viewer(tmp_path / "overview.png")
    assert "no xdg-open" in capsys.readouterr().err


def test_the_file_backend_draws_without_a_display():
    figures.use_file_backend()
    import matplotlib

    assert matplotlib.get_backend().lower() == "agg"
