"""render_graph.py: the graph file reader, where the figure goes and what it records."""

import contextlib
import io

import matplotlib

# Must precede any pyplot import.
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pytest
import yaml

from open_apero_graph_benchmark import render_graph
from open_apero_graph_benchmark.paths import INPUT_ROOT

GRAPH = "0 0.0 0.0 10.0 1\n1 2.0 0.0 30.0 0 2\n2 2.0 2.0 50.0 1\n"


@pytest.fixture(autouse=True)
def close_figures(monkeypatch):
    # A figure tool writes its file and exits; it never opens a window.
    monkeypatch.setattr(plt, "show", lambda *args, **kwargs: pytest.fail("a window opened"))
    yield
    plt.close("all")


def render(argv):
    output = io.StringIO()
    with contextlib.redirect_stdout(output):
        render_graph.main(argv)
    return output.getvalue()


def test_the_reader_keeps_ids_positions_gains_and_successors(tmp_path):
    (tmp_path / "graph.txt").write_text(GRAPH)
    graph = render_graph.load_graph(tmp_path / "graph.txt")
    assert sorted(graph) == ["0", "1", "2"]
    assert (graph["1"].x, graph["1"].y, graph["1"].gain) == (2.0, 0.0, 30.0)
    assert graph["1"].successors_id == ["0", "2"]
    assert graph["2"].successors_id == ["1"]


def test_a_missing_graph_file_raises_instead_of_drawing_nothing(tmp_path):
    with pytest.raises(FileNotFoundError):
        render_graph.load_graph(tmp_path / "graph.txt")


def test_an_explicit_output_gets_the_figure_and_its_record(tmp_path):
    (tmp_path / "graph.txt").write_text(GRAPH)
    output = tmp_path / "out" / "drawn.pdf"
    printed = render([str(tmp_path), "--output", str(output)])
    assert output.read_bytes().startswith(b"%PDF")
    assert "Wrote {}".format(output.resolve()) in printed
    record = yaml.safe_load(output.with_suffix(".yaml").read_text())
    assert record["figure"] == "drawn.pdf"
    assert record["inputs"] == [str(tmp_path / "graph.txt")]
    assert record["command"].startswith("render_graph.py ")


def test_a_dataset_name_reads_the_tracked_input_and_mirrors_the_results_tree(tmp_path):
    name = "ap_25x25_clustered_gain_0to100_cost_50/graph_0"
    render([name, "--out-dir", str(tmp_path)])
    figure = tmp_path / "graph_benchmark" / name / "graph.pdf"
    assert figure.read_bytes().startswith(b"%PDF")
    record = yaml.safe_load(figure.with_suffix(".yaml").read_text())
    assert record["inputs"] == [str(INPUT_ROOT / name / "graph.txt")]


def test_the_color_scale_follows_the_gains_unless_given(tmp_path, monkeypatch):
    (tmp_path / "graph.txt").write_text(GRAPH)
    seen = []
    original = render_graph.plot_graph

    def recording(graph, min_gain, max_gain, **kwargs):
        seen.append((min_gain, max_gain))
        return original(graph, min_gain, max_gain, **kwargs)

    monkeypatch.setattr(render_graph, "plot_graph", recording)
    render([str(tmp_path), "--output", str(tmp_path / "a.pdf")])
    render([str(tmp_path), "--output", str(tmp_path / "b.pdf"), "--min-gain", "0"])
    render([str(tmp_path), "--output", str(tmp_path / "c.pdf"), "--max-gain", "100"])
    assert seen == [(10.0, 50.0), (0.0, 50.0), (10.0, 100.0)]


def test_a_graph_with_one_gain_still_gets_a_scale(tmp_path):
    (tmp_path / "graph.txt").write_text("0 0.0 0.0 7.0 1\n1 1.0 0.0 7.0 0\n")
    output = tmp_path / "flat.pdf"
    render([str(tmp_path), "--output", str(output)])
    assert output.is_file()
