"""The input generators: what they generate, and that they write below the data root, not data/."""

import contextlib
import io
import itertools
import math
from unittest import mock

from open_apero_graph_benchmark import (
    generate_clustered_gain_model,
    generate_random_graph_topology,
)
from open_apero_graph_benchmark.paths import INPUT_ROOT
from open_apero_graph_benchmark.render_graph import load_graph

topology = generate_random_graph_topology


def tracked_inputs():
    return {path: path.read_bytes() for path in sorted(INPUT_ROOT.glob("*.txt"))}


def run_quietly(main, argv):
    with contextlib.redirect_stdout(io.StringIO()) as output:
        main(argv)
    return output.getvalue()


def test_generators_default_to_the_generated_folder(tmp_path):
    before = tracked_inputs()
    generated = tmp_path / "graph_benchmark" / "generated"
    for module in (generate_clustered_gain_model, generate_random_graph_topology):
        with mock.patch.object(module, "GENERATED_ROOT", generated):
            # A small topology keeps the rejection sampling fast.
            with mock.patch.object(topology, "NUM_NODES", 20):
                output = run_quietly(module.main, [])
        written = generated / module.OUTPUT_NAME
        assert written.read_text().strip()
        assert str(written) in output
    assert tracked_inputs() == before


def test_generators_write_their_output_argument(tmp_path):
    output = tmp_path / "nested" / "gain_model.txt"
    run_quietly(generate_clustered_gain_model.main, ["--output", str(output)])
    # Eight regions, then the all-zero fallback.
    lines = output.read_text().splitlines()
    assert len(lines) == 9
    assert lines[-1] == "0 0 0 0 0"


def test_topology_nodes_start_at_the_origin_and_keep_their_spacing():
    with mock.patch.object(topology, "NUM_NODES", 40):
        nodes = topology.generate_nodes()
    assert nodes[0] == (0.0, 0.0)
    assert 1 < len(nodes) <= 40
    for x, y in nodes:
        assert topology.X_MIN <= x <= topology.X_MAX
        assert topology.Y_MIN <= y <= topology.Y_MAX
    for first, second in itertools.combinations(nodes, 2):
        assert math.dist(first, second) >= topology.MIN_DISTANCE


def test_topology_connects_exactly_the_pairs_within_reach_both_ways():
    nodes = [(0.0, 0.0), (2.0, 0.0), (4.5, 0.0), (20.0, 20.0)]
    graph = topology.build_adjacency(nodes)
    assert graph == {0: [1], 1: [0, 2], 2: [1], 3: []}
    for first, second in itertools.combinations(range(len(nodes)), 2):
        within = math.dist(nodes[first], nodes[second]) <= topology.MAX_DISTANCE
        assert (second in graph[first]) == within == (first in graph[second])


def test_a_written_topology_reads_back_as_a_graph_file(tmp_path):
    with mock.patch.object(topology, "NUM_NODES", 25):
        nodes = topology.generate_nodes()
    graph = topology.build_adjacency(nodes)
    path = tmp_path / "graph_topology.txt"
    topology.write_graph(nodes, graph, path)

    loaded = load_graph(path)
    assert sorted(loaded, key=int) == [str(index) for index in range(len(nodes))]
    for index, (x, y) in enumerate(nodes):
        node = loaded[str(index)]
        assert (node.x, node.y) == (float(f"{x:.2f}"), float(f"{y:.2f}"))
        assert node.gain == 0.0
        assert node.successors_id == [str(neighbor) for neighbor in graph[index]]


def test_gain_regions_stay_apart_inside_the_area():
    regions = generate_clustered_gain_model.generate_gain_regions(8, 0, 25, 0, 25, 100)
    assert len(regions) == 8
    for x, y, radius, min_gain, max_gain in regions:
        assert 0 <= x <= 25 and 0 <= y <= 25
        assert 25 / 12 <= radius <= 25 / 6
        assert (min_gain, max_gain) == (0, 100)
    for first, second in itertools.combinations(regions, 2):
        assert math.dist(first[:2], second[:2]) >= first[2] + second[2]
