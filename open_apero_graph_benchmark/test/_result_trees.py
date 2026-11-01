"""Synthetic benchmark result trees for the reference and verifier tests."""

REPLANS = ("none", "goal", "everystep")
OBJECTIVES = ("path_gain", "path_ratio", "expected_gain", "table_gain")
RESULT_FILES = ("nbs_1.txt", "tsp_0.50.txt")


def write_results(dataset, gain_scale=10.0, time_scale=0.001, graphs=("graph_0", "graph_1")):
    """Write a result tree shaped like a try_every_option sweep; return the dataset folder."""
    for graph in graphs:
        (dataset / graph).mkdir(parents=True, exist_ok=True)
        (dataset / graph / "graph.txt").write_text("0 0.0 0.0 0.0 1\n")
        for replan in REPLANS:
            for objective in OBJECTIVES:
                folder = dataset / graph / replan / objective
                folder.mkdir(parents=True, exist_ok=True)
                for index, name in enumerate(RESULT_FILES):
                    rows = "".join(
                        "{} {} {} {} 40\n".format(
                            gain_scale * step * (index + 1), step, time_scale * step, step
                        )
                        for step in range(1, 6)
                    )
                    (folder / name).write_text(rows)
    return dataset
