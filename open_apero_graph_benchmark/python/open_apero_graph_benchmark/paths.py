"""Where the graph benchmark's inputs, results and figures live.

``INPUT_ROOT`` is the package's tracked ``data/``: the graph inputs and each dataset's
``reference.json``, the only file a tool writes there. ``RESULTS_ROOT`` is where
``run_benchmark.launch`` writes result trees by default, ``<data root>/graph_benchmark/<dataset>``,
the data root being ``$OPEN_APERO_DATA_ROOT`` or ``~/open_apero_data``
(:mod:`open_apero_analysis.paths`). ``GENERATED_ROOT``, below it, is where the input generators
write. The figures drawn from a result tree go below the data root's ``figures/`` at the tree's
path.
"""

from pathlib import Path

from open_apero_analysis.paths import data_root

PACKAGE_NAME = "open_apero_graph_benchmark"


def _package_root():
    """The source package in a checkout or devel space, else the installed share directory."""
    source = Path(__file__).resolve().parents[2]
    if (source / "package.xml").is_file():
        return source
    import rospkg  # an install space: the modules are under dist-packages, the data under share/

    return Path(rospkg.RosPack().get_path(PACKAGE_NAME))


PACKAGE_ROOT = _package_root()
INPUT_ROOT = PACKAGE_ROOT / "data"
RESULTS_ROOT = data_root() / "graph_benchmark"
GENERATED_ROOT = RESULTS_ROOT / "generated"
