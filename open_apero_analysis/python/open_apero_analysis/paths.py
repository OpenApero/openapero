"""The data root and the figure tree below it.

One setting places everything Open Apero writes: the environment variable
``OPEN_APERO_DATA_ROOT``, by default ``~/open_apero_data``. Runs and benchmark results are
recorded below it; the tools read them and write their figures below ``<data root>/figures/``, at
the path of the input each figure was drawn from. The launch files, ``run_experiments.sh``,
``paper_sweep.py`` and ``docker/run.sh`` read the same variable themselves, since they cannot
import this module; its name and default are spelled out in each of them.
"""

import os
from pathlib import Path

DATA_ROOT_VARIABLE = "OPEN_APERO_DATA_ROOT"
DEFAULT_DATA_ROOT = Path.home() / "open_apero_data"
FIGURES_DIRNAME = "figures"


def data_root():
    """The data root: ``$OPEN_APERO_DATA_ROOT`` with ``~`` expanded, else ``~/open_apero_data``."""
    value = os.environ.get(DATA_ROOT_VARIABLE, "")
    return Path(value).expanduser() if value else DEFAULT_DATA_ROOT


def figures_root(root=None):
    """``<root>/figures``, the figure tree of ``root`` (default: the data root)."""
    return (Path(root) if root is not None else data_root()) / FIGURES_DIRNAME


def figure_dir(source, out_dir=None):
    """The figure directory of ``source``, a run, scene or dataset directory.

    Below ``out_dir`` (default :func:`figures_root`), at the source's path below the data root,
    so inserting ``figures/`` into an input's path finds its figures. A source outside the data
    root keeps its whole absolute path, so two inputs never share a directory.
    """
    out_dir = Path(out_dir) if out_dir is not None else figures_root()
    source = os.path.expanduser(str(source))
    root = str(data_root())
    # Compare the paths as given first, then with symbolic links resolved, so a data root that is
    # a link and a source named through its target still line up.
    for normalize in (os.path.abspath, os.path.realpath):
        try:
            return out_dir / Path(normalize(source)).relative_to(normalize(root))
        except ValueError:
            continue
    return out_dir / Path(*Path(os.path.abspath(source)).parts[1:])
