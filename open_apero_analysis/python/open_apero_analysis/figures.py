"""Where a tool's figure goes, the record written beside it, and the closing ``Wrote`` line.

A figure tool writes its file and exits; it never opens a window (:func:`use_file_backend`).
Every figure tool takes the same arguments (:func:`add_figure_arguments`): ``--out-dir``, the
root of the figure tree, ``--output``, one exact file, and ``--open``, which opens the written
file in the system viewer. :func:`figure_path` resolves them for one figure, and
:func:`finish_figure` writes its record, prints its path and opens it when asked. The record,
``<figure>.yaml`` beside the figure, says what was drawn from which inputs by which command, so a
figure found later can be traced and redrawn.
"""

import argparse
import datetime
import shlex
import subprocess
import sys
from pathlib import Path

import matplotlib
import yaml

from open_apero_analysis.paths import figure_dir, figures_root

RECORD_SUFFIX = ".yaml"


def use_file_backend():
    """Draw with Matplotlib's Agg backend: files only, so a figure tool needs no display."""
    matplotlib.use("Agg")


def add_figure_arguments(parser, single_file=True):
    """Add ``--out-dir``, ``--open`` and, for a tool drawing one figure, ``--output``.

    The arguments default to absent (``argparse.SUPPRESS``), so a default-showing help formatter
    prints the defaults the help texts state; :func:`figure_path` and :func:`opens_viewer` read
    them.
    """
    group = parser.add_argument_group("figure output")
    group.add_argument(
        "--out-dir",
        type=Path,
        metavar="DIR",
        default=argparse.SUPPRESS,
        help="Root of the figure tree, where the figure goes at its input's path below the data "
        "root (default: <data root>/figures).",
    )
    if single_file:
        group.add_argument(
            "--output",
            type=Path,
            metavar="FILE",
            default=argparse.SUPPRESS,
            help="Write exactly this file.",
        )
    group.add_argument(
        "--open",
        action="store_true",
        default=argparse.SUPPRESS,
        help="Open the written file in the system viewer.",
    )
    return group


def figure_path(args, source, name):
    """Where the figure ``name`` drawn from ``source`` goes: ``--output``, else below ``--out-dir``.

    A figure drawn from no run or dataset (``source`` None) sits at the top of the tree.
    """
    output = getattr(args, "output", None)
    if output is not None:
        return Path(output)
    out_dir = getattr(args, "out_dir", None)
    if source is None:
        return (Path(out_dir) if out_dir is not None else figures_root()) / name
    return figure_dir(source, out_dir) / name


def opens_viewer(args):
    """Whether ``--open`` was given."""
    return getattr(args, "open", False)


def command_line(tool, argv=None):
    """The invocation as one shell line: the tool's name, then its arguments."""
    arguments = sys.argv[1:] if argv is None else list(argv)
    return " ".join(shlex.quote(str(argument)) for argument in [tool] + arguments)


def _code_version():
    """``git describe`` of the checkout this module runs from, or None outside one."""
    try:
        return subprocess.run(
            ["git", "-C", str(Path(__file__).resolve().parent), "describe", "--always", "--dirty"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=True,
            timeout=10,
            universal_newlines=True,
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def write_record(figure, inputs, command, extra=None):
    """Write ``<figure>.yaml`` beside ``figure``: the inputs, the command, the time and the code.

    ``inputs`` are the run, scene or dataset directories the figure was drawn from; ``extra``
    adds tool-specific keys. Returns the record's path.
    """
    figure = Path(figure)
    record = {
        "figure": figure.name,
        "command": command,
        "written_utc": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "inputs": [str(path) for path in inputs],
    }
    code = _code_version()
    if code:
        record["code"] = code
    if extra:
        record.update(extra)
    path = figure.with_suffix(RECORD_SUFFIX)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(record, sort_keys=False, default_flow_style=False))
    return path


def open_in_viewer(path):
    """Open ``path`` with the system viewer (``xdg-open``; ``open`` on macOS), without waiting."""
    opener = "open" if sys.platform == "darwin" else "xdg-open"
    try:
        subprocess.Popen([opener, str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError as error:
        print("Could not open %s with %s: %s" % (path, opener, error), file=sys.stderr)


def finish_figure(figure, inputs, command, open_file=False, extra=None):
    """Record ``figure``'s provenance beside it, print where it is, and open it when asked."""
    record = write_record(figure, inputs, command, extra)
    print("Wrote %s (+ %s)" % (Path(figure).resolve(), record.name))
    if open_file:
        open_in_viewer(figure)
