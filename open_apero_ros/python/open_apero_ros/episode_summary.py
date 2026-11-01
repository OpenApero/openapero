"""Stdlib reader for a run's ``episode_summary.yaml``, shared by the sweep tools in ``scripts/``.

``paper_sweep.py`` and ``compare_paper_sweep.py`` run under whatever interpreter the sweep uses,
so this reads the planner's own format without PyYAML.
"""

import os

EPISODE_SUMMARY = "episode_summary.yaml"
# The YAML 1.1 spellings the planner writes for non-finite floats.
YAML_FLOATS = {".inf": "inf", "-.inf": "-inf", ".nan": "nan"}


def _yaml_scalar(text):
    """A float for a numeric YAML scalar, else the unquoted text."""
    try:
        return float(YAML_FLOATS.get(text, text))
    except ValueError:
        return text.strip('"')


def read_episode_summary(run_dir):
    """The run's episode_summary.yaml as a dict, or None without the file.

    A stdlib reader for the planner's own format: ``key: value`` lines, and one indented
    level of ``"name": value`` lines under a key with no value (real_accumulated_gain).
    """
    try:
        with open(os.path.join(run_dir, EPISODE_SUMMARY)) as handle:
            lines = handle.read().splitlines()
    except OSError:
        return None
    summary = {}
    section = None
    for line in lines:
        key, _, value = line.strip().partition(":")
        key, value = key.strip().strip('"'), value.strip()
        if not key:
            continue
        if line[:1].isspace() and section is not None:
            section[key] = _yaml_scalar(value)
        elif value in ("", "{}"):
            section = summary[key] = {}
        else:
            section = None
            summary[key] = _yaml_scalar(value)
    return summary
