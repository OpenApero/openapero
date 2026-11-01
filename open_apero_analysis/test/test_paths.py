"""The data root setting and the figure tree that mirrors it."""

import os
from pathlib import Path

from open_apero_analysis import paths


def test_data_root_defaults_to_the_home_folder(monkeypatch):
    monkeypatch.delenv(paths.DATA_ROOT_VARIABLE, raising=False)
    assert paths.data_root() == Path.home() / "open_apero_data"


def test_data_root_reads_the_variable_and_expands_the_home_shorthand(monkeypatch):
    monkeypatch.setenv(paths.DATA_ROOT_VARIABLE, "/mnt/results")
    assert paths.data_root() == Path("/mnt/results")
    monkeypatch.setenv(paths.DATA_ROOT_VARIABLE, "~/elsewhere")
    assert paths.data_root() == Path.home() / "elsewhere"
    # An empty value is no setting.
    monkeypatch.setenv(paths.DATA_ROOT_VARIABLE, "")
    assert paths.data_root() == paths.DEFAULT_DATA_ROOT


def test_figures_root_is_below_the_data_root(monkeypatch):
    monkeypatch.setenv(paths.DATA_ROOT_VARIABLE, "/mnt/results")
    assert paths.figures_root() == Path("/mnt/results/figures")
    assert paths.figures_root("/tmp/other") == Path("/tmp/other/figures")


def test_figure_dir_mirrors_an_input_below_the_data_root(monkeypatch):
    monkeypatch.setenv(paths.DATA_ROOT_VARIABLE, "/mnt/results")
    run = "/mnt/results/explore/HSSD/102815835/nbs/20260101_120000"
    assert paths.figure_dir(run) == Path(
        "/mnt/results/figures/explore/HSSD/102815835/nbs/20260101_120000"
    )
    # --out-dir moves the whole tree; the input's path below the data root stays.
    assert paths.figure_dir(run, "/tmp/figs") == Path(
        "/tmp/figs/explore/HSSD/102815835/nbs/20260101_120000"
    )


def test_figure_dir_keeps_the_whole_path_of_an_input_elsewhere(monkeypatch):
    monkeypatch.setenv(paths.DATA_ROOT_VARIABLE, "/mnt/results")
    assert paths.figure_dir("/scratch/campaign/scene") == Path(
        "/mnt/results/figures/scratch/campaign/scene"
    )
    assert paths.figure_dir("/scratch/campaign/scene", "/tmp/figs") == Path(
        "/tmp/figs/scratch/campaign/scene"
    )


def test_figure_dir_follows_a_linked_data_root(monkeypatch, tmp_path):
    target = tmp_path / "target"
    (target / "campaign").mkdir(parents=True)
    link = tmp_path / "link"
    os.symlink(str(target), str(link))
    monkeypatch.setenv(paths.DATA_ROOT_VARIABLE, str(link))
    # Named through the link and through its target, the input lands in the same place.
    assert paths.figure_dir(link / "campaign") == link / "figures" / "campaign"
    assert paths.figure_dir(target / "campaign") == link / "figures" / "campaign"
