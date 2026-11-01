"""Scene resolution, depth post-processing, intrinsics and stderr filter of the Habitat renderer."""

import math
import subprocess
import sys

import numpy as np
import pytest

from open_apero_habitat.habitat_renderer import camera_intrinsics, process_depth, resolve_scene


def test_hm3d_id_resolves_to_its_glb():
    assert (
        resolve_scene("00014-nYYcLpSzihC", "/data/hm3d/val")
        == "/data/hm3d/val/00014-nYYcLpSzihC/nYYcLpSzihC.glb"
    )


@pytest.mark.parametrize(
    "scene_id",
    [
        "102816036",
        "103997562_171030642",
        "/data/scene_datasets/hssd-hab/stages/102344280.glb",
        "00014-nYYcLpSzihC.glb",
    ],
)
def test_other_scene_ids_pass_unchanged(scene_id):
    assert resolve_scene(scene_id, "/data/hm3d/val") == scene_id


def test_process_depth_replaces_invalid_and_out_of_range_readings():
    depth = np.array([[np.nan, np.inf, -np.inf, 0.05], [0.5, 3.0, 3.5, 0.0]], dtype=np.float32)
    processed = process_depth(depth, min_depth=0.1, max_depth=3.0, far_sentinel=10.0)
    expected = np.array([[0.0, 0.0, 0.0, 0.0], [0.5, 3.0, 10.0, 0.0]], dtype=np.float32)
    assert processed.dtype == np.float32
    np.testing.assert_array_equal(processed, expected)


def test_process_depth_leaves_its_input_unchanged():
    depth = np.array([np.nan, 4.0], dtype=np.float32)
    process_depth(depth, min_depth=0.0, max_depth=3.0, far_sentinel=10.0)
    assert np.isnan(depth[0]) and depth[1] == 4.0


def test_camera_intrinsics_of_the_shipped_camera():
    fx, fy, cx, cy = camera_intrinsics(480, 640, 42.0)
    assert fx == pytest.approx(240.0 / math.tan(math.radians(21.0)))
    assert fx == pytest.approx(625.22, abs=0.01)
    assert fy == fx
    assert (cx, cy) == (239.5, 319.5)


def test_the_stderr_filter_drops_only_the_asset_warnings():
    lines = [
        "Trade::BasisImporter::openData(): the image was not encoded Y-flipped, imported data will "
        "have wrong orientation",
        "Trade::GltfImporter::scene(): node 0 extras id property is "
        "Utility::JsonToken::Type::String, skipping",
        "Trade::GltfImporter::scene(): node 12 extras pid property is "
        "Utility::JsonToken::Type::String, skipping",
        "Platform::WindowlessEglApplication::tryCreateContext(): unable to find EGL device for "
        "CUDA device 0",
        "[WARN] kept",
    ]
    # Writes go to fd 2 directly, as the C++ libraries do, after the filter replaced it.
    script = (
        "import os, sys\n"
        "from open_apero_habitat.habitat_renderer import filter_stderr\n"
        "filter_stderr()\n"
        "for line in sys.argv[1:]:\n"
        "    os.write(2, (line + '\\n').encode())\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", script, *lines], stderr=subprocess.PIPE, timeout=30, check=True
    )
    assert result.stderr.decode().splitlines() == lines[3:]
