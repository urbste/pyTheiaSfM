# Copyright (C) 2026 The pyTheiaSfM Authors.
"""Round-trip and size tests for the compact Reconstruction binary format.

Reconstruction v2 / View v1 / Track v1 / Feature v1 write less data than the
legacy v1 layout. Files written by older builds (legacy_v1.recon) must still
load to exactly the same content.
"""

import json
import os
import sys

import numpy as np
import pytest

pt = pytest.importorskip("pytheia")

DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
sys.path.insert(0, DATA_DIR)
from dump_recon import dump  # noqa: E402

LEGACY_RECON = os.path.join(DATA_DIR, "legacy_v1.recon")
LEGACY_EXPECTED = os.path.join(DATA_DIR, "legacy_v1_expected.json")


def _load_expected():
    with open(LEGACY_EXPECTED, encoding="utf-8") as f:
        return json.load(f)


def _read(path):
    ok, recon = pt.io.ReadReconstruction(path)
    assert ok, f"ReadReconstruction failed for {path}"
    return recon


def test_legacy_file_loads_exactly():
    recon = _read(LEGACY_RECON)
    assert dump(recon) == _load_expected()


def test_new_format_round_trip_is_lossless(tmp_path):
    recon = _read(LEGACY_RECON)
    out = str(tmp_path / "compact.recon")
    assert pt.io.WriteReconstruction(recon, out)
    assert dump(_read(out)) == _load_expected()


def test_new_format_is_smaller_than_legacy(tmp_path):
    recon = _read(LEGACY_RECON)
    out = str(tmp_path / "compact.recon")
    assert pt.io.WriteReconstruction(recon, out)
    assert os.path.getsize(out) < os.path.getsize(LEGACY_RECON)


# Expected to fail: with the mandated v1 Track layout (84 bytes per track: point,
# bearing, color, inverse depth, reference view, empty descriptor header) plus
# ~13 bytes per feature, 7 observations per track already cost ~25 bytes per
# observation before any view overhead. The measured value is ~27.5 B/obs
# (legacy: ~191 B/obs). Strict xfail so an accidental pass is flagged.
@pytest.mark.xfail(
    strict=True,
    reason="Track layout floor is ~25 B/obs at 7 views per track; target 20 B/obs",
)
def test_synthetic_reconstruction_under_20_bytes_per_observation(tmp_path):
    rng = np.random.default_rng(1234)
    num_views, num_tracks, views_per_track = 50, 2000, 7

    recon = pt.sfm.Reconstruction()
    view_ids = []
    for i in range(num_views):
        vid = recon.AddView(f"img_{i}.png", 0, float(i) * 0.1)
        recon.MutableView(vid).SetIsEstimated(True)
        view_ids.append(vid)

    num_obs = 0
    for t in range(num_tracks):
        chosen = rng.choice(num_views, size=views_per_track, replace=False)
        obs = []
        for v in chosen:
            # float32-exact pixel coordinates, default covariance / depth prior
            xy = rng.uniform(0.0, 4000.0, size=2).astype(np.float32).astype(np.float64)
            obs.append((view_ids[v], pt.sfm.Feature(xy)))
        num_obs += len(obs)
        tid = recon.AddTrack(obs)
        track = recon.MutableTrack(tid)
        track.SetIsEstimated(True)
        track.SetPoint(np.array([rng.normal(), rng.normal(), 5.0 + rng.uniform(), 1.0]))

    out = str(tmp_path / "synthetic.recon")
    assert pt.io.WriteReconstruction(recon, out)
    bytes_per_obs = os.path.getsize(out) / num_obs
    print(f"synthetic: {os.path.getsize(out)} bytes, {num_obs} obs, {bytes_per_obs:.2f} B/obs")
    assert bytes_per_obs < 20.0
