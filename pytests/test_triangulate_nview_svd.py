"""TriangulateNViewSVD must be exact without noise and deterministic."""
import numpy as np
import pytest

pt = pytest.importorskip("pytheia")


def test_triangulate_nview_svd():
    rng = np.random.default_rng(0)
    X = np.array([0.3, -0.2, 5.0])
    P, x = [], []
    for _ in range(4):
        t = rng.normal(size=3) * 0.5
        P.append(np.c_[np.eye(3), t])
        Xc = X + t
        x.append(Xc[:2] / Xc[2])
    results = set()
    for k in range(200):
        # Leave non-zero data on the heap so that an uninitialized design
        # matrix would pick it up.
        junk = rng.normal(size=(3 * (k % 7 + 2), 12))
        pt.sfm.TriangulateNViewSVD([np.c_[np.eye(3), r[:3]] for r in junk[::3, :3]],
                                   [r[3:5] for r in junk[::3]])
        ok, Xh = pt.sfm.TriangulateNViewSVD(P, x)
        assert ok
        assert np.allclose(Xh[:3] / Xh[3], X, atol=1e-9)
        results.add(tuple(Xh))
    assert len(results) == 1
