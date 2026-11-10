"""MLPnP: the refined pose must reach the maximum-likelihood optimum, for general and planar scenes."""
import numpy as np
import pytest

pt = pytest.importorskip("pytheia")


def _random_rotation(rng):
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _rot_err_deg(R, R_gt):
    return np.degrees(np.arccos(np.clip((np.trace(R.T @ R_gt) - 1) / 2, -1, 1)))


def _scene(rng, n, planar, noise):
    R = _random_rotation(rng)
    t = rng.normal(size=3)
    if planar:
        # A plane that does not contain the world origin, seen from the front.
        E = _random_rotation(rng)
        offset = rng.normal(size=3) * 5
        Xc_plane = np.c_[rng.uniform(-2, 2, (n, 2)), np.full(n, 6.0)]
        Xw = (Xc_plane - t) @ R  # camera points on the plane z = 6
        Xw = Xw @ E.T + offset
        R = R @ E.T
        t = t - R @ offset
    else:
        Xc = np.c_[rng.uniform(-3, 3, (n, 2)), rng.uniform(4, 10, n)]
        Xw = (Xc - t) @ R
    Xc = Xw @ R.T + t
    x = Xc[:, :2] / Xc[:, 2:] + rng.normal(size=(n, 2)) * noise
    return R, t, x, Xw


def _tangent_cost(R, t, x, Xw):
    b = Xw @ R.T + t
    b /= np.linalg.norm(b, axis=1, keepdims=True)
    f = np.c_[x, np.ones(len(x))]
    f /= np.linalg.norm(f, axis=1, keepdims=True)
    return np.sum(1.0 - np.sum(b * f, axis=1))


@pytest.mark.parametrize("planar", [False, True])
def test_mlpnp_noise_free(planar):
    rng = np.random.default_rng(0)
    for _ in range(50):
        R_gt, t_gt, x, Xw = _scene(rng, 8, planar, 0.0)
        ok, R, t = pt.sfm.MLPnP(list(x), [], list(Xw))
        assert ok
        assert _rot_err_deg(R, R_gt) < 1e-4
        assert np.linalg.norm(t - t_gt) < 1e-4 * max(1.0, np.linalg.norm(t_gt))


@pytest.mark.parametrize("planar", [False, True])
def test_mlpnp_reaches_optimum_with_noise(planar):
    # With noise the ground-truth pose is not optimal, so the refined pose must
    # not be (noticeably) worse than it. Before the Jacobian fix of the
    # refinement, the median cost on six points was hundreds of times higher.
    rng = np.random.default_rng(1)
    ratios = []
    for _ in range(200):
        R_gt, t_gt, x, Xw = _scene(rng, 6, planar, 1e-3)
        ok, R, t = pt.sfm.MLPnP(list(x), [], list(Xw))
        assert ok
        ratios.append(_tangent_cost(R, t, x, Xw) / _tangent_cost(R_gt, t_gt, x, Xw))
    ratios = np.array(ratios)
    # Six noisy points are close to degenerate for the linear initialization,
    # which occasionally lands in a wrong basin; inside RANSAC this is harmless.
    assert np.median(ratios) <= 1.0
    assert np.mean(ratios <= 1.01) >= 0.97


def test_mlpnp_with_covariances():
    rng = np.random.default_rng(2)
    R_gt, t_gt, x, Xw = _scene(rng, 30, False, 0.0)
    covs = []
    for xi in x:
        p = np.r_[xi, 1.0]
        v = p / np.linalg.norm(p)
        J = (np.eye(3) - np.outer(v, v)) / np.linalg.norm(p)
        covs.append(J @ np.diag([1e-6, 1e-6, 0.0]) @ J.T)
    ok, R, t = pt.sfm.MLPnP(list(x), covs, list(Xw))
    assert ok
    assert _rot_err_deg(R, R_gt) < 1e-4
