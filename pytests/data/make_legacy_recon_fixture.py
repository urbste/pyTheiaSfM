"""Writes legacy_v1.recon with a pyTheia build that predates the compact format
(Reconstruction v1, View/Track/Feature v0). Kept for reproducibility; the committed
fixture was written by pytheia built from f32e69b."""
import sys
import numpy as np
import pytheia as pt

r = pt.sfm.Reconstruction()
for i in range(4):
    vid = r.AddView(f"img_{i}.png", 0, float(i) * 0.1)
    v = r.MutableView(vid)
    v.SetIsEstimated(True)
    cam = v.MutableCamera()
    cam.SetPosition(np.array([float(i), 0.1 * i, 0.0]))
    cam.SetFocalLength(800.0 + i)
    cam.SetImageSize(1000, 600)
    cam.SetPrincipalPoint(500.0, 300.0)
for t in range(6):
    obs = []
    for i in range(4):
        x, y = 100.25 + 10 * t + i, 200.5 + 5 * t - i  # float32-exact
        if t == 1 and i == 2:
            x = 123.456789012345  # not float32-exact
        obs.append((i, pt.sfm.Feature(np.array([x, y]))))
    tid = r.AddTrack(obs)
    tr = r.MutableTrack(tid)
    tr.SetIsEstimated(True)
    tr.SetPoint(np.array([t * 1.5, -t * 0.25, 10.0 + t, 1.0]))
    tr.SetColor(np.array([10 * t, 20, 30], dtype=np.uint8))
    tr.SetInverseDepth(0.1 + t)
    tr.SetReferenceBearingVector(np.array([0.0, 0.6, 0.8]))
    tr.SetReferenceDescriptor(np.arange(t, dtype=np.float32))
# one feature with non-default covariance and depth prior
f = pt.sfm.Feature(np.array([300.0, 310.0]), np.array([[2.0, 0.1], [0.1, 3.0]]))
f.depth_prior = 4.5
f.depth_prior_variance = 0.25
tid = r.AddTrack([(0, f), (1, pt.sfm.Feature(np.array([301.0, 311.0])))])
r.MutableTrack(tid).SetIsEstimated(True)  # WriteReconstruction keeps estimated tracks only
r.MutableTrack(tid).SetPoint(np.array([2.0, 4.0, 8.0, 2.0]))
assert pt.io.WriteReconstruction(r, sys.argv[1])
print("views", r.NumViews(), "tracks", r.NumTracks())
