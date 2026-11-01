"""Dump every serialized field of a reconstruction to JSON (used to pin legacy_v1.recon)."""
import json, sys
import numpy as np
import pytheia as pt

def dump(r):
    out = {"views": {}, "tracks": {}}
    for vid in sorted(r.ViewIds()):
        v = r.View(vid); c = v.Camera()
        feats = {}
        for tid in sorted(v.TrackIds()):
            f = v.GetFeature(tid)
            feats[str(tid)] = {"point": [float(x) for x in np.asarray(f.point)],
                               "cov": [float(x) for x in np.asarray(f.covariance).ravel()],
                               "dp": float(f.depth_prior), "dpv": float(f.depth_prior_variance)}
        out["views"][str(vid)] = {"name": v.Name(), "ts": v.GetTimestamp(), "est": v.IsEstimated(),
            "pos": [float(x) for x in c.GetPosition().ravel()], "f": c.FocalLength(),
            "size": [c.ImageWidth(), c.ImageHeight()], "features": feats}
    for tid in sorted(r.TrackIds()):
        t = r.Track(tid)
        out["tracks"][str(tid)] = {"est": t.IsEstimated(), "views": sorted(int(x) for x in t.ViewIds()),
            "ref": int(t.ReferenceViewId()), "invd": float(t.InverseDepth()),
            "point": [float(x) for x in np.asarray(t.Point())],
            "color": [int(x) for x in np.asarray(t.Color())],
            "bearing": [float(x) for x in np.asarray(t.ReferenceBearingVector())],
            "desc": [float(x) for x in np.asarray(t.ReferenceDescriptor())]}
    return out

if __name__ == "__main__":
    ok, r = pt.io.ReadReconstruction(sys.argv[1]); assert ok
    json.dump(dump(r), open(sys.argv[2], "w"), indent=1, sort_keys=True)
