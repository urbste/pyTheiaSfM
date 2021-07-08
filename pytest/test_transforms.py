import sys
import pytheia as pt
from scipy.spatial.transform import Rotation as R
import numpy as np

def test_TransformReconstruction():
    print("test_TransformReconstruction()")

    recon = pt.sfm.Reconstruction()
    # add a view
    view_id = recon.AddView("v1", 0, 0)
    view = recon.View(view_id)
    view.IsEstimated = True
    camera = view.Camera()
    cam_pos = np.array([0,0,10])
    camera.Position = np.array(cam_pos)

    # add a track
    track_id = recon.AddTrack()
    point = np.array([0,1,0,1],dtype=np.float32)
    track = recon.Track(track_id)
    track.Point = point
    track.IsEstimated = True

    # transform reconstruction
    scale = 2.0
    rotation = np.eye(3, dtype=np.float32)
    translation = np.array([1,1,1])
    pt.sfm.TransformReconstruction(recon, rotation, translation, scale)

    point_transformed = recon.Track(track_id).Point
    point_transformed = point_transformed[0:3] / point_transformed[3]

    assert (scale*rotation@point[0:3] + translation == point_transformed).any()
    assert (scale*rotation@cam_pos + translation == recon.View(view_id).Camera().Position).any()

if __name__ == "__main__":
    test_TransformReconstruction()
    print("All tests successfull.")
