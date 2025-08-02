import pytheia as pt
import numpy as np


def test_TransformReconstruction():
    recon = pt.sfm.Reconstruction()
    # add a view
    view_id = recon.AddView("v1", 0, 0)
    view = recon.View(view_id)
    view.SetIsEstimated(True)
    camera = view.MutableCamera()
    cam_pos = np.array([0, 0, 10])
    camera.SetPosition(cam_pos)

    # add a track
    track_id = recon.AddTrack()
    point = np.array([0, 1, 0, 1], dtype=np.float32)
    track = recon.MutableTrack(track_id)
    track.SetPoint(point)
    track.SetIsEstimated(True)

    # transform reconstruction
    scale = 2.0
    rotation = np.eye(3, dtype=np.float32)
    translation = np.array([1, 1, 1])
    pt.sfm.TransformReconstruction(recon, rotation, translation, scale)

    point_transformed = recon.Track(track_id).Point()
    point_transformed = point_transformed[0:3] / point_transformed[3]

    assert (scale * rotation @ point[0:3]
            + translation == point_transformed).any()
    assert (scale * rotation @ cam_pos + translation
            == recon.View(view_id).Camera().GetPosition()).any()


def test_AlignReconstructions():
    scale = 2.0
    rotation = np.eye(3, dtype=np.float32)
    translation = np.array([1, 1, 1])
    nr_cams = 10
    recon_fix = pt.sfm.Reconstruction()
    recon_to_align = pt.sfm.Reconstruction()

    # generate random cam poses
    X = np.random.random((nr_cams, 3))
    for i in range(nr_cams):
        view_id = recon_fix.AddView(str(i), 0, i)
        view = recon_fix.View(view_id)
        view.SetIsEstimated(True)
        view.MutableCamera().SetPosition(X[i, :])
        # add a track
        track_id = recon_fix.AddTrack()
        point = np.array([0, i, 0, 1], dtype=np.float32)
        track = recon_fix.MutableTrack(track_id)
        track.SetPoint(point)
        track.SetIsEstimated(True)

    for i in range(nr_cams):
        view_id = recon_to_align.AddView(str(i), 0, i)
        view = recon_to_align.View(view_id)
        view.SetIsEstimated(True)
        view.MutableCamera().SetPosition(X[i, :])

        # add a track
        track_id = recon_to_align.AddTrack()
        point = np.array([0, i, 0, 1], dtype=np.float32)
        track = recon_to_align.MutableTrack(track_id)
        track.SetPoint(point)
        track.SetIsEstimated(True)

    pt.sfm.TransformReconstruction(recon_fix, rotation, translation, scale)

    pt.sfm.AlignReconstructions(recon_fix, recon_to_align)

    track_ids = recon_to_align.TrackIds()
    for i in track_ids:
        assert np.linalg.norm(recon_to_align.Track(
            i).Point() - recon_fix.Track(i).Point()) < 1e-10

    view_ids = recon_to_align.ViewIds()
    for i in view_ids:
        assert np.linalg.norm(recon_to_align.View(i).Camera(
        ).GetPosition() - recon_fix.View(i).Camera().GetPosition()) < 1e-10


if __name__ == "__main__":
    test_TransformReconstruction()
    test_AlignReconstructions()
