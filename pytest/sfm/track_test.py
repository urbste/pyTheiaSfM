import pytheia as pt
import os
import numpy as np


def test_track_set_descriptor_read_write():
    recon = pt.sfm.Reconstruction()
    view_id1 = recon.AddView("0", 0.0)
    m_view1 = recon.MutableView(view_id1)
    m_view1.IsEstimated = True
    view_id2 = recon.AddView("1", 1.0)
    m_view2 = recon.MutableView(view_id2)
    m_view2.IsEstimated = True

    # add a track with descriptor
    t_id = recon.AddTrack()
    m_track = recon.MutableTrack(t_id)
    m_track.AddView(view_id1)
    m_track.AddView(view_id2)
    m_track.IsEstimated = True
    desc = np.asarray([100, 200, 300, 400])
    m_track.SetReferenceDescriptor(desc)

    assert (m_track.ReferenceDescriptor() == desc).all()

    # test read and write
    pt.io.WriteReconstruction(recon, "test")
    recon_loaded = pt.io.ReadReconstruction("test")[1]

    s_track = recon_loaded.Track(t_id)
    assert (s_track.ReferenceDescriptor() == desc).all()

    os.remove("test")


if __name__ == "__main__":
    test_track_set_descriptor_read_write()
