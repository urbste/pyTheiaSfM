import pytheia as pt
import numpy as np
from random_recon_gen import RandomReconGenerator, CameraPrior


def _check_BundleAdjustView(gen, ba_options):

    for vid in gen.recon.ViewIds():
        orig_pos = gen.recon.View(vid).Camera().GetPosition()
        gen.add_noise_to_views(noise_pos=1e-3, noise_angle=1e-1)
        result = pt.sfm.BundleAdjustView(gen.recon, ba_options, vid)
        dist_pos = np.linalg.norm(
            orig_pos[:2] - gen.recon.View(vid).Camera().GetPosition()[:2])
        assert dist_pos < 1e-4
        assert result.success

def _make_orthographic_generator():
    cam_prior = CameraPrior(focal_length=30000)
    cam_prior.set_to_orthographic()
    return RandomReconGenerator(cam_prior=cam_prior)


def _make_ba_options():
    ba_options = pt.sfm.BundleAdjustmentOptions()
    ba_options.loss_function_type = pt.sfm.LossFunctionType(0)
    ba_options.robust_loss_width = 1.345
    ba_options.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.NONE
    ba_options.orthographic_camera = True
    ba_options.verbose = False
    ba_options.use_position_priors = False
    return ba_options


def test_BundleAdjustView():
    gen = _make_orthographic_generator()
    gen.generate_random_recon(nr_views=1,
                              nr_tracks=20,
                              pt3_xyz_min=[-0.01, -0.01, 0],
                              pt3_xyz_max=[0.01, 0.01, 0],
                              cam_xyz_min=[0.0, 0.0, 0],
                              cam_xyz_max=[0.005, 0.005, 0],
                              cam_rot_ax_min=[-0.02, -0.02, -0.02],
                              cam_rot_ax_max=[0.01, 0.01, 0.01],
                              cam_rot_max_angle=np.pi / 10,
                              pixel_noise=0.0)
    _check_BundleAdjustView(gen, _make_ba_options())


def test_BundleAdjustViewWithNoise():
    gen = _make_orthographic_generator()
    gen.generate_random_recon(nr_views=1,
                              nr_tracks=20,
                              pt3_xyz_min=[-0.01, -0.01, 0],
                              pt3_xyz_max=[0.01, 0.01, 0],
                              # Orthographic cameras have no depth (tz is held
                              # constant in BA); keep them at z = 0 so the
                              # rotated points stay inside the image.
                              cam_xyz_min=[0.0, 0.0, 0],
                              cam_xyz_max=[0.005, 0.005, 0],
                              cam_rot_ax_min=[-0.02, -0.02, -0.02],
                              cam_rot_ax_max=[0.01, 0.01, 0.01],
                              cam_rot_max_angle=np.pi / 10,
                              pixel_noise=0.5)
    _check_BundleAdjustView(gen, _make_ba_options())


if __name__ == "__main__":
    test_BundleAdjustView()
    test_BundleAdjustViewWithNoise()
