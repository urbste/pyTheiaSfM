import pytheia as pt
import numpy as np
from random_recon_gen import RandomReconGenerator


def test_BundleAdjustView(gen, ba_options):

    for vid in gen.recon.ViewIds:
        orig_pos = gen.recon.View(vid).Camera().Position
        orig_axangle = gen.recon.View(vid).Camera().GetOrientationAsAngleAxis()
        gen.add_noise_to_views(noise_pos=1e-3, noise_angle=1e-1)
        result = pt.sfm.BundleAdjustView(gen.recon, ba_options, vid)
        dist_pos = np.linalg.norm(
            orig_pos - gen.recon.View(vid).Camera().Position)
        assert dist_pos < 1e-4
        assert result.success


def test_BundleAdjustWithPositionPrior(gen, ba_options):
    ba_options.use_position_priors = True

    for vid in gen.recon.ViewIds:
        # get the original camera pose as a prior
        pos_prior = gen.recon.View(vid).Camera().Position
        # now put noise on the position
        gen.add_noise_to_views(noise_pos=1e-2, noise_angle=1e-1)

        position_prior_sqrt_information = np.eye(3, dtype=np.float64)
        gen.recon.View(vid).SetPositionPrior(
            pos_prior, position_prior_sqrt_information)
        # check if setting the prior worked
        assert np.all(gen.recon.View(vid).GetPositionPrior() == pos_prior)

        result = pt.sfm.BundleAdjustView(gen.recon, ba_options, vid)

        dist_pos = np.linalg.norm(
            pos_prior - gen.recon.View(vid).Camera().Position)
        assert dist_pos < 1e-4
        assert result.success


def test_BundleAdjustWithPositionPriorAndInformation(gen, ba_options):
    ba_options.use_position_priors = True

    # set a low standard dev for the position prior
    # the sqrt_information will then be 1./std * eye(3)
    pos_priors_std_dev = 0.001
    for vid in gen.recon.ViewIds:
        view = gen.recon.View(vid)
        original_position = view.Camera().Position
        # get the original camera pose as a prior
        pos_prior = original_position + pos_priors_std_dev * np.random.randn(3)
        # now put a large noise on the position and see if the prior helps to recover the original position
        gen.add_noise_to_views(noise_pos=5.0, noise_angle=1e-1)

        position_prior_sqrt_information = 1. / \
            pos_priors_std_dev * np.eye(3, dtype=np.float64)
        view.SetPositionPrior(pos_prior, position_prior_sqrt_information)
        # check if setting the prior worked
        assert np.all(view.GetPositionPrior() == pos_prior)
        assert view.HasPositionPrior()

        result = pt.sfm.BundleAdjustView(gen.recon, ba_options, vid)

        dist_pos = np.linalg.norm(
            original_position - gen.recon.View(vid).Camera().Position)
        assert dist_pos < 3 * pos_priors_std_dev
        assert result.success


if __name__ == "__main__":
    gen = RandomReconGenerator()

    # test 1
    gen.generate_random_recon(nr_views=1, nr_tracks=20)
    ba_options = pt.sfm.BundleAdjustmentOptions()
    ba_options.constant_camera_orientation = False
    ba_options.constant_camera_position = False
    ba_options.loss_function_type = pt.sfm.LossFunctionType(0)
    ba_options.robust_loss_width = 1.345
    ba_options.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.NONE
    ba_options.verbose = True
    ba_options.use_position_priors = False
    test_BundleAdjustView(gen, ba_options)

    # test 2
    gen = RandomReconGenerator()
    gen.generate_random_recon(nr_views=1, nr_tracks=20)
    test_BundleAdjustWithPositionPrior(gen, ba_options)

    # test 2
    gen = RandomReconGenerator()
    gen.generate_random_recon(nr_views=1, nr_tracks=20, pixel_noise=1.0)
    test_BundleAdjustWithPositionPriorAndInformation(gen, ba_options)
