import pytheia as pt


def test_glomap_position_estimator_options_are_bound():
    options = pt.sfm.ReconstructionEstimatorOptions()

    options.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.GLOMAP
    options.glomap_position_estimator_options.use_pairwise_scale_priors = True
    options.glomap_position_estimator_options.min_track_length = 4

    assert options.global_position_estimator_type == pt.sfm.GlobalPositionEstimatorType.GLOMAP
    assert options.glomap_position_estimator_options.use_pairwise_scale_priors
    assert options.glomap_position_estimator_options.min_track_length == 4


def test_ligt_position_estimator_options_are_bound_on_reconstruction_options():
    options = pt.sfm.ReconstructionEstimatorOptions()

    options.ligt_position_estimator_options.max_num_views_svd = 123

    assert options.ligt_position_estimator_options.max_num_views_svd == 123
