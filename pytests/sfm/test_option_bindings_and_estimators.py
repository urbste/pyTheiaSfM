import pytheia as pt


def test_nonlinear_position_options_map_to_right_members():
    o = pt.sfm.NonlinearPositionEstimatorOptions()
    o.num_threads = 3
    o.max_num_iterations = 77
    o.robust_loss_width = 0.25
    o.min_num_points_per_view = 5
    o.point_to_camera_weight = 0.9
    assert (o.num_threads, o.max_num_iterations, o.min_num_points_per_view) == (3, 77, 5)
    assert o.robust_loss_width == 0.25 and o.point_to_camera_weight == 0.9


def test_nonlinear_position_options_deprecated_aliases():
    o = pt.sfm.NonlinearPositionEstimatorOptions()
    o.max_power_iterations = 12
    o.eigensolver_threshold = 0.5
    assert o.max_num_iterations == 12
    assert o.robust_loss_width == 0.5
