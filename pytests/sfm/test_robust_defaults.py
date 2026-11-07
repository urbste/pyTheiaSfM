"""Regression tests for the robustness-related defaults (pyTheia 1.3.2)."""
import pytheia as pt


def test_default_ba_loss_is_robust():
    o = pt.sfm.ReconstructionEstimatorOptions()
    assert o.bundle_adjustment_loss_function_type == pt.sfm.LossFunctionType.HUBER
    # A 10 px Huber width is practically L2 for reprojection errors; the default must be tight.
    assert o.bundle_adjustment_robust_loss_width <= 2.0

    b = pt.sfm.BundleAdjustmentOptions()
    assert b.loss_function_type == pt.sfm.LossFunctionType.HUBER
    assert b.robust_loss_width <= 2.0


def test_trivial_loss_is_still_selectable():
    o = pt.sfm.ReconstructionEstimatorOptions()
    o.bundle_adjustment_loss_function_type = pt.sfm.LossFunctionType.TRIVIAL
    assert o.bundle_adjustment_loss_function_type == pt.sfm.LossFunctionType.TRIVIAL


def test_ligt_robustness_options_exposed():
    o = pt.sfm.LiGTPositionEstimatorOptions()
    # defaults: row-normalised constraints + Huber IRLS
    assert o.normalize_constraints is True
    assert o.num_irls_iterations > 0
    assert o.irls_scale_multiplier > 0
    # the original, non-robust LiGT stays reachable
    o.normalize_constraints = False
    o.num_irls_iterations = 0
    assert o.normalize_constraints is False and o.num_irls_iterations == 0


def test_ligt_scale_constraint_option():
    o = pt.sfm.LiGTPositionEstimatorOptions()
    # default: scale-constrained solve instead of the smallest eigenvector
    assert o.use_scale_constraint is True
    o.use_scale_constraint = False
    assert o.use_scale_constraint is False
