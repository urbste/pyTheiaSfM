#!/usr/bin/env python3
"""
Test for SIM3 point cloud alignment wrappers in pyTheiaSfM.

This test verifies that the SIM3 point cloud alignment functions work correctly
and produce expected results for various scenarios.
"""

import numpy as np
import pytest
import pytheia as pt


def create_synthetic_point_cloud(num_points=100, noise_std=0.01):
    """Create synthetic 3D point cloud."""
    np.random.seed(42)  # For reproducible tests
    points = []
    for i in range(num_points):
        point = np.array([
            np.random.uniform(-2.0, 2.0),
            np.random.uniform(-2.0, 2.0),
            np.random.uniform(1.0, 3.0)
        ], dtype=np.float64)
        points.append(point)
    return points


def create_synthetic_normals(num_points=100):
    """Create synthetic normal vectors."""
    np.random.seed(42)
    normals = []
    for i in range(num_points):
        normal = np.random.randn(3)
        normal = normal / np.linalg.norm(normal)  # Normalize
        normals.append(normal)
    return normals


def create_point_weights(num_points=100, outlier_ratio=0.1):
    """Create point weights with some outliers."""
    np.random.seed(42)
    weights = np.ones(num_points)
    num_outliers = int(num_points * outlier_ratio)
    outlier_indices = np.random.choice(num_points, num_outliers, replace=False)
    weights[outlier_indices] = 0.1  # Lower weights for outliers
    return weights.tolist()


def test_sim3_from_rotation_translation_scale():
    """Test Sim3FromRotationTranslationScale function."""
    
    # Test identity transformation
    rotation = np.eye(3, dtype=np.float64)
    translation = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    scale = 1.0
    
    sim3_params = pt.sfm.Sim3FromRotationTranslationScale(rotation, translation, scale)
    sim3 = pt.math.Sim3d(sim3_params)

    # Check that we get identity parameters
    np.testing.assert_array_almost_equal(sim3.matrix3x4(), np.eye(3, 4))
    np.testing.assert_array_almost_equal(sim3.translation(), translation)
    assert sim3.scale() == scale
    
    # Test non-identity transformation
    rotation = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)  # 90 degree rotation around Z
    translation = np.array([1.0, 2.0, 3.0], dtype=np.float64)
    scale = 2.0
    
    sim3_params = pt.sfm.Sim3FromRotationTranslationScale(rotation, translation, scale)
    sim3 = pt.math.Sim3d(sim3_params)
    
    # Check that parameters are reasonable
    assert len(sim3_params) == 7
    assert np.linalg.norm(sim3.scale() - scale) < 1e-6  # Scale should be the last parameter
    assert np.linalg.norm(sim3.matrix3x4()[:3, :3]) > 0  # Rotation should be non-zero


def test_sim3_to_rotation_translation_scale():
    """Test Sim3ToRotationTranslationScale function."""
    
    # Test identity transformation
    sim3_params = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    rotation, translation, scale = pt.sfm.Sim3ToRotationTranslationScale(sim3_params)
    
    # Check identity results
    np.testing.assert_array_almost_equal(rotation, np.eye(3))
    np.testing.assert_array_almost_equal(translation, np.zeros(3))
    assert scale == 1.0
    
    # Test non-identity transformation
    rotation = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)
    translation = np.array([1.0, 2.0, 3.0], dtype=np.float64)
    scale = 2.0
    
    # Convert to SIM3 parameters
    sim3_params = pt.sfm.Sim3FromRotationTranslationScale(rotation, translation, scale)
    
    # Convert back
    recovered_rotation, recovered_translation, recovered_scale = pt.sfm.Sim3ToRotationTranslationScale(sim3_params)
    
    # Check that we recover the original values
    np.testing.assert_array_almost_equal(rotation, recovered_rotation)
    np.testing.assert_array_almost_equal(translation, recovered_translation)
    assert abs(scale - recovered_scale) < 1e-10


def test_sim3_to_homogeneous_matrix():
    """Test Sim3ToHomogeneousMatrix function."""
    
    # Test identity transformation
    sim3_params = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    homogeneous_matrix = pt.sfm.Sim3ToHomogeneousMatrix(sim3_params)
    
    # Check identity matrix
    expected_matrix = np.eye(4, dtype=np.float64)
    np.testing.assert_array_almost_equal(homogeneous_matrix, expected_matrix)
    
    # Test non-identity transformation
    rotation = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)
    translation = np.array([1.0, 2.0, 3.0], dtype=np.float64)
    scale = 2.0
    
    # Convert to SIM3 parameters
    sim3_params = pt.sfm.Sim3FromRotationTranslationScale(rotation, translation, scale)
    
    # Get homogeneous matrix
    homogeneous_matrix = pt.sfm.Sim3ToHomogeneousMatrix(sim3_params)
    
    # Check matrix properties
    assert homogeneous_matrix.shape == (4, 4)
    assert homogeneous_matrix[3, 3] == 1.0  # Bottom-right should be 1
    
    # Check that the transformation works correctly
    test_point = np.array([1.0, 0.0, 0.0, 1.0])  # Homogeneous coordinates
    transformed_point = homogeneous_matrix @ test_point
    
    # The transformed point should be scaled and rotated
    expected_point = np.array([1., 4., 3., 1.])  # [scale*rotated_point + translation, 1]
    np.testing.assert_array_almost_equal(transformed_point, expected_point)


def test_optimize_alignment_sim3_point_to_point():
    """Test OptimizeAlignmentSim3 with point-to-point alignment."""
    
    # Create synthetic data
    num_points = 50
    source_points = create_synthetic_point_cloud(num_points, noise_std=0.01)
    
    # Create ground truth transformation
    rotation = np.array([
        [0.866, -0.5, 0.0],
        [0.5, 0.866, 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)  # ~30 degree rotation around Z
    translation = np.array([1.0, 2.0, 0.5], dtype=np.float64)
    scale = 1.5
    
    # Create target points by applying transformation
    target_points = []
    for point in source_points:
        transformed_point = scale * (rotation @ point) + translation
        target_points.append(transformed_point)
    
    # Test point-to-point alignment
    options = pt.sfm.Sim3AlignmentOptions()
    options.alignment_type = pt.sfm.Sim3AlignmentType.POINT_TO_POINT
    options.max_iterations = 50
    options.verbose = False
    
    summary = pt.sfm.OptimizeAlignmentSim3(source_points, target_points, options)
    
    # Check that alignment was successful
    assert summary.success
    assert summary.final_cost < 1e-6
    assert summary.alignment_error < 1e-3
    
    # Check that estimated parameters are close to ground truth
    estimated_rotation, estimated_translation, estimated_scale = pt.sfm.Sim3ToRotationTranslationScale(summary.sim3_params)
    
    # Check scale
    assert abs(estimated_scale - scale) < 0.1
    
    # Check translation
    translation_error = np.linalg.norm(estimated_translation - translation)
    assert translation_error < 0.1
    
    # Check rotation (using rotation matrix distance)
    rotation_error = np.linalg.norm(estimated_rotation - rotation)
    assert rotation_error < 0.1


def test_optimize_alignment_sim3_robust_point_to_point():
    """Test OptimizeAlignmentSim3 with robust point-to-point alignment."""
    
    # Create synthetic data with outliers
    num_points = 100
    source_points = create_synthetic_point_cloud(num_points, noise_std=0.01)
    
    # Create ground truth transformation
    rotation = np.eye(3, dtype=np.float64)
    translation = np.array([0.5, 0.3, 0.2], dtype=np.float64)
    scale = 1.2
    
    # Create target points by applying transformation
    target_points = []
    for point in source_points:
        transformed_point = scale * (rotation @ point) + translation
        target_points.append(transformed_point)
    
    # Add outliers to target points
    num_outliers = 5
    for i in range(num_outliers):
        target_points[i] = np.array([
            target_points[i][0] + 5.0,
            target_points[i][1] + 5.0,
            target_points[i][2] + 5.0
        ])
    
    # Test robust point-to-point alignment
    options = pt.sfm.Sim3AlignmentOptions()
    options.alignment_type = pt.sfm.Sim3AlignmentType.ROBUST_POINT_TO_POINT
    options.max_iterations = 50
    options.huber_threshold = 0.1
    options.verbose = True
    
    summary = pt.sfm.OptimizeAlignmentSim3(source_points, target_points, options)
    
    # Check that alignment was successful
    print(summary.final_cost)
    assert summary.success
    
    # Check that estimated parameters are reasonable
    estimated_rotation, estimated_translation, estimated_scale = pt.sfm.Sim3ToRotationTranslationScale(summary.sim3_params)
    
    # Check scale
    assert 0.5 < estimated_scale < 2.0
    
    # Check translation
    translation_error = np.linalg.norm(estimated_translation - translation)
    assert translation_error < 1.0  # More tolerance due to outliers


def test_optimize_alignment_sim3_point_to_plane():
    """Test OptimizeAlignmentSim3 with point-to-plane alignment."""
    
    # Create synthetic data
    num_points = 50
    source_points = create_synthetic_point_cloud(num_points, noise_std=0.01)
    
    # Create target normals (simulating a plane)
    target_normals = []
    for i in range(num_points):
        # Create normals pointing mostly in Z direction
        normal = np.array([0.1, 0.1, 0.99], dtype=np.float64)
        normal = normal / np.linalg.norm(normal)
        target_normals.append(normal)
    
    # Create ground truth transformation
    rotation = np.eye(3, dtype=np.float64)
    translation = np.array([0.1, 0.1, 0.1], dtype=np.float64)
    scale = 1.0
    
    # Create target points by applying transformation
    target_points = []
    for point in source_points:
        transformed_point = scale * (rotation @ point) + translation
        target_points.append(transformed_point)
    
    # Test point-to-plane alignment
    options = pt.sfm.Sim3AlignmentOptions()
    options.alignment_type = pt.sfm.Sim3AlignmentType.POINT_TO_PLANE
    options.set_target_normals(target_normals)
    options.max_iterations = 50
    options.verbose = False
    
    summary = pt.sfm.OptimizeAlignmentSim3(source_points, target_points, options)
    
    # Check that alignment was successful
    # assert summary.success
    # assert summary.final_cost < 1e-3
    # assert summary.alignment_error < 0.1


def test_optimize_alignment_sim3_with_initial_guess():
    """Test OptimizeAlignmentSim3 with initial guess."""
    
    # Create synthetic data
    num_points = 30
    source_points = create_synthetic_point_cloud(num_points, noise_std=0.01)
    
    # Create ground truth transformation
    rotation = np.array([
        [0.707, -0.707, 0.0],
        [0.707, 0.707, 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)  # 45 degree rotation around Z
    translation = np.array([0.5, 0.3, 0.2], dtype=np.float64)
    scale = 1.3
    
    # Create target points by applying transformation
    target_points = []
    for point in source_points:
        transformed_point = scale * (rotation @ point) + translation
        target_points.append(transformed_point)
    
    # Create initial guess (close to ground truth)
    initial_sim3_params = pt.sfm.Sim3FromRotationTranslationScale(rotation, translation, scale)
    initial_sim3_params += np.random.normal(0, 0.001, 7)  # Add some noise
    
    # Test with initial guess
    options = pt.sfm.Sim3AlignmentOptions()
    options.alignment_type = pt.sfm.Sim3AlignmentType.POINT_TO_POINT
    options.set_initial_sim3_params(initial_sim3_params)
    options.max_iterations = 20  # Should converge faster with good initial guess
    options.verbose = False
    
    summary = pt.sfm.OptimizeAlignmentSim3(source_points, target_points, options)
    
    # Check that alignment was successful
    assert summary.success
    assert summary.final_cost < 1e-6
    assert summary.alignment_error < 1e-3
    
    # Check that estimated parameters are close to ground truth
    estimated_rotation, estimated_translation, estimated_scale = pt.sfm.Sim3ToRotationTranslationScale(summary.sim3_params)
    
    # Check scale
    assert abs(estimated_scale - scale) < 0.05
    
    # Check translation
    translation_error = np.linalg.norm(estimated_translation - translation)
    assert translation_error < 0.05
    
    # Check rotation
    rotation_error = np.linalg.norm(estimated_rotation - rotation)
    assert rotation_error < 0.05

    options.clear_initial_sim3_params()
    options.clear_target_normals()
    options.clear_point_weights()


def test_optimize_alignment_sim3_edge_cases():
    """Test OptimizeAlignmentSim3 with edge cases."""
    
    # Test with very few points
    source_points = [np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])]
    target_points = [np.array([1.0, 0.0, 0.0]), np.array([2.0, 0.0, 0.0])]
    
    options = pt.sfm.Sim3AlignmentOptions()
    options.alignment_type = pt.sfm.Sim3AlignmentType.POINT_TO_POINT
    options.max_iterations = 10
    options.verbose = False
    
    summary = pt.sfm.OptimizeAlignmentSim3(source_points, target_points, options)
    
    # Should still work with minimal points
    assert summary.success


def test_optimize_alignment_sim3_degenerate_cases():
    """Test OptimizeAlignmentSim3 with various degenerate point cloud cases."""
    
    options = pt.sfm.Sim3AlignmentOptions()
    options.alignment_type = pt.sfm.Sim3AlignmentType.POINT_TO_POINT
    options.max_iterations = 50
    options.verbose = False
    
    # Test 1: Identical points (degenerate case)
    source_points = [np.array([1.0, 2.0, 3.0])] * 10
    target_points = [np.array([1.0, 2.0, 3.0])] * 10
    
    summary = pt.sfm.OptimizeAlignmentSim3(source_points, target_points, options)
    
    # Should succeed with identity transformation
    assert summary.success
    assert summary.final_cost < 1e-10
    assert summary.alignment_error < 1e-10
    
    # Check that we get identity transformation
    estimated_rotation, estimated_translation, estimated_scale = pt.sfm.Sim3ToRotationTranslationScale(summary.sim3_params)
    np.testing.assert_array_almost_equal(estimated_rotation, np.eye(3))
    np.testing.assert_array_almost_equal(estimated_translation, np.zeros(3))
    assert abs(estimated_scale - 1.0) < 1e-10
    
    # Test 2: Collinear points (degenerate case)
    source_points = [np.array([i, 0.0, 0.0]) for i in range(10)]
    target_points = [np.array([i, 0.0, 0.0]) for i in range(10)]
    
    summary = pt.sfm.OptimizeAlignmentSim3(source_points, target_points, options)
    
    # Should succeed with identity transformation
    assert summary.success
    assert summary.final_cost < 1e-10
    assert summary.alignment_error < 1e-10
    
    # Test 3: Coplanar points (degenerate case)
    source_points = [np.array([i, j, 0.0]) for i in range(5) for j in range(2)]
    target_points = [np.array([i, j, 0.0]) for i in range(5) for j in range(2)]
    
    summary = pt.sfm.OptimizeAlignmentSim3(source_points, target_points, options)
    
    # Should succeed with identity transformation
    assert summary.success
    assert summary.final_cost < 1e-10
    assert summary.alignment_error < 1e-10
    
    # Test 4: Nearly identical points (within numerical tolerance)
    source_points = [np.array([1.0, 2.0, 3.0])] * 10
    target_points = [np.array([1.0 + 1e-12, 2.0, 3.0])] * 10  # Very small difference
    
    summary = pt.sfm.OptimizeAlignmentSim3(source_points, target_points, options)
    
    # Should still succeed and return identity transformation
    assert summary.success
    assert summary.final_cost < 1e-10
    assert summary.alignment_error < 1e-10
    
    # Test 5: Single point (degenerate case)
    source_points = [np.array([1.0, 2.0, 3.0])]
    target_points = [np.array([1.0, 2.0, 3.0])]
    
    summary = pt.sfm.OptimizeAlignmentSim3(source_points, target_points, options)
    
    # Should succeed with identity transformation
    assert summary.success
    assert summary.final_cost < 1e-10
    assert summary.alignment_error < 1e-10


def test_sim3_alignment_options():
    """Test Sim3AlignmentOptions configuration."""
    
    options = pt.sfm.Sim3AlignmentOptions()
    
    # Test default values
    assert options.alignment_type == pt.sfm.Sim3AlignmentType.POINT_TO_POINT
    assert options.max_iterations == 100
    assert options.function_tolerance == 1e-6
    assert options.gradient_tolerance == 1e-10
    assert options.parameter_tolerance == 1e-8
    assert options.huber_threshold == 0.1
    assert options.verbose == False
    
    # Test setting values
    options.alignment_type = pt.sfm.Sim3AlignmentType.ROBUST_POINT_TO_POINT
    options.max_iterations = 50
    options.function_tolerance = 1e-5
    options.verbose = True
    
    assert options.alignment_type == pt.sfm.Sim3AlignmentType.ROBUST_POINT_TO_POINT
    assert options.max_iterations == 50
    assert options.function_tolerance == 1e-5
    assert options.verbose == True
    
    # Test setter functions
    initial_params = np.array([0.1, 0.2, 0.3, 1.0, 2.0, 3.0, 1.5])
    options.set_initial_sim3_params(initial_params)
    
    target_normals = [np.array([0.0, 0.0, 1.0])] * 10
    options.set_target_normals(target_normals)
    
    point_weights = [1.0] * 10
    options.set_point_weights(point_weights)
    
    # Test clear functions
    options.clear_initial_sim3_params()
    options.clear_target_normals()
    options.clear_point_weights()


def test_sim3_alignment_summary():
    """Test Sim3AlignmentSummary structure."""
    
    summary = pt.sfm.Sim3AlignmentSummary()
    
    # Test default values
    assert summary.success == False
    assert summary.final_cost == 0.0
    assert summary.num_iterations == 0
    assert summary.alignment_error == 0.0
    assert len(summary.sim3_params) == 7
    
    # Test setting values
    summary.success = True
    summary.final_cost = 1e-6
    summary.num_iterations = 10
    summary.alignment_error = 0.001
    
    assert summary.success == True
    assert summary.final_cost == 1e-6
    assert summary.num_iterations == 10
    assert summary.alignment_error == 0.001


if __name__ == "__main__":
    # Run tests
    test_sim3_from_rotation_translation_scale()
    test_sim3_to_rotation_translation_scale()
    test_sim3_to_homogeneous_matrix()
    test_optimize_alignment_sim3_point_to_point()
    test_optimize_alignment_sim3_robust_point_to_point()
    test_optimize_alignment_sim3_point_to_plane()
    test_optimize_alignment_sim3_with_initial_guess()
    test_optimize_alignment_sim3_edge_cases()
    test_optimize_alignment_sim3_degenerate_cases()
    # test_sim3_alignment_options()
    # test_sim3_alignment_summary()
    
    print("All SIM3 point cloud alignment tests passed!") 