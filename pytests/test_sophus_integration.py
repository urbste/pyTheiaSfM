#!/usr/bin/env python3
"""
Integration tests for Sophus SE3d and Sim3d bindings.

This test demonstrates practical usage in computer vision scenarios:
- Camera pose estimation
- Point cloud alignment
- Trajectory optimization
- Bundle adjustment
"""

import numpy as np
import pytest
import pytheia as pt


class TestSophusIntegration:
    """Integration tests for Sophus bindings."""
    
    def test_camera_pose_estimation_scenario(self):
        """Test SE3d in a camera pose estimation scenario."""
        # Simulate camera pose estimation
        # Ground truth camera pose
        true_rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])  # 90 degree rotation around Z
        true_translation = np.array([1.0, 2.0, 3.0])
        true_pose = pt.math.SE3d(true_rotation, true_translation)
        
        # Create some 3D points
        points_3d = np.array([
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [1, 1, 0],
            [0, 0, 1]
        ])
        
        # Transform points to camera coordinates
        points_camera = []
        for point in points_3d:
            transformed_point = true_pose * point
            points_camera.append(transformed_point)
        
        # Simulate noisy measurements
        noise_std = 0.01
        noisy_points = []
        for point in points_camera:
            noise = np.random.normal(0, noise_std, 3)
            noisy_points.append(point + noise)
        
        # Now try to estimate the pose from the noisy measurements
        # This is a simplified version - in practice you'd use PnP algorithms
        
        # For this test, we'll just verify that the transformations work correctly
        estimated_pose = pt.math.SE3d(true_rotation, true_translation)  # Assume perfect estimation
        
        # Test that the estimated pose can transform points back
        reconstructed_points = []
        for point in noisy_points:
            # Transform back using inverse of estimated pose
            reconstructed_point = estimated_pose.inverse() * point
            reconstructed_points.append(reconstructed_point)
        
        # Check that reconstructed points are close to original
        for i, (original, reconstructed) in enumerate(zip(points_3d, reconstructed_points)):
            error = np.linalg.norm(original - reconstructed)
            assert error < 0.1, f"Point {i} reconstruction error too large: {error}"
    
    def test_point_cloud_alignment_scenario(self):
        """Test Sim3d in a point cloud alignment scenario."""
        # Simulate point cloud alignment with scale differences
        
        # Create source point cloud
        source_points = np.array([
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [1, 1, 0]
        ])
        
        # Define ground truth transformation
        true_rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])  # 90 degree rotation
        true_translation = np.array([1.0, 2.0, 3.0])
        true_scale = 2.0
        true_transform = pt.math.Sim3d(true_rotation, true_translation, true_scale)
        
        # Transform source points to create target point cloud
        target_points = []
        for point in source_points:
            transformed_point = true_transform * point
            target_points.append(transformed_point)
        
        # Add some noise to target points
        noise_std = 0.01
        noisy_target_points = []
        for point in target_points:
            noise = np.random.normal(0, noise_std, 3)
            noisy_target_points.append(point + noise)
        
        # Test that we can recover the transformation
        # In practice, you'd use ICP or similar algorithms
        estimated_transform = pt.math.Sim3d(true_rotation, true_translation, true_scale)
        
        # Verify the transformation properties

        assert np.abs(estimated_transform.scale() - true_scale) < 1e-6

        np.testing.assert_array_almost_equal(estimated_transform.translation(), true_translation)
        np.testing.assert_array_almost_equal(
            estimated_transform.matrix3x4()[:3, :3]/estimated_transform.scale(), true_rotation)
        
        # Test point transformation consistency
        for i, (source_point, target_point) in enumerate(zip(source_points, noisy_target_points)):
            # Transform source point with estimated transform
            transformed_source = estimated_transform * source_point
            
            # Check that transformed point is close to target
            error = np.linalg.norm(transformed_source - target_point)
            assert error < 0.1, f"Point {i} alignment error too large: {error}"
    
    def test_trajectory_optimization_scenario(self):
        """Test SE3d in a trajectory optimization scenario."""
        # Simulate a sequence of camera poses
        poses = []
        
        # Create a simple trajectory: move in a circle
        num_poses = 8
        radius = 2.0
        height = 1.0
        
        for i in range(num_poses):
            angle = 2 * np.pi * i / num_poses
            
            # Position
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            z = height
            
            # Orientation: always look at center
            # Simplified: just rotate around Z axis
            rotation = np.array([
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle), np.cos(angle), 0],
                [0, 0, 1]
            ])
            
            translation = np.array([x, y, z])
            pose = pt.math.SE3d(rotation, translation)
            poses.append(pose)
        
        # Test trajectory properties
        assert len(poses) == num_poses
        
        # Test that poses are valid transformations
        for i, pose in enumerate(poses):
            # Check that inverse * pose = identity
            identity = pose.inverse() * pose
            identity_matrix = identity.matrix()
            np.testing.assert_array_almost_equal(identity_matrix, np.eye(4), decimal=10)
            
            # Check that pose can transform points
            test_point = np.array([1.0, 0.0, 0.0])
            transformed_point = pose * test_point
            assert transformed_point.shape == (3,)
    
    def test_bundle_adjustment_scenario(self):
        """Test SE3d in a bundle adjustment scenario."""
        # Simulate bundle adjustment with multiple cameras and 3D points
        
        # Create multiple camera poses
        camera_poses = []
        for i in range(3):
            # Different camera positions
            translation = np.array([i * 2.0, 0.0, 1.0])
            rotation = np.eye(3)  # Identity rotation for simplicity
            pose = pt.math.SE3d(rotation, translation)
            camera_poses.append(pose)
        
        # Create 3D points
        points_3d = np.array([
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [1, 1, 0],
            [0, 0, 1],
            [1, 0, 1],
            [0, 1, 1],
            [1, 1, 1]
        ])
        
        # Project points to each camera
        projections = {}
        for i, pose in enumerate(camera_poses):
            projections[i] = []
            for point_3d in points_3d:
                # Transform point to camera coordinates
                point_camera = pose * point_3d
                
                # Simple perspective projection (assuming camera at origin)
                if point_camera[2] > 0:  # Only project points in front of camera
                    u = point_camera[0] / point_camera[2]
                    v = point_camera[1] / point_camera[2]
                    projections[i].append((u, v))
                else:
                    projections[i].append(None)
        
        # Test that projections are reasonable
        for camera_id, camera_projections in projections.items():
            for j, projection in enumerate(camera_projections):
                if projection is not None:
                    u, v = projection
                    # Check that projections are finite
                    assert np.isfinite(u) and np.isfinite(v)
    
    def test_lie_algebra_operations(self):
        """Test Lie algebra operations (exp/log)."""
        # Test SE3 exp/log operations
        # Create a tangent vector (6D for SE3)
        tangent_se3 = np.array([0.1, 0.2, 0.3, 1.0, 2.0, 3.0])
        
        # Apply exponential map
        se3_from_exp = pt.math.SE3d.exp(tangent_se3)
        
        # Apply logarithmic map
        log_result = se3_from_exp.log()
        
        # Check that exp(log(x)) ≈ x
        se3_reconstructed = pt.math.SE3d.exp(log_result)
        
        # Compare matrices
        original_matrix = se3_from_exp.matrix()
        reconstructed_matrix = se3_reconstructed.matrix()
        np.testing.assert_array_almost_equal(original_matrix, reconstructed_matrix, decimal=10)
        
        # Test Sim3 exp/log operations
        # Create a tangent vector (7D for Sim3)
        tangent_sim3 = np.array([0.1, 0.2, 0.3, 1.0, 2.0, 3.0, 0.5])
        
        # Apply exponential map
        sim3_from_exp = pt.math.Sim3d.exp(tangent_sim3)
        
        # Apply logarithmic map
        log_result_sim3 = sim3_from_exp.log()
        
        # Check that exp(log(x)) ≈ x
        sim3_reconstructed = pt.math.Sim3d.exp(log_result_sim3)
        
        # Compare matrices
        original_matrix_sim3 = sim3_from_exp.matrix()
        reconstructed_matrix_sim3 = sim3_reconstructed.matrix()
        np.testing.assert_array_almost_equal(original_matrix_sim3, reconstructed_matrix_sim3, decimal=10)
    
    def test_geometric_operations(self):
        """Test various geometric operations."""
        # Test rotation composition
        rotation1 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])  # 90 degree around Z
        rotation2 = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])  # 90 degree around X
        
        se3_1 = pt.math.SE3d(rotation1, np.array([1.0, 0.0, 0.0]))
        se3_2 = pt.math.SE3d(rotation2, np.array([0.0, 1.0, 0.0]))
        
        # Compose transformations
        se3_combined = se3_1 * se3_2
        
        # Test that composition is correct
        expected_rotation = rotation1 @ rotation2
        result_rotation = se3_combined.rotation_matrix()
        np.testing.assert_array_almost_equal(result_rotation, expected_rotation)
        
        # Test scale operations with Sim3
        scale1 = 2.0
        scale2 = 1.5
        
        sim3_1 = pt.math.Sim3d(rotation1, np.array([1.0, 0.0, 0.0]), scale1)
        sim3_2 = pt.math.Sim3d(rotation2, np.array([0.0, 1.0, 0.0]), scale2)
        
        # Compose transformations
        sim3_combined = sim3_1 * sim3_2
        
        # Test that scale composition is correct
        expected_scale = scale1 * scale2
        result_scale = sim3_combined.scale()
        assert result_scale == expected_scale
    
    def test_error_handling_and_edge_cases(self):
        """Test error handling and edge cases."""
        # Test with identity transformations
        identity_se3 = pt.math.SE3d()
        identity_matrix = identity_se3.matrix()
        np.testing.assert_array_almost_equal(identity_matrix, np.eye(4))
        
        identity_sim3 = pt.math.Sim3d()
        identity_matrix_sim3 = identity_sim3.matrix()
        np.testing.assert_array_almost_equal(identity_matrix_sim3, np.eye(4))
        
        # Test with zero translation
        zero_translation = np.array([0.0, 0.0, 0.0])
        se3_zero = pt.math.SE3d(np.eye(3), zero_translation)
        result_translation = se3_zero.translation()
        np.testing.assert_array_almost_equal(result_translation, zero_translation)
        
        # Test with unit scale
        unit_scale = 1.0
        sim3_unit = pt.math.Sim3d(np.eye(3), zero_translation, unit_scale)
        result_scale = sim3_unit.scale()
        assert result_scale == unit_scale
        
        # Test point transformation with identity
        test_point = np.array([1.0, 2.0, 3.0])
        transformed_point = identity_se3 * test_point
        np.testing.assert_array_almost_equal(transformed_point, test_point)


if __name__ == "__main__":
    # Run the tests
    import sys
    import pytest
    sys.exit(pytest.main([__file__, "-v"])) 