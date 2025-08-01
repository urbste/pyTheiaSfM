#!/usr/bin/env python3
"""
Example demonstrating SIM3 point cloud alignment using pytheia.

This example shows how to:
1. Create synthetic point clouds with known transformations
2. Use different alignment types (point-to-point, robust point-to-point, point-to-plane)
3. Use point weights for weighted alignment
4. Use Ceres's built-in Huber loss for robust alignment
5. Convert between different SIM3 representations
"""

import numpy as np
import pytheia as pt

def create_synthetic_point_cloud(num_points=100, noise_std=0.01):
    """Create a synthetic point cloud with random 3D points."""
    points = np.random.randn(num_points, 3) * 2.0
    return [np.array([p[0], p[1], p[2]]) for p in points]

def create_synthetic_normals(num_points=100):
    """Create synthetic surface normals for point-to-plane alignment."""
    normals = np.random.randn(num_points, 3)
    # Normalize the normals
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = normals / norms
    return [np.array([n[0], n[1], n[2]]) for n in normals]

def create_point_weights(num_points=100, outlier_ratio=0.1):
    """Create point weights with some outliers having lower weights.
    
    Higher weights indicate higher confidence in the point correspondence.
    Lower weights are used for outliers or uncertain correspondences.
    """
    weights = np.ones(num_points)
    num_outliers = int(num_points * outlier_ratio)
    outlier_indices = np.random.choice(num_points, num_outliers, replace=False)
    weights[outlier_indices] = 0.1  # Lower weight for outliers (lower confidence)
    
    # Add some variation to other weights to simulate different confidence levels
    for i in range(num_points):
        if i not in outlier_indices:
            # Random confidence between 0.5 and 1.0
            weights[i] = 0.5 + 0.5 * np.random.random()
    
    return weights.tolist()

def main():
    print("SIM3 Point Cloud Alignment Example")
    print("=" * 40)
    
    # Create synthetic data
    num_points = 100
    source_points = create_synthetic_point_cloud(num_points, noise_std=0.01)
    target_normals = create_synthetic_normals(num_points)
    point_weights = create_point_weights(num_points, outlier_ratio=0.1)
    
    # Define ground truth transformation
    # Translation: [1.0, 2.0, 0.5]
    # Rotation: small rotation around [0.1, 0.05, 0.02] axis
    # Scale: 1.5
    ground_truth_sim3 = pt.sfm.Sim3FromRotationTranslationScale(
        rotation=np.eye(3),  # Start with identity
        translation=np.array([1.0, 2.0, 0.5]),
        scale=1.5
    )
    
    # Apply small rotation
    angle =  0.1
    axis = np.array([0.1, 0.05, 0.02]).normalized()
    rotation = np.eye(3)
    ground_truth_sim3 = pt.sfm.Sim3FromRotationTranslationScale(
        rotation=rotation,
        translation=np.array([1.0, 2.0, 0.5]),
        scale=1.5
    )
    
    # Transform source points to create target points
    target_points = []
    for point in source_points:
        # Create SIM3 transformation and apply it
        sim3_obj = pt.Sophus.Sim3d.exp(ground_truth_sim3)
        transformed_point = sim3_obj * point
        target_points.append(transformed_point)
    
    # Add some outliers to target points
    num_outliers = 5
    for i in range(num_outliers):
        target_points[i] = np.array([
            target_points[i][0] + 3.0,
            target_points[i][1] + 3.0,
            target_points[i][2] + 3.0
        ])
    
    print(f"Created {num_points} point pairs with {num_outliers} outliers")
    print(f"Ground truth SIM3 parameters: {ground_truth_sim3}")
    
    # Test 1: Point-to-point alignment
    print("\n1. Point-to-point alignment")
    print("-" * 30)
    
    options = pt.sfm.Sim3AlignmentOptions()
    options.alignment_type = pt.sfm.Sim3AlignmentType.POINT_TO_POINT
    options.max_iterations = 100
    options.verbose = False
    
    summary = pt.sfm.OptimizeAlignmentSim3(source_points, target_points, options)
    
    print(f"Success: {summary.success}")
    print(f"Final cost: {summary.final_cost:.6f}")
    print(f"Alignment error: {summary.alignment_error:.6f}")
    print(f"Estimated SIM3: {summary.sim3_params}")
    
    # Convert to homogeneous matrix
    homogeneous_matrix = pt.sfm.Sim3ToHomogeneousMatrix(summary.sim3_params)
    print(f"Homogeneous matrix:\n{homogeneous_matrix}")
    
    # Test 2: Robust point-to-point alignment with Huber loss
    print("\n2. Robust point-to-point alignment (Huber loss)")
    print("-" * 50)
    
    options.alignment_type = pt.sfm.Sim3AlignmentType.ROBUST_POINT_TO_POINT
    options.huber_threshold = 1.0
    
    summary_robust = pt.sfm.OptimizeAlignmentSim3(source_points, target_points, options)
    
    print(f"Success: {summary_robust.success}")
    print(f"Final cost: {summary_robust.final_cost:.6f}")
    print(f"Alignment error: {summary_robust.alignment_error:.6f}")
    print(f"Estimated SIM3: {summary_robust.sim3_params}")
    
    # Test 3: Point-to-plane alignment
    print("\n3. Point-to-plane alignment")
    print("-" * 30)
    
    options.alignment_type = pt.sfm.Sim3AlignmentType.POINT_TO_PLANE
    options.target_normals = target_normals
    
    summary_plane = pt.sfm.OptimizeAlignmentSim3(source_points, target_points, options)
    
    print(f"Success: {summary_plane.success}")
    print(f"Final cost: {summary_plane.final_cost:.6f}")
    print(f"Alignment error: {summary_plane.alignment_error:.6f}")
    print(f"Estimated SIM3: {summary_plane.sim3_params}")
    
    # Test 4: Weighted alignment
    print("\n4. Weighted alignment")
    print("-" * 20)
    print("Using individual point weights where higher weights indicate higher confidence")
    print(f"Point weights range: {min(point_weights):.2f} to {max(point_weights):.2f}")
    
    options.alignment_type = pt.sfm.Sim3AlignmentType.ROBUST_POINT_TO_POINT
    options.point_weights = point_weights
    # Note: point_weight is only used if point_weights is not provided
    
    summary_weighted = pt.sfm.OptimizeAlignmentSim3(source_points, target_points, options)
    
    print(f"Success: {summary_weighted.success}")
    print(f"Final cost: {summary_weighted.final_cost:.6f}")
    print(f"Alignment error: {summary_weighted.alignment_error:.6f}")
    print(f"Estimated SIM3: {summary_weighted.sim3_params}")
    
    # Test 5: Scale-constrained alignment
    print("\n5. Scale-constrained alignment")
    print("-" * 30)
    
    options.alignment_type = pt.sfm.Sim3AlignmentType.POINT_TO_POINT
    options.scale_constraint = pt.sfm.Sim3ScaleConstraint(target_scale=1.0, weight=10.0)
    
    summary_constrained = pt.sfm.OptimizeAlignmentSim3(source_points, target_points, options)
    
    print(f"Success: {summary_constrained.success}")
    print(f"Final cost: {summary_constrained.final_cost:.6f}")
    print(f"Alignment error: {summary_constrained.alignment_error:.6f}")
    print(f"Estimated SIM3: {summary_constrained.sim3_params}")
    
    # Convert estimated parameters back to components
    rotation, translation, scale = pt.sfm.Sim3ToRotationTranslationScale(summary_robust.sim3_params)
    print(f"\nDecomposed transformation:")
    print(f"Rotation:\n{rotation}")
    print(f"Translation: {translation}")
    print(f"Scale: {scale}")
    
    # Compare with ground truth
    gt_rotation, gt_translation, gt_scale = pt.sfm.Sim3ToRotationTranslationScale(ground_truth_sim3)
    print(f"\nGround truth:")
    print(f"Rotation:\n{gt_rotation}")
    print(f"Translation: {gt_translation}")
    print(f"Scale: {gt_scale}")
    
    # Compute errors
    rotation_error = (rotation - gt_rotation).norm()
    translation_error = (translation - gt_translation).norm()
    scale_error = abs(scale - gt_scale)
    
    print(f"\nErrors:")
    print(f"Rotation error: {rotation_error:.6f}")
    print(f"Translation error: {translation_error:.6f}")
    print(f"Scale error: {scale_error:.6f}")
    
    print("\nExample completed successfully!")

if __name__ == "__main__":
    main() 