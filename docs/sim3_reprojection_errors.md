# SIM3 Reprojection Errors

This document describes the SIM3 reprojection error implementations in TheiaSfM, which provide scale-aware optimization capabilities for Structure-from-Motion problems.

## Overview

SIM3 (Similarity Transform in 3D) reprojection errors extend the standard SE3 reprojection errors by including scale as an additional parameter. This is particularly useful for:

- Monocular SfM where scale is ambiguous
- Multi-view scenarios with varying scales
- Bundle adjustment with scale-aware optimization
- Relative pose estimation with scale consistency

## Available Error Functions

### 1. Sim3ReprojectionError

The standard SIM3 reprojection error for single camera pose optimization.

**Parameters:**
- `extrinsic_parameters`: 7-dimensional SIM3 pose [rx, ry, rz, tx, ty, tz, scale]
- `intrinsic_parameters`: Camera intrinsic parameters
- `point`: 3D point coordinates [x, y, z]
- `reprojection_error`: Output 2D reprojection error [dx, dy]

**Usage:**
```cpp
ceres::CostFunction* cost_function = 
    new ceres::AutoDiffCostFunction<Sim3ReprojectionError<PinholeCameraModel>, 2, 7, 4, 3>(
        new Sim3ReprojectionError<PinholeCameraModel>(feature));
```

### 2. Sim3RelativeReprojectionError

Reprojection error for relative pose estimation between two SIM3 poses.

**Parameters:**
- `sim3_pose1`: First SIM3 pose [7 parameters]
- `sim3_pose2`: Second SIM3 pose [7 parameters]
- `intrinsic_parameters`: Camera intrinsic parameters
- `point`: 3D point coordinates [x, y, z]
- `reprojection_error`: Output 4D reprojection error [dx1, dy1, dx2, dy2]

**Usage:**
```cpp
ceres::CostFunction* cost_function = 
    new ceres::AutoDiffCostFunction<Sim3RelativeReprojectionError<PinholeCameraModel>, 4, 7, 7, 4, 3>(
        new Sim3RelativeReprojectionError<PinholeCameraModel>(feature));
```

### 3. Sim3PoseOnlyReprojectionError

Pose-only optimization with fixed 3D points.

**Parameters:**
- `extrinsic_parameters`: SIM3 pose [7 parameters]
- `intrinsic_parameters`: Camera intrinsic parameters
- `reprojection_error`: Output 2D reprojection error [dx, dy]

**Usage:**
```cpp
ceres::CostFunction* cost_function = 
    new ceres::AutoDiffCostFunction<Sim3PoseOnlyReprojectionError<PinholeCameraModel>, 2, 7, 4>(
        new Sim3PoseOnlyReprojectionError<PinholeCameraModel>(feature, world_point));
```

## SIM3 Parameterization

The SIM3 pose is parameterized using the Lie algebra representation:

```
[ω_x, ω_y, ω_z, v_x, v_y, v_z, s]
```

Where:
- `[ω_x, ω_y, ω_z]`: Rotation (angle-axis representation)
- `[v_x, v_y, v_z]`: Translation
- `s`: Scale factor

The exponential map converts this to the SIM3 group:
```
SIM3 = exp([ω, v, s])
```

## Advantages of SIM3 Reprojection Errors

1. **Scale Consistency**: Handles scale changes consistently across multiple views
2. **Monocular Compatibility**: Works well with monocular SfM where scale is ambiguous
3. **Robust Optimization**: Better convergence for scenes with varying scales
4. **Flexible Parameterization**: Can optimize scale independently or fix it to 1.0

## Example Applications

### Bundle Adjustment with SIM3
```cpp
// Add residual blocks for multiple observations
for (const auto& observation : observations) {
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<Sim3ReprojectionError<PinholeCameraModel>, 2, 7, 4, 3>(
            new Sim3ReprojectionError<PinholeCameraModel>(observation.feature));
    
    problem.AddResidualBlock(cost_function, loss_function, 
                            sim3_pose, intrinsics, point);
}
```

### Relative Pose Estimation
```cpp
// Estimate relative SIM3 transformation
ceres::CostFunction* cost_function =
    new ceres::AutoDiffCostFunction<Sim3RelativeReprojectionError<PinholeCameraModel>, 4, 7, 7, 4, 3>(
        new Sim3RelativeReprojectionError<PinholeCameraModel>(feature));

problem.AddResidualBlock(cost_function, loss_function, 
                        sim3_pose1, sim3_pose2, intrinsics, point);
```

### Pose-Only Optimization
```cpp
// Optimize only the camera pose with fixed 3D points
ceres::CostFunction* cost_function =
    new ceres::AutoDiffCostFunction<Sim3PoseOnlyReprojectionError<PinholeCameraModel>, 2, 7, 4>(
        new Sim3PoseOnlyReprojectionError<PinholeCameraModel>(feature, fixed_point));

problem.AddResidualBlock(cost_function, loss_function, sim3_pose, intrinsics);
```

## Integration with TheiaSfM

The SIM3 reprojection errors are designed to integrate seamlessly with TheiaSfM's existing pipeline:

1. **Camera Models**: Compatible with all camera models (Pinhole, Fisheye, etc.)
2. **Bundle Adjustment**: Can be used in bundle adjustment routines
3. **Reconstruction**: Works with Theia's reconstruction data structures
4. **Optimization**: Compatible with Ceres Solver optimization framework

## Performance Considerations

- **Computational Cost**: SIM3 operations are slightly more expensive than SE3
- **Memory Usage**: 7 parameters per pose instead of 6
- **Convergence**: May require more iterations but often achieves better final results
- **Numerical Stability**: Careful handling of scale parameters is important

## Best Practices

1. **Initialization**: Start with reasonable scale estimates (close to 1.0)
2. **Constraints**: Consider adding scale priors for known-scale scenarios
3. **Regularization**: Use appropriate loss functions for robust optimization
4. **Validation**: Check scale consistency across the reconstruction

## Related Work

- [Sophus](https://github.com/strasdat/Sophus): C++ Lie groups library
- [Ceres Solver](http://ceres-solver.org/): Nonlinear optimization library
- [TheiaSfM](https://github.com/sweeneychris/TheiaSfM): Structure-from-Motion library

## References

1. Strasdat, H., Montiel, J. M. M., & Davison, A. J. (2010). Scale drift-aware large scale monocular SLAM. RSS.
2. Barfoot, T. D. (2017). State estimation for robotics. Cambridge University Press.
3. Sweeney, C. (2015). Theia multiview geometry library: Tutorial & reference. University of California, Santa Barbara. 