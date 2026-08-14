#!/usr/bin/env python3
"""
Comprehensive benchmark comparing two-view essential matrix solvers in pytheia:
  1. FivePointRelativePose (Stewenius Groebner basis 5-point solver)
  2. FivePointRelativePoseSturm (Nister / PoseLib Sturm sequence 5-point solver)
  3. FastIterativeFivePoint (Hedborg & Felsberg Powell's Dogleg solver, forward-facing prior)
  4. MonodepthThreePoint (PoseLib / RePoseD 3-point solver with monocular depth priors)

Evaluates:
  - Raw minimal solver latency (microseconds per solve)
  - End-to-end RANSAC runtime and iteration count
  - Robustness to noise and outlier ratios
  - Forward-facing motion trajectory regime vs. general motion
"""

import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import pytheia as pt


def generate_synthetic_points(num_points, z_min=3.0, z_max=12.0, rng=None):
    if rng is None:
        rng = np.random.default_rng(42)
    x = rng.uniform(-1.5, 1.5, size=num_points)
    y = rng.uniform(-1.5, 1.5, size=num_points)
    z = rng.uniform(z_min, z_max, size=num_points)
    return np.column_stack([x, y, z])


def generate_scene(
    num_points=100,
    rot_deg=(1.0, 1.0, 0.5),
    translation=np.array([0.02, 0.01, -1.0]),
    noise_std=0.001,
    outlier_ratio=0.0,
    focal=1000.0,
    principal_point=(500.0, 500.0),
    rng=None,
):
    if rng is None:
        rng = np.random.default_rng(42)

    pts3d = generate_synthetic_points(num_points, rng=rng)
    rot = R.from_euler("xyz", rot_deg, degrees=True).as_matrix()
    trans = np.array(translation, dtype=float)
    trans = trans / np.linalg.norm(trans)

    # Camera 1 normalized coords
    x1_norm = pts3d[:, :2] / pts3d[:, 2:3]

    # Camera 2 3D points and normalized coords
    pts3d_cam2 = (rot @ pts3d.T).T + trans
    x2_norm = pts3d_cam2[:, :2] / pts3d_cam2[:, 2:3]

    # Convert to pixel coords
    px1 = x1_norm * focal + np.array(principal_point)
    px2 = x2_norm * focal + np.array(principal_point)

    # Add Gaussian noise
    if noise_std > 0:
        px1 += rng.normal(0, noise_std * focal, size=px1.shape)
        px2 += rng.normal(0, noise_std * focal, size=px2.shape)

    # Add outliers
    num_outliers = int(num_points * outlier_ratio)
    if num_outliers > 0:
        outlier_indices = rng.choice(num_points, size=num_outliers, replace=False)
        px2[outlier_indices] = rng.uniform(0, 1000.0, size=(num_outliers, 2))

    # Feature correspondences
    corrs = []
    for i in range(num_points):
        f1 = pt.sfm.Feature(px1[i])
        f2 = pt.sfm.Feature(px2[i])
        f1.depth_prior = pts3d[i, 2]
        f2.depth_prior = pts3d_cam2[i, 2]
        corrs.append(pt.matching.FeatureCorrespondence(f1, f2))

    return {
        "pts3d": pts3d,
        "x1_norm": x1_norm,
        "x2_norm": x2_norm,
        "px1": px1,
        "px2": px2,
        "rot_gt": rot,
        "trans_gt": trans,
        "corrs": corrs,
        "focal": focal,
        "pp": principal_point,
    }


def benchmark_raw_minimal_solvers(num_trials=5000):
    print("\n" + "=" * 70)
    print(f"1. Raw Minimal Solver Latency Benchmark ({num_trials} iterations)")
    print("=" * 70)

    rng = np.random.default_rng(123)
    scene = generate_scene(num_points=10, rot_deg=(1.0, -1.0, 0.5), translation=[0.02, 0.01, -1.0], rng=rng)
    x1_5 = scene["x1_norm"][:5]
    x2_5 = scene["x2_norm"][:5]

    # Warmup
    for _ in range(50):
        pt.sfm.FivePointRelativePose(x1_5, x2_5)
        pt.sfm.FivePointRelativePoseSturm(x1_5, x2_5)
        pt.sfm.FastIterativeFivePointRelativePose(x1_5, x2_5)

    # Stewenius 5pt
    t0 = time.perf_counter()
    for _ in range(num_trials):
        pt.sfm.FivePointRelativePose(x1_5, x2_5)
    t_stew = (time.perf_counter() - t0) / num_trials * 1e6

    # Sturm 5pt (PoseLib)
    t0 = time.perf_counter()
    for _ in range(num_trials):
        pt.sfm.FivePointRelativePoseSturm(x1_5, x2_5)
    t_sturm = (time.perf_counter() - t0) / num_trials * 1e6

    # Fast Iterative (Dogleg) 5pt (max 8 iters)
    opts8 = pt.sfm.FastIterativeFivePointOptions()
    opts8.max_iterations = 8
    t0 = time.perf_counter()
    for _ in range(num_trials):
        pt.sfm.FastIterativeFivePointRelativePose(x1_5, x2_5, opts8)
    t_dogleg8 = (time.perf_counter() - t0) / num_trials * 1e6

    # Fast Iterative (Dogleg) 5pt (max 4 iters)
    opts4 = pt.sfm.FastIterativeFivePointOptions()
    opts4.max_iterations = 4
    t0 = time.perf_counter()
    for _ in range(num_trials):
        pt.sfm.FastIterativeFivePointRelativePose(x1_5, x2_5, opts4)
    t_dogleg4 = (time.perf_counter() - t0) / num_trials * 1e6

    print(f"  Stewenius 5-point (Groebner)       : {t_stew:8.2f} µs / call  (1.00x)")
    print(f"  Sturm 5-point (PoseLib/Nister)     : {t_sturm:8.2f} µs / call  ({t_stew / t_sturm:5.2f}x speedup vs Stew)")
    print(f"  Fast Iterative (Dogleg, <=8 iters) : {t_dogleg8:8.2f} µs / call  ({t_stew / t_dogleg8:5.2f}x speedup vs Stew)")
    print(f"  Fast Iterative (Dogleg, <=4 iters) : {t_dogleg4:8.2f} µs / call  ({t_stew / t_dogleg4:5.2f}x speedup vs Stew)")


def benchmark_ransac_two_view_info(num_trials=200):
    print("\n" + "=" * 70)
    print(f"2. End-to-End RANSAC Two-View Estimation Benchmark ({num_trials} trials)")
    print("=" * 70)

    configs = [
        ("Forward motion (clean, 100 pts)", (1.0, -0.5, 0.2), [0.02, 0.01, 1.0], 0.0, 0.0),
        ("Forward motion (noisy 0.5px, 20% outliers)", (1.5, -1.0, 0.5), [0.03, -0.02, 1.0], 0.0005, 0.2),
        ("Forward motion (noisy 1.0px, 40% outliers)", (2.0, -1.5, 1.0), [-0.05, 0.04, 1.0], 0.001, 0.4),
    ]

    solvers = [
        ("Stewenius 5pt", pt.sfm.TwoViewEstimationMethod.FIVE_POINT_STEWENIUS, False),
        ("Sturm 5pt", pt.sfm.TwoViewEstimationMethod.FIVE_POINT_STURM, False),
        ("Fast Iterative 5pt", pt.sfm.TwoViewEstimationMethod.FAST_ITERATIVE_FIVE_POINT, False),
        ("Monodepth 3pt", pt.sfm.TwoViewEstimationMethod.MONODEPTH_THREE_POINT, True),
    ]

    for regime_name, rot_deg, trans, noise, outlier_r in configs:
        print(f"\n--- Scenario: {regime_name} ---")
        print(f"{'Solver':<22} | {'Time (ms)':<10} | {'R_err (deg)':<12} | {'t_err (deg)':<12} | {'Success':<8}")
        print("-" * 72)

        rng = np.random.default_rng(42)
        scene = generate_scene(
            num_points=120,
            rot_deg=rot_deg,
            translation=trans,
            noise_std=noise,
            outlier_ratio=outlier_r,
            rng=rng,
        )

        cam1 = pt.sfm.Camera()
        cam1.SetFocalLength(scene["focal"])
        cam1.SetPrincipalPoint(scene["pp"][0], scene["pp"][1])
        cam1.SetImageSize(1000, 1000)

        prior1 = cam1.CameraIntrinsicsPriorFromIntrinsics()
        prior2 = cam1.CameraIntrinsicsPriorFromIntrinsics()

        for solver_name, method, use_mono in solvers:
            opts = pt.sfm.EstimateTwoViewInfoOptions()
            opts.max_sampson_error_pixels = 3.0
            opts.estimation_method = method
            opts.use_monodepth = use_mono
            opts.min_ransac_iterations = 10
            opts.max_ransac_iterations = 500

            successes = 0
            rot_errors = []
            trans_errors = []

            t0 = time.perf_counter()
            for t in range(num_trials):
                succ, info, inliers = pt.sfm.EstimateTwoViewInfo(
                    opts, prior1, prior2, scene["corrs"]
                )
                if succ:
                    successes += 1
                    rot_mat = R.from_rotvec(np.array(info.rotation_2, copy=True)).as_matrix()
                    r_err = R.from_matrix(rot_mat @ scene["rot_gt"].T).magnitude() * 180.0 / np.pi
                    rot_errors.append(r_err)

                    t_dir = info.position_2 / np.linalg.norm(info.position_2)
                    cos_t = np.clip(np.abs(np.dot(t_dir, scene["trans_gt"])), -1.0, 1.0)
                    t_err = np.arccos(cos_t) * 180.0 / np.pi
                    trans_errors.append(t_err)

            elapsed_ms = (time.perf_counter() - t0) / num_trials * 1000.0
            mean_r_err = np.mean(rot_errors) if rot_errors else float("nan")
            mean_t_err = np.mean(trans_errors) if trans_errors else float("nan")
            succ_pct = (successes / num_trials) * 100.0

            print(f"{solver_name:<22} | {elapsed_ms:8.3f} ms | {mean_r_err:10.3f}° | {mean_t_err:10.3f}° | {succ_pct:6.1f}%")


def benchmark_motion_regime_boundaries():
    print("\n" + "=" * 70)
    print("3. Motion Regime Analysis: Forward vs. Sideways / Large Rotation")
    print("=" * 70)
    print("Testing convergence and pose accuracy across varying trajectory types:")

    trajectories = [
        ("Pure Forward [0, 0, 1], 1° rot", (1.0, 0.0, 0.0), [0.0, 0.0, 1.0]),
        ("Forward-Dominant [0.1, 0.05, 1], 3° rot", (2.0, -1.5, 1.0), [0.1, 0.05, 1.0]),
        ("Diagonal [0.5, 0.2, 1], 5° rot", (3.0, 3.0, -2.0), [0.5, 0.2, 1.0]),
        ("Sideways [1, 0, 0.1], 2° rot", (1.0, 1.0, 0.0), [1.0, 0.0, 0.1]),
        ("Large Rotation [0, 0, 1], 25° rot", (15.0, 15.0, 10.0), [0.0, 0.0, 1.0]),
    ]

    print(f"{'Trajectory Type':<38} | {'FastIter Dogleg (R / t err)':<28} | {'Sturm 5pt (R / t err)':<25}")
    print("-" * 97)

    for traj_name, rot_deg, trans in trajectories:
        scene = generate_scene(num_points=100, rot_deg=rot_deg, translation=trans, noise_std=0.0005)

        cam = pt.sfm.Camera()
        cam.SetFocalLength(1000.0)
        cam.SetPrincipalPoint(500.0, 500.0)
        cam.SetImageSize(1000, 1000)
        prior = cam.CameraIntrinsicsPriorFromIntrinsics()

        # Fast iterative
        opts_fi = pt.sfm.EstimateTwoViewInfoOptions()
        opts_fi.estimation_method = pt.sfm.TwoViewEstimationMethod.FAST_ITERATIVE_FIVE_POINT
        s_fi, info_fi, _ = pt.sfm.EstimateTwoViewInfo(opts_fi, prior, prior, scene["corrs"])

        if s_fi:
            r_mat = R.from_rotvec(np.array(info_fi.rotation_2, copy=True)).as_matrix()
            r_err_fi = R.from_matrix(r_mat @ scene["rot_gt"].T).magnitude() * 180.0 / np.pi
            t_dir = info_fi.position_2 / np.linalg.norm(info_fi.position_2)
            t_err_fi = np.arccos(np.clip(np.abs(np.dot(t_dir, scene["trans_gt"])), -1.0, 1.0)) * 180.0 / np.pi
            fi_str = f"{r_err_fi:5.2f}° / {t_err_fi:5.2f}°"
        else:
            fi_str = "FAILED"

        # Sturm 5pt
        opts_st = pt.sfm.EstimateTwoViewInfoOptions()
        opts_st.estimation_method = pt.sfm.TwoViewEstimationMethod.FIVE_POINT_STURM
        s_st, info_st, _ = pt.sfm.EstimateTwoViewInfo(opts_st, prior, prior, scene["corrs"])

        if s_st:
            r_mat = R.from_rotvec(np.array(info_st.rotation_2, copy=True)).as_matrix()
            r_err_st = R.from_matrix(r_mat @ scene["rot_gt"].T).magnitude() * 180.0 / np.pi
            t_dir = info_st.position_2 / np.linalg.norm(info_st.position_2)
            t_err_st = np.arccos(np.clip(np.abs(np.dot(t_dir, scene["trans_gt"])), -1.0, 1.0)) * 180.0 / np.pi
            st_str = f"{r_err_st:5.2f}° / {t_err_st:5.2f}°"
        else:
            st_str = "FAILED"

        print(f"{traj_name:<38} | {fi_str:<28} | {st_str:<25}")


if __name__ == "__main__":
    print("Running Two-View Essential Matrix Solvers Benchmark in pytheia...")
    benchmark_raw_minimal_solvers()
    benchmark_ransac_two_view_info()
    benchmark_motion_regime_boundaries()
    print("\nBenchmark completed successfully.")
