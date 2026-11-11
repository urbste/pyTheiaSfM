"""Initialization-free SfM (INIT_FREE) on a synthetic scene. No data download.

A ring of cameras looks at a cloud of points. The observations get pixel noise
and, optionally, outliers. Two parts run on the same kind of scene:

Part A (estimator): INIT_FREE with POSE_ROT, calibrated rays, the unit-circle
camera init, random restarts, the Cauchy loss, projective refinement and the
linear + nonlinear metric upgrade. The global SfM pipeline runs on an identical
rebuild of the input for comparison.

Part B (stages): the stages run one by one (see docs/content/init_free.md,
Usage). Then a self-calibration variant with PIXEL_ISOTROPIC observations and
one unknown focal length per camera.

Errors are the rotation and camera-centre errors after a Sim(3) (Umeyama)
alignment of the estimated centres to the ground truth.

Robustness: the Cauchy loss (VarPro) and the Huber refinement (3 px) handle
moderate outliers. The self-calibration part is less robust: at 1 px noise with
20 % outliers its nonlinear refinement can diverge. The global pipeline has no
outlier handling here and may estimate no views with outliers.

Usage (with a pyTheia build that contains INIT_FREE):
  python pyexamples/sfm/init_free_synthetic.py
  python pyexamples/sfm/init_free_synthetic.py --noise_px 1.0 --restarts 5
  python pyexamples/sfm/init_free_synthetic.py --outlier_ratio 0.2 --seed 1
"""
import argparse
import collections
import time

import numpy as np

import pytheia as pt

WIDTH, HEIGHT = 1000, 800
FOCAL = 800.0
NUM_VIEWS = 12
NUM_POINTS = 300
KEEP_PROB = 0.8          # a point visible in a view is observed with this probability
MIN_COMMON_TRACKS = 20   # view pairs with fewer shared tracks get no edge
ROBUST_LOSS_PX = 3.0     # Cauchy scale (VarPro) and Huber width (refinement), pixels


def make_camera(focal):
    cam = pt.sfm.Camera()
    cam.SetCameraIntrinsicsModelType(pt.sfm.CameraIntrinsicsModelType.PINHOLE)
    cam.SetImageSize(WIDTH, HEIGHT)
    intr = cam.CameraIntrinsics()
    intr.SetFocalLength(focal)
    intr.SetPrincipalPoint(WIDTH / 2.0, HEIGHT / 2.0)
    return cam


def look_at(center, target):
    """World-to-camera rotation of a camera at `center` that looks at `target`."""
    z = (target - center) / np.linalg.norm(target - center)
    x = np.cross([0.0, 1.0, 0.0], z)  # image y axis points down
    x /= np.linalg.norm(x)
    return np.vstack([x, np.cross(z, x), z])


def rotmat_to_aa(R):
    angle = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-12:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return axis / (2.0 * np.sin(angle)) * angle


def rotation_angle_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))


# Ground truth of a scene. View id i is camera i. rotations: world-to-camera
# 3x3 matrices, centers: (n, 3), focals: (n,) in pixels.
Scene = collections.namedtuple("Scene", "recon view_graph rotations centers focals")


def make_scene(seed, noise_px, outlier_ratio, focals=None):
    """Builds the synthetic input. The same arguments give the same scene."""
    rng = np.random.default_rng(seed)
    if focals is None:
        focals = np.full(NUM_VIEWS, FOCAL)
    centers = []
    for i in range(NUM_VIEWS):
        theta = 1.2 * np.pi * i / NUM_VIEWS  # partial ring
        radius = 6.0 + rng.uniform(-0.5, 0.5)
        centers.append([radius * np.sin(theta), rng.uniform(-0.5, 0.5),
                        -radius * np.cos(theta)])
    centers = np.array(centers)
    # Each camera looks at its own random target near the origin. If all optical
    # axes met in one point, the linear self-calibration of Part B would be off by
    # about 50 % in the focal lengths (orthogonality error near 3, which the
    # estimator's checks reject); the nonlinear refinement still recovers them.
    rotations = [look_at(c, rng.uniform(-1.0, 1.0, 3)) for c in centers]

    recon = pt.sfm.Reconstruction()
    view_ids = [recon.AddView(str(i), i, float(i)) for i in range(NUM_VIEWS)]
    assert view_ids == list(range(NUM_VIEWS))
    for i, vid in enumerate(view_ids):
        cam = recon.MutableView(vid).MutableCamera()
        cam.DeepCopy(make_camera(focals[i]))
        cam.SetPosition(centers[i])
        cam.SetOrientationFromRotationMatrix(rotations[i])

    for X in rng.uniform(-1.5, 1.5, size=(NUM_POINTS, 3)):
        obs = []
        for i, vid in enumerate(view_ids):
            x = rotations[i] @ (X - centers[i])
            if x[2] <= 0.0:
                continue
            pixel = np.array([focals[i] * x[0] / x[2] + WIDTH / 2.0,
                              focals[i] * x[1] / x[2] + HEIGHT / 2.0])
            if not (0 < pixel[0] < WIDTH and 0 < pixel[1] < HEIGHT):
                continue
            if rng.uniform() > KEEP_PROB:
                continue
            if rng.uniform() < outlier_ratio:
                pixel = rng.uniform([0.0, 0.0], [WIDTH, HEIGHT])
            else:
                pixel = pixel + rng.normal(scale=noise_px, size=2)
            obs.append((vid, pixel))
        if len(obs) < 3:
            continue
        tid = recon.AddTrack()
        for vid, pixel in obs:
            recon.AddObservation(vid, tid, pt.sfm.Feature(pixel))

    view_graph = pt.sfm.ViewGraph()
    tracks = [set(recon.View(vid).TrackIds()) for vid in view_ids]
    for a in range(NUM_VIEWS):
        for b in range(a + 1, NUM_VIEWS):
            common = len(tracks[a] & tracks[b])
            if common < MIN_COMMON_TRACKS:
                continue
            # Convention of docs/content/view_graph.md:
            # rotation_2 = R_b R_a^T, position_2 = normalize(R_a (c_b - c_a)).
            info = pt.sfm.TwoViewInfo()
            info.focal_length_1 = focals[a]
            info.focal_length_2 = focals[b]
            info.rotation_2 = rotmat_to_aa(rotations[b] @ rotations[a].T)
            direction = rotations[a] @ (centers[b] - centers[a])
            info.position_2 = direction / np.linalg.norm(direction)
            info.num_verified_matches = common
            view_graph.AddEdge(view_ids[a], view_ids[b], info)

    return Scene(recon, view_graph, rotations, centers, np.asarray(focals, float))


def umeyama(src, dst):
    """Similarity (s, R, t) with dst ~ s R src + t, for (N, 3) arrays."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    U, sig, Vt = np.linalg.svd(D.T @ S / len(src))
    W = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        W[2, 2] = -1
    R = U @ W @ Vt
    s = np.trace(np.diag(sig) @ W) / S.var(0).sum()
    return s, R, mu_d - s * R @ mu_s


def evaluate(rot_est, centers_est, rot_gt, centers_gt):
    """Rotation and centre errors after a Sim(3) alignment of the centres."""
    s, R_al, t_al = umeyama(centers_est, centers_gt)
    c_err = np.linalg.norm(s * centers_est @ R_al.T + t_al - centers_gt, axis=1)
    r_err = [rotation_angle_deg(R @ R_al.T @ G.T) for R, G in zip(rot_est, rot_gt)]
    return {"rot_med": np.median(r_err), "rot_max": np.max(r_err),
            "centre_med": np.median(c_err)}


def print_errors(name, err):
    print(f"  {name}: rotation error median {err['rot_med']:.4f} deg "
          f"(max {err['rot_max']:.4f}), centre error median "
          f"{err['centre_med']:.5f} (scene units)")


def recon_poses(recon):
    """Estimated views (ids), world-to-camera rotations and centres."""
    est = [v for v in sorted(recon.ViewIds()) if recon.View(v).IsEstimated()]
    rot = [np.asarray(recon.View(v).Camera().GetOrientationAsRotationMatrix())
           for v in est]
    cen = np.array([recon.View(v).Camera().GetPosition() for v in est])
    return est, rot, cen


def metric_poses(result):
    """Rotations and centres (c = -R^T t) of a MetricUpgradeResult."""
    rot = [np.asarray(R) for R in result.rotations]
    cen = np.array([-R.T @ np.asarray(t) for R, t in zip(rot, result.translations)])
    return rot, cen


def part_a_estimator(args):
    print("Part A: estimator (INIT_FREE vs global SfM)")
    scene = make_scene(args.seed, args.noise_px, args.outlier_ratio)
    ro = pt.sfm.ReconstructionEstimatorOptions()
    ro.reconstruction_estimator_type = pt.sfm.ReconstructionEstimatorType.INIT_FREE
    ro.num_threads = 1
    ro.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.NONE
    ifo = ro.init_free_options
    ifo.formulation = pt.sfm.OseFormulationType.POSE_ROT
    ifo.observation_normalization = pt.sfm.OseObservationNormalization.CALIBRATED_RAYS
    ifo.camera_init = pt.sfm.InitFreeCameraInitType.UNIT_CIRCLE
    ifo.num_random_restarts = args.restarts
    ifo.seed = args.seed
    ifo.varpro.robust_loss_type = pt.sfm.VarProRobustLossType.CAUCHY
    ifo.varpro.robust_loss_scale = ROBUST_LOSS_PX / FOCAL
    ifo.projective_refinement.max_num_iterations = 50
    ifo.projective_refinement.robust_loss_width = ROBUST_LOSS_PX / FOCAL
    ifo.metric_upgrade = pt.sfm.MetricUpgradeType.LINEAR_AND_NONLINEAR

    estimator = pt.sfm.InitFreeReconstructionEstimator(ro)
    t0 = time.perf_counter()
    summary = estimator.Estimate(scene.view_graph, scene.recon)
    dt = time.perf_counter() - t0
    est, rot, cen = recon_poses(scene.recon)
    print(f"[init_free] success={summary.success} time {dt:.2f} s "
          f"views {len(est)}/{NUM_VIEWS}")
    if len(est) >= 3:
        print_errors("init_free", evaluate(rot, cen, [scene.rotations[v] for v in est],
                                           scene.centers[est]))
    diag = estimator.Diagnostics()
    print(f"    best restart {diag.best_restart}, fraction_in_front "
          f"{diag.fraction_in_front:.3f}, mean orthogonality error "
          f"{diag.mean_orthogonality_error:.4g}")
    for k, r in enumerate(diag.restarts):
        print(f"    restart {k} (seed {r.seed}): OSE cost {r.final_cost:.4g}, refined RMS "
              f"{r.refined_rms_error * FOCAL:.3f} px")
        if not r.upgrade_attempted:
            print("      upgrade not attempted (a higher-ranked restart passed the checks)")
            continue
        print(f"      upgrade: {r.upgrade_message!r}, orthogonality error "
              f"{r.upgrade_mean_orthogonality_error:.4g}, fraction in front "
              f"{r.upgrade_fraction_in_front:.3f}, conditioning {r.upgrade_conditioning:.4g}")

    # Global SfM on a second, identical build of the scene.
    scene = make_scene(args.seed, args.noise_px, args.outlier_ratio)
    ro = pt.sfm.ReconstructionEstimatorOptions()
    ro.num_threads = 1
    ro.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.LIGT
    ro.global_rotation_estimator_type = pt.sfm.GlobalRotationEstimatorType.HYBRID
    t0 = time.perf_counter()
    summary = pt.sfm.GlobalReconstructionEstimator(ro).Estimate(
        scene.view_graph, scene.recon)
    dt = time.perf_counter() - t0
    est, rot, cen = recon_poses(scene.recon)
    print(f"[global] success={summary.success} time {dt:.2f} s "
          f"views {len(est)}/{NUM_VIEWS}")
    if len(est) >= 3:
        print_errors("global", evaluate(rot, cen, [scene.rotations[v] for v in est],
                                        scene.centers[est]))


def part_b_stages(args):
    print("Part B: stages one by one")
    scene = make_scene(args.seed, args.noise_px, args.outlier_ratio)
    ifo = pt.sfm.InitFreeOptions()
    ifo.formulation = pt.sfm.OseFormulationType.POSE_ROT
    ifo.observation_normalization = pt.sfm.OseObservationNormalization.CALIBRATED_RAYS
    ifo.camera_init = pt.sfm.InitFreeCameraInitType.UNIT_CIRCLE
    ifo.alpha = 0.05
    ifo.varpro.robust_loss_type = pt.sfm.VarProRobustLossType.CAUCHY
    ifo.varpro.robust_loss_scale = ROBUST_LOSS_PX / FOCAL
    ifo.projective_refinement.max_num_iterations = 50
    ifo.projective_refinement.robust_loss_width = ROBUST_LOSS_PX / FOCAL

    t0 = time.perf_counter()
    ok, problem, err = pt.sfm.BuildOseProblem(scene.recon, scene.view_graph, ifo)
    assert ok, err
    cams0 = pt.sfm.SampleInitialCameras(problem.NumCameras(), ifo.camera_init, args.seed)
    ok, sol = pt.sfm.SolveOse(problem, ifo, cams0)
    assert ok, sol.message
    ok, ref = pt.sfm.RefineProjectiveReconstruction(
        problem, sol.cameras, sol.points, ifo.projective_refinement)
    assert ok
    uo = pt.sfm.MetricUpgradeOptions()
    uo.nonlinear_refinement = True
    ok, metric, err = pt.sfm.UpgradeToMetric(problem, ref.cameras, ref.points, uo)
    assert ok, err
    dt = time.perf_counter() - t0

    gt = list(problem.view_ids)  # view ids are the ground-truth camera indices
    rot, cen = metric_poses(metric)
    print(f"[calibrated] {problem.NumCameras()} cameras, {problem.NumPoints()} points, "
          f"{problem.NumObservations()} observations, time {dt:.2f} s")
    print(f"    projective RMS {ref.final_rms_error * FOCAL:.3f} px after refinement, "
          f"fraction_in_front {metric.fraction_in_front:.3f}, conditioning "
          f"{metric.conditioning:.4g}")
    print_errors("metric", evaluate(rot, cen, [scene.rotations[i] for i in gt],
                                    scene.centers[gt]))

    # Self-calibration: PIXEL_ISOTROPIC observations, one focal length per camera.
    focals = np.random.default_rng(args.seed).uniform(650.0, 1000.0, NUM_VIEWS)
    scene = make_scene(args.seed, args.noise_px, args.outlier_ratio, focals)
    ifo = pt.sfm.InitFreeOptions()
    ifo.formulation = pt.sfm.OseFormulationType.POSE
    ifo.observation_normalization = pt.sfm.OseObservationNormalization.PIXEL_ISOTROPIC
    ifo.camera_init = pt.sfm.InitFreeCameraInitType.UNIT_CIRCLE
    ifo.alpha = 0.05
    ifo.projective_refinement.max_num_iterations = 50
    ok, problem, err = pt.sfm.BuildOseProblem(scene.recon, scene.view_graph, ifo)
    assert ok, err
    # Observations are scaled by the normalization, so the robust scales are
    # converted from pixels with the mean scale.
    scale = np.array([T[0, 0] for T in problem.normalizations])
    ifo.varpro.robust_loss_type = pt.sfm.VarProRobustLossType.CAUCHY
    ifo.varpro.robust_loss_scale = ROBUST_LOSS_PX * scale.mean()
    ifo.projective_refinement.robust_loss_width = ROBUST_LOSS_PX * scale.mean()
    cams0 = pt.sfm.SampleInitialCameras(problem.NumCameras(), ifo.camera_init, args.seed)
    ok, sol = pt.sfm.SolveOse(problem, ifo, cams0)
    assert ok, sol.message
    ok, ref = pt.sfm.RefineProjectiveReconstruction(
        problem, sol.cameras, sol.points, ifo.projective_refinement)
    assert ok

    gt = list(problem.view_ids)
    true_f = scene.focals[gt]
    print("[self-calibration] focal lengths in pixels (true / estimated)")
    # The linear upgrade is biased by its priors; the nonlinear refinement removes the bias.
    # A conditioning near zero (below about 3e-3) means a critical motion: the focal
    # lengths are then not determined (InitFreeOptions.min_upgrade_conditioning).
    for nonlinear in [False, True]:
        uo = pt.sfm.MetricUpgradeOptions()
        uo.nonlinear_refinement = nonlinear
        ok, result, err = pt.sfm.UpgradeToMetric(problem, ref.cameras, ref.points, uo)
        assert ok, err
        est_f = np.array(result.focal_lengths) / scale  # observation units -> pixels
        name = "nonlinear" if nonlinear else "linear"
        print(f"    {name:9s}: max relative focal error "
              f"{100 * np.max(np.abs(est_f / true_f - 1.0)):.3f} %, conditioning "
              f"{result.conditioning:.3g}")
        if nonlinear:
            for k in range(0, len(gt), 3):  # three views per line
                print("     " + "".join(
                    f"  view {gt[j]:2d}: {true_f[j]:6.1f} / {est_f[j]:6.1f} px"
                    for j in range(k, min(k + 3, len(gt)))))
            rot, cen = metric_poses(result)
            print_errors("metric", evaluate(rot, cen, [scene.rotations[i] for i in gt],
                                            scene.centers[gt]))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--noise_px", type=float, default=0.5,
                    help="pixel noise standard deviation")
    ap.add_argument("--outlier_ratio", type=float, default=0.0,
                    help="fraction of observations replaced by uniform random pixels")
    ap.add_argument("--restarts", type=int, default=3,
                    help="random restarts of the OSE stage (Part A)")
    ap.add_argument("--seed", type=int, default=0,
                    help="scene seed and first restart seed")
    args = ap.parse_args()

    pt.solvers.SetDefaultRandomSeed(0)
    part_a_estimator(args)
    print()
    part_b_stages(args)


if __name__ == "__main__":
    main()
