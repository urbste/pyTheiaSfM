"""Loaders and geometry helpers for ETH3D COLMAP-text ground truth.

Layout expected under ``ETH3D_ROOT`` (default below, override with the env var)::

    <root>/<scene>/dslr_calibration_undistorted/{cameras,images,points3D}.txt
    <root>/<scene>/images/dslr_images_undistorted/*.JPG

Conventions (COLMAP): images.txt stores world-to-camera poses
(qvec = w,x,y,z and tvec) so that X_cam = R X_world + t. Cameras are PINHOLE
and already undistorted. Images are never decoded here; all examples work on
ground-truth 2D observations and sparse points.
"""

import os
from dataclasses import dataclass, field

import numpy as np
import pytheia as pt
from scipy.spatial.transform import Rotation as Rot

ETH3D_ROOT = os.environ.get(
    "ETH3D_ROOT", "/media/steffen/Data2/sfm_bench/data/eth3d/extracted")
SCENES = ["courtyard", "delivery_area", "electro", "facade", "kicker",
          "meadow", "office", "pipes", "playground", "relief", "relief_2",
          "terrace", "terrains"]


@dataclass
class Camera:
    id: int
    model: str
    width: int
    height: int
    params: np.ndarray

    @property
    def K(self):
        """3x3 intrinsics. Only PINHOLE (fx, fy, cx, cy) is used by ETH3D."""
        fx, fy, cx, cy = self.params[:4]
        return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])


@dataclass
class Image:
    id: int
    name: str
    qvec: np.ndarray           # w, x, y, z (world -> camera)
    tvec: np.ndarray           # world -> camera translation
    camera_id: int
    xys: np.ndarray            # (N, 2) observed pixels
    point3d_ids: np.ndarray    # (N,) -1 if the observation has no 3D point

    @property
    def R(self):
        """World-to-camera rotation matrix from the quaternion."""
        w, x, y, z = self.qvec / np.linalg.norm(self.qvec)
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

    @property
    def t(self):
        return np.asarray(self.tvec, dtype=float)

    @property
    def center(self):
        """Camera center in world coordinates, C = -R^T t."""
        return -self.R.T @ self.t


@dataclass
class Model:
    scene: str
    cameras: dict
    images: dict                       # name -> Image
    points: dict = field(default_factory=dict)   # point3d_id -> xyz (3,)

    def image_list(self):
        return [self.images[k] for k in sorted(self.images)]

    def image_path(self, image):
        return os.path.join(ETH3D_ROOT, self.scene, "images", image.name)

    def camera(self, image):
        return self.cameras[image.camera_id]

    def K(self, image):
        return self.camera(image).K


def _model_dir(scene):
    path = scene if os.path.isabs(scene) else os.path.join(ETH3D_ROOT, scene)
    if os.path.isdir(os.path.join(path, "dslr_calibration_undistorted")):
        path = os.path.join(path, "dslr_calibration_undistorted")
    return path


def _data_lines(path):
    with open(path, "r") as f:
        for line in f:
            if line.strip() and not line.startswith("#"):
                yield line.split()


def load_model(scene):
    """Load cameras, GT poses + 2D observations and GT sparse points."""
    scene_name = os.path.basename(os.path.normpath(scene))
    mdir = _model_dir(scene)
    cameras = {}
    for tok in _data_lines(os.path.join(mdir, "cameras.txt")):
        cid = int(tok[0])
        cameras[cid] = Camera(cid, tok[1], int(tok[2]), int(tok[3]),
                              np.array([float(v) for v in tok[4:]]))

    images = {}
    img_lines = list(_data_lines(os.path.join(mdir, "images.txt")))
    for head, obs in zip(img_lines[0::2], img_lines[1::2]):
        arr = np.array(obs, dtype=float).reshape(-1, 3)
        img = Image(
            id=int(head[0]),
            name=head[9],
            qvec=np.array([float(v) for v in head[1:5]]),
            tvec=np.array([float(v) for v in head[5:8]]),
            camera_id=int(head[8]),
            xys=arr[:, :2],
            point3d_ids=arr[:, 2].astype(np.int64))
        images[img.name] = img

    points = {}
    for tok in _data_lines(os.path.join(mdir, "points3D.txt")):
        points[int(tok[0])] = np.array([float(v) for v in tok[1:4]])
    return Model(scene_name, cameras, images, points)


def project(K, R, t, X):
    """Pinhole projection of world points X (N,3) with world-to-camera R, t."""
    Xc = X @ R.T + t
    uv = Xc[:, :2] / Xc[:, 2:3]
    return uv @ K[:2, :2].T + K[:2, 2]


def camera_points(R, t, X):
    """World points (N,3) expressed in the camera frame."""
    return X @ R.T + t


def normalize(K, pix):
    """Pixels (N,2) -> normalized image coordinates (N,2)."""
    uv1 = np.hstack([pix, np.ones((len(pix), 1))])
    return (np.linalg.inv(K) @ uv1.T).T[:, :2]


def shared_observations(model, img_a, img_b):
    """GT correspondences of two images: rows (point3d_id, pixel_a, pixel_b)."""
    ids_a = {pid: k for k, pid in enumerate(img_a.point3d_ids) if pid >= 0}
    rows = []
    for k_b, pid in enumerate(img_b.point3d_ids):
        if pid >= 0 and pid in ids_a:
            rows.append((pid, img_a.xys[ids_a[pid]], img_b.xys[k_b]))
    pids = np.array([r[0] for r in rows], dtype=np.int64)
    pa = np.array([r[1] for r in rows]).reshape(-1, 2)
    pb = np.array([r[2] for r in rows]).reshape(-1, 2)
    return pids, pa, pb


def gt_points_in_image(model, image, min_depth=0.0):
    """GT sparse points observed in an image: (point3d_ids, X_world, pixels)."""
    mask = image.point3d_ids >= 0
    pids = image.point3d_ids[mask]
    pixels = image.xys[mask]
    X = np.array([model.points[p] for p in pids]).reshape(-1, 3)
    depth = camera_points(image.R, image.t, X)[:, 2]
    keep = depth > min_depth
    return pids[keep], X[keep], pixels[keep]


def relative_gt(img_a, img_b):
    """Ground-truth (R, t) with X_b = R X_a + t for points in camera a frame."""
    R = img_b.R @ img_a.R.T
    t = img_b.t - R @ img_a.t
    return R, t


def rotation_error_deg(R_est, R_gt):
    c = (np.trace(R_est.T @ R_gt) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def direction_error_deg(t_est, t_gt):
    a = t_est / np.linalg.norm(t_est)
    b = t_gt / np.linalg.norm(t_gt)
    return float(np.degrees(np.arccos(np.clip(a @ b, -1.0, 1.0))))


def add_pixel_noise(pixels, sigma_px, outlier_ratio=0.0, image_size=None,
                    rng=None):
    """Gaussian pixel noise plus a fraction of uniformly random outliers.

    Returns (noisy_pixels, outlier_mask). image_size = (width, height) is used
    to draw outlier locations uniformly over the image.
    """
    rng = np.random.default_rng() if rng is None else rng
    out = pixels + rng.normal(scale=sigma_px, size=pixels.shape)
    mask = np.zeros(len(pixels), dtype=bool)
    if outlier_ratio > 0:
        n = int(round(outlier_ratio * len(pixels)))
        idx = rng.choice(len(pixels), size=n, replace=False)
        w, h = image_size if image_size else (np.max(pixels[:, 0]),
                                             np.max(pixels[:, 1]))
        out[idx, 0] = rng.uniform(0, w, size=n)
        out[idx, 1] = rng.uniform(0, h, size=n)
        mask[idx] = True
    return out, mask


def rotvec_from_matrix(R):
    """Rotation matrix -> angle-axis vector (Rodrigues inverse)."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(c)
    if angle < 1e-12:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return axis / (2.0 * np.sin(angle)) * angle


def umeyama(src, dst):
    """Similarity (s, R, t) with dst ~ s R src + t, src/dst (N,3). NumPy only."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    U, sig, Vt = np.linalg.svd(D.T @ S / len(src))
    W = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        W[2, 2] = -1
    R = U @ W @ Vt
    s = np.trace(np.diag(sig) @ W) / (S.var(0).sum())
    return s, R, mu_d - s * R @ mu_s


def division_distort(u, k):
    """Division-model distortion used by pyTheia's radial-distortion PnP.

    u: centred pixel coordinates (2,) relative to the principal point.
    Mirrors DistortPoint() in theia/sfm/estimators/estimate_radial_dist_*.cc.
    """
    r2 = u @ u
    denom, inner = 2.0 * k * r2, 1.0 - 4.0 * k * r2
    if abs(denom) < 1e-15 or inner < 0:
        return u.copy()
    return u * ((1.0 - np.sqrt(inner)) / denom)


def pt_camera(model, image):
    """pytheia.sfm.Camera with ETH3D intrinsics and GT world-to-camera pose.

    Note: PINHOLE fx/fy differ slightly in ETH3D, so the aspect ratio is set.
    """
    cm = model.camera(image)
    K = model.K(image)
    cam = pt.sfm.Camera()
    cam.SetCameraIntrinsicsModelType(pt.sfm.CameraIntrinsicsModelType.PINHOLE)
    cam.SetImageSize(cm.width, cm.height)
    intr = cam.CameraIntrinsics()
    intr.SetFocalLength(K[0, 0])
    intr.SetAspectRatio(K[1, 1] / K[0, 0])
    intr.SetPrincipalPoint(K[0, 2], K[1, 2])
    intr.SetSkew(0.0)
    cam.SetOrientationFromRotationMatrix(image.R)
    cam.SetPosition(image.center)
    return cam


def gt_tracks(model, images):
    """GT tracks restricted to ``images``: {point3d_id: [(image, pixel), ...]}.

    Keeps one observation per (image, point) so each view contributes once.
    """
    tracks = {}
    for im in images:
        seen = set()
        for x, pid in zip(im.xys, im.point3d_ids):
            if pid >= 0 and pid not in seen:
                seen.add(pid)
                tracks.setdefault(int(pid), []).append((im, x))
    return {k: v for k, v in tracks.items() if len(v) >= 2}


def quat_wxyz_to_matrix(q):
    """(w, x, y, z) unit quaternion -> 3x3 rotation matrix."""
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose_errors(R_est, C_est, image):
    """Rotation error (deg) and camera-centre error (scene units) vs GT."""
    return (rotation_error_deg(R_est, image.R),
            float(np.linalg.norm(C_est - image.center)))


def intrinsics_prior(model, image):
    """pytheia.sfm.CameraIntrinsicsPrior with the ETH3D PINHOLE intrinsics."""
    K, cm = model.K(image), model.camera(image)
    pr = pt.sfm.CameraIntrinsicsPrior()
    pr.focal_length.value, pr.aspect_ratio.value = [K[0, 0]], [K[1, 1] / K[0, 0]]
    pr.principal_point.value, pr.skew.value = [K[0, 2], K[1, 2]], [0.0]
    pr.image_width, pr.image_height = cm.width, cm.height
    pr.camera_intrinsics_model_type = "PINHOLE"
    return pr


def partial_rotation_inliers(R, t, ra, rb, thr):
    """Inlier mask: epipolar residual small and both depths positive.

    Depths solve la * R ra - lb * rb = -t per match (2x2 normal equations).
    """
    t = t / np.linalg.norm(t)
    n = np.cross(t, ra @ R.T)                       # epipolar plane normals
    resid = np.abs(np.sum(rb * n, 1)) / np.linalg.norm(n, axis=1)
    u, v = ra @ R.T, rb
    a, c, b = np.sum(u * u, 1), np.sum(v * v, 1), np.sum(u * v, 1)
    p, q = -(u @ t), v @ t
    det = a * c - b * b
    la = (c * p + b * q) / det
    lb = (b * p + a * q) / det
    return (resid < thr) & (la > 0) & (lb > 0)


# ---- bundle adjustment test problem (used by bundle_adjustment_priors.py) ----

def build_ba_problem(m, ims, flags, seed):
    """Reconstruction for one condition. Noise streams are drawn identically
    regardless of ``flags`` so every condition starts from the same state."""
    r_pose, r_prior, r_obs = (np.random.default_rng(seed + i) for i in range(3))
    recon = pt.sfm.Reconstruction()
    vids = []
    for k, im in enumerate(ims):
        v = recon.AddView(im.name, 0, k)
        vids.append(v)
        cam = recon.MutableView(v).MutableCamera()
        cam.DeepCopy(pt_camera(m, im))
        dR = Rot.from_rotvec(r_pose.normal(size=3) * np.radians(0.5) / np.sqrt(3))
        cam.SetOrientationFromRotationMatrix(dR.as_matrix() @ im.R)
        cam.SetPosition(im.center + r_pose.normal(size=3) * 0.10)
        recon.MutableView(v).SetIsEstimated(True)
        pos_p = im.center + r_prior.normal(size=3) * 0.02
        ori_p = rotvec_from_matrix(im.R) + r_prior.normal(size=3) * np.radians(0.2)
        g = im.R @ np.array([0.0, 0.0, -1.0]) + r_prior.normal(size=3) * 0.01
        if "pos" in flags:
            recon.MutableView(v).SetPositionPrior(pos_p, np.eye(3) / 0.02)
        if "ori" in flags:
            recon.MutableView(v).SetOrientationPrior(ori_p, np.eye(3) / np.radians(0.2))
        if "grav" in flags:
            recon.MutableView(v).SetGravityPrior(g / np.linalg.norm(g), np.eye(3) / 0.01)
    pairs = []
    for pid, obs in gt_tracks(m, ims).items():
        tid = recon.AddTrack()
        pairs.append((tid, pid))
        for im, x in obs:
            z = camera_points(im.R, im.t, m.points[pid][None])[0, 2]
            z_noisy = z * (1 + r_obs.normal() * 0.02)
            f = pt.sfm.Feature(x)
            if "depth" in flags:
                f = pt.sfm.Feature(x, z_noisy)
                f.depth_prior_variance = (0.02 * z) ** 2
            recon.AddObservation(vids[ims.index(im)], tid, f)
        tr = recon.MutableTrack(tid)
        tr.SetPoint(np.append(m.points[pid] + r_obs.normal(size=3) * 0.05, 1.0))
        tr.SetIsEstimated(True)
        pt_prior = m.points[pid] + r_obs.normal(size=3) * 0.02
        if "point" in flags:
            tr.SetPositionPrior(pt_prior, np.eye(3) / 0.02)
    return recon, vids, pairs


def ba_alignment_errors(m, ims, recon, vids, pairs):
    C_gt = np.array([im.center for im in ims])
    C = np.array([recon.View(v).Camera().GetPosition() for v in vids])
    s, R_al, t_al = umeyama(C, C_gt)
    c_err = np.linalg.norm(s * C @ R_al.T + t_al - C_gt, axis=1)
    r_err = [rotation_error_deg(recon.View(v).Camera().GetOrientationAsRotationMatrix()
                                  @ R_al.T, im.R) for v, im in zip(vids, ims)]
    X = np.array([recon.Track(t).Point()[:3] / recon.Track(t).Point()[3] for t, _ in pairs])
    X_gt = np.array([m.points[p] for _, p in pairs])
    p_err = np.linalg.norm(s * X @ R_al.T + t_al - X_gt, axis=1)
    return np.median(c_err), np.median(r_err), np.median(p_err)
