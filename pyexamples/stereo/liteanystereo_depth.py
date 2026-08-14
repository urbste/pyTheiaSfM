#!/usr/bin/env python3
# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""Precompute LiteAnyStereo disparity/depth, then fuse keyframe clouds.

Precompute (disparity `.npy` + optional camera-frame PLYs)::

  python pyexamples/stereo/liteanystereo_depth.py precompute \\
    --frames_dir /home/steffen/Dokumente/ZED/test_frames \\
    --las_root /home/steffen/external_projects/LiteAnyStereo \\
    --model_size h --write_clouds --z_far 1.0

Fuse only SfM keyframes (same stride as reconstruction) into a colored PLY::

  python pyexamples/stereo/liteanystereo_depth.py fuse \\
    --frames_dir /home/steffen/Dokumente/ZED/test_frames \\
    --recon /home/steffen/Dokumente/ZED/test_frames/stereo_rig.recon \\
    --recon_stride 5
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np

_THIS = os.path.dirname(os.path.abspath(__file__))
_EXAMPLES = os.path.dirname(_THIS)
if _EXAMPLES not in sys.path:
    sys.path.insert(0, _EXAMPLES)
if _THIS not in sys.path:
    sys.path.insert(0, _THIS)

from calibrated_stereo_rig import reconstruction_keyframe_indices  # noqa: E402

DEFAULT_LAS_ROOT = "/home/steffen/external_projects/LiteAnyStereo"
DEPTH_DIRNAME = "depth_las2_m"
DISP_DIRNAME = "disparity_las2_m"
META_NAME = "liteanystereo.json"
OUT_SUBDIR = "liteanystereo"


def _sorted_images(folder: str, ext: str) -> list[str]:
    paths = sorted(glob.glob(os.path.join(folder, f"*.{ext}")))
    if not paths:
        paths = sorted(glob.glob(os.path.join(folder, f"*.{ext.upper()}")))
    return paths


def _guess_img_ext(left_dir: str) -> str:
    for ext in ("png", "jpg", "jpeg", "PNG", "JPG", "JPEG"):
        if glob.glob(os.path.join(left_dir, f"*.{ext}")):
            return ext.lower()
    return "png"


def _load_calib(frames_dir: str) -> dict:
    path = os.path.join(frames_dir, "rig_calibration.json")
    if not os.path.isfile(path):
        raise FileNotFoundError(f"Missing {path}")
    with open(path, encoding="utf-8") as f:
        calib = json.load(f)
    py = calib.get("pytheia", calib)
    fx = float(py.get("focal", calib.get("left", {}).get("fx")))
    fy = float(calib.get("left", {}).get("fy", fx))
    cx = float(py.get("cx", calib.get("left", {}).get("cx")))
    cy = float(py.get("cy", calib.get("left", {}).get("cy")))
    baseline = float(py.get("baseline", calib.get("baseline_meters")))
    return {
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "baseline": baseline,
        "width": int(py.get("width", calib.get("image_width", 0))),
        "height": int(py.get("height", calib.get("image_height", 0))),
        "K": np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64),
    }


def _stem(path: str) -> str:
    return os.path.splitext(os.path.basename(path))[0]


def _depth_dirs(frames_dir: str) -> tuple[str, str]:
    return (
        os.path.join(frames_dir, DISP_DIRNAME),
        os.path.join(frames_dir, DEPTH_DIRNAME),
    )


def _las_out_dirs(frames_dir: str, out_subdir: str) -> tuple[str, str]:
    root = os.path.join(frames_dir, out_subdir)
    return os.path.join(root, "disparity"), os.path.join(root, "pointclouds")


def _write_ply_rgb(path: str, xyz: np.ndarray, rgb: np.ndarray) -> None:
    n = int(xyz.shape[0])
    verts = np.empty(
        n,
        dtype=[
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("red", "u1"),
            ("green", "u1"),
            ("blue", "u1"),
        ],
    )
    verts["x"] = xyz[:, 0]
    verts["y"] = xyz[:, 1]
    verts["z"] = xyz[:, 2]
    verts["red"] = rgb[:, 0]
    verts["green"] = rgb[:, 1]
    verts["blue"] = rgb[:, 2]
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        verts.tofile(f)


def _default_ckpt(las_root: str, model_size: str) -> str:
    return os.path.join(las_root, "checkpoints", f"LAS2_{str(model_size).upper()}.pth")


def _vis_disparity(disp: np.ndarray) -> np.ndarray:
    valid = np.isfinite(disp) & (disp > 0)
    vis = np.zeros((*disp.shape, 3), dtype=np.uint8)
    if not np.any(valid):
        return vis
    lo, hi = np.percentile(disp[valid], [1.0, 99.0])
    hi = max(float(hi), float(lo) + 1e-3)
    norm = np.clip((disp - lo) / (hi - lo), 0.0, 1.0)
    norm[~valid] = 0.0
    bgr = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    bgr[~valid] = 0
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def _write_disp_vis(path: str, disp: np.ndarray) -> None:
    cv2.imwrite(path, cv2.cvtColor(_vis_disparity(disp), cv2.COLOR_RGB2BGR))


def _scale_calib(calib: dict, scale: float) -> dict:
    if abs(scale - 1.0) < 1e-9:
        return calib
    out = dict(calib)
    out["fx"] = float(calib["fx"]) * scale
    out["fy"] = float(calib["fy"]) * scale
    out["cx"] = float(calib["cx"]) * scale
    out["cy"] = float(calib["cy"]) * scale
    out["width"] = max(1, int(round(int(calib["width"]) * scale)))
    out["height"] = max(1, int(round(int(calib["height"]) * scale)))
    out["K"] = np.array(
        [[out["fx"], 0.0, out["cx"]], [0.0, out["fy"], out["cy"]], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return out


def _resize_pair(left_rgb: np.ndarray, right_rgb: np.ndarray, scale: float):
    if abs(scale - 1.0) < 1e-9:
        return left_rgb, right_rgb
    left = cv2.resize(left_rgb, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    right = cv2.resize(right_rgb, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return left, right


def _disparity_to_native(
    disp: np.ndarray, scale: float, out_hw: tuple[int, int]
) -> np.ndarray:
    """Resize disparity to the original image and convert to native pixel units.

    Stereo disparity (pixels) scales with image width: ``d_native = d_scaled / scale``.
    Depth with the original intrinsics is then ``Z = fx_full * B / d_native``.
    """
    h0, w0 = out_hw
    if abs(scale - 1.0) < 1e-9 and disp.shape == (h0, w0):
        return disp
    valid = np.isfinite(disp) & (disp > 0)
    work = np.nan_to_num(disp, nan=0.0).astype(np.float32)
    up = cv2.resize(work, (w0, h0), interpolation=cv2.INTER_LINEAR)
    wts = cv2.resize(valid.astype(np.float32), (w0, h0), interpolation=cv2.INTER_LINEAR)
    up = up / max(float(scale), 1e-6)
    up[wts < 0.5] = np.nan
    return up.astype(np.float32)


def _disp_to_depth(disp: np.ndarray, fx: float, baseline: float) -> np.ndarray:
    depth = np.full(disp.shape, np.nan, dtype=np.float32)
    valid = np.isfinite(disp) & (disp > 0.5)
    depth[valid] = (fx * baseline) / disp[valid]
    return depth


def cmd_precompute(args: argparse.Namespace) -> int:
    import torch

    las_root = os.path.abspath(args.las_root)
    if las_root not in sys.path:
        sys.path.insert(0, las_root)

    from core.models import (  # noqa: E402
        build_model,
        load_model_weights,
        model_label,
        normalize_model_size,
        normalize_version,
        require_checkpoint,
    )
    from core.utils.utils import InputPadder  # noqa: E402

    frames_dirs = [os.path.abspath(p) for p in args.frames_dir]
    version = normalize_version(args.version)
    model_size = normalize_model_size(version, args.model_size)
    label = model_label(version, model_size)
    ckpt = os.path.abspath(args.restore_ckpt or _default_ckpt(las_root, model_size))
    require_checkpoint(ckpt)

    device = torch.device(
        "cuda" if args.device == "cuda" and torch.cuda.is_available() else "cpu"
    )
    print(f"LiteAnyStereo {label}  device={device}  ckpt={ckpt}")
    if args.scale <= 0 or args.scale > 1:
        print("--scale must be in (0, 1]", file=sys.stderr)
        return 1

    model = build_model(
        version, fnet_pretrained=False, model_size=model_size, max_disp=args.max_disp
    )
    try:
        checkpoint = torch.load(ckpt, map_location=device, weights_only=False)
    except TypeError:
        checkpoint = torch.load(ckpt, map_location=device)
    load_model_weights(model, checkpoint, strict=True)
    model = model.to(device).eval()

    eye = np.eye(3, dtype=np.float64)
    origin = np.zeros(3, dtype=np.float64)
    torch.autograd.set_grad_enabled(False)

    for frames_dir in frames_dirs:
        left_dir = os.path.join(frames_dir, "left")
        right_dir = os.path.join(frames_dir, "right")
        ext = args.img_ext or _guess_img_ext(left_dir)
        left_paths = _sorted_images(left_dir, ext)
        right_paths = _sorted_images(right_dir, ext)
        n = min(len(left_paths), len(right_paths))
        if n < 1:
            print(f"No stereo pairs under {frames_dir}", file=sys.stderr)
            continue

        calib_full = _load_calib(frames_dir)
        calib = _scale_calib(calib_full, args.scale)
        if args.out_subdir:
            disp_dir, cloud_dir = _las_out_dirs(frames_dir, args.out_subdir)
            depth_dir = os.path.join(frames_dir, args.out_subdir, "depth")
            vis_dir = os.path.join(disp_dir, "vis")
            meta_path = os.path.join(frames_dir, args.out_subdir, "meta.json")
        else:
            disp_dir, depth_dir = _depth_dirs(frames_dir)
            cloud_dir = os.path.join(frames_dir, "pointclouds_las2")
            vis_dir = os.path.join(disp_dir, "vis")
            meta_path = os.path.join(frames_dir, META_NAME)
        os.makedirs(disp_dir, exist_ok=True)
        if args.write_vis:
            os.makedirs(vis_dir, exist_ok=True)
        if args.write_depth:
            os.makedirs(depth_dir, exist_ok=True)
        if args.write_clouds:
            os.makedirs(cloud_dir, exist_ok=True)

        print(
            f"{os.path.basename(frames_dir)}  {n} pairs  "
            f"native={calib_full['width']}x{calib_full['height']}  "
            f"infer={calib['width']}x{calib['height']}  scale={args.scale:g}  "
            f"fx_full={calib_full['fx']:.3f}  fx_infer={calib['fx']:.3f}  "
            f"B={calib['baseline']:.6f} m"
        )

        wrote = 0
        skipped = 0
        for i, (lp, rp) in enumerate(zip(left_paths[:n], right_paths[:n])):
            stem = _stem(lp)
            disp_path = os.path.join(disp_dir, f"{stem}.npy")
            ply_path = os.path.join(cloud_dir, f"{stem}.ply")
            vis_path = os.path.join(vis_dir, f"{stem}.png")
            need_disp = args.overwrite or not os.path.isfile(disp_path)
            need_cloud = args.write_clouds and (
                args.overwrite or not os.path.isfile(ply_path)
            )
            need_vis = args.write_vis and (
                args.overwrite or not os.path.isfile(vis_path)
            )
            if not need_disp and not need_cloud and not need_vis:
                skipped += 1
                continue
            if not need_disp and os.path.isfile(disp_path) and (need_vis or need_cloud):
                disp = np.load(disp_path)
                left = cv2.imread(lp, cv2.IMREAD_COLOR)
                if left is None:
                    print(f"  skip unreadable left {stem}", file=sys.stderr)
                    continue
                left_full = cv2.cvtColor(left, cv2.COLOR_BGR2RGB)
                h0, w0 = left_full.shape[:2]
                if disp.shape != (h0, w0):
                    disp = _disparity_to_native(disp, args.scale, (h0, w0))
                if need_vis:
                    _write_disp_vis(vis_path, disp)
                if need_cloud:
                    depth = _disp_to_depth(
                        disp, calib_full["fx"], calib_full["baseline"]
                    )
                    xyz, col = _backproject_colored(
                        depth,
                        left_full,
                        calib_full["K"],
                        eye,
                        origin,
                        args.z_min,
                        args.z_far,
                        args.pixel_stride,
                    )
                    _write_ply_rgb(ply_path, xyz, col)
                wrote += 1
                continue

            left = cv2.imread(lp, cv2.IMREAD_COLOR)
            right = cv2.imread(rp, cv2.IMREAD_COLOR)
            if left is None or right is None:
                print(f"  skip unreadable pair {stem}", file=sys.stderr)
                continue
            left_rgb = cv2.cvtColor(left, cv2.COLOR_BGR2RGB)
            right_rgb = cv2.cvtColor(right, cv2.COLOR_BGR2RGB)
            left_full = left_rgb
            h0, w0 = left_full.shape[:2]
            left_in, right_in = _resize_pair(left_rgb, right_rgb, args.scale)
            h, w = left_in.shape[:2]
            img0 = (
                torch.as_tensor(left_in, device=device).float()[None].permute(0, 3, 1, 2)
            )
            img1 = (
                torch.as_tensor(right_in, device=device)
                .float()[None]
                .permute(0, 3, 1, 2)
            )
            padder = InputPadder(img0.shape, divis_by=32)
            img0_p, img1_p = padder.pad(img0, img1)
            with torch.no_grad():
                disp_t = model(img0_p, img1_p, max_disp=args.max_disp, test_mode=True)
            disp_infer = padder.unpad(disp_t.float()).detach().cpu().numpy().reshape(h, w)

            if args.remove_invisible:
                xx = np.arange(w, dtype=np.float32)[None, :].repeat(h, axis=0)
                disp_infer = disp_infer.copy()
                disp_infer[xx - disp_infer < 0] = np.nan

            disp = _disparity_to_native(disp_infer, args.scale, (h0, w0))
            depth = _disp_to_depth(disp, calib_full["fx"], calib_full["baseline"])
            np.save(disp_path, disp.astype(np.float32))
            if args.write_vis:
                _write_disp_vis(vis_path, disp)
            if args.write_depth:
                np.save(os.path.join(depth_dir, f"{stem}.npy"), depth)
                mm = np.zeros(depth.shape, dtype=np.uint16)
                finite = np.isfinite(depth) & (depth > 0)
                mm[finite] = np.clip(
                    np.round(depth[finite] * 1000.0), 1, 65535
                ).astype(np.uint16)
                cv2.imwrite(os.path.join(depth_dir, f"{stem}.mm.png"), mm)
            n_pts = 0
            if args.write_clouds:
                xyz, col = _backproject_colored(
                    depth,
                    left_full,
                    calib_full["K"],
                    eye,
                    origin,
                    args.z_min,
                    args.z_far,
                    args.pixel_stride,
                )
                _write_ply_rgb(ply_path, xyz, col)
                n_pts = int(xyz.shape[0])
            wrote += 1
            if i == 0 or i + 1 == n or (i + 1) % 10 == 0:
                extra = f"  cloud={n_pts}" if args.write_clouds else ""
                print(
                    f"  {i + 1}/{n}  {stem}  disp med={np.nanmedian(disp):.2f} px{extra}"
                )

        meta = {
            "model": version,
            "model_size": model_size,
            "label": label,
            "checkpoint": ckpt,
            "max_disp": int(args.max_disp),
            "fx": calib_full["fx"],
            "fy": calib_full["fy"],
            "cx": calib_full["cx"],
            "cy": calib_full["cy"],
            "fx_infer": calib["fx"],
            "scale": float(args.scale),
            "inference_size": [int(calib["width"]), int(calib["height"])],
            "native_size": [int(calib_full["width"]), int(calib_full["height"])],
            "disparity_units": "pixels at native image resolution (d_native = d_infer / scale)",
            "baseline_m": calib["baseline"],
            "depth_formula": "Z = fx * baseline / disparity",
            "n_frames": n,
            "wrote": wrote,
            "skipped_existing": skipped,
            "disparity_dir": os.path.relpath(disp_dir, frames_dir),
            "pointcloud_dir": os.path.relpath(cloud_dir, frames_dir)
            if args.write_clouds
            else "",
            "z_min": args.z_min,
            "z_far": args.z_far,
            "pixel_stride": args.pixel_stride,
            "coordinate_frame": "left_camera",
        }
        os.makedirs(os.path.dirname(meta_path), exist_ok=True)
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)
        print(
            f"  wrote {wrote} ({skipped} already present)  "
            f"disp={disp_dir}"
            + (f"  ply={cloud_dir}" if args.write_clouds else "")
        )
    return 0


def _left_view_id(recon, index: int, left_path: str):
    name = f"L_{index:05d}_{os.path.basename(left_path)}"
    vid = recon.ViewIdFromName(name)
    if vid is not None and int(vid) >= 0 and recon.View(vid) is not None:
        return vid
    suffix = os.path.basename(left_path)
    for cand in recon.ViewIds():
        view = recon.View(cand)
        if view is None:
            continue
        vn = view.Name()
        if vn.startswith("L_") and vn.endswith(suffix):
            return cand
    return None


def _backproject_colored(
    depth: np.ndarray,
    rgb: np.ndarray,
    K: np.ndarray,
    R_w2c: np.ndarray,
    center: np.ndarray,
    z_min: float,
    z_far: float,
    pixel_stride: int,
) -> tuple[np.ndarray, np.ndarray]:
    h, w = depth.shape
    us = np.arange(0, w, pixel_stride)
    vs = np.arange(0, h, pixel_stride)
    uu, vv = np.meshgrid(us, vs)
    z = depth[vv, uu]
    keep = np.isfinite(z) & (z >= z_min) & (z <= z_far)
    uu, vv, z = uu[keep], vv[keep], z[keep]
    if z.size == 0:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.uint8)
    x = (uu.astype(np.float64) - K[0, 2]) * z / K[0, 0]
    y = (vv.astype(np.float64) - K[1, 2]) * z / K[1, 1]
    cam = np.stack([x, y, z.astype(np.float64)], axis=1)
    world = (R_w2c.T @ cam.T).T + center.reshape(1, 3)
    col = rgb[vv, uu]
    return world, col


def cmd_fuse(args: argparse.Namespace) -> int:
    import open3d as o3d
    import pytheia as pt

    frames_dir = os.path.abspath(args.frames_dir)
    left_dir = os.path.join(frames_dir, "left")
    ext = args.img_ext or _guess_img_ext(left_dir)
    left_paths = _sorted_images(left_dir, ext)
    n = len(left_paths)
    if n < 1:
        print(f"No left images in {left_dir}", file=sys.stderr)
        return 1

    calib = _load_calib(frames_dir)
    _, depth_dir = _depth_dirs(frames_dir)
    recon_path = args.recon or os.path.join(frames_dir, "stereo_rig.recon")
    ok, recon = pt.io.ReadReconstruction(recon_path)
    if not ok:
        print(f"Failed to read reconstruction {recon_path}", file=sys.stderr)
        return 1

    keyframes = reconstruction_keyframe_indices(n, args.recon_stride)
    print(
        f"Fusing {len(keyframes)}/{n} keyframes  stride={args.recon_stride}  "
        f"z=[{args.z_min}, {args.z_far}] m  voxel={args.voxel_size} m"
    )

    all_xyz = []
    all_rgb = []
    used = 0
    for i in keyframes:
        stem = _stem(left_paths[i])
        depth_path = os.path.join(depth_dir, f"{stem}.npy")
        if not os.path.isfile(depth_path):
            print(f"  missing depth {depth_path}", file=sys.stderr)
            continue
        vid = _left_view_id(recon, i, left_paths[i])
        if vid is None:
            print(f"  no left view for frame {i} {stem}", file=sys.stderr)
            continue
        view = recon.View(vid)
        if view is None or not view.IsEstimated():
            print(f"  left view not estimated: {view.Name() if view else vid}")
            continue
        depth = np.load(depth_path)
        bgr = cv2.imread(left_paths[i], cv2.IMREAD_COLOR)
        if bgr is None:
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        cam = view.Camera()
        R = np.asarray(cam.GetOrientationAsRotationMatrix(), dtype=np.float64)
        c = np.asarray(cam.GetPosition(), dtype=np.float64).reshape(3)
        xyz, col = _backproject_colored(
            depth,
            rgb,
            calib["K"],
            R,
            c,
            args.z_min,
            args.z_far,
            args.pixel_stride,
        )
        if xyz.shape[0] == 0:
            continue
        all_xyz.append(xyz)
        all_rgb.append(col)
        used += 1
        print(f"  keyframe {i:03d}  {stem}  {xyz.shape[0]} pts")

    if not all_xyz:
        print("No keyframe clouds to fuse.", file=sys.stderr)
        return 2

    xyz = np.vstack(all_xyz)
    rgb = np.vstack(all_rgb)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(np.clip(rgb.astype(np.float64) / 255.0, 0, 1))
    if args.voxel_size > 0:
        pcd = pcd.voxel_down_sample(args.voxel_size)
    out = args.out_ply or os.path.join(frames_dir, "keyframes_las2_m_fused.ply")
    o3d.io.write_point_cloud(out, pcd)
    print(
        f"Fused {used} keyframes → {len(pcd.points)} points  wrote {out}"
    )
    if args.visualize:
        o3d.visualization.draw_geometries([pcd], window_name="LAS2-M keyframe fusion")
    return 0


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="LiteAnyStereo disparity/depth precompute + keyframe fusion"
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    pc = sub.add_parser("precompute", help="Run LAS2 on every stereo pair")
    pc.add_argument("--frames_dir", nargs="+", required=True)
    pc.add_argument("--las_root", default=DEFAULT_LAS_ROOT)
    pc.add_argument("--restore_ckpt", default="", help="Default: checkpoints/LAS2_{SIZE}.pth")
    pc.add_argument("--version", default="las2")
    pc.add_argument("--model_size", default="m", choices=("s", "m", "l", "h"))
    pc.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Downsample factor before inference (<=1). Disparity is in the scaled pixel units; K is scaled to match.",
    )
    pc.add_argument("--max_disp", type=int, default=192)
    pc.add_argument("--device", default="cuda", choices=("cuda", "cpu"))
    pc.add_argument("--img_ext", default="")
    pc.add_argument(
        "--out_subdir",
        default=OUT_SUBDIR,
        help="Write disparity/pointclouds under <frames_dir>/<this>/ (empty = legacy las2_m dirs)",
    )
    pc.add_argument(
        "--write_clouds",
        action="store_true",
        help="Write per-frame colored PLYs in the left-camera frame",
    )
    pc.add_argument(
        "--write_vis",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Write Turbo-colormap disparity PNGs under disparity/vis/ (default on)",
    )
    pc.add_argument("--write_depth", action="store_true", help="Also write depth npy/png")
    pc.add_argument("--z_min", type=float, default=0.05, help="Cloud near clip (m)")
    pc.add_argument("--z_far", type=float, default=1.0, help="Cloud far clip (m); keep points closer than this")
    pc.add_argument("--pixel_stride", type=int, default=1, help="Use every Nth pixel in clouds")
    pc.add_argument(
        "--remove_invisible",
        action="store_true",
        default=True,
        help="Invalidate pixels whose match would fall left of the image (default on)",
    )
    pc.add_argument("--keep_invisible", action="store_true")
    pc.add_argument("--overwrite", action="store_true")

    fu = sub.add_parser("fuse", help="Backproject keyframe depths with SfM poses")
    fu.add_argument("--frames_dir", required=True)
    fu.add_argument("--recon", default="", help="Default: <frames_dir>/stereo_rig.recon")
    fu.add_argument("--recon_stride", type=int, default=5)
    fu.add_argument("--z_min", type=float, default=0.3)
    fu.add_argument("--z_far", type=float, default=15.0)
    fu.add_argument("--voxel_size", type=float, default=0.02)
    fu.add_argument("--pixel_stride", type=int, default=2, help="Use every Nth pixel")
    fu.add_argument("--out_ply", default="")
    fu.add_argument("--img_ext", default="")
    fu.add_argument("--visualize", action="store_true")
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    if args.cmd == "precompute":
        if args.keep_invisible:
            args.remove_invisible = False
        return cmd_precompute(args)
    return cmd_fuse(args)


if __name__ == "__main__":
    raise SystemExit(main())
