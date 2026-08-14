#!/usr/bin/env python3
# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""
Extract rectified stereo frames and calibrated rig extrinsics from a ZED SVO.

Requires the Stereolabs ZED SDK Python package (`pyzed`). Opens a recorded
`.svo` / `.svo2`, writes left/right **rectified** image sequences, and dumps
intrinsics + stereo extrinsics for use with `stereo_rig_reconstruction.py`.

Example:
  python pyexamples/preprocess/zed_svo_extract_stereo.py \\
    --svo /data/capture.svo2 --out_dir /data/zed_frames --every 5 \\
    --depth_mode neural_plus

  python pyexamples/stereo/stereo_rig_reconstruction.py \\
    --frames_dir /data/zed_frames --matcher edm --method global
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Extract rectified left/right frames and rig calibration from a "
            "ZED SVO recording (requires pyzed)."
        )
    )
    p.add_argument(
        "--svo",
        type=str,
        required=True,
        help="Path to .svo / .svo2 recording",
    )
    p.add_argument(
        "--out_dir",
        type=str,
        required=True,
        help="Output directory (creates left/, right/, rig_calibration.json)",
    )
    p.add_argument(
        "--img_ext",
        type=str,
        default="png",
        choices=("png", "jpg", "jpeg"),
    )
    p.add_argument(
        "--every",
        type=int,
        default=1,
        help="Keep every N-th SVO frame (default: 1 = all)",
    )
    p.add_argument("--start_frame", type=int, default=0, help="First SVO index")
    p.add_argument(
        "--end_frame",
        type=int,
        default=-1,
        help="Last SVO index inclusive (-1 = end of file)",
    )
    p.add_argument(
        "--max_frames",
        type=int,
        default=0,
        help="Stop after this many saved frames (0 = unlimited)",
    )
    p.add_argument(
        "--disable_self_calib",
        action="store_true",
        help=(
            "Disable ZED self-calibration on open (more reproducible params "
            "across replays of the same SVO)."
        ),
    )
    p.add_argument(
        "--jpeg_quality",
        type=int,
        default=95,
        help="JPEG quality when --img_ext is jpg/jpeg",
    )
    p.add_argument(
        "--depth_mode",
        type=str,
        default="neural_plus",
        help=(
            "ZED DEPTH_MODE: none, neural_light, neural, neural_plus "
            "(highest neural), ultra (highest classical), quality, performance. "
            "Default: neural_plus."
        ),
    )
    p.add_argument(
        "--skip_images",
        action="store_true",
        help="Do not rewrite left/right/timestamps (depth-only pass on an existing extract).",
    )
    p.add_argument(
        "--depth_subdir",
        type=str,
        default="",
        help="Output folder under out_dir (default: zed_<depth_mode>).",
    )
    p.add_argument(
        "--write_clouds",
        action="store_true",
        help="Write colored left-camera PLYs from ZED depth (default off).",
    )
    p.add_argument("--z_min", type=float, default=0.05, help="Cloud near clip (m)")
    p.add_argument("--z_far", type=float, default=1.0, help="Cloud far clip (m)")
    p.add_argument("--pixel_stride", type=int, default=1)
    return p.parse_args()


def _parse_depth_mode(sl, name: str):
    key = str(name).strip().lower().replace("-", "_")
    aliases = {
        "none": sl.DEPTH_MODE.NONE,
        "off": sl.DEPTH_MODE.NONE,
        "neural_light": sl.DEPTH_MODE.NEURAL_LIGHT,
        "neural": sl.DEPTH_MODE.NEURAL,
        "neural_plus": sl.DEPTH_MODE.NEURAL_PLUS,
        "ultra": sl.DEPTH_MODE.ULTRA,
        "quality": sl.DEPTH_MODE.QUALITY,
        "performance": sl.DEPTH_MODE.PERFORMANCE,
    }
    if key not in aliases:
        raise ValueError(
            f"Unknown depth mode '{name}'. Choose from: {', '.join(sorted(aliases))}"
        )
    return aliases[key]


def _sanitize_measure(arr):
    import numpy as np

    out = np.asarray(arr)
    if out.ndim == 3:
        out = out[..., 0]
    out = out.astype(np.float32, copy=True)
    invalid = ~np.isfinite(out) | (out <= 0)
    out[invalid] = np.nan
    return out


def _write_turbo_vis(path: str, values, valid_min: float = 0.0) -> None:
    import cv2
    import numpy as np

    valid = np.isfinite(values) & (values > valid_min)
    vis = np.zeros((*values.shape, 3), dtype=np.uint8)
    if np.any(valid):
        lo, hi = np.percentile(values[valid], [1.0, 99.0])
        hi = max(float(hi), float(lo) + 1e-3)
        norm = np.zeros(values.shape, dtype=np.float32)
        norm[valid] = np.clip((values[valid] - lo) / (hi - lo), 0.0, 1.0)
        vis = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
        vis[~valid] = 0
    cv2.imwrite(path, vis)


def _write_ply_rgb(path: str, xyz, rgb) -> None:
    import numpy as np

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


def _backproject_colored(depth, rgb, K, z_min: float, z_far: float, pixel_stride: int):
    import numpy as np

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
    col = rgb[vv, uu]
    return cam, col


def _camera_params_dict(cam) -> dict[str, Any]:
    disto = []
    try:
        disto = [float(x) for x in list(cam.disto)]
    except Exception:
        pass
    return {
        "fx": float(cam.fx),
        "fy": float(cam.fy),
        "cx": float(cam.cx),
        "cy": float(cam.cy),
        "distortion": disto,
    }


def _transform_to_dict(transform) -> dict[str, Any]:
    """Serialize sl.Transform to rotation (row-major 3x3) + translation."""
    try:
        m = transform.get_data()  # typically 4x4
        import numpy as np

        mat = np.asarray(m, dtype=float).reshape(4, 4)
        R = mat[:3, :3]
        t = mat[:3, 3]
        return {
            "rotation_3x3": R.reshape(-1).tolist(),
            "translation": t.tolist(),
        }
    except Exception:
        pass
    try:
        t = transform.get_translation().get()
        r = transform.get_rotation_matrix().r
        import numpy as np

        R = np.asarray(r, dtype=float).reshape(3, 3)
        return {
            "rotation_3x3": R.reshape(-1).tolist(),
            "translation": [float(t[0]), float(t[1]), float(t[2])],
        }
    except Exception as exc:
        return {"error": f"could not serialize transform: {exc}"}


def _build_calibration(
    zed,
    svo_path: str,
    units_name: str,
) -> dict[str, Any]:
    info = zed.get_camera_information()
    cam_conf = info.camera_configuration
    calib = cam_conf.calibration_parameters
    resolution = cam_conf.resolution

    left = _camera_params_dict(calib.left_cam)
    right = _camera_params_dict(calib.right_cam)
    stereo = _transform_to_dict(calib.stereo_transform)

    baseline = None
    try:
        baseline = float(calib.get_camera_baseline())
    except Exception:
        pass
    if baseline is None and "translation" in stereo:
        baseline = abs(float(stereo["translation"][0]))

    # ZED reports baseline / stereo_transform in the InitParameters units.
    # We open with UNIT.METER so values are meters for pyTheia.
    half_b = 0.5 * float(baseline) if baseline is not None else 0.0
    focal = 0.5 * (left["fx"] + left["fy"])

    model_name = ""
    try:
        model_name = str(info.camera_model)
    except Exception:
        model_name = repr(getattr(info, "camera_model", ""))

    calib_json: dict[str, Any] = {
        "source": "zed_svo",
        "svo": os.path.abspath(svo_path),
        "rectified": True,
        "view": {"left": "VIEW.LEFT", "right": "VIEW.RIGHT"},
        "coordinate_units": units_name,
        "image_width": int(resolution.width),
        "image_height": int(resolution.height),
        "fps": float(cam_conf.fps),
        "serial_number": int(info.serial_number),
        "camera_model": model_name,
        "left": left,
        "right": right,
        "stereo_transform": stereo,
        "baseline_meters": baseline,
        "pytheia": {
            "focal": focal,
            "cx": left["cx"],
            "cy": left["cy"],
            "width": int(resolution.width),
            "height": int(resolution.height),
            "baseline": baseline,
            "left_position_in_rig": [-half_b, 0.0, 0.0],
            "right_position_in_rig": [half_b, 0.0, 0.0],
            "notes": (
                "Use with pyexamples/stereo/stereo_rig_reconstruction.py. "
                "Rectified ZED images are effectively pinhole (distortion≈0). "
                "Body frame at stereo mid-point; +X toward the right camera."
            ),
        },
    }
    return calib_json


def _imwrite(path: str, bgr, img_ext: str, jpeg_quality: int) -> None:
    import cv2

    if img_ext in ("jpg", "jpeg"):
        cv2.imwrite(path, bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)])
    else:
        cv2.imwrite(path, bgr)


def main() -> int:
    args = _parse_args()
    if args.every < 1:
        print("--every must be >= 1", file=sys.stderr)
        return 1
    if not os.path.isfile(args.svo):
        print(f"SVO not found: {args.svo}", file=sys.stderr)
        return 1
    if not args.svo.lower().endswith((".svo", ".svo2")):
        print("Expected a .svo / .svo2 file", file=sys.stderr)
        return 1

    try:
        import pyzed.sl as sl
    except ImportError:
        print(
            "pyzed is required. Install the ZED SDK and its Python API:\n"
            "  https://www.stereolabs.com/docs/installation\n"
            "  https://www.stereolabs.com/docs/app-development/python/install",
            file=sys.stderr,
        )
        return 1

    try:
        import cv2
        import numpy as np
    except ImportError:
        print("opencv-python (cv2) and numpy are required", file=sys.stderr)
        return 1

    try:
        depth_mode = _parse_depth_mode(sl, args.depth_mode)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 1
    depth_mode_name = str(args.depth_mode).strip().lower().replace("-", "_")
    save_depth = depth_mode != sl.DEPTH_MODE.NONE
    depth_subdir = args.depth_subdir or (f"zed_{depth_mode_name}" if save_depth else "")

    left_dir = os.path.join(args.out_dir, "left")
    right_dir = os.path.join(args.out_dir, "right")
    if not args.skip_images:
        os.makedirs(left_dir, exist_ok=True)
        os.makedirs(right_dir, exist_ok=True)

    depth_dir = disp_dir = vis_depth_dir = vis_disp_dir = ""
    if save_depth:
        root = os.path.join(args.out_dir, depth_subdir)
        depth_dir = os.path.join(root, "depth")
        disp_dir = os.path.join(root, "disparity")
        vis_depth_dir = os.path.join(depth_dir, "vis")
        vis_disp_dir = os.path.join(disp_dir, "vis")
        cloud_dir = os.path.join(root, "pointclouds")
        for d in (depth_dir, disp_dir, vis_depth_dir, vis_disp_dir):
            os.makedirs(d, exist_ok=True)
        if args.write_clouds:
            os.makedirs(cloud_dir, exist_ok=True)

    init = sl.InitParameters()
    init.set_from_svo_file(args.svo)
    init.svo_real_time_mode = False
    init.coordinate_units = sl.UNIT.METER
    init.depth_mode = depth_mode
    if args.disable_self_calib:
        init.camera_disable_self_calib = True

    zed = sl.Camera()
    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open SVO: {repr(err)}", file=sys.stderr)
        return 1

    units_name = "meter"
    calib = _build_calibration(zed, args.svo, units_name)
    calib["depth_mode"] = depth_mode_name
    calib_path = os.path.join(args.out_dir, "rig_calibration.json")
    if not args.skip_images:
        with open(calib_path, "w", encoding="utf-8") as f:
            json.dump(calib, f, indent=2)
            f.write("\n")

    nb = int(zed.get_svo_number_of_frames())
    end_frame = nb - 1 if args.end_frame < 0 else min(args.end_frame, nb - 1)
    print(
        f"SVO frames={nb}  save [{args.start_frame}, {end_frame}] "
        f"every={args.every}  resolution="
        f"{calib['image_width']}x{calib['image_height']}  "
        f"baseline={calib['baseline_meters']} m  depth={depth_mode_name}"
    )

    left_mat = sl.Mat()
    right_mat = sl.Mat()
    depth_mat = sl.Mat()
    disp_mat = sl.Mat()
    runtime = sl.RuntimeParameters()
    timestamps_path = os.path.join(args.out_dir, "timestamps_ns.txt")
    saved = 0
    img_ext = args.img_ext.lower()
    K = np.array(
        [
            [calib["left"]["fx"], 0.0, calib["left"]["cx"]],
            [0.0, calib["left"]["fy"], calib["left"]["cy"]],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    ts_ctx = (
        open(os.devnull, "w", encoding="utf-8")
        if args.skip_images
        else open(timestamps_path, "w", encoding="utf-8")
    )
    with ts_ctx as ts_file:
        if not args.skip_images:
            ts_file.write("# save_index svo_index timestamp_ns\n")
        while True:
            err = zed.grab(runtime)
            if err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                break
            if err != sl.ERROR_CODE.SUCCESS:
                print(f"grab failed: {repr(err)}", file=sys.stderr)
                break

            svo_idx = int(zed.get_svo_position())
            if svo_idx < args.start_frame:
                continue
            if svo_idx > end_frame:
                break
            if (svo_idx - args.start_frame) % args.every != 0:
                continue

            name = f"{saved:06d}.{img_ext}"
            stem = f"{saved:06d}"
            if not args.skip_images:
                zed.retrieve_image(left_mat, sl.VIEW.LEFT)
                zed.retrieve_image(right_mat, sl.VIEW.RIGHT)

                left_bgra = left_mat.get_data()
                right_bgra = right_mat.get_data()
                left_bgr = cv2.cvtColor(left_bgra, cv2.COLOR_BGRA2BGR)
                right_bgr = cv2.cvtColor(right_bgra, cv2.COLOR_BGRA2BGR)

                _imwrite(
                    os.path.join(left_dir, name),
                    left_bgr,
                    img_ext,
                    args.jpeg_quality,
                )
                _imwrite(
                    os.path.join(right_dir, name),
                    right_bgr,
                    img_ext,
                    args.jpeg_quality,
                )

                ts_ns = 0
                try:
                    ts_ns = int(
                        zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
                    )
                except Exception:
                    pass
                ts_file.write(f"{saved} {svo_idx} {ts_ns}\n")

            if save_depth:
                zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
                zed.retrieve_measure(disp_mat, sl.MEASURE.DISPARITY)
                depth = _sanitize_measure(depth_mat.get_data())
                disp = np.asarray(disp_mat.get_data())
                if disp.ndim == 3:
                    disp = disp[..., 0]
                disp = disp.astype(np.float32, copy=True)
                disp[~np.isfinite(disp)] = np.nan
                # ZED disparity is typically negative (right-minus-left).
                disp_abs = np.abs(disp)
                np.save(os.path.join(depth_dir, f"{stem}.npy"), depth)
                np.save(os.path.join(disp_dir, f"{stem}.npy"), disp)
                mm = np.zeros(depth.shape, dtype=np.uint16)
                finite = np.isfinite(depth) & (depth > 0)
                mm[finite] = np.clip(np.round(depth[finite] * 1000.0), 1, 65535).astype(
                    np.uint16
                )
                cv2.imwrite(os.path.join(depth_dir, f"{stem}.mm.png"), mm)
                _write_turbo_vis(os.path.join(vis_depth_dir, f"{stem}.png"), depth, 0.0)
                _write_turbo_vis(os.path.join(vis_disp_dir, f"{stem}.png"), disp_abs, 0.0)
                if args.write_clouds:
                    left_bgr_c = None
                    if not args.skip_images:
                        left_bgr_c = left_bgr
                    else:
                        left_bgr_c = cv2.imread(os.path.join(left_dir, name))
                    if left_bgr_c is not None:
                        left_rgb = cv2.cvtColor(left_bgr_c, cv2.COLOR_BGR2RGB)
                        xyz, col = _backproject_colored(
                            depth,
                            left_rgb,
                            K,
                            args.z_min,
                            args.z_far,
                            args.pixel_stride,
                        )
                        _write_ply_rgb(
                            os.path.join(cloud_dir, f"{stem}.ply"), xyz, col
                        )

            saved += 1
            if saved % 50 == 0:
                print(f"  saved {saved} stereo pairs (svo_idx={svo_idx})")
            if args.max_frames > 0 and saved >= args.max_frames:
                break

    zed.close()

    if save_depth:
        meta = {
            "source": "zed_svo",
            "svo": os.path.abspath(args.svo),
            "depth_mode": depth_mode_name,
            "units": "meter",
            "n_frames": saved,
            "every": args.every,
            "depth_dir": os.path.join(depth_subdir, "depth"),
            "disparity_dir": os.path.join(depth_subdir, "disparity"),
            "pointcloud_dir": os.path.join(depth_subdir, "pointclouds")
            if args.write_clouds
            else "",
            "z_min": args.z_min,
            "z_far": args.z_far,
            "notes": (
                "MEASURE.DEPTH in meters (NaN invalid). "
                "MEASURE.DISPARITY in pixels (ZED sign, typically negative). "
                "Vis uses |disparity|. Filenames match left/right stems."
            ),
        }
        meta_path = os.path.join(args.out_dir, depth_subdir, "meta.json")
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)
            f.write("\n")

    print(f"Wrote {saved} pairs under {args.out_dir}/{{left,right}}")
    if not args.skip_images:
        print(f"Calibration: {calib_path}")
    if save_depth:
        print(f"ZED depth ({depth_mode_name}): {os.path.join(args.out_dir, depth_subdir)}")
    print(
        "Suggested next step:\n"
        f"  python pyexamples/stereo/stereo_rig_reconstruction.py \\\n"
        f"    --frames_dir {args.out_dir} --matcher edm --method global"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
