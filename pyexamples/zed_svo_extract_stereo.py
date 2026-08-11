#!/usr/bin/env python3
# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""
Extract rectified stereo frames and calibrated rig extrinsics from a ZED SVO.

Requires the Stereolabs ZED SDK Python package (`pyzed`). Opens a recorded
`.svo` / `.svo2`, writes left/right **rectified** image sequences, and dumps
intrinsics + stereo extrinsics for use with `stereo_rig_reconstruction.py`.

Example:
  python pyexamples/zed_svo_extract_stereo.py \\
    --svo /data/capture.svo2 --out_dir /data/zed_frames --every 5

  python pyexamples/stereo_rig_reconstruction.py \\
    --left_dir /data/zed_frames/left --right_dir /data/zed_frames/right \\
    --baseline $(python -c "import json; print(json.load(open('/data/zed_frames/rig_calibration.json'))['pytheia']['baseline'])") \\
    --focal $(python -c "import json; print(json.load(open('/data/zed_frames/rig_calibration.json'))['pytheia']['focal'])") \\
    --cx $(python -c "import json; print(json.load(open('/data/zed_frames/rig_calibration.json'))['pytheia']['cx'])") \\
    --cy $(python -c "import json; print(json.load(open('/data/zed_frames/rig_calibration.json'))['pytheia']['cy'])") \\
    --matcher edm --method global
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
    return p.parse_args()


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
                "Use with pyexamples/stereo_rig_reconstruction.py. "
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
    except ImportError:
        print("opencv-python (cv2) is required", file=sys.stderr)
        return 1

    left_dir = os.path.join(args.out_dir, "left")
    right_dir = os.path.join(args.out_dir, "right")
    os.makedirs(left_dir, exist_ok=True)
    os.makedirs(right_dir, exist_ok=True)

    init = sl.InitParameters()
    init.set_from_svo_file(args.svo)
    init.svo_real_time_mode = False
    init.coordinate_units = sl.UNIT.METER
    if args.disable_self_calib:
        init.camera_disable_self_calib = True

    zed = sl.Camera()
    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open SVO: {repr(err)}", file=sys.stderr)
        return 1

    units_name = "meter"
    calib = _build_calibration(zed, args.svo, units_name)
    calib_path = os.path.join(args.out_dir, "rig_calibration.json")
    with open(calib_path, "w", encoding="utf-8") as f:
        json.dump(calib, f, indent=2)
        f.write("\n")

    nb = int(zed.get_svo_number_of_frames())
    end_frame = nb - 1 if args.end_frame < 0 else min(args.end_frame, nb - 1)
    print(
        f"SVO frames={nb}  save [{args.start_frame}, {end_frame}] "
        f"every={args.every}  resolution="
        f"{calib['image_width']}x{calib['image_height']}  "
        f"baseline={calib['baseline_meters']} m"
    )

    left_mat = sl.Mat()
    right_mat = sl.Mat()
    runtime = sl.RuntimeParameters()
    timestamps_path = os.path.join(args.out_dir, "timestamps_ns.txt")
    saved = 0
    img_ext = args.img_ext.lower()

    with open(timestamps_path, "w", encoding="utf-8") as ts_file:
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

            zed.retrieve_image(left_mat, sl.VIEW.LEFT)
            zed.retrieve_image(right_mat, sl.VIEW.RIGHT)

            left_bgra = left_mat.get_data()
            right_bgra = right_mat.get_data()
            left_bgr = cv2.cvtColor(left_bgra, cv2.COLOR_BGRA2BGR)
            right_bgr = cv2.cvtColor(right_bgra, cv2.COLOR_BGRA2BGR)

            name = f"{saved:06d}.{img_ext}"
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
                ts_ns = int(zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds())
            except Exception:
                pass
            ts_file.write(f"{saved} {svo_idx} {ts_ns}\n")

            saved += 1
            if saved % 50 == 0:
                print(f"  saved {saved} stereo pairs (svo_idx={svo_idx})")
            if args.max_frames > 0 and saved >= args.max_frames:
                break

    zed.close()

    py = calib["pytheia"]
    print(f"Wrote {saved} pairs under {args.out_dir}/{{left,right}}")
    print(f"Calibration: {calib_path}")
    print(
        "Suggested next step:\n"
        f"  python pyexamples/stereo_rig_reconstruction.py \\\n"
        f"    --left_dir {left_dir} --right_dir {right_dir} \\\n"
        f"    --baseline {py['baseline']} --focal {py['focal']} \\\n"
        f"    --cx {py['cx']} --cy {py['cy']} \\\n"
        f"    --width {py['width']} --height {py['height']} \\\n"
        f"    --img_ext {img_ext} --matcher edm --method global"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
