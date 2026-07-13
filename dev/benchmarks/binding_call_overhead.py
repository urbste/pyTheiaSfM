#!/usr/bin/env python3
"""Micro-benchmarks for pyTheia Python binding call overhead.

Usage:
  PYTHONPATH=src python dev/benchmarks/binding_call_overhead.py
  PYTHONPATH=src python dev/benchmarks/binding_call_overhead.py --iterations 50000
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO / "pytests" / "sfm") not in sys.path:
    sys.path.insert(0, str(_REPO / "pytests" / "sfm"))

import pytheia as pt
from random_recon_gen import RandomReconGenerator


def _detect_backend() -> str:
    so = next(Path(_REPO / "src" / "pytheia").glob("pytheia*.so"), None)
    if so is None:
        return "unknown"
    try:
        import subprocess

        out = subprocess.check_output(["nm", "-D", str(so)], text=True, errors="ignore")
        if "nanobind" in out or "PyInit_pytheia" in out:
            # nanobind modules still export PyInit_; check linked symbols
            if "nanobind" in subprocess.check_output(
                ["ldd", str(so)], text=True, errors="ignore"
            ):
                return "nanobind"
    except OSError:
        pass
    try:
        import subprocess

        ldd = subprocess.check_output(["ldd", str(so)], text=True, errors="ignore")
        if "nanobind" in ldd.lower() or "libnanobind" in ldd:
            return "nanobind"
        # pybind11 is header-only; use file size heuristic from migration baseline
        size = so.stat().st_size
        if size > 16_500_000:
            return "pybind11 (heuristic)"
        return "nanobind (heuristic)"
    except OSError:
        return "unknown"


def _timeit(name: str, fn, iterations: int, warmup: int = 200) -> dict:
    for _ in range(warmup):
        fn()
    t0 = time.perf_counter()
    for _ in range(iterations):
        fn()
    elapsed = time.perf_counter() - t0
    per_call_ns = elapsed / iterations * 1e9
    return {
        "name": name,
        "iterations": iterations,
        "total_s": elapsed,
        "per_call_ns": per_call_ns,
        "calls_per_s": iterations / elapsed,
    }


def _fixtures():
    gen = RandomReconGenerator(seed=0)
    recon = gen.generate_random_recon(nr_views=6, nr_tracks=80)
    vid = next(iter(recon.ViewIds()))
    view = recon.View(vid)
    cam = view.Camera()
    point3d = np.array([0.1, 0.2, 5.0, 1.0], dtype=np.float64)
    rot = np.eye(3)
    trans = np.array([1.0, 2.0, 3.0])
    return recon, vid, view, cam, point3d, rot, trans


def run_benchmarks(iterations: int, skip_mvs: bool = False) -> list[dict]:
    recon, vid, view, cam, point3d, rot, trans = _fixtures()
    results = []

    results.append(
        _timeit(
            "Camera.ProjectPoint",
            lambda: cam.ProjectPoint(point3d),
            iterations,
        )
    )
    results.append(
        _timeit(
            "Camera.GetPosition",
            lambda: cam.GetPosition(),
            iterations,
        )
    )
    results.append(
        _timeit(
            "Reconstruction.View",
            lambda: recon.View(vid),
            iterations,
        )
    )
    results.append(
        _timeit(
            "View.MutableCamera + SetPosition",
            lambda: recon.MutableView(vid).MutableCamera().SetPosition(trans),
            max(iterations // 4, 1000),
        )
    )
    results.append(
        _timeit(
            "math.SE3d construct",
            lambda: pt.math.SE3d(rot, trans),
            iterations,
        )
    )
    results.append(
        _timeit(
            "math.SE3d * point",
            lambda: pt.math.SE3d(rot, trans) * trans,
            iterations,
        )
    )
    if not skip_mvs:
        results.append(
            _timeit(
                "mvs.ViewSelectionMVSNet",
                lambda: pt.mvs.ViewSelectionMVSNet(recon, 3, 5.0, 1.0, 10.0),
                max(iterations // 200, 50),
            )
        )
    return results


def main() -> int:
    parser = argparse.ArgumentParser(description="pyTheia binding call overhead benchmark")
    parser.add_argument("--iterations", type=int, default=20_000)
    parser.add_argument("--label", type=str, default=None, help="Backend label for output")
    parser.add_argument("--skip-mvs", action="store_true", help="Skip ViewSelectionMVSNet (pybind11 HEAD can segfault)")
    args = parser.parse_args()

    label = args.label or _detect_backend()
    print(f"backend: {label}")
    print(f"iterations (default): {args.iterations}")
    print(f"python: {sys.version.split()[0]}")
    print()
    print(f"{'benchmark':<36} {'iter':>8} {'total_ms':>10} {'ns/call':>10} {'calls/s':>12}")
    print("-" * 80)

    for row in run_benchmarks(args.iterations, skip_mvs=args.skip_mvs):
        print(
            f"{row['name']:<36} "
            f"{row['iterations']:8d} "
            f"{row['total_s'] * 1e3:10.2f} "
            f"{row['per_call_ns']:10.1f} "
            f"{row['calls_per_s']:12.0f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
