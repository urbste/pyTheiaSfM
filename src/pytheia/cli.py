"""Command line interface for pyTheia (``python -m pytheia`` or ``pytheia``).

Subcommands::

    pytheia info RECON
    pytheia convert IN OUT --format {colmap,colmap-bin,nvm,bundler,ply,nerfstudio,json}
    pytheia view RECON [--html OUT.html]
    pytheia export-splat RECON --images DIR --out DIR [--no-undistort --max-image-size N]
    pytheia align A B
    pytheia sfm --input DIR --out DIR [--method {global,incremental,hybrid}]
                [--seed N] [--config FILE.json] [--set key=value ...]

``RECON`` is a binary reconstruction written by ``pytheia.io.WriteReconstruction``
(``.recon``). Feature detection and matching are not part of pyTheia; ``sfm``
takes the tracks and view graph that the application produced (see the CLI docs).

Exit status: 0 on success, 1 on a runtime failure, 2 on a usage error.
``main`` returns the status instead of calling ``sys.exit``.
"""

from __future__ import annotations

import argparse
import importlib
import json
import os
import sys
import time
from typing import Any, Callable, Iterator, Sequence, Tuple

import numpy as np

import pytheia as pt

PROG = "pytheia"
RECON_FILE = "reconstruction.recon"
VIEW_GRAPH_FILE = "view_graph.bin"
CONVERT_FORMATS = ("colmap", "colmap-bin", "nvm", "bundler", "ply", "nerfstudio", "json")
SFM_METHODS = ("global", "incremental", "hybrid")

_TRUE_WORDS = {"1", "true", "yes", "on"}
_FALSE_WORDS = {"0", "false", "no", "off"}


class CLIError(Exception):
    """User-facing error. Printed on stderr; ``code`` is the exit status."""

    def __init__(self, message: str, code: int = 1):
        super().__init__(message)
        self.code = code


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _read_recon(path: str):
    if not os.path.isfile(path):
        raise CLIError(f"reconstruction not found: {path}")
    try:
        ok, recon = pt.io.ReadReconstruction(path)
    except RuntimeError as exc:
        raise CLIError(f"could not read reconstruction {path}: {exc}") from exc
    if not ok:
        raise CLIError(
            f"could not read reconstruction {path} (expected a binary .recon file "
            "written by pytheia.io.WriteReconstruction)")
    return recon


def _load_optional(module_name: str, attr: str) -> Callable[..., Any]:
    """Import a module that another part of pyTheia may provide, lazily."""
    try:
        module = importlib.import_module(module_name)
    except ImportError as exc:
        raise CLIError(
            f"module '{module_name}' is not available in this pyTheia install ({exc}); "
            "this command needs it") from exc
    fn = getattr(module, attr, None)
    if fn is None:
        raise CLIError(f"module '{module_name}' has no function '{attr}'; this command needs it")
    return fn


def _ensure_parent(path: str) -> None:
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)


def _mean_reprojection_error(recon) -> float:
    return float(pt.sfm.ComputeMeanReprojectionError(recon))


# ---------------------------------------------------------------------------
# info
# ---------------------------------------------------------------------------

def cmd_info(args) -> int:
    recon = _read_recon(args.recon)
    view_ids = recon.ViewIds()
    track_ids = recon.TrackIds()
    info = {
        "file": os.path.abspath(args.recon),
        "views": len(view_ids),
        "registered_views": sum(1 for v in view_ids if recon.View(v).IsEstimated()),
        "tracks": len(track_ids),
        "estimated_tracks": sum(1 for t in track_ids if recon.Track(t).IsEstimated()),
        "observations": sum(recon.View(v).NumFeatures() for v in view_ids),
        "mean_reprojection_error_px": _mean_reprojection_error(recon),
    }
    if args.json:
        print(json.dumps(info, indent=2))
        return 0
    print(f"file:                    {info['file']}")
    print(f"views:                   {info['views']} (registered {info['registered_views']})")
    print(f"tracks:                  {info['tracks']} (estimated {info['estimated_tracks']})")
    print(f"observations:            {info['observations']}")
    print(f"mean reprojection error: {info['mean_reprojection_error_px']:.4f} px")
    return 0


# ---------------------------------------------------------------------------
# convert
# ---------------------------------------------------------------------------

def cmd_convert(args) -> int:
    recon = _read_recon(args.input)
    fmt, out = args.format, args.output
    ok = None
    if fmt == "colmap":
        os.makedirs(out, exist_ok=True)
        ok = pt.io.WriteColmapFiles(reconstruction=recon, output_directory=out)
    elif fmt == "colmap-bin":
        os.makedirs(out, exist_ok=True)
        try:
            ok = pt.io.WriteColmapFiles(reconstruction=recon, output_directory=out, binary=True)
        except TypeError as exc:
            raise CLIError(
                "this pyTheia build has no binary COLMAP writer "
                "(pytheia.io.WriteColmapFiles has no 'binary' argument)") from exc
    elif fmt == "nvm":
        _ensure_parent(out)
        ok = pt.io.WriteNVMFile(nvm_filepath=out, reconstruction=recon)
    elif fmt == "bundler":
        os.makedirs(out, exist_ok=True)
        ok = pt.io.WriteBundlerFiles(
            reconstruction=recon,
            lists_file=os.path.join(out, "list.txt"),
            bundle_file=os.path.join(out, "bundle.out"))
    elif fmt == "ply":
        _ensure_parent(out)
        ok = pt.io.WritePlyFile(
            ply_file=out,
            reconstruction=recon,
            camera_color=np.array([255, 0, 0], dtype=np.int32),
            min_num_observations_per_point=args.min_observations)
    elif fmt == "nerfstudio":
        if not args.images:
            raise CLIError("--images DIR is required for --format nerfstudio", code=2)
        _ensure_parent(out)
        ok = pt.io.WriteNerfStudio(
            path_to_images=args.images,
            reconstruction=recon,
            aabb_scale=args.aabb_scale,
            out_json_nerfstudio_file=out)
    elif fmt == "json":
        _ensure_parent(out)
        ok = pt.io.WriteReconstructionJson(reconstruction=recon, output_json_file=out)
    else:  # pragma: no cover - argparse restricts the choices
        raise CLIError(f"unknown format: {fmt}", code=2)

    if ok is False:
        raise CLIError(f"writing {fmt} output to {out} failed")
    print(f"wrote {fmt}: {out}")
    return 0


# ---------------------------------------------------------------------------
# view / export-splat (optional modules, imported lazily)
# ---------------------------------------------------------------------------

def cmd_view(args) -> int:
    recon = _read_recon(args.recon)
    export_html = _load_optional("pytheia.viewer", "export_html")
    html = args.html or os.path.splitext(os.path.basename(args.recon))[0] + ".html"
    export_html(recon, html)
    print(f"wrote viewer: {html}")
    return 0


def cmd_export_splat(args) -> int:
    recon = _read_recon(args.recon)
    if not os.path.isdir(args.images):
        raise CLIError(f"image directory not found: {args.images}")
    export = _load_optional("pytheia.export", "export_for_gaussian_splatting")
    try:
        export(
            recon,
            args.out,
            args.images,
            undistort=not args.no_undistort,
            max_image_size=args.max_image_size,
        )
    except (FileNotFoundError, ValueError) as exc:
        raise CLIError(str(exc))
    print(f"wrote Gaussian splatting input: {args.out}")
    return 0


# ---------------------------------------------------------------------------
# align
# ---------------------------------------------------------------------------

def umeyama(src: np.ndarray, dst: np.ndarray) -> Tuple[float, np.ndarray, np.ndarray]:
    """Least-squares similarity transform ``dst ~ s * R @ src + t`` (Umeyama 1991).

    ``src`` and ``dst`` are (N, 3) arrays with N >= 3 and non-collinear points.
    Returns ``(s, R, t)``.
    """
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    n = src.shape[0]
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    x = src - mu_src
    y = dst - mu_dst
    var_src = float((x * x).sum() / n)
    cov = y.T @ x / n
    u, sing, vt = np.linalg.svd(cov)
    sign = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0.0:
        sign[2, 2] = -1.0
    rot = u @ sign @ vt
    scale = float(np.trace(np.diag(sing) @ sign) / var_src)
    trans = mu_dst - scale * rot @ mu_src
    return scale, rot, trans


def _estimated_camera_centers(recon) -> dict:
    centers = {}
    for vid in recon.ViewIds():
        view = recon.View(vid)
        if view.IsEstimated():
            centers[view.Name()] = np.asarray(view.Camera().GetPosition(), dtype=np.float64).reshape(3)
    return centers


def cmd_align(args) -> int:
    rec_a = _read_recon(args.recon_a)
    rec_b = _read_recon(args.recon_b)
    centers_a = _estimated_camera_centers(rec_a)
    centers_b = _estimated_camera_centers(rec_b)
    names = sorted(set(centers_a) & set(centers_b))
    if len(names) < 3:
        raise CLIError(
            f"need at least 3 registered views with the same name in both reconstructions, "
            f"found {len(names)}")
    src = np.array([centers_a[n] for n in names])
    dst = np.array([centers_b[n] for n in names])
    try:
        scale, rot, trans = umeyama(src, dst)
    except np.linalg.LinAlgError as exc:
        raise CLIError(f"alignment failed: {exc}") from exc
    aligned = scale * src @ rot.T + trans
    errors = np.linalg.norm(aligned - dst, axis=1)
    result = {
        "common_views": len(names),
        "scale": scale,
        "camera_center_error_mean": float(errors.mean()),
        "camera_center_error_median": float(np.median(errors)),
        "camera_center_error_rmse": float(np.sqrt((errors ** 2).mean())),
        "camera_center_error_max": float(errors.max()),
    }
    if args.json:
        print(json.dumps(result, indent=2))
        return 0
    print(f"Sim(3) alignment of {os.path.basename(args.recon_a)} onto {os.path.basename(args.recon_b)}")
    print(f"common registered views: {result['common_views']}")
    print(f"scale:                   {scale:.6g}")
    print(f"camera center error (units of {os.path.basename(args.recon_b)}):")
    print(f"  mean   {result['camera_center_error_mean']:.6g}")
    print(f"  median {result['camera_center_error_median']:.6g}")
    print(f"  rmse   {result['camera_center_error_rmse']:.6g}")
    print(f"  max    {result['camera_center_error_max']:.6g}")
    return 0


# ---------------------------------------------------------------------------
# sfm
# ---------------------------------------------------------------------------

def _is_primitive(value: Any) -> bool:
    return isinstance(value, (bool, int, float, str)) or value is None


def _enum_members(value: Any):
    """Name -> member dict for pybind11 enums, else None."""
    if _is_primitive(value):
        return None
    return getattr(type(value), "__members__", None)


def _option_names(obj: Any) -> list:
    return [n for n in dir(obj) if not n.startswith("_") and not callable(getattr(obj, n))]


def _leaf_options(obj: Any, prefix: str = "") -> Iterator[Tuple[str, Any]]:
    """Yield (dotted name, value) for every settable leaf option, recursing into groups."""
    for name in _option_names(obj):
        value = getattr(obj, name)
        if _is_primitive(value) or _enum_members(value) is not None:
            yield prefix + name, value
        else:
            yield from _leaf_options(value, prefix + name + ".")


def _coerce(current: Any, value: Any) -> Any:
    """Convert a JSON value or a ``--set`` string to the type of the current option value."""
    members = _enum_members(current)
    if members is not None:
        if isinstance(value, str):
            key = value if value in members else value.upper()
            if key not in members:
                raise ValueError("expected one of " + ", ".join(sorted(members)))
            return members[key]
        if isinstance(value, int) and not isinstance(value, bool):
            for member in members.values():
                if int(member) == value:
                    return member
            raise ValueError("no enum member with value %d" % value)
        raise ValueError("expected an enum name")

    if isinstance(current, bool):
        if isinstance(value, bool):
            return value
        if isinstance(value, str) and value.strip().lower() in _TRUE_WORDS:
            return True
        if isinstance(value, str) and value.strip().lower() in _FALSE_WORDS:
            return False
        if isinstance(value, int) and value in (0, 1):
            return bool(value)
        raise ValueError("expected true or false")

    if isinstance(current, (int, float)):
        if isinstance(value, bool):
            raise ValueError("expected a number")
        if isinstance(value, str):
            value = float(value.strip())
        if not isinstance(value, (int, float)):
            raise ValueError("expected a number")
        if isinstance(current, int):
            if float(value) != int(value):
                raise ValueError("expected an integer")
            return int(value)
        return float(value)

    if isinstance(current, str):
        if not isinstance(value, str):
            raise ValueError("expected a string")
        return value

    # Anything else (e.g. vectors): accept JSON text or an already parsed value.
    if isinstance(value, str):
        return json.loads(value)
    return value


def _apply_settings(obj: Any, settings: dict, prefix: str = "") -> None:
    """Apply a (possibly nested) mapping of option names to values to ``obj``."""
    names = set(_option_names(obj))
    for key, value in settings.items():
        full = prefix + key
        if key not in names:
            raise CLIError(f"unknown option '{full}' for ReconstructionEstimatorOptions", code=2)
        current = getattr(obj, key)
        if isinstance(value, dict):
            if _is_primitive(current) or _enum_members(current) is not None:
                raise CLIError(f"option '{full}' is not a group of options", code=2)
            _apply_settings(current, value, full + ".")
            # Nested options are returned as copies; write the modified group back.
            setattr(obj, key, current)
            continue
        try:
            new_value = _coerce(current, value)
        except (ValueError, json.JSONDecodeError) as exc:
            raise CLIError(f"invalid value for '{full}': {value!r} ({exc})", code=2) from exc
        try:
            setattr(obj, key, new_value)
        except (AttributeError, TypeError) as exc:
            raise CLIError(f"cannot set '{full}': {exc}", code=2) from exc


def _set_pairs_to_dict(pairs: Sequence[str]) -> dict:
    """Turn ``a.b=1`` ``c=x`` into ``{"a": {"b": "1"}, "c": "x"}``."""
    result: dict = {}
    for pair in pairs:
        if "=" not in pair:
            raise CLIError(f"--set expects key=value, got '{pair}'", code=2)
        key, _, value = pair.partition("=")
        key = key.strip()
        if not key or any(not part for part in key.split(".")):
            raise CLIError(f"--set has an empty key in '{pair}'", code=2)
        node = result
        parts = key.split(".")
        for part in parts[:-1]:
            child = node.setdefault(part, {})
            if not isinstance(child, dict):
                raise CLIError(f"conflicting --set keys around '{key}'", code=2)
            node = child
        node[parts[-1]] = value
    return result


def _load_config(path: str) -> dict:
    if path.lower().endswith((".yaml", ".yml")):
        raise CLIError("YAML config files are not supported; use a JSON object", code=2)
    try:
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
    except OSError as exc:
        raise CLIError(f"cannot read config {path}: {exc}", code=2) from exc
    except json.JSONDecodeError as exc:
        raise CLIError(f"config {path} is not valid JSON: {exc}", code=2) from exc
    if not isinstance(data, dict):
        raise CLIError(f"config {path} must contain a JSON object", code=2)
    return data


def build_estimator_options(config_path=None, set_pairs=()):
    """ReconstructionEstimatorOptions from defaults, then a JSON file, then ``--set`` pairs."""
    options = pt.sfm.ReconstructionEstimatorOptions()
    if config_path:
        _apply_settings(options, _load_config(config_path))
    if set_pairs:
        _apply_settings(options, _set_pairs_to_dict(set_pairs))
    return options


def _format_value(value: Any) -> str:
    members = _enum_members(value)
    if members is not None:
        for name, member in members.items():
            if member == value:
                return name
    return repr(value)


def cmd_sfm(args) -> int:
    if args.list_options:
        for name, value in _leaf_options(pt.sfm.ReconstructionEstimatorOptions()):
            print(f"{name} = {_format_value(value)}")
        return 0

    if not args.input:
        raise CLIError("--input DIR is required (see 'pytheia sfm --help')", code=2)
    if not args.out:
        raise CLIError("--out DIR is required", code=2)

    options = build_estimator_options(args.config, args.set or ())

    recon_path = os.path.join(args.input, RECON_FILE)
    graph_path = args.view_graph or os.path.join(args.input, VIEW_GRAPH_FILE)
    recon = _read_recon(recon_path)
    view_graph = pt.sfm.ViewGraph()
    if not view_graph.ReadFromDisk(graph_path):
        raise CLIError(f"could not read view graph: {graph_path}")

    # The tracks and the view graph are the input; poses from the file are re-estimated.
    for vid in recon.ViewIds():
        recon.MutableView(vid).SetIsEstimated(False)
    for tid in recon.TrackIds():
        recon.MutableTrack(tid).SetIsEstimated(False)

    if args.seed is not None:
        pt.solvers.SetDefaultRandomSeed(args.seed)

    estimator_cls = {
        "global": pt.sfm.GlobalReconstructionEstimator,
        "incremental": pt.sfm.IncrementalReconstructionEstimator,
        "hybrid": pt.sfm.HybridReconstructionEstimator,
    }[args.method]
    estimator = estimator_cls(options)

    start = time.time()
    summary = estimator.Estimate(view_graph, recon)
    elapsed = time.time() - start

    os.makedirs(args.out, exist_ok=True)
    success = bool(summary.success)
    report = {
        "method": args.method,
        "seed": args.seed,
        "input": os.path.abspath(args.input),
        "success": success,
        "message": summary.message,
        "num_views": recon.NumViews(),
        "estimated_views": len(summary.estimated_views),
        "num_tracks": recon.NumTracks(),
        "estimated_tracks": len(summary.estimated_tracks),
        "mean_reprojection_error_px": _mean_reprojection_error(recon) if success else None,
        "wall_time_s": round(elapsed, 3),
    }
    if success:
        pt.io.WriteReconstruction(recon, os.path.join(args.out, RECON_FILE))
        pt.io.WritePlyFile(os.path.join(args.out, "reconstruction.ply"), recon,
                           np.array([255, 0, 0], dtype=np.int32), 2)
    with open(os.path.join(args.out, "summary.json"), "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2)

    print(json.dumps(report, indent=2))
    if not success:
        print(f"{PROG} sfm: estimation failed: {summary.message}", file=sys.stderr)
        return 1
    return 0


# ---------------------------------------------------------------------------
# Parser and entry point
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=PROG,
        description="pyTheia command line tools: SfM, reconstruction info, format conversion, "
                    "viewer and exporters.")
    parser.add_argument("--version", action="version", version=f"{PROG} {pt.__version__}")
    sub = parser.add_subparsers(dest="command", metavar="COMMAND")
    sub.required = True

    p = sub.add_parser("info", help="Print views, tracks and mean reprojection error")
    p.add_argument("recon", help="Reconstruction file (.recon)")
    p.add_argument("--json", action="store_true", help="Print JSON")
    p.set_defaults(func=cmd_info)

    p = sub.add_parser("convert", help="Convert a reconstruction to another format")
    p.add_argument("input", help="Input reconstruction (.recon)")
    p.add_argument("output", help="Output directory (colmap, colmap-bin, bundler) or file")
    p.add_argument("--format", required=True, choices=CONVERT_FORMATS)
    p.add_argument("--images", default=None,
                   help="Image directory written into nerfstudio json (required for nerfstudio)")
    p.add_argument("--aabb-scale", type=int, default=16, help="nerfstudio aabb_scale (default 16)")
    p.add_argument("--min-observations", type=int, default=2,
                   help="ply: minimum observations per point (default 2)")
    p.set_defaults(func=cmd_convert)

    p = sub.add_parser("view", help="Write a self-contained HTML viewer (needs pytheia.viewer)")
    p.add_argument("recon", help="Reconstruction file (.recon)")
    p.add_argument("--html", default=None,
                   help="Output HTML file (default: <recon name>.html in the current directory)")
    p.set_defaults(func=cmd_view)

    p = sub.add_parser("export-splat",
                       help="Export undistorted COLMAP-style input for Gaussian splatting "
                            "(needs pytheia.export)")
    p.add_argument("recon", help="Reconstruction file (.recon)")
    p.add_argument("--images", required=True, help="Directory with the original images")
    p.add_argument("--out", required=True, help="Output directory")
    p.add_argument("--no-undistort", action="store_true",
                   help="Keep the original (distorted) images and camera models")
    p.add_argument("--max-image-size", type=int, default=-1,
                   help="Downscale images so the longer side is at most N px (default: no limit)")
    p.set_defaults(func=cmd_export_splat)

    p = sub.add_parser("align",
                       help="Sim(3)-align reconstruction A onto B and report camera-center error")
    p.add_argument("recon_a", help="Estimated reconstruction (.recon), moved onto B")
    p.add_argument("recon_b", help="Reference reconstruction (.recon), e.g. ground truth")
    p.add_argument("--json", action="store_true", help="Print JSON")
    p.set_defaults(func=cmd_align)

    p = sub.add_parser("sfm", help="Run global, incremental or hybrid SfM from tracks and a view graph")
    p.add_argument("--input", default=None,
                   help="Directory with reconstruction.recon (views + tracks with features) and "
                        "view_graph.bin (ViewGraph.WriteToDisk)")
    p.add_argument("--view-graph", default=None,
                   help="Override the view graph file (default: INPUT/view_graph.bin)")
    p.add_argument("--out", default=None, help="Output directory")
    p.add_argument("--method", default="global", choices=SFM_METHODS)
    p.add_argument("--seed", type=int, default=None,
                   help="Fixed RNG seed (pt.solvers.SetDefaultRandomSeed)")
    p.add_argument("--config", default=None,
                   help="JSON object with ReconstructionEstimatorOptions fields (nested groups allowed)")
    p.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                   help="Override one option, e.g. --set max_num_iterations=50 or "
                        "--set ligt_position_estimator_options.use_scale_constraint=false. "
                        "Repeatable; applied after --config.")
    p.add_argument("--list-options", action="store_true",
                   help="List all option names with their default values and exit")
    p.set_defaults(func=cmd_sfm)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the pyTheia CLI. Returns the exit status."""
    parser = build_parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit as exc:
        return exc.code if isinstance(exc.code, int) else 2
    try:
        return int(args.func(args) or 0)
    except CLIError as exc:
        print(f"{PROG} {args.command}: error: {exc}", file=sys.stderr)
        return exc.code


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
