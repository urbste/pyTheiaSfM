#!/usr/bin/env python3
# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""Print MGSfM-style KITTI odometry comparison table (markdown + LaTeX)."""

from __future__ import annotations

import argparse
import json
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
if _THIS not in sys.path:
    sys.path.insert(0, _THIS)

from kitti_mgsfm_reference import (  # noqa: E402
    MGSFM_TABLE_REF,
    MgsfmRow,
    REFERENCE_METHOD_ORDER,
    TRAINING_SEQUENCES,
)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Print MGSfM Table 1 comparison")
    p.add_argument(
        "--metrics_json",
        type=str,
        required=True,
        help="JSON list of per-sequence metrics (from kitti_rig_benchmark batch).",
    )
    p.add_argument(
        "--out_tex",
        type=str,
        default="",
        help="Optional path to write LaTeX table.",
    )
    p.add_argument(
        "--method_name",
        type=str,
        default="pyTheia",
        help="Label for the measured method column.",
    )
    return p.parse_args()


def _fmt(v: float) -> str:
    if v >= 100:
        return f"{v:.0e}"
    if abs(v - round(v)) < 1e-6:
        return str(int(round(v)))
    return f"{v:.1f}"


def _row_from_metrics(entry: dict) -> MgsfmRow | None:
    if entry.get("mgsfm_n", 0) <= 0:
        return None
    return MgsfmRow(
        n=int(entry.get("mgsfm_n", entry.get("sequence_n", 0))),
        er_median=float(entry.get("mgsfm_er_median", float("nan"))),
        er_mean=float(entry.get("mgsfm_er_mean", float("nan"))),
        et_median=float(entry.get("mgsfm_et_median", float("nan"))),
        et_mean=float(entry.get("mgsfm_et_mean", float("nan"))),
    )


def _collect_table(
    metrics: list[dict],
    method_name: str,
) -> dict[str, dict[str, MgsfmRow | None]]:
    """method -> seq -> row."""
    out: dict[str, dict[str, MgsfmRow | None]] = {
        m: dict(MGSFM_TABLE_REF[m]) for m in REFERENCE_METHOD_ORDER
    }
    out[method_name] = {}
    for entry in metrics:
        seq = str(entry.get("sequence", "")).zfill(2)
        out[method_name][seq] = _row_from_metrics(entry)
    return out


def _best_values(
    seq: str,
    table: dict[str, dict[str, MgsfmRow | None]],
    fields: list[str],
    methods: list[str],
):
    bests: dict[str, float] = {}
    for field in fields:
        vals = []
        for method in methods:
            row = table.get(method, {}).get(seq)
            if row is None:
                continue
            v = getattr(row, field)
            if v is not None and not (isinstance(v, float) and v != v):
                vals.append(float(v))
        bests[field] = min(vals) if vals else float("nan")
    return bests


def _cell(row: MgsfmRow | None, field: str, best: float) -> str:
    if row is None:
        return "---"
    v = float(getattr(row, field))
    s = _fmt(v)
    if best == v or abs(v - best) < 1e-9:
        return f"**{s}**"
    return s


def print_markdown(
    table: dict[str, dict[str, MgsfmRow | None]],
    method_name: str,
) -> str:
    methods = REFERENCE_METHOD_ORDER + [method_name]
    lines = ["| Seq | N |"]
    for m in methods:
        lines[0] += f" {m} e~r | {m} e~r | {m} e~t | {m} e~t |"
    lines[0] += "\n"
    lines.append("|-----|---|" + "----|----|----|----|" * len(methods) + "\n")

    fields = ["er_median", "er_mean", "et_median", "et_mean"]
    for seq in TRAINING_SEQUENCES:
        bests = _best_values(seq, table, fields, methods)
        n_ref = None
        for m in methods:
            row = table.get(m, {}).get(seq)
            if row is not None:
                n_ref = row.n
                break
        line = f"| {seq} | {n_ref or '---'} |"
        for m in methods:
            row = table.get(m, {}).get(seq)
            for field in fields:
                best = bests.get(field, float("nan"))
                line += f" {_cell(row, field, best)} |"
        lines.append(line + "\n")

    text = "".join(lines)
    print(text)
    return text


def write_latex(
    table: dict[str, dict[str, MgsfmRow | None]],
    method_name: str,
    path: str,
) -> None:
    methods = REFERENCE_METHOD_ORDER + [method_name]
    fields = ["er_median", "er_mean", "et_median", "et_mean"]
    ncol = 2 + len(methods) * 4

    lines = [
        "\\begin{tabular}{" + "l" + "r" + "r" * (ncol - 1) + "}\n",
        "\\hline\n",
    ]
    header = "Seq & N"
    for m in methods:
        header += f" & \\multicolumn{{4}}{{c}}{{{m}}}"
    header += " \\\\\n"
    lines.append(header)
    sub = " & "
    for _ in methods:
        sub += " & $\\tilde e_r$ & $\\bar e_r$ & $\\tilde e_t$ & $\\bar e_t$"
    sub += " \\\\\n\\hline\n"
    lines.append(sub)

    for seq in TRAINING_SEQUENCES:
        bests = _best_values(seq, table, fields, methods)
        n_ref = None
        for m in methods:
            row = table.get(m, {}).get(seq)
            if row is not None:
                n_ref = row.n
                break
        row_line = f"{seq} & {n_ref or '---'}"
        for m in methods:
            row = table.get(m, {}).get(seq)
            for field in fields:
                if row is None:
                    row_line += " & ---"
                else:
                    v = float(getattr(row, field))
                    s = _fmt(v)
                    if bests.get(field) == v:
                        row_line += f" & \\textbf{{{s}}}"
                    else:
                        row_line += f" & {s}"
        row_line += " \\\\\n"
        lines.append(row_line)

    lines.append("\\hline\n\\end{tabular}\n")
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(lines)
    print(f"Wrote LaTeX table to {path}")


def main() -> int:
    args = _parse_args()
    with open(args.metrics_json, encoding="utf-8") as f:
        metrics = json.load(f)
    if not isinstance(metrics, list):
        print("Expected JSON array", file=sys.stderr)
        return 1
    table = _collect_table(metrics, args.method_name)
    print_markdown(table, args.method_name)
    if args.out_tex:
        write_latex(table, args.method_name, os.path.abspath(args.out_tex))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
