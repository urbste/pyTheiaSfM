#!/usr/bin/env python3
"""Mechanical pybind11 -> nanobind substitutions for pytheia binding sources."""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "src" / "pytheia"

SUBS = [
    (r"#include \"\.\./pytheia_pybind\.h\"", '#include "../pytheia_nanobind.h"'),
    (r"#include \"pytheia/pytheia_pybind\.h\"", '#include "pytheia/pytheia_nanobind.h"'),
    (r"namespace py = pybind11;", "namespace nb = nanobind;"),
    (r"\bpy::", "nb::"),
    (r"\bPYBIND11_MODULE\b", "NB_MODULE"),
    (r"\.def_readwrite\(", ".def_rw("),
    (r"\.def_readonly\(", ".def_ro("),
    (r"\.def_property_readonly\(", ".def_prop_ro("),
    (r"\.def_property\(", ".def_prop_rw("),
    (r"nb::return_value_policy::reference_internal", "nb::rv_policy::reference_internal"),
    (r"<pybind11/", "<nanobind/"),
    (r"#include <pybind11/", "#include <nanobind/"),
]

SHARED_PTR_CLASS = re.compile(
    r"nb::class_<([^,>]+),\s*std::shared_ptr<[^>]+>>([^;{]*)",
    re.MULTILINE,
)


def convert_text(text: str) -> str:
    for pat, repl in SUBS:
        text = re.sub(pat, repl, text)
    text = SHARED_PTR_CLASS.sub(r"nb::class_<\1>\2", text)
    text = text.replace("py::module&", "nb::module_&")
    text = text.replace("nb::module&", "nb::module_&")
    text = text.replace("nb::module ", "nb::module_ ")
    return text


def main() -> int:
    patterns = ["**/*.cc", "**/*.h"]
    files: list[Path] = []
    for pat in patterns:
        files.extend(ROOT.glob(pat))
    files = sorted({f for f in files if "spike" not in str(f) and f.name != "pytheia_pybind.h"})

    for path in files:
        if path.name == "pytheia_nanobind.h":
            continue
        original = path.read_text(encoding="utf-8")
        if "pybind11" not in original and "py::" not in original and "PYBIND11" not in original:
            continue
        converted = convert_text(original)
        if converted != original:
            path.write_text(converted, encoding="utf-8")
            print("converted", path.relative_to(ROOT.parent.parent))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
