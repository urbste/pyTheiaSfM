#!/usr/bin/env bash
set -euo pipefail

# Usage: dev/generate_stubs.sh
# Writes PEP 561 stubs into src/pytheia/ (same layout as setup.py wheel build).

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

if command -v python >/dev/null 2>&1; then
	PY=python
elif command -v python3 >/dev/null 2>&1; then
	PY=python3
else
	echo "No python interpreter found (python/python3)." >&2
	exit 1
fi

if ! "$PY" -c "import pybind11_stubgen" >/dev/null 2>&1; then
	echo "pybind11-stubgen not found. Install it via: $PY -m pip install pybind11-stubgen" >&2
	exit 1
fi

export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

echo "Generating stubs for 'pytheia.pytheia' into src/..."
"$PY" -m pybind11_stubgen pytheia.pytheia -o src

TARGET_PKG="src/pytheia/pytheia"
ALT_STUBS="src/pytheia-stubs"

if [ -d "${ALT_STUBS}" ]; then
	mkdir -p "${TARGET_PKG}"
	rsync -a "${ALT_STUBS}/" "${TARGET_PKG}/" 2>/dev/null || cp -r "${ALT_STUBS}/." "${TARGET_PKG}/"
fi

INIT_PYI="${TARGET_PKG}/__init__.pyi"
FLAT_PYI="src/pytheia/pytheia.pyi"
if [ -f "${INIT_PYI}" ]; then
	cp "${INIT_PYI}" "${FLAT_PYI}"
fi

echo "Stub files updated under src/pytheia (see pytheia.pyi and pytheia/*.pyi)"
