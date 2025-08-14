#!/usr/bin/env bash
set -euo pipefail

# Usage: dev/generate_stubs.sh [DEST_DIR]
# Default destination is ./typings (recommended for editor consumption).
DEST_DIR="${1:-typings}"
mkdir -p "${DEST_DIR}"

# Pick a Python interpreter
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

# Ensure the built extension in src/ is importable
export PYTHONPATH="$(pwd)/src:${PYTHONPATH:-}"

TMP_DIR="${DEST_DIR}/.gen"
rm -rf "${TMP_DIR}"
mkdir -p "${TMP_DIR}"

echo "Generating stubs for 'pytheia' into temporary dir '${TMP_DIR}'..."
"$PY" -m pybind11_stubgen pytheia -o "${TMP_DIR}"

# pybind11-stubgen typically outputs into <out>/pytheia-stubs/pytheia
if [ -d "${TMP_DIR}/pytheia-stubs/pytheia" ]; then
	mkdir -p "${DEST_DIR}/pytheia"
	rsync -a --delete "${TMP_DIR}/pytheia-stubs/pytheia/" "${DEST_DIR}/pytheia/" || cp -r "${TMP_DIR}/pytheia-stubs/pytheia/." "${DEST_DIR}/pytheia/"
else
	# Fallback: copy any .pyi files under TMP_DIR
	find "${TMP_DIR}" -name '*.pyi' -exec bash -c '
		for f; do
			rel="${f#${TMP_DIR}/}"
			dir="${DEST_DIR}/$(dirname "$rel")"
			mkdir -p "$dir"
			cp "$f" "$dir/"
		done
	' bash {} +
fi

# Create PEP 561 marker in stub folder for editors
: > "${DEST_DIR}/pytheia/py.typed"

rm -rf "${TMP_DIR}"
echo "Stubs ready in '${DEST_DIR}/pytheia'. Ensure your editor uses this path."