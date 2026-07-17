#!/usr/bin/env bash
# Capture pybind11 binding baseline metrics before nanobind migration.
# Usage: ./dev/capture_binding_baseline.sh [output_file]
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

OUTPUT="${1:-docs/plans/baseline-pybind11-$(date +%Y-%m-%d).txt}"
PYTHON="${PYTHON:-python3.12}"
export PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:$PYTHONPATH}"

mkdir -p "$(dirname "$OUTPUT")"

{
  echo "pyTheia binding baseline (pybind11)"
  echo "Date: $(date -Iseconds)"
  echo "Python: $($PYTHON --version 2>&1)"
  echo "Host: $(uname -a)"
  echo ""

  echo "=== Clean rebuild: pytheia extension ==="
  rm -f cmake_build/src/pytheia/CMakeFiles/pytheia.dir/*.o 2>/dev/null || true
  /usr/bin/time -f "elapsed_sec %e" make -C cmake_build -j"$(nproc)" pytheia 2>&1 || true
  echo ""

  SO_FILE="$(ls -1 src/pytheia/pytheia*.so 2>/dev/null | head -1)"
  if [[ -n "$SO_FILE" ]]; then
    echo "=== Extension binary ==="
    ls -lh "$SO_FILE"
    echo "bytes: $(stat -c%s "$SO_FILE")"
    echo ""
  fi

  echo "=== pytest pytests/ ==="
  /usr/bin/time -f "elapsed_sec %e" "$PYTHON" -m pytest pytests/ -q 2>&1 || true
  echo ""

  echo "=== Import smoke ==="
  "$PYTHON" -c "import pytheia as pt; print('submodules:', [s for s in ('io','matching','math','mvs','sfm','solvers') if hasattr(pt,s)])"
} | tee "$OUTPUT"

echo "Baseline written to $OUTPUT"
