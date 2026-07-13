#!/usr/bin/env bash
# Build and benchmark pybind11 (HEAD sources) vs nanobind (working tree).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PY="${PYTHON:-/home/steffen/anaconda3/envs/taawn_algo_cu13_pt312/bin/python}"
PYINC="$("$PY" -c "import sysconfig; print(sysconfig.get_path('include'))")"
OUT="${1:-$REPO/docs/plans/benchmark-binding-backends-$(date +%Y-%m-%d).txt}"
BENCH="$REPO/dev/benchmarks/binding_call_overhead.py"
BUILD="$REPO/cmake_build"
ITERATIONS="${ITERATIONS:-20000}"

mkdir -p "$(dirname "$OUT")"
export PYTHONPATH="$REPO/src"

run_bench() {
  local label="$1"
  local extra_flags="${2:-}"
  echo ""
  echo "========== $label =========="
  "$PY" "$BENCH" --iterations "$ITERATIONS" --label "$label" $extra_flags
}

{
  echo "pyTheia binding runtime benchmark"
  echo "Date: $(date -Iseconds)"
  echo "Host: $(uname -a)"
  echo "Python: $($PY --version 2>&1)"
  echo "Iterations (default workload): $ITERATIONS"
  echo ""

  # --- nanobind (current tree) ---
  echo "=== Building nanobind extension ==="
  cmake -S "$REPO" -B "$BUILD" \
    -DPYTHON_BUILD=ON \
    -DPYTHEIA_USE_NANOBIND=ON \
    -DPYTHEIA_NANOBIND_SPIKE=OFF \
    -DPYTHON_EXECUTABLE="$PY" \
    -DPYTHON_INCLUDE_DIR="$PYINC" >/dev/null
  cmake --build "$BUILD" --target pytheia -j"$(nproc)" >/dev/null
  cp "$BUILD/lib/pytheia"*.so "$REPO/src/pytheia/"
  NB_SIZE=$(stat -c%s "$REPO/src/pytheia"/pytheia*.so | head -1)
  echo "nanobind .so bytes: $NB_SIZE"
  run_bench "nanobind" ""

  # --- pybind11 (HEAD sources, stashed working tree) ---
  echo ""
  echo "=== Building pybind11 extension (git HEAD sources) ==="
  STASHED=0
  if ! git -C "$REPO" diff --quiet HEAD -- src/pytheia libraries/CMakeLists.txt src/theia/mvs/view_selection_mvsnet.cc 2>/dev/null; then
    git -C "$REPO" stash push -q -m "nanobind-bench-temp" -- \
      src/pytheia libraries/CMakeLists.txt src/theia/mvs/view_selection_mvsnet.cc pytests/sfm/random_recon_gen.py || true
    STASHED=1
  fi

  cmake -S "$REPO" -B "$BUILD" \
    -DPYTHON_BUILD=ON \
    -DPYTHEIA_USE_NANOBIND=OFF \
    -DPYTHEIA_NANOBIND_SPIKE=OFF \
    -DPYTHON_EXECUTABLE="$PY" \
    -DPYTHON_INCLUDE_DIR="$PYINC" >/dev/null
  cmake --build "$BUILD" --target pytheia -j"$(nproc)" >/dev/null
  cp "$BUILD/lib/pytheia"*.so "$REPO/src/pytheia/"
  PB_SIZE=$(stat -c%s "$REPO/src/pytheia"/pytheia*.so | head -1)
  echo "pybind11 .so bytes: $PB_SIZE"
  run_bench "pybind11" "--skip-mvs"

  if [[ "$STASHED" -eq 1 ]]; then
    git -C "$REPO" stash pop -q || git -C "$REPO" stash pop -q --index || true
    cmake -S "$REPO" -B "$BUILD" \
      -DPYTHON_BUILD=ON \
      -DPYTHEIA_USE_NANOBIND=ON \
      -DPYTHEIA_NANOBIND_SPIKE=OFF \
      -DPYTHON_EXECUTABLE="$PY" \
      -DPYTHON_INCLUDE_DIR="$PYINC" >/dev/null
    cmake --build "$BUILD" --target pytheia -j"$(nproc)" >/dev/null
    cp "$BUILD/lib/pytheia"*.so "$REPO/src/pytheia/"
  fi

  echo ""
  echo "=== Summary ==="
  echo "pybind11 .so: $PB_SIZE bytes"
  echo "nanobind .so: $NB_SIZE bytes"
  if [[ "$PB_SIZE" -gt 0 ]]; then
    python3 - <<EOF
pb, nb = $PB_SIZE, $NB_SIZE
print(f"size ratio nanobind/pybind11: {nb/pb:.3f} ({(1-nb/pb)*100:.1f}% smaller)" if pb else "")
EOF
  fi
} | tee "$OUT"

echo "Wrote $OUT"
