#!/usr/bin/env bash
# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
#
# Batch KITTI color odometry benchmark for pyTheia stereo rig pipeline.
#
# Examples:
#   ./pyexamples/stereo/run_kitti_rig_benchmark_all.sh
#   RUN_MODE=match_only ./pyexamples/stereo/run_kitti_rig_benchmark_all.sh
#   RUN_MODE=recon EXTRA_ARGS="--ba_loss cauchy" ./pyexamples/stereo/run_kitti_rig_benchmark_all.sh
#   SEQUENCES=04,05 MAX_FRAMES=200 ./pyexamples/stereo/run_kitti_rig_benchmark_all.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# stereo/ -> pyexamples/ -> repo root (pyTheiaSfM)
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

KITTI_DATA="${KITTI_DATA:-/media/steffen/Data2/kitti}"
COLOR_ROOT="${COLOR_ROOT:-${KITTI_DATA}/data_odometry_color/dataset}"
POSES_SRC="${POSES_SRC:-${KITTI_DATA}/data_odometry_poses/dataset/poses}"

if [[ ! -d "${COLOR_ROOT}/sequences" ]]; then
  echo "Missing color dataset sequences under ${COLOR_ROOT}/sequences" >&2
  exit 1
fi

if [[ ! -e "${COLOR_ROOT}/poses" ]]; then
  ln -sfn "${POSES_SRC}" "${COLOR_ROOT}/poses"
  echo "Linked poses: ${COLOR_ROOT}/poses -> ${POSES_SRC}"
fi

KITTI_ROOT="${KITTI_ROOT:-${COLOR_ROOT}}"
CAMERAS="${CAMERAS:-color}"
MATCHER="${MATCHER:-disk-lightglue}"
RESIZE="${RESIZE:-960}"
MAX_KEYPOINTS="${MAX_KEYPOINTS:-2048}"
CACHE_ROOT="${CACHE_ROOT:-${KITTI_ROOT}/.pytheia_cache}"
CACHE_TAG="${MATCHER}_w${RESIZE}_k${MAX_KEYPOINTS}"
FEATURE_CACHE_DIR="${FEATURE_CACHE_DIR:-${CACHE_ROOT}/features/${CACHE_TAG}}"
MATCH_CACHE_DIR="${MATCH_CACHE_DIR:-${CACHE_ROOT}/matches/${CACHE_TAG}}"
OUT_ROOT="${OUT_ROOT:-${KITTI_ROOT}/pytheia_rig}"
RUN_MODE="${RUN_MODE:-full}"
EXTRA_ARGS="${EXTRA_ARGS:-}"
PYTHON="${PYTHON:-python3}"
MAX_FRAMES="${MAX_FRAMES:-0}"
STRIDE="${STRIDE:-1}"
RECON_STRIDE="${RECON_STRIDE:-1}"

LOOP_SEQS="00 02 05 06 07 08 09"

if [[ -z "${SEQUENCES:-}" ]]; then
  SEQUENCES="$(ls -1 "${KITTI_ROOT}/sequences" | sort | tr '\n' ',' | sed 's/,$//')"
fi

mkdir -p "${OUT_ROOT}/logs" "${FEATURE_CACHE_DIR}" "${MATCH_CACHE_DIR}"

normalize_seq() {
  local raw="${1// /}"
  if [[ -z "${raw}" ]]; then
    return 1
  fi
  # Force decimal so 08/09 are not parsed as invalid octal.
  printf '%02d' "$((10#${raw}))"
}

BENCHMARK="${SCRIPT_DIR}/kitti_rig_benchmark.py"
TABLE_SCRIPT="${SCRIPT_DIR}/print_kitti_mgsfm_table.py"
if [[ ! -f "${BENCHMARK}" ]]; then
  echo "Missing benchmark script: ${BENCHMARK}" >&2
  exit 1
fi
MGSFM_JSON="${OUT_ROOT}/mgsfm_metrics.json"

run_one() {
  local seq="$1"
  local extra=()
  if [[ " ${LOOP_SEQS} " == *" ${seq} "* ]]; then
    extra+=(--trajectory_has_loops)
  fi
  if [[ "${RUN_MODE}" == "match_only" ]]; then
    extra+=(--match_cache_only)
  fi

  local log="${OUT_ROOT}/logs/${seq}.log"
  echo "=== Sequence ${seq} (RUN_MODE=${RUN_MODE}) ==="

  set +e
  "${PYTHON}" "${BENCHMARK}" \
    --kitti_root "${KITTI_ROOT}" \
    --sequences "${seq}" \
    --cameras "${CAMERAS}" \
    --matcher "${MATCHER}" \
    --resize "${RESIZE}" \
    --max_keypoints "${MAX_KEYPOINTS}" \
    --stride "${STRIDE}" \
    --recon_stride "${RECON_STRIDE}" \
    --max_frames "${MAX_FRAMES}" \
    --feature_cache_dir "${FEATURE_CACHE_DIR}" \
    --match_cache_dir "${MATCH_CACHE_DIR}" \
    --out_dir "${OUT_ROOT}/${seq}" \
    --out_json "${OUT_ROOT}/summary_${seq}.json" \
    --write_kitti_poses \
    "${extra[@]}" \
    ${EXTRA_ARGS} \
    2>&1 | tee "${log}"
  local rc=${PIPESTATUS[0]}
  set -e
  return "${rc}"
}

IFS=',' read -ra SEQ_ARR <<< "${SEQUENCES}"
failures=0
for seq in "${SEQ_ARR[@]}"; do
  if [[ -z "${seq// /}" ]]; then
    continue
  fi
  seq="$(normalize_seq "${seq}")"
  img_dir="${KITTI_ROOT}/sequences/${seq}/image_2"
  if [[ ! -d "${img_dir}" ]]; then
    echo "Skip ${seq}: no image_2 under ${KITTI_ROOT}/sequences/${seq}" >&2
    continue
  fi
  if ! run_one "${seq}"; then
    failures=$((failures + 1))
  fi
done

# Aggregate MGSfM metrics from per-sequence summaries
"${PYTHON}" - <<PY
import glob, json, os, subprocess, sys

out_root = "${OUT_ROOT}"
mgsfm_json = "${MGSFM_JSON}"
table_script = "${TABLE_SCRIPT}"
sequences = "${SEQUENCES}".split(",")

rows = []
for raw in sequences:
    seq = raw.strip()
    if not seq:
        continue
    seq = f"{int(seq):02d}"
    path = os.path.join(out_root, f"summary_{seq}.json")
    if not os.path.isfile(path):
        continue
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    if isinstance(data, list) and data:
        rows.append(data[0])
    elif isinstance(data, dict):
        rows.append(data)

if rows:
    with open(mgsfm_json, "w", encoding="utf-8") as f:
        json.dump(rows, f, indent=2)
    print(f"Wrote {mgsfm_json} ({len(rows)} sequences)")
    subprocess.run(
        [sys.executable, table_script, "--metrics_json", mgsfm_json,
         "--out_tex", os.path.join(out_root, "mgsfm_table.tex")],
        check=False,
    )
PY

if [[ "${failures}" -gt 0 ]]; then
  echo "${failures} sequence(s) failed — see ${OUT_ROOT}/logs/" >&2
  exit 2
fi

echo "Done. Results under ${OUT_ROOT}"
