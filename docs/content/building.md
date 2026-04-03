# Building pyTheia Library {#chapter-building}

Source and issues: [pyTheiaSfM on GitHub](https://github.com/urbste/pyTheiaSfM).

## Dependencies {#section-dependencies}

pyTheia / Theia need a **C++17-capable** toolchain, **CMake 3.15+** (see root `CMakeLists.txt`), and:

| Dependency | Role |
|------------|------|
| **[Eigen3](https://eigen.tuxfamily.org/)** | Dense linear algebra |
| **[Ceres Solver](http://ceres-solver.org/)** | Non-linear least squares (bundle adjustment, many pose / alignment solvers) |
| **glog**, **gflags** | Logging and flags (typical Ceres dependencies) |

Install Eigen and Ceres from your distribution or build from source. Follow **[Ceres installation](http://ceres-solver.org/installation.html)** for BLAS/LAPACK or **SuiteSparse** choices; this fork does not require SuiteSparse in pyTheia’s math layer, but Ceres may still use it if you enable it there.

### Ceres version

Use a **current Ceres 2.x** release (build from the latest stable tag in [ceres-solver/ceres-solver](https://github.com/ceres-solver/ceres-solver/releases)). Older 2.1.x builds remain fine for **CPU-only** workflows; **CUDA sparse** (cuDSS) support needs a Ceres version and build that actually compile the **cuDSS** component (see Ceres release notes and installation guide).

### Optional: Ceres with CUDA (dense and sparse solvers)

Ceres can accelerate **bundle adjustment** (and other solves that use Ceres’ linear algebra backends) on NVIDIA GPUs:

1. **CUDA dense** — Ceres built with **CUDA** enabled (`USE_CUDA=ON` in Ceres; see upstream docs). Exposes Ceres’ **`CUDA`** dense linear algebra backend (`DenseLinearAlgebraLibraryType`). Typical pairing: dense Schur-style solvers with `dense_linear_algebra_library_type = ceres::CUDA`.
2. **CUDA sparse** — Ceres built with **NVIDIA cuDSS** so the compiled library includes the **cuDSS** component (see Ceres documentation for cuDSS paths and CMake hints). Exposes **`CUDA_SPARSE`** (`SparseLinearAlgebraLibraryType`) for sparse Schur / sparse normal Cholesky–style solves.

When you **configure pyTheia**, CMake detects what your installed Ceres supports:

- **Dense CUDA:** a small `try_compile` check against Ceres headers (or override with `-DTHEIA_CERES_USE_CUDA=ON|OFF`).
- **Sparse CUDA:** inspection of `CERES_COMPILED_COMPONENTS` for `cuDSS` (or override with `-DTHEIA_CERES_USE_CUDA_SPARSE=ON|OFF`).

Configure log lines look like:

- `Ceres was built with CUDA (dense linear algebra available)` or the fallback message if not.
- `Ceres was built with CUDA sparse (cuDSS); BA can use CUDA_SPARSE` if cuDSS is present.

These flags are passed into the **pybind11** extension so bindings match your Ceres capabilities. At **runtime**, GPU use is still controlled by **`BundleAdjustmentOptions`** (and the same Ceres enums elsewhere), e.g. `dense_linear_algebra_library_type` / `sparse_linear_algebra_library_type` — see [Bundle adjustment](bundle_adjustment.md).

!!! note
    Manylinux **wheels** shipped by the project are built against a **CPU** Ceres in the base image. To use **CUDA** dense or **CUDA_SPARSE** in Python, build pyTheia locally against a Ceres that was compiled with the desired GPU options.

## Building on Linux or WSL2 (on Windows) {#section-building}

Example: Eigen and Ceres from source on Ubuntu (adjust paths and generators as needed).

```bash
sudo apt install cmake build-essential libgflags-dev libgoogle-glog-dev libatlas-base-dev
# optional: NVIDIA CUDA toolkit + cuDSS for Ceres GPU backends (see Ceres docs)

mkdir -p LIBS && cd LIBS

# Eigen (example: 3.4.x)
git clone https://gitlab.com/libeigen/eigen.git
cd eigen && git checkout 3.4.0
mkdir -p build && cd build && cmake .. && sudo cmake --install .

# Ceres — use a current 2.x release; add -DUSE_CUDA=ON and cuDSS hints per Ceres docs for GPU
cd ../../
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver && git fetch --tags && git checkout "$(git tag -l '2.*' | sort -V | tail -1)"
mkdir build && cd build
cmake .. \
  -DBUILD_TESTING=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_BENCHMARKS=OFF
# Example GPU Ceres line (details in Ceres installation guide):
# cmake .. ... -DUSE_CUDA=ON

cmake --build . -j"$(nproc)"
sudo cmake --install .
```

Then build the **Python** package from the pyTheia repo root (see repository `README.md` for `build_and_install.sh` and wheel flows):

```bash
python3 -m pip install -e .   # or: python3 setup.py bdist_wheel && pip install dist/*.whl
```

Manylinux wheels can be built with Docker or scripts under `pypackage/` (see README).

## Building the documentation (MkDocs) {#section-documentation-mkdocs}

The manual uses [MkDocs](https://www.mkdocs.org/) and the [Material theme](https://squidfunk.github.io/mkdocs-material/). From the repository root:

```bash
pip install -r docs/requirements.txt
# or: pip install ".[docs]"
mkdocs serve -f docs/mkdocs.yml
```

Open the URL printed in the terminal (usually [http://127.0.0.1:8000/](http://127.0.0.1:8000/)). Static site:

```bash
mkdocs build --strict -f docs/mkdocs.yml
```

With `-DBUILD_DOCUMENTATION=ON`, CMake can generate the `theia_docs` target (MkDocs; install under `share/doc/theia` when the `Doc` component is enabled).
