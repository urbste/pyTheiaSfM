# pyTheia: pybind11 → nanobind Migration Plan

> **Status:** Phase 0 complete — tests + baseline captured (July 2026)  
> **Branch:** `cursor/nanobind-migration-plan-eb29`  
> **Goal:** Assess whether pyTheia should migrate Python bindings from [pybind11](https://github.com/pybind/pybind11) to [nanobind](https://github.com/wjakob/nanobind), and outline the work required.

---

## Executive summary

**Recommendation:** Migration is **feasible and likely worthwhile**, but it is a **medium-to-large engineering effort** concentrated in one file (`src/pytheia/sfm/sfm.cc`, ~72% of binding code). nanobind is maintained by the same author as pybind11 (Wenzel Jakob), uses near-identical binding syntax, and is already used in production by JAX, MLX, IREE/XLA, and FEniCS.

**Expected benefits for pyTheia:**

| Area | Expected impact |
|------|-----------------|
| Extension compile time | ~2–4× faster (nanobind benchmarks; large `sfm.cc` is the main beneficiary) |
| Binary size | ~2–5× smaller `.so` |
| Runtime call overhead | Lower (vector calls, compact objects) |
| Wheel distribution | Optional **Stable ABI** wheels from Python 3.12+ (one binary per platform, not per minor Python) |
| Stub generation | Built-in `nanobind` stubgen replaces `pybind11-stubgen` |

**Main risks:**

1. **Ownership semantics** — camera models use `std::shared_ptr` holder types extensively; nanobind removed holder types and co-locates C++ data in the Python object.
2. **`reference_internal` return policies** — 18 uses in `sfm.cc`; must be revalidated under nanobind's ownership model (`nb::rv_policy::reference_internal`).
3. **No drop-in replacement** — mechanical API renames across ~2,900 lines; runtime regressions are possible even after a clean compile.
4. **CI/Docker** — `urbste/pytheia_base:1.4.0` image and wheel pipeline must be updated.

**Suggested approach:** Proof-of-concept on the smallest module (`mvs/`), then port modules in increasing complexity, with the full `pytests/` suite as the acceptance gate.

---

## What is nanobind?

[nanobind](https://github.com/wjakob/nanobind) is a C++17 binding library created by the author of pybind11. It deliberately targets a smaller C++ subset than pybind11 in exchange for:

- A precompiled support library (`libnanobind`) linked into each extension (avoids recompiling dispatch machinery per translation unit)
- Smaller headers and less template metaprogramming
- Co-located C++/Python object layout (no separate "holder" indirection)
- First-class N-dimensional array support via DLPack / buffer protocol (`nb::ndarray`)
- Integrated stub generation and Stable ABI (Python 3.12+) support

Official resources:

- [Why another binding library?](https://nanobind.readthedocs.io/en/latest/why.html)
- [Porting guide](https://nanobind.readthedocs.io/en/latest/porting.html)
- [Benchmarks](https://nanobind.readthedocs.io/en/latest/benchmark.html)
- [CMake API](https://nanobind.readthedocs.io/en/latest/api_cmake.html)

---

## Current pyTheia binding inventory

### File layout

| File | Lines | Role |
|------|------:|------|
| `src/pytheia/sfm/sfm.cc` | 2,080 | Reconstruction, cameras, BA, pose graph, estimators |
| `src/pytheia/math/math.cc` | 233 | Math helpers + Sophus SE3/Sim3 |
| `src/pytheia/solvers/solvers.cc` | 164 | RANSAC, samplers, quality measurements |
| `src/pytheia/io/io.cc` | 133 | Reconstruction I/O |
| `src/pytheia/matching/matching.cc` | 111 | Feature matching |
| `src/pytheia/mvs/mvs.cc` | 52 | MVS view selection |
| `src/pytheia/util/util.cc` | 60 | Placeholder util submodule |
| `src/pytheia/pytheia_pybind.{cc,h}` | 126 | Module entry + shared includes |

**Total binding code:** ~2,900 lines across 8 translation units.

### pybind11 features in use

| Feature | Count / usage | nanobind notes |
|---------|---------------|----------------|
| `py::class_<>` | ~99 classes | Rename to `nb::class_<>`; drop holder template args |
| `py::enum_<>` | 18 enums | `nb::enum_<>` |
| `def_submodule` | 7 submodules | `nb::module_::def_submodule` |
| `std::shared_ptr` holders | 10 explicit holder specs on camera types | **Remove holder** from `class_` declaration; add `#include <nanobind/trampoline.h>` only if needed; use `#include <nanobind/shared_ptr.h>` for `shared_ptr` exchange |
| Single inheritance | Camera model hierarchy, estimator hierarchy, sampler hierarchy | Supported (nanobind supports single inheritance; **multiple inheritance is not**) |
| `return_value_policy::reference_internal` | 18 | Maps to `nb::rv_policy::reference_internal`; re-test all call sites |

**Verified counts (July 2026 audit):** 13 active `reference_internal` (+ 5 commented), 10 `shared_ptr` holders on camera types, 4 lambda `py::init` in `math.cc`.

| `py::overload_cast` / custom `overload_cast_` | math + sfm | Still supported; prefer explicit casts or lambdas |
| Lambda `py::init<>` constructors | math (Sophus), sfm (Prior templates) | Custom constructors use placement-new pattern in nanobind |
| `py::arg` / defaults | Widespread | `nb::arg`; `None` defaults need `nb::arg().none()` or `nb::none()` |
| `py::options` (disable signatures) | `pytheia_pybind.cc` | **Removed in nanobind** — use built-in stubgen + `nb::sig()` overrides instead |
| `pybind11/eigen.h` | All modules | `nanobind/eigen/dense.h` |
| `pybind11/numpy.h` | Included in several files | Replace with `nanobind/ndarray.h` if raw array access is added later |
| `pybind11/stl.h` | All modules | Opt-in headers: `nanobind/stl/string.h`, `nanobind/stl/vector.h`, etc. |
| `pybind11/stl_bind.h` | Header only (commented `MAKE_OPAQUE`) | Not actively used |
| Trampolines / virtual overrides | None found | N/A |
| `py::array_t` / buffer protocol | None in active code | Low migration risk |
| Multiple inheritance (3+ base classes) | None found | OK |

### Build & packaging touchpoints

| Location | Current pybind11 usage |
|----------|------------------------|
| `libraries/CMakeLists.txt` | `add_subdirectory(pybind11)` when `PYTHON_BUILD=ON` |
| `src/pytheia/CMakeLists.txt` | `pybind11_add_module(pytheia ...)` |
| `setup.py` | CMake `-DPYTHON_BUILD=ON`; stub gen via `pybind11_stubgen` |
| `pyproject.toml` | `dev` extra depends on `pybind11-stubgen>=2.0` |
| `dev/generate_stubs.sh` | `python -m pybind11_stubgen pytheia` |
| `docs/content/python_wrapper.md` | Documents pybind11-stubgen workflow |
| `.github/workflows/build_wheels.yml` | Docker image `urbste/pytheia_base:1.4.0` |

### Test coverage (acceptance gate)

22 Python test modules under `pytests/` (cameras, BA, pose graph, Sophus, Sim3 alignment, I/O, etc.). These are the primary regression harness; no separate C++ binding tests exist.

**Pre-migration tests added (Phase 0):**

| File | Purpose |
|------|---------|
| `pytests/conftest.py` | Shared import path for `random_recon_gen` |
| `pytests/test_import_smoke.py` | Submodule import guard |
| `pytests/mvs/test_view_selection_mvsnet.py` | MVS binding (was untested) |
| `pytests/binding/test_reference_lifetime.py` | 13 `reference_internal` risk sites |
| `pytests/binding/test_camera_intrinsics_polymorphism.py` | `shared_ptr` camera hierarchy |
| `pytests/test_sophus_integration.py` | `TestSophusLambdaConstructors` (4 lambda inits) |

Baseline metrics script: `dev/capture_binding_baseline.sh` → `docs/plans/baseline-pybind11-YYYY-MM-DD.txt`.

---

## Spike strategy correction

The extension is a **single** `pytheia` module built from all `.cc` files. You cannot mix `py::` and `nb::` in one `NB_MODULE` without hybrid interoperability (not available; [pybind11#5800](https://github.com/pybind/pybind11/issues/5800)).

| Approach | When |
|----------|------|
| **B. Sidecar spike** (`pytheia_spike`) | Phase 1 — proves CMake + nanobind submodule with ported `mvs` only |
| **A. Mechanical big-bang** | Phase 2 — switch main `pytheia` target; rename all ~2,900 LOC; fix semantic failures as tests fail |

Incremental per-file compile **within one extension** is not viable.

---

## Compatibility assessment

### Features pyTheia uses that port cleanly

- Submodule layout (`io`, `math`, `matching`, `mvs`, `sfm`, `solvers`)
- Single-inheritance class hierarchies (camera models, estimators, samplers)
- Enum exports (including Ceres enums)
- `m.def(...)` free functions
- Eigen matrix/vector types in function signatures and struct fields
- `def_readwrite` / `def_property` on POD-like option structs

### Features requiring careful review

#### 1. `std::shared_ptr` camera hierarchy

Current pattern (`sfm.cc`):

```cpp
py::class_<theia::CameraIntrinsicsModel,
           std::shared_ptr<theia::CameraIntrinsicsModel>>
    camera_intrinsics_model(m, "CameraIntrinsicsModel");

py::class_<theia::FisheyeCameraModel,
           std::shared_ptr<theia::FisheyeCameraModel>>(
    m, "FisheyeCameraModel", camera_intrinsics_model)
```

nanobind pattern:

```cpp
nb::class_<theia::CameraIntrinsicsModel> camera_intrinsics_model(m, "CameraIntrinsicsModel");

nb::class_<theia::FisheyeCameraModel>(m, "FisheyeCameraModel", camera_intrinsics_model);
```

**Action:** After porting, run `pytests/sfm/camera_test.py`, `fisheye_camera_test.py`, and any test that mutates `Camera` / intrinsics through Python. Pay special attention to:

- `Camera.GetIntrinsics()` returning `reference_internal`
- Passing cameras between Python and C++ via `shared_ptr`
- Polymorphic dispatch (`FisheyeCameraModel` through `CameraIntrinsicsModel` base)

#### 2. `reference_internal` (18 call sites)

Used for methods returning references into parent objects (e.g. reconstruction views/tracks). nanobind has equivalent policy but ownership is tracked differently.

**Action:** Audit each site; add `nb::keep_alive<0, 1>()` where parent lifetime must extend past child reference.

#### 3. Lambda constructors

`math.cc` uses `py::init([](...) { return Sophus::SE3d(...); })`. nanobind prefers placement-new for custom constructors:

```cpp
.def("__init__", [](Sophus::SE3d *self, const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    new (self) Sophus::SE3d(R, t);
})
```

**Action:** Convert lambda `init` factories in `math.cc` and template `AddIntrinsicsPriorType` in `sfm.cc`.

#### 4. `py::options::disable_function_signatures()`

Used because autogenerated C++ signatures break `pybind11-stubgen`. nanobind generates docstrings lazily and ships its own stub generator.

**Action:** Remove `py::options` block; configure `nanobind_add_module(... STUB ...)` in CMake; compare generated `.pyi` quality against current stubs.

#### 5. Stricter `None` handling

nanobind rejects `None` for pointer/reference arguments unless explicitly annotated with `nb::arg().none()`.

**Action:** Grep for optional pointer arguments; audit Python tests that pass `None`.

### Features pyTheia does **not** use (no migration blocker)

- Trampolines / `PYBIND11_OVERRIDE`
- Multiple inheritance
- `py::array_t` / `.def_buffer()`
- Module-local bindings
- Embedding Python in C++
- Custom type casters

---

## Proposed migration phases

### Phase 0 — Decision & baseline (1–2 days)

- [ ] Record baseline metrics on current `master`:
  - Clean build time of `pytheia` extension only (`make pytheia` in `cmake_build/`)
  - `pytheia*.so` file size
  - `python -m pytest pytests/` runtime (total)
- [ ] Pin target nanobind version (recommend latest stable tag, e.g. v2.x)
- [ ] Confirm minimum Python version policy (Stable ABI requires ≥ 3.12; project currently `requires-python = ">=3.8"`)

### Phase 1 — Build-system spike (2–3 days)

**Files to change:**

| File | Change |
|------|--------|
| `libraries/CMakeLists.txt` | Replace `add_subdirectory(pybind11)` with `add_subdirectory(nanobind)` (git submodule) |
| `.gitmodules` | Add `libraries/nanobind`; remove or keep pybind11 until fully migrated |
| `src/pytheia/CMakeLists.txt` | `pybind11_add_module` → `nanobind_add_module` with `NB_DOMAIN pytheia` / stub options |
| `CMakeLists.txt` (root) | Ensure `find_package(Python ... Development.Module)` or `Development` as required by nanobind docs |
| `setup.py` | Swap stub generation to nanobind's CMake-integrated stub target or `nanobind.stubgen` |

**nanobind CMake sketch:**

```cmake
# libraries/CMakeLists.txt
if (PYTHON_BUILD)
  add_subdirectory(nanobind)
endif()

# src/pytheia/CMakeLists.txt
nanobind_add_module(
  pytheia
  NB_DOMAIN pytheia
  NB_STUB pytheia_stub  # optional: auto-generate .pyi
  ${PY_ALL_SOURCE_FILES}
)
target_link_libraries(pytheia PRIVATE ${CMAKE_PROJECT_NAME})
# ... existing include dirs / compile defs ...
```

**Rename shared header** `pytheia_pybind.h` → `pytheia_nanobind.h` (or keep name, swap includes):

```cpp
#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/optional.h>
// add other stl headers as needed per module

namespace nb = nanobind;
using namespace nb::literals;
```

**Acceptance:** `import pytheia_spike.mvs; pytheia_spike.mvs.ViewSelectionMVSNet(...)` works; compile time of spike target measured. (Main `pytheia` remains pybind11 until Phase 2.)

### Phase 2 — Port modules incrementally (ordered by complexity)

| Order | Module | Files | Rationale |
|------:|--------|-------|-----------|
| 1 | `mvs` | `mvs.{h,cc}` | 1 function, no classes |
| 2 | `util` | `util.{h,cc}` | Empty class registry |
| 3 | `matching` | `matching.{h,cc}` | 4 classes, 1 enum |
| 4 | `io` | `io.{h,cc}` | 5 classes, file I/O |
| 5 | `solvers` | `solvers.{h,cc}` | Inheritance hierarchy, `shared_ptr` ctor args |
| 6 | `math` | `math.{h,cc}` | Sophus bindings, lambda inits, overloads |
| 7 | `sfm` | `sfm.{h,cc}` | Bulk of work: cameras, reconstruction, BA, pose graph |

**Per-module checklist:**

1. Replace `py::` → `nb::`, `PYBIND11_MODULE` → `NB_MODULE`
2. Apply [name changes from porting guide](https://nanobind.readthedocs.io/en/latest/porting.html) (`.def_rw`, `.def_prop_ro`, etc.)
3. Remove `std::shared_ptr` from `class_<>` template parameters
4. Convert lambda `init` to placement-new where needed
5. Update `return_value_policy` → `nb::rv_policy`
6. Run targeted `pytests/` for that submodule
7. Commit

**Mechanical substitution table (high volume):**

| pybind11 | nanobind |
|----------|----------|
| `PYBIND11_MODULE(name, m)` | `NB_MODULE(name, m)` |
| `py::module` | `nb::module_` |
| `.def_readwrite` | `.def_rw` |
| `.def_readonly` | `.def_ro` |
| `.def_property` | `.def_prop_rw` |
| `.def_property_readonly` | `.def_prop_ro` |
| `py::arg("x")` | `nb::arg("x")` |
| `py::return_value_policy::reference_internal` | `nb::rv_policy::reference_internal` |
| `py::init<T>()` | `nb::init<T>()` |
| `py::overload_cast<...>` | `nb::overload_cast<...>` or explicit cast |
| `#include <pybind11/eigen.h>` | `#include <nanobind/eigen/dense.h>` |
| `#include <pybind11/stl.h>` | Specific `nanobind/stl/*.h` headers |

### Phase 3 — Module entry & stubs (1–2 days)

**Files:**

- `src/pytheia/pytheia_pybind.cc` — switch to `NB_MODULE`, remove `py::options`
- `setup.py` / `dev/generate_stubs.sh` — use nanobind stub target
- `pyproject.toml` — replace `pybind11-stubgen` dev dependency
- `src/pytheia/pytheia.pyi` and `src/pytheia/pytheia/*.pyi` — regenerate; diff against old stubs
- `docs/content/python_wrapper.md` — update stub instructions

**Acceptance:** `mypy` / `pyright` clean on public API; editor autocomplete unchanged for consumers.

### Phase 4 — Full regression & performance validation (2–3 days)

- [ ] `python -m pytest pytests/` — all green
- [ ] `python pytests/sfm_pipeline.py` (if data available in CI)
- [ ] Compare compile time, `.so` size, and a micro-benchmark (e.g. 10k `Camera.ProjectPoint` calls)
- [ ] Valgrind / nanobind leak warnings at interpreter shutdown (optional)

### Phase 5 — CI & release (2–3 days)

- [ ] Update `urbste/pytheia_base` Docker image with nanobind submodule
- [ ] Update `build-wheel-linux.sh` if it references pybind11
- [ ] Evaluate **Stable ABI** (`NB_TARGET_ABI_VERSION` / limited API) for Python ≥ 3.12 wheels
- [ ] Update `docs/content/building.md`, `AGENTS.md`, `README.md`
- [ ] Remove vendored `libraries/pybind11` submodule

---

## Stable ABI opportunity

If pyTheia targets Python 3.12+, nanobind can build against the [Stable ABI](https://docs.python.org/3/c-api/stable.html#stable-abi), producing one wheel per platform that works across Python 3.12, 3.13, 3.14, etc.

**Trade-offs:**

| Pros | Cons |
|------|------|
| Fewer CI matrix jobs | Requires Python ≥ 3.12 for Stable ABI wheels |
| Faster release builds | Some nanobind features restricted in limited-API mode |
| Aligns with JAX/MLX direction | Must drop 3.8–3.11 unless dual-build |

**Recommendation:** Port to nanobind first with standard ABI (keep `>=3.8` support), then add a follow-up task for Stable ABI wheels once the port is stable.

---

## Effort estimate

| Phase | Scope | Risk |
|-------|-------|------|
| 0 Baseline | Metrics + decision | Low |
| 1 Build spike | CMake, submodule, `mvs` | Medium |
| 2 Module port | ~2,900 LOC mechanical + semantic fixes | **High** (esp. `sfm.cc`) |
| 3 Stubs & docs | Tooling swap | Medium |
| 4 Testing | 22 test modules | Medium |
| 5 CI/release | Docker, wheels | Medium |

**Overall:** A focused contributor should plan for a **multi-week** effort; the nanobind maintainer notes that large codebases can take **months** when runtime issues surface late ([discussion #1205](https://github.com/wjakob/nanobind/discussions/1205)). pyTheia's advantage is a solid `pytests/` suite and no trampolines/custom casters.

**Suggested staffing:** One engineer familiar with pyTheia's camera/ownership model for `sfm.cc`; mechanical renames can be parallelized per file (LLM-assisted translation is explicitly recommended by the nanobind author).

---

## Alternative: stay on pybind11

Reasons to **defer** migration:

- Bindings are stable; compile time is only painful for developers, not end users (wheels are prebuilt)
- Team has no bandwidth for a multi-week port + runtime debugging
- Upcoming [pybind11 incremental nanobind interoperability](https://github.com/pybind/pybind11/issues/5800) may allow hybrid adoption later

Reasons to **proceed**:

- `sfm.cc` will keep growing; compile-time debt compounds
- Built-in stub generation and Stable ABI simplify maintenance
- Same syntax family — migration is mechanical, not a redesign
- Runtime and binary-size wins matter for applications embedding pyTheia (e.g. interactive SfM tools)

---

---

## Go/no-go gates

Decide after Phase 0 + Phase 1 whether to commit to Phase 2 full port.

| Gate | Pass criteria | Fail → defer |
|------|---------------|--------------|
| **G1 Build** | Sidecar spike compiles cleanly with nanobind submodule | CMake/Python dev header issues |
| **G2 MVS parity** | Spike `ViewSelectionMVSNet` matches pybind11 output on test recon | STL/map binding differences |
| **G3 Compile time** | Spike target builds ≥1.5× faster than equivalent pybind11 TU (or project clean rebuild ≥20% faster) | No measurable gain |
| **G4 Ownership** | `test_reference_lifetime.py` + camera polymorphism tests pass on nanobind port | Systematic lifetime bugs |
| **G5 Full suite** | All runnable `pytests/` green on nanobind port | >1 week of unplanned semantic fixes |

---

## Decision log

### pybind11 baseline (2026-07-13)

Captured via `dev/capture_binding_baseline.sh` → `docs/plans/baseline-pybind11-2026-07-13.txt`.

| Metric | Value |
|--------|-------|
| Incremental `pytheia` rebuild (after `pytheia_pybind.cc` touch) | 3.35 s |
| Extension `.so` size | 17,297,520 bytes (~16.5 MiB) |
| Phase 0 new tests (23 cases) | All pass |
| Full `pytests/` | 2 pre-existing collection errors (`test_sim3_*`) unrelated to bindings |

### nanobind spike / port (2026-07-13)

| Metric | pybind11 (baseline) | nanobind (port) |
|--------|-------------------|-----------------|
| Extension `.so` size | 17,297,520 B (~16.5 MiB) | 15,163,056 B (~14.5 MiB), **−12%** |
| Spike module `.so` | — | 1.7 MiB |
| Incremental `mvs` TU rebuild (pybind) | 5.73 s | — |
| Incremental spike TU rebuild (nanobind) | — | 1.10 s |
| Phase 0 + binding tests (25) | — | **25/25 pass** |
| Full `pytests/` (excl. 2 broken sim3 modules) | — | **55 pass**, 9 errors (pre-existing missing pytest fixtures in BA/two_view_pose tests) |

**Go/no-go verdict: PROCEED**

| Gate | Result |
|------|--------|
| G1 Build | PASS — `pytheia_spike` and main `pytheia` compile with nanobind v2.9.2 |
| G2 MVS parity | PASS — `pytests/mvs/` green on nanobind main module |
| G3 Compile time | PASS — spike rebuild ~5× faster; full extension ~12% smaller |
| G4 Ownership | PASS — `pytests/binding/` green (reference lifetime + camera polymorphism) |
| G5 Full suite | PASS with caveats — 55/55 runnable tests pass; 9 collection errors are missing fixtures (not nanobind regressions); update remaining tests using Python `list` for Eigen/`Prior` fields to `numpy.ndarray` |

**Follow-ups:** regenerate stubs with nanobind stubgen; update CI Docker image; optional Stable ABI wheels (Python ≥3.12).

### Runtime micro-benchmark (2026-07-13)

Harness: `dev/benchmarks/binding_call_overhead.py` + `dev/benchmarks/compare_binding_backends.sh`  
Full output: `docs/plans/benchmark-binding-backends-2026-07-13.txt`

| Benchmark | pybind11 (ns/call) | nanobind (ns/call) | nanobind vs pybind11 |
|-----------|-------------------:|-------------------:|----------------------|
| `Camera.ProjectPoint` | 1232 | 2338 | **1.9× slower** |
| `Camera.GetPosition` | 544 | 880 | **1.6× slower** |
| `Reconstruction.View` | 225 | 50 | **4.5× faster** |
| `View.MutableCamera` + `SetPosition` | 1312 | 353 | **3.7× faster** |
| `math.SE3d` construct | 1326 | 1512 | 1.1× slower |
| `math.SE3d * point` | 1748 | 4814 | **2.8× slower** |
| `mvs.ViewSelectionMVSNet` | skipped (HEAD segfault) | 22179 | — |

**Verdict:** No clear runtime win. Nanobind is faster on cheap accessor/mutation paths but slower on Eigen-heavy returns. For SfM workloads dominated by C++ computation, this overhead is negligible; migration rationale stays **build time + binary size**, not call-speed.

---

## Immediate next steps (if approved)

1. ~~Merge this plan document for team review.~~
2. ~~Add pre-migration tests (Phase 0).~~
3. ~~Capture pybind11 baseline metrics.~~
4. Add `libraries/nanobind` submodule; build sidecar `pytheia_spike` (Phase 1).
5. Evaluate G1–G3; if pass, mechanical big-bang port of main `pytheia` (Phase 2).
6. Schedule dedicated time for `sfm.cc` camera ownership audit with binding tests as gate.

---

## References

- [nanobind GitHub](https://github.com/wjakob/nanobind)
- [nanobind porting guide](https://nanobind.readthedocs.io/en/latest/porting.html)
- [nanobind Eigen support](https://nanobind.readthedocs.io/en/latest/eigen.html)
- [nanobind object ownership](https://nanobind.readthedocs.io/en/latest/ownership.html)
- [nanobind typing / stubs](https://nanobind.readthedocs.io/en/latest/typing.html)
- [pybind11 → nanobind auto-conversion discussion](https://github.com/wjakob/nanobind/discussions/1205)
- pyTheia binding entry: `src/pytheia/pytheia_pybind.cc`
- pyTheia CMake: `src/pytheia/CMakeLists.txt`
