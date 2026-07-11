# pyTheia: pybind11 → nanobind Migration Plan

> **Status:** Investigation / feasibility sketch (July 2026)  
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

**Acceptance:** `mvs` module compiles and `import pytheia.mvs` works.

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

## Immediate next steps (if approved)

1. Merge this plan document for team review.
2. Add `libraries/nanobind` submodule (pinned tag).
3. Implement Phase 1 on a `cursor/nanobind-spike-eb29` branch — port `mvs` only, verify build + import.
4. Port `matching` + `io`, run `pytests/sfm/write_reconstruction_json_test.py` and matching-related tests.
5. Schedule dedicated time for `sfm.cc` with camera ownership audit.

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
