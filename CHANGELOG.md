# Changelog

## 1.3.1 (2026-11-05)

Changes since 1.3.0 (compact reconstruction format, `e51ff49`).

### Added
- **Reproducible RNG seeding.** Default-constructed `RandomNumberGenerator`s
  (used by RANSAC samplers and global estimators) can now be seeded with a fixed
  value instead of the clock, so repeated runs draw identical sample sequences.
  - C++: `theia::SetDefaultRandomSeed(seed)`, `theia::ClearDefaultRandomSeed()`,
    `theia::HasDefaultRandomSeed()` in `theia/util/random.h`.
  - Python: `pt.solvers.SetDefaultRandomSeed`, `pt.solvers.ClearDefaultRandomSeed`,
    `pt.solvers.HasDefaultRandomSeed`.
  - Environment: `THEIA_RANDOM_SEED=<unsigned decimal>` sets the seed at process
    start. An invalid value is ignored and the clock is used.
  - The seed is process-wide and re-applied on every default construction, so
    each RANSAC/estimator run restarts from the same state.
  - Without a seed, behavior is unchanged (clock seeding).

### Fixed
- `BundleAdjuster::SetCameraExtrinsicsConstant` no longer aborts in Ceres when
  the view has no residual block yet (e.g. all its observations were removed and
  it has no gravity prior). The extrinsics block is added before being marked
  constant.

### Changed
- Type stubs (`src/pytheia/pytheia/*.pyi`) regenerated with pybind11-stubgen 3.0.0.
  They now include the new seed functions and previously missing entries such as
  `io.WriteRigPlyFile`. Output ordering differs from the previous stubs.
- The compact-format size test now asserts the measured layout (< 30 B/obs)
  instead of being marked xfail.
- Bumped vulnerable pins in the vendored pybind11 `requirements.txt` files
  (certifi, urllib3, soupsieve, numpy). This does not affect the built package.
