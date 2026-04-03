# Releases {#chapter-releases}

This page records **pyTheia** releases. It no longer mirrors the historical [TheiaSfM](http://www.theia-sfm.org) upstream changelog; older version notes referred to upstream tarballs and features that this fork no longer ships.

---

## 0.6.0

Major cleanup: smaller surface area, Python-first imaging and features, fewer vendored or optional dependencies in the default build path.

### Removed or deprecated in this line

- **Raster / EXIF in C++:** The `theia/image` stack and **OpenImageIO** integration are gone. Images and EXIF are expected in Python (OpenCV, Pillow, etc.). C++ keeps geometry, reconstructions, cameras, BA, and solvers.
- **Feature and descriptor I/O:** Reading/writing bundled keypoint–descriptor files and **SIFT** key formats has been removed from the library and from Python bindings. Feature detection and matching stay outside the core package; you pass correspondences and use `Feature`, `FeatureCorrespondence`, and estimators as needed.
- **Python API trim:** Descriptor-centric matching helpers (e.g. `KeypointsAndDescriptors`, brute-force / cascade hash matchers, Fisher-vector globals, two-view verification types that depended on that stack) are no longer exposed on `pytheia.matching` / `pytheia.sfm`. **`FeaturesAndMatchesDatabase`**, **`InMemoryFeaturesAndMatchesDatabase`**, **`ReconstructionBuilder`**, and **`ReconstructionBuilderOptions`** are also removed from Python (C++ Theia still uses the match database inside its own reconstruction-builder path).
- **Exports and extras:** Legacy **PMVS** export and other dead or redundant tooling paths have been removed or stripped from docs and CMake where applicable.
- **Documentation:** Sphinx-era pages were replaced with **MkDocs**; narrative docs focus on what pyTheia actually exposes and how to wire Python-side features into SfM.

### Still here

- Core **SfM** pipelines, **bundle adjustment**, **cameras**, **pose / triangulation** solvers, **view graph**, **I/O** for reconstructions (binary/JSON, Bundler, NVM, COLMAP text, PLY, Nerfstudio, SDFStudio, etc.), and **pybind11** Python API for the remaining types and functions.

If you need the old upstream release notes for academic comparison, see the [TheiaSfM repository](https://github.com/sweeneychris/TheiaSfM) tags and history.
