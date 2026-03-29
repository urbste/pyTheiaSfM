# Feature matching (`pytheia.matching`) {#documentation-matching}

The **`pytheia.matching`** submodule holds **data structures** and **options** used when building a [`ViewGraph`](view_graph.md) from putative correspondences. In pyTheia, **descriptor extraction and brute-force / FLANN matching** are usually done in **Python** (OpenCV, [vismatch](https://github.com/gmberton/vismatch), etc.); you then fill these types and call **`EstimateTwoViewInfo`** from [`pytheia.sfm`](estimators.md) for geometric verification.

Bindings: [`matching.cc`](https://github.com/urbste/pyTheiaSfM/blob/master/src/pytheia/matching/matching.cc).

---

## Core types

### `FeatureCorrespondence`

Pair of 2D features: **`feature1`**, **`feature2`**, each a **`pytheia.sfm.Feature`** (pixel location and optional orientation / scale / descriptor).

### `IndexedFeatureMatch`

Integer indices into per-image feature lists plus a scalar **`distance`** (e.g. descriptor distance): **`feature1_ind`**, **`feature2_ind`**, **`distance`**.

### `ImagePairMatch`

Container for one edge in the view graph:

- **`image1`**, **`image2`** — image names (strings).
- **`twoview_info`** — [`TwoViewInfo`](view_graph.md#twoview-info) after geometric estimation.
- **`correspondences`** — list of **`FeatureCorrespondence`**.

### `FeaturesAndMatchesDatabase` / `InMemoryFeaturesAndMatchesDatabase`

Abstract store for **`ImagePairMatch`** entries and optional **camera intrinsics priors** per image name.

**`InMemoryFeaturesAndMatchesDatabase`** (Python-concrete):

- **`PutImagePairMatch`**, **`GetImagePairMatch`**, **`NumMatches`**
- **`ImageNamesOfMatches`**
- **`PutCameraIntrinsicsPrior`**, **`GetCameraIntrinsicsPrior`**, **`ContainsCameraIntrinsicsPrior`**, **`NumCameraIntrinsicsPrior`**, **`ImageNamesOfCameraIntrinsicsPriors`**

Used by C++ matching pipelines; in Python you can use it to stage matches before reconstruction.

---

## `FeatureMatcherOptions`

Options for **Theia’s** batch matcher (C++). pyTheia exposes a **subset** of fields:

| Field | Role |
|-------|------|
| **`num_threads`** | Parallelism for matching. |
| **`keep_only_symmetric_matches`** | Keep only mutual nearest neighbors. |
| **`use_lowes_ratio`** / **`lowes_ratio`** | Lowe’s ratio test on descriptor distances. |
| **`perform_geometric_verification`** | Run 2-view geometry after putative matching. |
| **`min_num_feature_matches`** | Minimum matches to keep an edge. |
| **`geometric_verification_options`** | **`pytheia.sfm.GeometricVerificationOptions`** (thresholds, RANSAC, etc.). |

Other C++ fields (out-of-core paths, cache directories) exist in [`feature_matcher_options.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/matching/feature_matcher_options.h) but are **not** bound in Python.

---

## `GraphMatch`

**`GraphMatch(image_names, match_list)`** — builds an **image connectivity** structure from a list of string pairs `(name_i, name_j)` that have been matched. Used to schedule which image pairs to process in larger collections.

---

## `MatchingStrategy` (Python caveat)

The enum exposes **`GLOBAL`** and **`INCREMENTAL`**, but **both values map to the same underlying C++ strategy (`BRUTE_FORCE`)** in the current bindings. Treat this as a **placeholder** unless the binding is extended.

---

## See also

- [View graph](view_graph.md) — `TwoViewInfo`, edges, filtering  
- [Geometric estimators](estimators.md) — `EstimateTwoViewInfo`, RANSAC types  
- [SfM](sfm.md) — `TrackBuilder`, reconstruction pipelines  
- [Examples showcase](examples_showcase.md) — vismatch + pyTheia workflows  
