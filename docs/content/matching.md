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

**`GraphMatch(image_names, global_descriptors, num_nearest_neighbors_for_global_descriptor_matching)`** — proposes **which image pairs** deserve pairwise (local) feature matching. For each image, it keeps the **k** other images whose **global descriptors** are closest under **squared L2 distance**, then applies **query expansion** (neighbors-of-neighbors). It returns a **deduplicated, sorted** list of **`(name_a, name_b)`** string pairs where **`name_a`** is from the **lower** index and **`name_b`** from the **higher** index in the original `image_names` list (not alphabetical by string).

Arguments:

- **`image_names`** — `list[str]`, one name per image, same length as the descriptor list.
- **`global_descriptors`** — one floating-point descriptor per image: typically a **2D NumPy array** of shape `(n_images, dim)` or an equivalent sequence of 1D vectors (same `dim` for every row). These are **whole-image** embeddings (e.g. from a place-recognition network), not per-keypoint descriptors.
- **`num_nearest_neighbors_for_global_descriptor_matching`** — **k**; capped internally at `n_images - 1`.

Use this to avoid an **all-pairs** \(O(n^2)\) local matching schedule on large collections. Example: [`pyexamples/sfm_pipeline_loftr_aqualoc.py`](https://github.com/urbste/pyTheiaSfM/blob/master/pyexamples/sfm_pipeline_loftr_aqualoc.py) (CosPlace-style global vectors + `GraphMatch`, then LoFTR on the selected pairs).

---

## `MatchingStrategy` (Python caveat)

The enum exposes **`GLOBAL`** and **`INCREMENTAL`**, but **both values map to the same underlying C++ strategy (`BRUTE_FORCE`)** in the current bindings. Treat this as a **placeholder** unless the binding is extended.

---

## See also

- [View graph](view_graph.md) — `TwoViewInfo`, edges, filtering  
- [Geometric estimators](estimators.md) — `EstimateTwoViewInfo`, RANSAC types  
- [SfM](sfm.md) — `TrackBuilder`, reconstruction pipelines  
- [Examples showcase](examples_showcase.md) — vismatch + pyTheia workflows  
