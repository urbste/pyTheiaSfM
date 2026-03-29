# Triangulation {#documentation-triangulation}

Triangulation recovers a 3D point from its projections in two or more calibrated views. Theia implements several algebraic and multi-view refinements on top of classic DLT and midpoint closures. In pyTheia these live under **`pytheia.sfm`**, alongside RANSAC-wrapped **robust** triangulation for noisy correspondences.

For **Python** entry points and NumPy types, see [Python API overview](python_wrapper.md). C++ sources: [`src/theia/sfm/triangulation/`](https://github.com/urbste/pyTheiaSfM/tree/master/src/theia/sfm/triangulation).

## Concepts {#triangulation-concepts}

- **Projection matrices** \(P_i \in \mathbb{R}^{3 \times 4}\) map homogeneous 3D points \(X\) to image pixels (or normalized coordinates, depending on how you built \(P\)). Two-view helpers assume consistent conventions with the rest of Theia (see [`fundamental_matrix_utils`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/fundamental_matrix_utils.h) if you only have \(E\) or \(F\)).
- **Output** is a homogeneous 4-vector \([x, y, z, w]^\top\); Euclidean 3D is \((x/w,\, y/w,\, z/w)\) when \(w \neq 0\).
- **Cheirality**: a candidate should lie **in front of** the cameras that observe it. Use the library’s cheirality / angle checks before accepting a point.

## Closed-form and multi-view methods {#triangulation-methods}

| API (Python module `pt.sfm`) | Role |
|------------------------------|------|
| `Triangulate` | Two-view triangulation (“Triangulation Made Easy”, Lindstrom CVPR 2010). |
| `TriangulateDLT` | Two-view DLT (Hartley & Zisserman). |
| `TriangulateMidpoint` | Fast midpoint of rays; unit directions from each camera center. |
| `TriangulateNViewSVD` | \(N\)-view SVD triangulation; approximately minimizes algebraic error. |
| `TriangulateNView` | \(N\)-view L2 algebraic minimization; scalable and often better reprojection error than SVD alone. |
| `EstimateTriangulation` | RANSAC over two-view samples; **inputs:** `RansacParameters`, list of **`Camera`**, list of **2D points** (`Vector2d` / NumPy); **returns:** success, triangulated point, `RansacSummary`. |

**C++ declarations:** [`triangulation.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/triangulation/triangulation.h), robust wrapper [`estimators_wrapper.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/estimators_wrapper.h).

## Quality checks {#triangulation-quality}

- **`IsTriangulatedPointInFrontOfCameras`** — Given a **`FeatureCorrespondence`** (two-image match) and a **relative** rotation / translation between the two views, tests whether the implied structure is in front of both cameras.
- **`SufficientTriangulationAngle`** — Given **unit bearing directions** from several views, returns whether the **maximum angular baseline** between any pair of rays is large enough (parameter: minimum angle in **degrees**). Use this to reject ill-conditioned points before or after bundle adjustment.

`TriangulationMethodType` (MIDPOINT / SVD / L2_MINIMIZATION) appears in reconstruction / track logic where multiple backends are selectable.

## Typical use in a pipeline {#triangulation-pipeline}

1. Estimate relative or absolute **poses** and build **`Camera`** objects with correct intrinsics.
2. For each track, collect **2D observations** and corresponding **projection matrices** (or camera centers + calibrated rays for `TriangulateMidpoint`).
3. **Triangulate** (or `EstimateTriangulation` if matches are noisy).
4. Filter with **cheirality** and **triangulation angle**, then refine with [**Bundle adjustment**](bundle_adjustment.md).

## See also {#triangulation-see-also}

- [Pose](pose.md) — two-view geometry and projection utilities  
- [SfM](sfm.md) — `Reconstruction`, `Track`, `View`  
- [Bundle adjustment](bundle_adjustment.md) — joint refinement of structure and motion  
